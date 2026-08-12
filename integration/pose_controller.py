"""
pose_controller.py

Turns a raw ArUco detection (camera-frame x, y, z) into a velocity setpoint
(vx, vy, yaw_rate) ready to hand to mavlink_interface.send_velocity().

This is the "Frame transform + PID" box from our architecture diagram --
it sits between camFinal.py (raw pose) and the decision engine / MAVLink
layer (which sends the command).

Two separate jobs happen here, kept as two separate pieces of code on
purpose, since they solve different problems:

  1. camera_to_body()  -- FRAME TRANSFORM
     Converts a pose from camera coordinates (defined by which way the
     lens points) into platform body coordinates (defined by the
     platform's own surge/sway/heave axes). Camera and platform axes are
     almost never perfectly aligned once mounted, so skipping this step
     means your controller could push the wrong direction with total
     confidence.

  2. PID / PoseController -- CONTROL LAW
     Once you know the AUV's position in the RIGHT coordinate frame, this
     decides how hard/fast to correct for it -- proportional to the
     error, accumulating for persistent drift, damping to avoid overshoot.
"""

import time
import numpy as np


# ---------------------------------------------------------------------
# 1. FRAME TRANSFORM
# ---------------------------------------------------------------------
#
# IMPORTANT: the angles below are placeholders. They describe how the
# camera is rotated relative to the platform's own body frame (surge =
# forward, sway = right, heave = down). You need to set these to match
# YOUR physical mounting -- measure/estimate them once, then verify
# empirically (see the calibration check at the bottom of this file).
#
# Convention used here:
#   Camera frame (OpenCV):  x = right in image, y = down in image,
#                            z = straight out of the lens
#   Body frame (platform):  x = surge (forward), y = sway (right),
#                            z = heave (down)
#
CAMERA_MOUNT_ROLL_DEG = 0.0    # rotation of camera around its own x-axis
CAMERA_MOUNT_PITCH_DEG = 0.0   # rotation around its own y-axis
CAMERA_MOUNT_YAW_DEG = 0.0     # live-calibrated 2026-08-13: held the marker
                                # square-on to the camera and read yaw_body
                                # via hardware.py's yaw_debug -- the old
                                # 90deg placeholder was turning that into a
                                # false ~90deg body-yaw error, saturating the
                                # yaw PID regardless of the marker's real
                                # orientation. 90deg was clearly wrong either
                                # way, but treat 0.0 here as a rough fix, not
                                # a trusted calibration: estimatePoseSingleMarkers()
                                # has a real two-solution ambiguity for a
                                # near-head-on marker (~180deg apart), and a
                                # repeat of this same square-on test on a
                                # different detection session read ~178deg
                                # instead of ~0deg. Don't trust either number
                                # until camFinal.py picks between the two PnP
                                # solutions by reprojection error instead of
                                # whichever one estimatePoseSingleMarkers()
                                # happens to return first.


def _rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Standard XYZ Euler rotation matrix, in degrees."""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


# Precomputed once, since the mounting doesn't change at runtime
_R_CAM_TO_BODY = _rotation_matrix(
    CAMERA_MOUNT_ROLL_DEG, CAMERA_MOUNT_PITCH_DEG, CAMERA_MOUNT_YAW_DEG
)


def camera_to_body(x_cam, y_cam, z_cam):
    """Convert a camera-frame pose into platform body-frame pose."""
    v_cam = np.array([x_cam, y_cam, z_cam])
    x_body, y_body, z_body = _R_CAM_TO_BODY @ v_cam
    # Cast off numpy.float64 back to plain float -- left as numpy, this
    # propagates into PID output and eventually the Flask /api/state
    # response, where numpy scalars (numpy>=2.0's numpy.bool_/float64)
    # aren't JSON-serializable and 500 the endpoint the instant the
    # engine starts actively controlling.
    return float(x_body), float(y_body), float(z_body)


def marker_yaw_from_rvec(rvec):
    """Extract the marker's yaw angle (rotation about the vertical/z axis)
    from the ArUco pose estimate's rotation vector. Call this in
    camFinal.py right after cv2.aruco.estimatePoseSingleMarkers(), e.g.:

        import cv2
        yaw_cam = marker_yaw_from_rvec(rvecs[i])

    This is the angle you were already computing implicitly for
    drawFrameAxes() but not extracting as a number -- this is that
    missing piece. Returns radians, camera-frame (apply the same mount
    yaw offset as position when converting to body frame -- see
    camera_to_body_yaw() below)."""
    import cv2
    rmat, _ = cv2.Rodrigues(rvec)
    return np.arctan2(rmat[1, 0], rmat[0, 0])


def camera_to_body_yaw(yaw_cam):
    """Apply the camera's mounting yaw offset to a detected marker yaw
    angle, same idea as camera_to_body() but for orientation rather
    than position -- a pure rotation only needs the yaw component of
    the mount offset, not the full 3D rotation matrix.

    yaw_cam is negated here -- live-checked 2026-08-13: rotating the
    marker clockwise (as seen from the camera) reads yaw_cam positive,
    but MANUAL_CONTROL.r positive is clockwise (MAVLink spec) and the
    control law wants the vehicle to turn the SAME way the marker did
    (see PoseController's docstring), so a positive yaw_cam should
    produce a positive r. Without the negation it produced negative r
    instead -- the yaw correction was turning the vehicle away from
    matching the marker's orientation, not toward it."""
    return float(-yaw_cam + np.radians(CAMERA_MOUNT_YAW_DEG))


def _wrap_pi(angle):
    """Wrap an angle (radians) to [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


DEFAULT_YAW_SNAP_AXES = 4


def snapped_yaw_error(yaw_body, n_axes=DEFAULT_YAW_SNAP_AXES):
    """Signed shortest yaw error to the nearest of n_axes equivalent headings.

    n_axes=4 (hive with 4 symmetric openings) -> targets every pi/2. The error
    is invariant under yaw_body -> yaw_body + pi: a 180deg ArUco PnP flip maps
    onto another valid opening, so it yields the SAME error and cannot saturate
    the yaw PID. Returns radians in (-pi/2, pi/2]; for n_axes=4, (-pi/4, pi/4].
    n_axes < 2 disables snapping (targets heading 0)."""
    if n_axes < 2:
        return _wrap_pi(-yaw_body)
    step = 2.0 * np.pi / n_axes
    nearest = round(yaw_body / step) * step
    return _wrap_pi(nearest - yaw_body)


# ---------------------------------------------------------------------
# 2. PID CONTROLLER
# ---------------------------------------------------------------------

class PID:
    """A single-axis PID controller. One instance per controlled axis
    (you'll want one for x/sway correction and one for y/surge
    correction -- see PoseController below)."""

    def __init__(self, kp, ki, kd, output_limit=0.5, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit      # clamp final output (m/s)
        self.integral_limit = integral_limit  # prevent integral windup

        self._integral = 0.0
        self._prev_error = None

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral (clamped so a long-standing error can't wind up
        # forever and cause a huge overshoot once it's finally corrected)
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative (based on rate of change of error)
        if self._prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))
        return float(output)

    def reset(self):
        """Call this when re-entering a controlling state (e.g. going
        from SEARCHING back into ALIGNING) so old integral/derivative
        history doesn't cause a jolt."""
        self._integral = 0.0
        self._prev_error = None


class PoseController:
    """Combines the frame transform and three PID loops (surge, sway,
    yaw) into one call: raw camera pose in, velocity setpoint out.

    Yaw target is 0 -- meaning the net's own heading reference is
    aligned with the AUV's marker orientation, since the goal is for
    the net to rotate to face the same way as the incoming AUV, not to
    hold some fixed compass heading."""

    def __init__(self, kp=0.6, ki=0.05, kd=0.15, output_limit=0.4,
                 yaw_kp=0.8, yaw_ki=0.0, yaw_kd=0.1, yaw_output_limit=0.6,
                 yaw_snap_axes=DEFAULT_YAW_SNAP_AXES):
        self.pid_surge = PID(kp, ki, kd, output_limit=output_limit)
        self.pid_sway = PID(kp, ki, kd, output_limit=output_limit)
        # Yaw often responds differently than translation (platform
        # inertia around its vertical axis vs. linear drag), so it gets
        # its own gains and output limit rather than sharing surge/sway's.
        # No integral term by default (yaw_ki=0) -- a slowly-rotating
        # AUV target doesn't need windup correction the way a steady
        # current pushing the platform sideways does; add it back if
        # you see persistent steady-state yaw error in testing.
        self.pid_yaw = PID(yaw_kp, yaw_ki, yaw_kd, output_limit=yaw_output_limit)

        self.yaw_snap_axes = yaw_snap_axes
        self.snap_hysteresis_rad = np.radians(10.0)   # boundary hysteresis (doubled-angle space)
        self._snap_locked_axis = None                  # 0.0 or pi (the two opening axes)

    def _snap_yaw_error(self, yaw_body):
        """4-axis snap error for the yaw PID, with boundary hysteresis.

        Flip-invariant via doubled-angle space (a +pi yaw flip = +2pi there =
        no-op), so an ArUco PnP 180deg flip yields the identical error and
        cannot saturate the yaw PID. Hysteresis stops the target axis toggling
        when the vessel sits near a 45deg cell boundary."""
        if self.yaw_snap_axes < 2:
            return _wrap_pi(-yaw_body)
        a = _wrap_pi(2.0 * yaw_body)            # doubled-angle: flip-invariant
        d0 = abs(_wrap_pi(a - 0.0))
        dpi = abs(_wrap_pi(a - np.pi))
        nearest = 0.0 if d0 <= dpi else np.pi
        if self._snap_locked_axis is None:
            self._snap_locked_axis = nearest
        else:
            d_locked = abs(_wrap_pi(a - self._snap_locked_axis))
            d_nearest = d0 if nearest == 0.0 else dpi
            if nearest != self._snap_locked_axis and d_nearest < d_locked - self.snap_hysteresis_rad:
                self._snap_locked_axis = nearest
        return _wrap_pi(self._snap_locked_axis - a) / 2.0

    def compute(self, x_cam, y_cam, z_cam, yaw_cam, dt):
        """Returns (vx, vy, yaw_rate) ready for
        mavlink_interface.send_velocity(). yaw_cam is the marker's
        detected yaw angle in camera frame, radians (see
        marker_yaw_from_rvec() above)."""
        x_body, y_body, _z_body = camera_to_body(x_cam, y_cam, z_cam)
        yaw_body = camera_to_body_yaw(yaw_cam)

        # Target is 0 (centered / aligned) on all three axes
        # error_surge/error_sway sign flipped 2026-08-13 at the user's
        # request, live-tested manually: the un-flipped sign was
        # confirmed backwards for both axes. NOTE: this does not fix
        # the deeper cross-wiring issue found the same session -- x_body
        # (labeled surge) is currently fed camera left/right position,
        # not real depth (z_cam is discarded entirely, see
        # camera_to_body()'s _z_body) -- that redesign (routing z_cam
        # into surge, plus deciding a standoff-distance target instead
        # of raw-zero) was explicitly deferred, not done here.
        error_surge = x_body
        error_sway = y_body
        error_yaw = self._snap_yaw_error(yaw_body)

        vx = self.pid_surge.update(error_surge, dt)
        vy = self.pid_sway.update(error_sway, dt)
        yaw_rate = self.pid_yaw.update(error_yaw, dt)

        return vx, vy, yaw_rate

    def reset(self):
        self.pid_surge.reset()
        self.pid_sway.reset()
        self.pid_yaw.reset()
        self._snap_locked_axis = None


# ---------------------------------------------------------------------
# Calibration check -- run this standalone to sanity-check your mounting
# angles before trusting the transform in the full system
# ---------------------------------------------------------------------
if __name__ == '__main__':
    print("Frame transform sanity check")
    print("Enter a camera-frame pose (as if the AUV were detected there)")
    print("and confirm the resulting body-frame values match what you'd")
    print("physically expect for your mounting.\n")

    # Example: AUV appears 0.3m to the right in the image
    x_cam, y_cam, z_cam = 0.3, 0.0, 1.0
    x_body, y_body, z_body = camera_to_body(x_cam, y_cam, z_cam)
    print(f"camera(x={x_cam}, y={y_cam}, z={z_cam})  ->  "
          f"body(surge={x_body:.3f}, sway={y_body:.3f}, heave={z_body:.3f})")
    print("Does 'sway' match the direction you'd expect for a real AUV")
    print("appearing to the right in the camera image? If not, adjust")
    print("CAMERA_MOUNT_*_DEG at the top of this file and re-run.\n")

    # Quick PID demo with a fixed, decaying synthetic error on all
    # three axes -- surge/sway position plus a yaw misalignment
    print("PID demo (synthetic, decaying error, no real camera needed):")
    controller = PoseController()
    fake_x_cam = 0.5
    fake_yaw_cam = np.radians(30)  # AUV starts 30 deg misaligned
    dt = 0.1
    for i in range(10):
        vx, vy, yaw_rate = controller.compute(
            fake_x_cam, 0.0, 1.0, fake_yaw_cam, dt
        )
        print(f"  step {i}: x_cam={fake_x_cam:+.3f}  "
              f"yaw_cam={np.degrees(fake_yaw_cam):+.1f} deg  ->  "
              f"vx={vx:+.3f} vy={vy:+.3f} yaw_rate={yaw_rate:+.3f}")
        fake_x_cam *= 0.8      # pretend the position error is shrinking
        fake_yaw_cam *= 0.8    # and the yaw misalignment too
        time.sleep(0.05)
