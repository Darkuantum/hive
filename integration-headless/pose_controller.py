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
CAMERA_MOUNT_YAW_DEG = 90.0    # set from the live right/left test -- verify
                                # sign against the real net before trusting it


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
    return x_body, y_body, z_body


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
    the mount offset, not the full 3D rotation matrix."""
    return yaw_cam + np.radians(CAMERA_MOUNT_YAW_DEG)


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
        return output

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
                 yaw_kp=0.8, yaw_ki=0.0, yaw_kd=0.1, yaw_output_limit=0.6):
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

    def compute(self, x_cam, y_cam, z_cam, yaw_cam, dt):
        """Returns (vx, vy, yaw_rate) ready for
        mavlink_interface.send_velocity(). yaw_cam is the marker's
        detected yaw angle in camera frame, radians (see
        marker_yaw_from_rvec() above)."""
        x_body, y_body, _z_body = camera_to_body(x_cam, y_cam, z_cam)
        yaw_body = camera_to_body_yaw(yaw_cam)

        # Target is 0 (centered / aligned) on all three axes
        error_surge = -x_body
        error_sway = -y_body
        error_yaw = -yaw_body

        vx = self.pid_surge.update(error_surge, dt)
        vy = self.pid_sway.update(error_sway, dt)
        yaw_rate = self.pid_yaw.update(error_yaw, dt)

        return vx, vy, yaw_rate

    def reset(self):
        self.pid_surge.reset()
        self.pid_sway.reset()
        self.pid_yaw.reset()


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
