"""
camFinal.py

ArUco detection on the Arducam IMX708, refactored into an importable
ArucoDetector class (get_pose() method) so it can be called from other
code -- previously this only ran as a standalone script.

Three ways to run this file:

  1. As a library:
       from camFinal import ArucoDetector
       detector = ArucoDetector()
       detector.start()
       pose = detector.get_pose()   # dict or None
       detector.stop()

  2. Standalone preview (original behaviour, for eyeballing detection):
       python3 camFinal.py

  3. Live mounting-calibration check against pose_controller.py --
     physically move a real marker and watch the camera-frame AND
     body-frame values update together, to confirm CAMERA_MOUNT_*_DEG
     is set correctly:
       python3 camFinal.py --calibration-check
"""

import argparse
import math
import subprocess
import time
import cv2
import numpy as np
from picamera2 import Picamera2

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def get_screen_resolution():
    """Try to detect the connected display's resolution via xrandr."""
    try:
        output = subprocess.check_output(["xrandr"]).decode()
        for line in output.splitlines():
            if " connected" in line:
                for part in line.split():
                    if "x" in part and part[0].isdigit():
                        res = part.split("+")[0]
                        w, h = res.split("x")
                        return int(w), int(h)
    except Exception:
        pass
    return None


def half_area_window_size(screen_w, screen_h):
    scale = math.sqrt(0.5)
    return int(screen_w * scale), int(screen_h * scale)


def load_calibration(path):
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]


def approximate_camera_matrix(capture_width, capture_height, hfov_deg=100.0, vfov_deg=72.0):
    """Build a camera matrix from the IMX708 B0311 spec sheet's H/V FOV.
    See original docstring notes: this assumes zero lens distortion and
    a centered principal point -- less accurate than real checkerboard
    calibration, but avoids the earlier diagonal-FOV source of error."""
    fx = capture_width / (2 * math.tan(math.radians(hfov_deg / 2)))
    fy = capture_height / (2 * math.tan(math.radians(vfov_deg / 2)))
    cx = capture_width / 2.0
    cy = capture_height / 2.0

    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    dist_coeffs = np.zeros(5, dtype=np.float64)
    return camera_matrix, dist_coeffs


def marker_yaw_from_rvec(rvec):
    """Extract yaw (rotation about the camera's z-axis) from an ArUco
    rotation vector. Duplicated here (also defined in pose_controller.py)
    so camFinal.py has no hard dependency on that file -- keeps this
    module usable standalone."""
    rmat, _ = cv2.Rodrigues(rvec)
    return float(np.arctan2(rmat[1, 0], rmat[0, 0]))


class ArucoDetector:
    """Wraps the camera + ArUco detection loop. Call start() once, then
    get_pose() repeatedly (e.g. once per main-loop iteration or in a
    background thread), then stop() on shutdown."""

    def __init__(self, dict_name="DICT_4X4_50", width=1280, height=720,
                 hfov_deg=100.0, vfov_deg=72.0, marker_size=0.05,
                 z_correction=1.6, exposure_us=8000, gain=2.0,
                 calib_path=None):
        self.width = width
        self.height = height
        self.marker_size = marker_size
        self.z_correction = z_correction

        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        if calib_path:
            self.camera_matrix, self.dist_coeffs = load_calibration(calib_path)
        else:
            self.camera_matrix, self.dist_coeffs = approximate_camera_matrix(
                width, height, hfov_deg=hfov_deg, vfov_deg=vfov_deg
            )

        self.exposure_us = exposure_us
        self.gain = gain
        self.picam2 = None

    def start(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)  # let auto-exposure/focus settle before overriding

        self.picam2.set_controls({
            "ExposureTime": self.exposure_us,
            "AnalogueGain": self.gain,
        })

    def stop(self):
        if self.picam2:
            self.picam2.stop()

    def get_pose(self):
        """Capture one frame and return the first detected marker's
        pose, or None if nothing was detected.

        Returns a dict:
            {id, x, y, z, yaw, frame}
        x, y, z are in metres, camera-frame (OpenCV convention:
        x=right, y=down, z=out of the lens). yaw is radians,
        camera-frame. frame is the raw BGR image, useful for the
        operator video overlay -- discard it if you don't need it.
        """
        pose, _frame = self.capture_and_detect()
        return pose

    def capture_and_detect(self):
        """Like get_pose(), but always returns (pose_or_None, frame) --
        the BGR frame is returned even when no marker is detected, so
        a live preview window can keep showing video while you position
        the marker. get_pose() is a thin wrapper around this for callers
        that only care about the pose."""
        frame = self.picam2.capture_array()  # RGB888
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params
        )

        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if ids is None:
            return None, bgr

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        marker_id = int(ids.flatten()[0])
        x, y, z = tvecs[0][0]
        x *= self.z_correction
        y *= self.z_correction
        z *= self.z_correction
        yaw = marker_yaw_from_rvec(rvecs[0])

        cv2.drawFrameAxes(bgr, self.camera_matrix, self.dist_coeffs,
                           rvecs[0], tvecs[0], self.marker_size * 0.5)

        pose = {
            "id": marker_id,
            "x": float(x), "y": float(y), "z": float(z),
            "yaw": yaw,
            "frame": bgr,
        }
        return pose, bgr


# ---------------------------------------------------------------------
def _run_preview(args):
    """Original standalone behaviour: live window, prints on detection."""
    detector = ArucoDetector(
        dict_name=args.dict, width=args.width, height=args.height,
        marker_size=args.marker_size, z_correction=args.z_correction,
        exposure_us=args.exposure_us, gain=args.gain, calib_path=args.calib,
    )
    detector.start()
    print(f"Camera started ({args.width}x{args.height}), dictionary: {args.dict}")
    print("Press 'q' in the preview window to quit.")

    window_name = "ArUco Detection - IMX708"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    screen_res = get_screen_resolution()
    if screen_res:
        win_w, win_h = half_area_window_size(*screen_res)
    else:
        win_w, win_h = half_area_window_size(args.width, args.height)
    cv2.resizeWindow(window_name, win_w, win_h)

    last_print_time = 0.0
    try:
        while True:
            pose, frame = detector.capture_and_detect()
            if pose:
                now = time.time()
                if now - last_print_time >= 1.0:
                    print(f"aruco detected: id={pose['id']}  "
                          f"x={pose['x']:.3f}m y={pose['y']:.3f}m "
                          f"z={pose['z']:.3f}m yaw={math.degrees(pose['yaw']):.1f} deg")
                    last_print_time = now
            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        cv2.destroyAllWindows()


def _run_calibration_check(args):
    """Live camera-frame -> body-frame comparison, for setting
    CAMERA_MOUNT_*_DEG in pose_controller.py against your real mount.
    Shows the live video feed with the numbers overlaid, plus prints
    them to the console. Press 'q' in the video window to quit."""
    from pose_controller import camera_to_body, camera_to_body_yaw

    detector = ArucoDetector(
        dict_name=args.dict, width=args.width, height=args.height,
        marker_size=args.marker_size, z_correction=args.z_correction,
        exposure_us=args.exposure_us, gain=args.gain, calib_path=args.calib,
    )
    detector.start()
    print("Mounting calibration check -- move the marker to a known")
    print("position (e.g. 'to the platform's right') and confirm the")
    print("body-frame values match what you physically expect.")
    print("Press 'q' in the video window (or Ctrl+C here) to quit.\n")

    window_name = "Mounting Calibration Check"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    screen_res = get_screen_resolution()
    if screen_res:
        win_w, win_h = half_area_window_size(*screen_res)
    else:
        win_w, win_h = half_area_window_size(args.width, args.height)
    cv2.resizeWindow(window_name, win_w, win_h)

    last_print_time = 0.0
    try:
        while True:
            pose, frame = detector.capture_and_detect()

            if pose is None:
                cv2.putText(frame, "no marker detected", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                x_body, y_body, z_body = camera_to_body(pose["x"], pose["y"], pose["z"])
                yaw_body = camera_to_body_yaw(pose["yaw"])

                cam_line = (f"cam:  x={pose['x']:+.3f} y={pose['y']:+.3f} "
                            f"z={pose['z']:+.3f} yaw={math.degrees(pose['yaw']):+.1f}deg")
                body_line = (f"body: surge={x_body:+.3f} sway={y_body:+.3f} "
                             f"heave={z_body:+.3f} yaw={math.degrees(yaw_body):+.1f}deg")

                cv2.putText(frame, cam_line, (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, body_line, (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

                now = time.time()
                if now - last_print_time >= 0.2:
                    print(f"{cam_line}   |   {body_line}")
                    last_print_time = now

            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArUco detection + pose on IMX708 CSI camera")
    parser.add_argument("--dict", default="DICT_4X4_50", choices=ARUCO_DICTS.keys())
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--calib", default=None,
                         help="Path to .npz calibration file (most accurate)")
    parser.add_argument("--marker-size", type=float, default=0.05,
                         help="Marker BLACK SQUARE side length in meters")
    parser.add_argument("--z-correction", type=float, default=1.6,
                         help="Empirical multiplier on x,y,z (default: 1.6)")
    parser.add_argument("--exposure-us", type=int, default=8000,
                         help="Manual exposure time in microseconds")
    parser.add_argument("--gain", type=float, default=2.0,
                         help="Manual analogue gain")
    parser.add_argument("--calibration-check", action="store_true",
                         help="Run the live camera-to-body mounting check "
                              "instead of the preview window")
    args = parser.parse_args()

    if args.calibration_check:
        _run_calibration_check(args)
    else:
        _run_preview(args)
