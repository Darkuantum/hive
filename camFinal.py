'''
-> just run python3 camFinal.py

get_screen_resolution()
    Detects the connected display's resolution via the `xrandr` command
    (e.g. 800x480 for a 7" screen). Used only to size the preview window.
    Returns None if detection fails, so the rest of the script degrades
    gracefully instead of crashing.
 
half_area_window_size(screen_w, screen_h)
    Shrinks the screen resolution so the preview window covers HALF the
    screen's pixel AREA (not half the width/height — that would only be
    a quarter of the area). Since area scales with the square of a linear
    scale factor, each dimension is multiplied by sqrt(0.5) ≈ 0.707.
 
approximate_camera_matrix(capture_width, capture_height, hfov_deg, vfov_deg)
    Builds a "camera matrix" — the set of numbers OpenCV needs to convert
    pixel measurements into real-world distances:
 
        [ fx   0   cx ]
        [ 0    fy  cy ]
        [ 0    0   1  ]
 
        fx, fy = focal length in PIXELS (how many pixels wide a 1-meter
                 object would appear at 1 meter away), computed from the
                 camera's known horizontal/vertical field of view (spec
                 sheet: 100° H x 72° V) using the pinhole camera formula:
                     f = size / (2 * tan(FOV / 2))
        cx, cy = optical center, assumed to be the exact middle of frame
 
    NOTE: This is an APPROXIMATION derived from the camera's spec sheet


main loop sequence:
frame = picam2.capture_array()
        Grabs one frame from the camera (RGB pixel array).
 
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, aruco_params)
        Scans the frame for ArUco markers.
        corners = pixel coordinates of each marker's 4 edges
        ids     = which marker ID(s) were found (None if nothing detected)
 
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, args.marker_size, camera_matrix, dist_coeffs
    )
        Converts 2D pixel corners into 3D real-world position, using the
        marker's known physical size + the camera matrix. Returns:
        tvecs = translation (x, y, z position in meters)
        rvecs = rotation (orientation) — used here just to draw 3D axes
 
    x, y, z = tvecs[i][0]
    x *= args.z_correction
    y *= args.z_correction
    z *= args.z_correction
        Applies the empirical correction factor (see --z-correction above)
        before printing/displaying the result.
 
    cv2.imshow(window_name, bgr)
        Displays the annotated frame. Note: Picamera2 outputs RGB, but
        OpenCV's display expects BGR, hence the color conversion just
        before this line.
'''


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
                        res = part.split("+")[0]  # strip position offset like "800x480+0+0"
                        w, h = res.split("x")
                        return int(w), int(h)
    except Exception:
        pass
    return None


def half_area_window_size(screen_w, screen_h):
    """
    Scale down (screen_w, screen_h) so the resulting window has HALF the
    screen's pixel area, while keeping the same aspect ratio.
    Area scales with the square of the linear scale factor, so to halve
    the area we scale each dimension by sqrt(0.5) (~0.707), not by 0.5.
    """
    scale = math.sqrt(0.5)
    return int(screen_w * scale), int(screen_h * scale)


def load_calibration(path):
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]


def approximate_camera_matrix(capture_width, capture_height, hfov_deg=100.0, vfov_deg=72.0):
    """
    Build a camera matrix from the IMX708 B0311 spec sheet's HORIZONTAL and
    VERTICAL field of view (measured independently, not derived from the
    diagonal). This ties fx directly to capture_width and fy directly to
    capture_height, with no dependency on full-sensor resolution or pixel
    size -- avoiding the earlier source of error where an assumed width
    scaling factor (against the full 4608px sensor) didn't match how
    Picamera2 actually bins/crops the frame.

    This still assumes zero lens distortion and a centered principal point,
    so it's less accurate than a real checkerboard calibration -- but it's
    a more direct, less error-prone approximation than the diagonal-FOV
    method.

    hfov_deg: horizontal field of view in degrees (spec: 100 for B0311)
    vfov_deg: vertical field of view in degrees   (spec: 72 for B0311)
    """
    fx = capture_width / (2 * math.tan(math.radians(hfov_deg / 2)))
    fy = capture_height / (2 * math.tan(math.radians(vfov_deg / 2)))
    cx = capture_width / 2.0
    cy = capture_height / 2.0

    camera_matrix = np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,  0,  1],
    ], dtype=np.float64)

    dist_coeffs = np.zeros(5, dtype=np.float64)  # assume no distortion

    return camera_matrix, dist_coeffs


def main():
    parser = argparse.ArgumentParser(description="ArUco detection + pose on IMX708 CSI camera")
    parser.add_argument("--dict", default="DICT_4X4_50", choices=ARUCO_DICTS.keys(),
                         help="ArUco dictionary to use (default: DICT_4X4_50)")
    parser.add_argument("--width", type=int, default=1280, help="Capture width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="Capture height (default: 720)")
    parser.add_argument("--no-preview", action="store_true",
                         help="Run headless, no on-screen preview window")
    parser.add_argument("--notify-cooldown", type=float, default=1.0,
                         help="Seconds between repeated 'aruco detected' prints (default: 1.0)")
    parser.add_argument("--calib", default=None,
                         help="Path to .npz file with camera_matrix and dist_coeffs "
                              "(most accurate; from a real checkerboard calibration)")
    parser.add_argument("--approx-calib", action="store_true", default=True,
                         help="Skip checkerboard calibration and instead build an "
                              "approximate camera matrix from the IMX708 B0311's "
                              "published H/V field of view. Enabled by default.")
    parser.add_argument("--hfov", type=float, default=100.0,
                         help="Horizontal field of view in degrees, for --approx-calib "
                              "(default: 100, per the B0311 spec sheet)")
    parser.add_argument("--vfov", type=float, default=72.0,
                         help="Vertical field of view in degrees, for --approx-calib "
                              "(default: 72, per the B0311 spec sheet)")
    parser.add_argument("--marker-size", type=float, default=0.05,
                         help="Marker side length in meters -- measure the BLACK SQUARE "
                              "only, not including the white border. Used with --calib "
                              "or --approx-calib (default: 0.05 = 5cm)")
    parser.add_argument("--z-correction", type=float, default=1.6,
                         help="Multiplier applied to x,y,z after estimation, empirically "
                              "calibrated against a tape measure for this camera+marker "
                              "setup (default: 1.6)")
    args = parser.parse_args()

    # --- Set up the CSI camera ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # let auto-exposure/focus settle

    # --- Set up ArUco detector (old-style API, for OpenCV < 4.7) ---
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[args.dict])
    aruco_params = cv2.aruco.DetectorParameters_create()

    # --- Calibration (needed for real-world x,y,z) ---
    camera_matrix, dist_coeffs = None, None
    if args.calib:
        camera_matrix, dist_coeffs = load_calibration(args.calib)
        print(f"Loaded calibration from {args.calib} -> real x,y,z (meters) enabled")
    elif args.approx_calib:
        camera_matrix, dist_coeffs = approximate_camera_matrix(
            args.width, args.height, hfov_deg=args.hfov, vfov_deg=args.vfov
        )
        print(f"Using approximate camera matrix (HFOV={args.hfov}\u00b0, VFOV={args.vfov}\u00b0, "
              f"no distortion correction) -> real x,y,z enabled but LESS ACCURATE than --calib")
        if args.z_correction != 1.0:
            print(f"Applying manual z-correction multiplier: {args.z_correction}")
    else:
        print("No --calib or --approx-calib provided -> showing pixel coordinates only")

    print(f"Camera started ({args.width}x{args.height}), dictionary: {args.dict}")
    print("Press Ctrl+C to quit." if args.no_preview else "Press 'q' in the preview window to quit.")

    if not args.no_preview:
        window_name = "ArUco Detection - IMX708"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        screen_res = get_screen_resolution()
        if screen_res:
            screen_w, screen_h = screen_res
            win_w, win_h = half_area_window_size(screen_w, screen_h)
            cv2.resizeWindow(window_name, win_w, win_h)
            print(f"Detected screen {screen_w}x{screen_h}, preview window set to "
                  f"{win_w}x{win_h} (half the screen area)")
        else:
            win_w, win_h = half_area_window_size(args.width, args.height)
            cv2.resizeWindow(window_name, win_w, win_h)
            print(f"Could not detect screen resolution, using half capture area instead: "
                  f"{win_w}x{win_h}")

    last_print_time = 0.0

    try:
        while True:
            frame = picam2.capture_array()  # RGB888 numpy array

            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            if ids is not None:
                now = time.time()
                should_print = (now - last_print_time) >= args.notify_cooldown

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                if camera_matrix is not None:
                    # Real-world pose: x,y,z in meters, relative to the camera
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, args.marker_size, camera_matrix, dist_coeffs
                    )
                    for i, marker_id in enumerate(ids.flatten()):
                        x, y, z = tvecs[i][0]
                        x *= args.z_correction
                        y *= args.z_correction
                        z *= args.z_correction
                        if should_print:
                            print(f"aruco detected: id={marker_id}  "
                                  f"x={x:.3f}m y={y:.3f}m z={z:.3f}m")
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                           rvecs[i], tvecs[i], args.marker_size * 0.5)
                        c = corners[i][0]
                        label_pos = (int(c[:, 0].mean()) - 40, int(c[:, 1].mean()) - 15)
                        cv2.putText(frame, f"id={marker_id} z={z:.2f}m", label_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    # No calibration: only pixel-space position available
                    for i, marker_id in enumerate(ids.flatten()):
                        c = corners[i][0]
                        px = float(c[:, 0].mean())
                        py = float(c[:, 1].mean())
                        if should_print:
                            print(f"aruco detected: id={marker_id}  "
                                  f"pixel_x={px:.0f} pixel_y={py:.0f}  "
                                  f"(no z / real-world coords without --calib)")

                if should_print:
                    last_print_time = now

            if not args.no_preview:
                # Picamera2 gives RGB; OpenCV's imshow expects BGR for correct colors
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow(window_name, bgr)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        if not args.no_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
