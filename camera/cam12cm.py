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


def enhance_low_light(frame_rgb):
    """
    Boost local contrast (CLAHE) and lightly denoise a frame, to help ArUco
    detection in dim/hazy/underwater conditions where the marker has low
    contrast against its surroundings. Operates on a copy; does not modify
    the original frame (which is still needed in RGB for display).

    Returns a grayscale, enhanced frame suitable for detectMarkers().
    """
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

    # CLAHE: contrast-limited adaptive histogram equalization. Unlike a
    # global histogram equalization, this adapts per local region, so it
    # boosts contrast in dim areas without blowing out already-bright ones.
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    # Mild bilateral denoise: reduces sensor noise (common at high gain in
    # low light) while preserving sharp edges -- important because ArUco
    # detection depends on clean black/white edge transitions.
    denoised = cv2.bilateralFilter(enhanced, d=5, sigmaColor=50, sigmaSpace=50)

    return denoised


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
    parser.add_argument("--marker-size", type=float, default=0.10,
                         help="Marker side length in meters -- measure the BLACK SQUARE "
                              "only (the 1cm white quiet-zone margin is NOT included). "
                              "Used with --calib or --approx-calib (default: 0.10 = 10cm, "
                              "matching the team's 4x4 ArUco label spec)")
    parser.add_argument("--z-correction", type=float, default=1.6,
                         help="Multiplier applied to x,y,z after estimation, empirically "
                              "calibrated against a tape measure for this camera+marker "
                              "setup (default: 1.6)")
    parser.add_argument("--enhance-low-light", action="store_true", default=True,
                         help="Apply CLAHE contrast enhancement + mild denoising before "
                              "detection. Enabled by default for underwater use "
                              "(daytime, 2-5m depth, open water).")
    parser.add_argument("--exposure-us", type=int, default=20000,
                         help="Manually fix exposure time in microseconds. Default: "
                              "20000 (20ms) -- a moderate starting point for daytime "
                              "underwater light at 2-5m. Raise if frames look dark, "
                              "lower if markers blur during movement. Pass 0 to leave "
                              "auto-exposure on instead.")
    parser.add_argument("--gain", type=float, default=4.0,
                         help="Manually fix analogue gain. Default: 4.0 -- a moderate "
                              "boost for underwater daytime light without adding too "
                              "much sensor noise. Raise for darker/deeper/murkier "
                              "conditions, lower if frames look noisy/grainy. Pass 0 "
                              "to leave auto-gain on instead.")
    args = parser.parse_args()

    # --- Set up the CSI camera ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # let auto-exposure/focus settle

    # Manual exposure/gain override -- helpful in low light where
    # auto-exposure can hunt/oscillate between frames, causing intermittent
    # over/under-exposed frames and inconsistent detection. Pass 0 to
    # either flag to leave that control on auto instead.
    controls = {}
    if args.exposure_us and args.exposure_us > 0:
        controls["ExposureTime"] = args.exposure_us
        controls["AeEnable"] = False
    if args.gain and args.gain > 0:
        controls["AnalogueGain"] = args.gain
    if controls:
        picam2.set_controls(controls)
        print(f"Manual exposure controls set: {controls}")
    else:
        print("Using camera auto-exposure/auto-gain")

    # --- Set up ArUco detector (old-style API, for OpenCV < 4.7) ---
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[args.dict])
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Sub-pixel corner refinement: slightly slower per frame, but corner
    # positions are noticeably more stable/accurate, which directly improves
    # both detection consistency and pose estimation accuracy.
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.cornerRefinementWinSize = 5
    aruco_params.cornerRefinementMaxIterations = 30
    aruco_params.cornerRefinementMinAccuracy = 0.1

    # Widen the adaptive thresholding window range slightly -- helps catch
    # markers under uneven lighting (shadows, glare) which is a common
    # cause of intermittent/flaky detection.
    aruco_params.adaptiveThreshWinSizeMin = 3
    aruco_params.adaptiveThreshWinSizeMax = 43
    aruco_params.adaptiveThreshWinSizeStep = 4

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

            if args.enhance_low_light:
                detect_input = enhance_low_light(frame)
            else:
                detect_input = frame

            corners, ids, _ = cv2.aruco.detectMarkers(detect_input, aruco_dict, parameters=aruco_params)

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
