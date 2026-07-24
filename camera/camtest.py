"""
camTest.py -- Trial-logging test harness for ArUco detection experiments.

Built on top of camFinal.py's detection/pose pipeline, but instead of
continuously printing every frame, this logs ONE ROW PER TRIAL to a CSV
file, on keypress -- matching how you actually run the experiments
(position the marker at a known distance/angle/lux, then log N trials).

WHY A KEYPRESS-DRIVEN TRIAL MODEL, NOT AUTO-LOGGING EVERY FRAME:
  Your Experiment 1-4 designs are all "5 trials per condition". Auto-logging
  every frame gives you hundreds of correlated readings per condition (not
  independent trials) and, worse, never logs a FAILURE -- if the marker
  isn't detected, nothing would get printed at all, and your success-rate
  denominator would be wrong. This script logs a trial (success OR failure)
  every time you press a key, so your CSV row count == your actual trial
  count, and failures are captured.

USAGE:
  python3 camTest.py --condition-label "clear_20cm" --distance-cm 20 \
      --csv results/exp1_range.csv

  Then, for each trial:
    - position the marker as required for that trial
    - press SPACE to log a trial using whatever the camera currently sees
      (records Detected=Y with pose, or Detected=N if nothing is in frame)
    - press 'n' to force-log a trial as Detected=N even if something WAS
      detected (e.g. you judge it a false positive -- wrong ID, garbage pose)
    - press 'u' to log the last few frames' exposure/gain/lux metadata
      (see LOGGING LIGHTING CONDITIONS below)
    - press 'q' to quit

  Change --condition-label / --distance-cm / --angle-deg / --lateral-pct /
  --lux for each new block of trials, or just edit the CSV's condition
  columns after the fact -- whichever is less friction on the day.

LOGGING LIGHTING CONDITIONS:
  Picamera2/libcamera estimates lux as part of its own auto-exposure
  algorithm. Every logged trial also records ExposureTime, AnalogueGain,
  and Lux from capture_metadata() at the moment you pressed the key, so
  you get a lighting readout for free, time-aligned with the trial --
  no separate lux meter required unless you want a cross-check.

CSV COLUMNS:
  timestamp, condition_label, distance_cm, angle_deg, lateral_pct,
  detected, marker_id, x_m, y_m, z_m, exposure_us, analogue_gain, lux,
  underwater_tuning, notes
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime

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

CSV_FIELDS = [
    "timestamp", "condition_label", "distance_cm", "angle_deg", "lateral_pct",
    "detected", "marker_id", "x_m", "y_m", "z_m",
    "exposure_us", "analogue_gain", "lux",
    "underwater_tuning", "notes",
]


def approximate_camera_matrix(capture_width, capture_height, hfov_deg=100.0, vfov_deg=72.0):
    """Same approach as camFinal.py -- direct H/V FOV camera matrix approximation."""
    fx = capture_width / (2 * math.tan(math.radians(hfov_deg / 2)))
    fy = capture_height / (2 * math.tan(math.radians(vfov_deg / 2)))
    cx = capture_width / 2.0
    cy = capture_height / 2.0
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros(5, dtype=np.float64)
    return camera_matrix, dist_coeffs


def load_calibration(path):
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]


def build_aruco_params(underwater_tuning):
    """
    Default params if underwater_tuning is False.
    Widened adaptive-threshold range + subpixel refinement + looser error
    correction if True -- see the underwater tuning discussion for why
    each of these helps with blur/contrast loss.
    """
    params = cv2.aruco.DetectorParameters_create()
    if underwater_tuning:
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 5
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.errorCorrectionRate = 0.8
    return params


def preprocess_underwater(frame_rgb):
    """CLAHE contrast enhancement + edge-preserving denoise, then to grayscale."""
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    denoised = cv2.bilateralFilter(enhanced, d=5, sigmaColor=50, sigmaSpace=50)
    return denoised


def ensure_csv(path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    is_new = not os.path.exists(path)
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    if is_new:
        writer.writeheader()
        f.flush()
    return f, writer


def main():
    parser = argparse.ArgumentParser(description="Trial-logging ArUco test harness")
    parser.add_argument("--dict", default="DICT_4X4_50", choices=ARUCO_DICTS.keys())
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--calib", default=None,
                         help="Path to .npz with camera_matrix/dist_coeffs (real checkerboard calib)")
    parser.add_argument("--hfov", type=float, default=100.0)
    parser.add_argument("--vfov", type=float, default=72.0)
    parser.add_argument("--marker-size", type=float, default=0.05,
                         help="Marker BLACK SQUARE side length in meters")
    parser.add_argument("--z-correction", type=float, default=1.6,
                         help="Empirical multiplier on x,y,z (see camFinal.py notes)")
    parser.add_argument("--underwater-tuning", action="store_true",
                         help="Enable CLAHE+bilateral preprocessing and tuned ArUco params")
    parser.add_argument("--no-preview", action="store_true")

    # --- experiment/trial labeling ---
    parser.add_argument("--csv", required=True, help="Path to CSV file to append trial rows to")
    parser.add_argument("--condition-label", default="",
                         help="Free-text label for this test block, e.g. 'clear_water', 'dark_acrylic'")
    parser.add_argument("--distance-cm", type=float, default=None,
                         help="Target/ruler-measured Z distance for this block of trials")
    parser.add_argument("--angle-deg", type=float, default=None,
                         help="Marker tilt angle for this block of trials (Experiment 2)")
    parser.add_argument("--lateral-pct", type=float, default=None,
                         help="Marker lateral offset as %% of half-frame-width (Experiment 3)")
    args = parser.parse_args()

    # --- camera setup ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[args.dict])
    aruco_params = build_aruco_params(args.underwater_tuning)

    if args.calib:
        camera_matrix, dist_coeffs = load_calibration(args.calib)
        print(f"Loaded real calibration from {args.calib}")
    else:
        camera_matrix, dist_coeffs = approximate_camera_matrix(
            args.width, args.height, hfov_deg=args.hfov, vfov_deg=args.vfov
        )
        print(f"Using approximate camera matrix (HFOV={args.hfov} VFOV={args.vfov})")

    csv_file, writer = ensure_csv(args.csv)
    print(f"Logging trials to {args.csv}")
    print(f"Condition: label='{args.condition_label}' distance_cm={args.distance_cm} "
          f"angle_deg={args.angle_deg} lateral_pct={args.lateral_pct} "
          f"underwater_tuning={args.underwater_tuning}")
    print("Controls: SPACE = log trial (auto Y/N) | n = force-log as NOT detected | q = quit")

    window_name = "ArUco Test Harness"
    if not args.no_preview:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height)

    trial_count = 0
    last_flash_time = 0.0
    flash_text = ""

    try:
        while True:
            frame = picam2.capture_array()  # RGB888

            if args.underwater_tuning:
                detect_input = preprocess_underwater(frame)
            else:
                detect_input = frame

            corners, ids, _ = cv2.aruco.detectMarkers(detect_input, aruco_dict, parameters=aruco_params)

            detected = ids is not None
            marker_id, x, y, z = None, None, None, None

            display = frame.copy()
            if detected:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.marker_size, camera_matrix, dist_coeffs
                )
                marker_id = int(ids.flatten()[0])
                x, y, z = tvecs[0][0]
                x *= args.z_correction
                y *= args.z_correction
                z *= args.z_correction
                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                                   rvecs[0], tvecs[0], args.marker_size * 0.5)
                cv2.putText(display, f"id={marker_id} z={z:.3f}m", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                cv2.putText(display, "NOT DETECTED", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.putText(display, f"trials logged: {trial_count}", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            if flash_text and (time.time() - last_flash_time) < 1.0:
                cv2.putText(display, flash_text, (20, args.height - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if not args.no_preview:
                bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
                cv2.imshow(window_name, bgr)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break

            if key == ord(" ") or key == ord("n"):
                force_not_detected = (key == ord("n"))
                meta = picam2.capture_metadata()
                row = {
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                    "condition_label": args.condition_label,
                    "distance_cm": args.distance_cm,
                    "angle_deg": args.angle_deg,
                    "lateral_pct": args.lateral_pct,
                    "detected": "N" if force_not_detected else ("Y" if detected else "N"),
                    "marker_id": "" if force_not_detected else marker_id,
                    "x_m": "" if force_not_detected else x,
                    "y_m": "" if force_not_detected else y,
                    "z_m": "" if force_not_detected else z,
                    "exposure_us": meta.get("ExposureTime", ""),
                    "analogue_gain": meta.get("AnalogueGain", ""),
                    "lux": meta.get("Lux", ""),
                    "underwater_tuning": args.underwater_tuning,
                    "notes": "forced N (false positive override)" if force_not_detected else "",
                }
                writer.writerow(row)
                csv_file.flush()
                trial_count += 1
                flash_text = f"logged trial {trial_count}: {row['detected']}"
                last_flash_time = time.time()
                print(f"  trial {trial_count}: {row}")

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        picam2.stop()
        if not args.no_preview:
            cv2.destroyAllWindows()
        print(f"Done. {trial_count} trials logged to {args.csv}")


if __name__ == "__main__":
    main()
