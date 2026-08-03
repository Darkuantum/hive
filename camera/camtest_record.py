"""
camtest_record.py -- camtest.py, unchanged detection-wise, with two things
bolted on from camtest_v2.py/camtest_v3.py: auto screen recording
(wf-recorder) and live latency tracking. Everything else (dehaze,
software white balance, always-on CLAHE/denoise, temporal filter,
auto-log, confidence-signal columns) is intentionally NOT included --
this is camtest.py's exact raw-by-default / --underwater-tuning-bundled
detection pipeline, so it stays the version that's actually been shown to
detect best at close range and in near-total darkness, just with a
review recording and a latency readout added on top.

AUTO SCREEN RECORDING (wf-recorder):
  Starts automatically at launch (default ON, --no-record to disable),
  stops cleanly on exit (normal quit, Ctrl+C, or crash). Requires
  wf-recorder on PATH (Wayland only). If it's not installed, the script
  WARNS and keeps running the trial-logging session anyway rather than
  crashing -- recording is a convenience, not a dependency. Codec is
  pinned to libx264 explicitly (pairs cleanly with the .mp4 container)
  rather than relying on wf-recorder's own build-time default.
  Run WITHOUT --no-preview if you want the recording to actually show
  the annotated camera feed; with --no-preview there's no OpenCV window,
  so the recording only captures your terminal.

LATENCY TRACKING:
  Every frame's capture+preprocess+detect time is measured as
  proc_time_ms and logged per trial. A rolling window (mean/p95/max) is
  also displayed live, so a one-off slow frame doesn't hide a pipeline
  config that's too slow to sustain. If the rolling p95 crosses
  --latency-warn-ms (default 100ms), the overlay flags it in red and a
  throttled warning prints to the terminal. rolling_latency_p95_ms is
  logged per trial row.

Everything below this point (controls, dark-condition tuning, CSV
columns for the ORIGINAL camtest.py fields) is identical to camtest.py --
see that file's docstring for the full explanation. Only session_id,
proc_time_ms, and rolling_latency_p95_ms are new CSV columns.

USAGE:
  python3 camtest_record.py --condition-label "clear_20cm" --distance-cm 20 \
      --csv results/exp1_range.csv
"""

import argparse
import csv
import math
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
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
    "timestamp", "session_id", "condition_label", "distance_cm", "angle_deg", "lateral_pct",
    "detected", "marker_id", "x_m", "y_m", "z_m",
    "exposure_us", "analogue_gain", "lux",
    "manual_exposure", "manual_gain", "underwater_tuning",
    "proc_time_ms", "rolling_latency_p95_ms",
    "notes",
]


def start_recording(record_enabled, recording_file):
    """
    Launch wf-recorder as a background subprocess capturing the whole
    screen. Returns the Popen handle (or None if recording is disabled
    or wf-recorder isn't available) -- callers should treat None as
    "recording didn't start" and keep running the trial-logging session
    regardless, since recording is a convenience, not a dependency.
    """
    if not record_enabled:
        return None
    if shutil.which("wf-recorder") is None:
        print("WARNING: --record was on but 'wf-recorder' isn't on PATH -- "
              "continuing WITHOUT screen recording. Install with your "
              "distro's package manager (Wayland/wlroots only) if you want it.")
        return None
    os.makedirs(os.path.dirname(recording_file) or ".", exist_ok=True)
    try:
        proc = subprocess.Popen(
            # -c libx264: pin the codec explicitly so it pairs cleanly
            # with the .mp4 container regardless of wf-recorder's own
            # build-time default codec.
            ["wf-recorder", "-c", "libx264", "-f", recording_file],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(0.3)  # let it actually start before we report success
        if proc.poll() is not None:
            print(f"WARNING: wf-recorder exited immediately (code {proc.returncode}) -- "
                  f"continuing WITHOUT screen recording.")
            return None
        print(f"Screen recording started -> {recording_file}")
        return proc
    except Exception as e:
        print(f"WARNING: couldn't start wf-recorder ({e}) -- continuing WITHOUT screen recording.")
        return None


def stop_recording(proc, recording_file):
    """Send SIGINT (not SIGKILL) so wf-recorder/ffmpeg finalizes the file
    properly instead of leaving a corrupt/unplayable video."""
    if proc is None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)
        print(f"Screen recording saved -> {recording_file}")
    except subprocess.TimeoutExpired:
        proc.terminate()
        print(f"Screen recording stopped (had to terminate, may be truncated) -> {recording_file}")
    except Exception as e:
        print(f"WARNING: error stopping wf-recorder cleanly ({e})")


def latency_stats(history):
    """Mean/p95/max over the rolling window of recent proc_time_ms values."""
    if not history:
        return 0.0, 0.0, 0.0
    s = sorted(history)
    mean = sum(s) / len(s)
    p95_idx = min(int(len(s) * 0.95), len(s) - 1)
    return mean, s[p95_idx], s[-1]


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


def preprocess_underwater(frame_rgb, clip_limit=3.0):
    """CLAHE contrast enhancement + edge-preserving denoise, then to grayscale."""
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
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


def start_stdin_listener():
    """
    Headless (--no-preview) key capture.

    cv2.waitKey() only reads keys when the OpenCV preview WINDOW has focus,
    so it does nothing over SSH / with no display. This runs a background
    thread that blocks on stdin.readline() (via `input()`) and pushes
    whatever you typed into a thread-safe queue, which the main loop polls
    without blocking on it. daemon=True so it doesn't keep the process
    alive after the main loop exits.
    """
    q = queue.Queue()

    def _reader():
        print("(headless mode) Type a command + Enter: space/s = log trial | n = force NOT-detected | "
              "e+/e- = exposure up/down | g+/g- = gain up/down | a = toggle auto-exposure | q = quit")
        while True:
            try:
                line = input()
            except EOFError:
                q.put("q")
                break
            cmd = line.strip().lower()
            if cmd in ("", "space", "s"):
                q.put(" ")
            elif cmd in ("n", "e+", "e-", "g+", "g-", "a", "q"):
                q.put(cmd)
                if cmd == "q":
                    break
            else:
                print(f"  (unrecognized input '{line}' -- use space/s, n, e+/e-, g+/g-, a, or q)")

    t = threading.Thread(target=_reader, daemon=True)
    t.start()
    return q


def main():
    parser = argparse.ArgumentParser(description="Trial-logging ArUco test harness "
                                                   "(camtest.py detection pipeline, "
                                                   "with auto screen recording + latency tracking)")
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
    parser.add_argument("--clahe-clip", type=float, default=3.0,
                         help="CLAHE clipLimit for --underwater-tuning (default 3.0). "
                              "Higher = more contrast boost, but amplifies noise/turbidity graininess too.")
    parser.add_argument("--manual-exposure", type=int, default=None,
                         help="Lock exposure time in microseconds (disables auto-exposure). "
                              "Use with --manual-gain to fully control the sensor for dark-condition "
                              "tuning tests instead of trusting AE's guess. e.g. 20000 = 20ms")
    parser.add_argument("--manual-gain", type=float, default=None,
                         help="Lock analogue gain (disables auto-exposure). Typical range ~1.0-16.0 "
                              "depending on sensor; higher = brighter but noisier. Used with --manual-exposure.")
    parser.add_argument("--no-preview", action="store_true")

    # --- solo-testing helpers: recording, latency ---
    solo_group = parser.add_argument_group("solo testing")
    solo_group.add_argument("--record", action=argparse.BooleanOptionalAction, default=True,
                             help="Auto-start wf-recorder screen recording at launch, stop "
                                  "cleanly on exit (default ON). --no-record to disable. Run "
                                  "WITHOUT --no-preview for the recording to show the actual "
                                  "annotated camera feed, not just the terminal.")
    solo_group.add_argument("--recording-file", default=None,
                             help="Explicit path for the screen recording. Default: auto-named "
                                  "under --recording-dir from the condition label + timestamp.")
    solo_group.add_argument("--recording-dir", default="recordings",
                             help="Directory for auto-named recordings (default: recordings/).")
    solo_group.add_argument("--latency-warn-ms", type=float, default=100.0,
                             help="Rolling p95 latency threshold (ms) above which the overlay "
                                  "flags red (default 100ms). Doesn't stop anything -- it's a "
                                  "signal to back off --underwater-tuning/resolution, not a hard limit.")
    solo_group.add_argument("--latency-window", type=int, default=90,
                             help="Number of recent frames the rolling latency stats are "
                                  "computed over (default 90, i.e. a few seconds of history).")

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

    # --- session id + recording (started as early as possible, before camera
    # setup, so nothing is missed even if camera init itself hangs/fails) ---
    session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.recording_file:
        recording_file = args.recording_file
        if not recording_file.lower().endswith(".mp4"):
            recording_file = os.path.splitext(recording_file)[0] + ".mp4"
    else:
        label_part = args.condition_label.replace(" ", "_") or "session"
        recording_file = os.path.join(args.recording_dir, f"{label_part}_{session_id}.mp4")
    if args.record and args.no_preview:
        print("NOTE: --record with --no-preview will only capture your terminal, not the "
              "annotated camera feed -- drop --no-preview if you want the visual overlay recorded.")
    recorder_proc = start_recording(args.record, recording_file)

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
    print(f"Latency warning threshold: {args.latency_warn_ms:.0f}ms rolling p95 "
          f"(window={args.latency_window} frames)")
    print("Controls: SPACE = log trial | n = force-log as NOT detected | "
          "e/d = exposure up/down | g/f = gain up/down | a = toggle auto-exposure | "
          "v = toggle raw/processed view | q = quit")

    window_name = "ArUco Test Harness"
    if not args.no_preview:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height)
    else:
        stdin_queue = start_stdin_listener()

    trial_count = 0
    last_flash_time = 0.0
    flash_text = ""
    last_status_print = 0.0
    proc_time_history = deque(maxlen=args.latency_window)
    last_latency_warn_print = 0.0

    # --- live exposure/gain state, adjustable with keypresses while running ---
    ae_enabled = not (args.manual_exposure is not None or args.manual_gain is not None)
    current_exposure = args.manual_exposure if args.manual_exposure is not None else 10000  # 10ms
    current_gain = args.manual_gain if args.manual_gain is not None else 4.0
    EXPOSURE_STEP_FACTOR = 1.25   # multiplicative step, so it scales sensibly at any exposure level
    GAIN_STEP = 1.0
    EXPOSURE_MIN, EXPOSURE_MAX = 100, 1_000_000   # 0.1ms to 1s, sane sensor bounds
    GAIN_MIN, GAIN_MAX = 1.0, 16.0                 # typical AnalogueGain range for this sensor

    def apply_manual_controls():
        picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": int(current_exposure),
            "AnalogueGain": current_gain,
        })

    def apply_auto_controls():
        picam2.set_controls({"AeEnable": True})

    # --- debug view toggle: watch the raw feed, or what the detector actually sees ---
    view_processed = False  # False = raw camera feed (what your eyes see normally)
                             # True  = the CLAHE/bilateral-processed image handed to detectMarkers
                             #         (only meaningfully different when --underwater-tuning is on)

    if not ae_enabled:
        apply_manual_controls()
        time.sleep(0.5)  # let manual settings actually take effect before capturing
        print(f"Manual exposure control ON: ExposureTime={current_exposure}us AnalogueGain={current_gain}")

    try:
        while True:
            t_proc_start = time.perf_counter()
            frame = picam2.capture_array()  # RGB888

            if args.underwater_tuning:
                detect_input = preprocess_underwater(frame, clip_limit=args.clahe_clip)
            else:
                detect_input = frame

            corners, ids, _ = cv2.aruco.detectMarkers(detect_input, aruco_dict, parameters=aruco_params)

            detected = ids is not None
            marker_id, x, y, z = None, None, None, None

            if view_processed and args.underwater_tuning:
                # detect_input is grayscale here -- convert back to 3-channel so the
                # colored marker/axis overlays below still render in color on top of it
                display = cv2.cvtColor(detect_input, cv2.COLOR_GRAY2RGB)
            else:
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

            proc_time_ms = (time.perf_counter() - t_proc_start) * 1000
            proc_time_history.append(proc_time_ms)
            rolling_mean, rolling_p95, rolling_max = latency_stats(proc_time_history)
            latency_over_budget = rolling_p95 > args.latency_warn_ms

            # Live exposure/gain/lux readout every frame -- this is the actual
            # fix for "can't tell what auto-exposure is doing until after I
            # log a trial". Cheap enough to call every frame for a diagnostic
            # tool (not meant to run this way in the final deployed system).
            live_meta = picam2.capture_metadata()
            live_exposure = live_meta.get("ExposureTime", "?")
            live_gain = live_meta.get("AnalogueGain", "?")
            live_lux = live_meta.get("Lux", "?")
            meta_text = f"exp={live_exposure}us gain={live_gain} lux={live_lux}"

            if latency_over_budget and (time.time() - last_latency_warn_print) >= 5.0:
                print(f"\nWARNING: rolling p95 latency {rolling_p95:.0f}ms exceeds "
                      f"--latency-warn-ms {args.latency_warn_ms:.0f}ms -- consider "
                      f"disabling --underwater-tuning or lowering --width/--height.")
                last_latency_warn_print = time.time()

            cv2.putText(display, f"trials logged: {trial_count}", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(display, meta_text, (20, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 0), 2)
            ae_label = "AUTO-EXPOSURE" if ae_enabled else f"MANUAL exp={current_exposure:.0f}us gain={current_gain:.1f}"
            cv2.putText(display, ae_label, (20, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            view_label = "VIEW: PROCESSED (what detector sees)" if (view_processed and args.underwater_tuning) \
                else "VIEW: RAW"
            cv2.putText(display, view_label, (20, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 255), 2)
            latency_color = (0, 0, 255) if latency_over_budget else (100, 255, 100)
            latency_label = (f"LATENCY mean={rolling_mean:.0f} p95={rolling_p95:.0f} "
                              f"max={rolling_max:.0f}ms (warn>{args.latency_warn_ms:.0f}ms)"
                              + ("  HIGH" if latency_over_budget else " OK"))
            cv2.putText(display, latency_label, (20, 195),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, latency_color, 2)

            if flash_text and (time.time() - last_flash_time) < 1.0:
                cv2.putText(display, flash_text, (20, args.height - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if not args.no_preview:
                bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
                cv2.imshow(window_name, bgr)
                raw_key = cv2.waitKey(1) & 0xFF
                key_map = {
                    ord(" "): " ", ord("n"): "n", ord("q"): "q",
                    ord("e"): "e+", ord("d"): "e-",
                    ord("g"): "g+", ord("f"): "g-",
                    ord("a"): "a", ord("v"): "v",
                }
                cmd = key_map.get(raw_key)
            else:
                # Non-blocking check of whatever the stdin listener thread
                # has queued up since the last frame -- keeps the capture
                # loop running at full speed instead of blocking on input().
                try:
                    cmd = stdin_queue.get_nowait()
                except queue.Empty:
                    cmd = None
                # print a live status line a couple times a second since
                # there's no preview window to look at
                if time.time() - last_status_print >= 0.5:
                    status = f"id={marker_id} z={z:.3f}m" if detected else "NOT DETECTED"
                    ae_status = "AUTO" if ae_enabled else f"MANUAL(exp={current_exposure}us gain={current_gain:.1f})"
                    print(f"\rlive: {status}  {meta_text}  [{ae_status}]   (trials logged: {trial_count})   ",
                          end="", flush=True)
                    last_status_print = time.time()

            # --- dispatch ---
            if cmd == "q":
                break

            elif cmd in (" ", "n"):
                force_not_detected = (cmd == "n")
                meta = live_meta  # reuse this frame's metadata so the logged row matches
                                   # exactly what was on screen when you pressed the key
                row = {
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                    "session_id": session_id,
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
                    "manual_exposure": "" if ae_enabled else current_exposure,
                    "manual_gain": "" if ae_enabled else current_gain,
                    "underwater_tuning": args.underwater_tuning,
                    "proc_time_ms": round(proc_time_ms, 1),
                    "rolling_latency_p95_ms": round(rolling_p95, 1),
                    "notes": "forced N (false positive override)" if force_not_detected else "",
                }
                writer.writerow(row)
                csv_file.flush()
                trial_count += 1
                flash_text = f"logged trial {trial_count}: {row['detected']}"
                last_flash_time = time.time()
                print(f"  trial {trial_count}: {row}")

            elif cmd == "a":
                ae_enabled = not ae_enabled
                if ae_enabled:
                    apply_auto_controls()
                    print("\nAuto-exposure: ON")
                else:
                    apply_manual_controls()
                    print(f"\nAuto-exposure: OFF -- locked to exp={current_exposure}us gain={current_gain:.1f}")

            elif cmd == "v":
                if not args.underwater_tuning:
                    print("\n'v' toggle has nothing to show -- --underwater-tuning is off, "
                          "so there's no separate processed image (raw and processed are identical)")
                else:
                    view_processed = not view_processed
                    print(f"\nView: {'PROCESSED (what the detector sees)' if view_processed else 'RAW'}")

            elif cmd in ("e+", "e-", "g+", "g-"):
                if ae_enabled:
                    # switching to manual the moment you try to nudge a value --
                    # adjusting a value only makes sense once AE is out of the way
                    ae_enabled = False
                    print("\nAuto-exposure disabled automatically (you adjusted a value manually)")
                if cmd == "e+":
                    current_exposure = min(current_exposure * EXPOSURE_STEP_FACTOR, EXPOSURE_MAX)
                elif cmd == "e-":
                    current_exposure = max(current_exposure / EXPOSURE_STEP_FACTOR, EXPOSURE_MIN)
                elif cmd == "g+":
                    current_gain = min(current_gain + GAIN_STEP, GAIN_MAX)
                elif cmd == "g-":
                    current_gain = max(current_gain - GAIN_STEP, GAIN_MIN)
                apply_manual_controls()
                print(f"\nexposure={current_exposure:.0f}us  gain={current_gain:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        picam2.stop()
        if not args.no_preview:
            cv2.destroyAllWindows()
        stop_recording(recorder_proc, recording_file)
        print(f"Done. {trial_count} trials logged to {args.csv}")


if __name__ == "__main__":
    main()
