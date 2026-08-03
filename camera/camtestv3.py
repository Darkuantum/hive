"""
camtest_v3.py -- camtest_v2.py, extended for SOLO testing: auto screen
recording (wf-recorder) so a review record exists without needing
someone else to operate anything, optional hands-free auto-logging (no
spacebar required), and live latency tracking so you know when the
pipeline is too slow for the control loop rather than finding out later.

NEW IN THIS VERSION (on top of everything camtest_v2.py already does):

AUTO SCREEN RECORDING (wf-recorder):
  Starts automatically at launch (default ON, --no-record to disable),
  stops cleanly on exit (normal quit, Ctrl+C, or crash). Requires
  wf-recorder on PATH (Wayland only -- this is what Raspberry Pi OS
  Bookworm+ uses by default). If it's not installed, the script WARNS
  and keeps running the trial-logging session anyway rather than
  crashing -- recording is a convenience, not a dependency.
  Run WITHOUT --no-preview if you want the recording to actually show
  the annotated camera feed (overlay, detection box, latency readout);
  with --no-preview there's no OpenCV window, so the recording only
  captures your terminal.

AUTO-LOGGING (--auto-log):
  Logs a trial automatically every frame (or every --auto-log-interval
  seconds, if you want to throttle it) instead of requiring SPACE.
  SPACE/n still work on top of this if you want to hand-mark a specific
  moment. Changes what a "trial" means, worth being clear-eyed about:
  camtest.py's original keypress model gave you independent,
  deliberately-repositioned trials; auto-logging at a fixed condition
  instead gives you a continuous reliability sample over time at that
  condition (useful for its own reasons -- e.g. catching intermittent
  dropouts from drifting particulates -- but it's a different
  measurement, and adjacent auto-logged rows are correlated, not
  independent, the same concern the original camtest.py docstring
  raised about naive per-frame logging). Still change condition_label /
  turbidity_ntu / distance_cm etc. between BLOCKS by relaunching with
  new values, same as before -- this doesn't do that for you.

LATENCY TRACKING:
  Every frame's capture+pipeline+detect time already got logged as
  proc_time_ms in v2. This version adds a rolling window (mean/p95/max)
  displayed live, so a one-off slow frame doesn't hide a pipeline
  config that's too slow to sustain. If your rolling p95 crosses
  --latency-warn-ms (default 100ms -- see the accompanying chat message
  for how that default was derived from your own SDR's position-error
  budget), the overlay flags it in red. rolling_latency_p95_ms is also
  logged per trial row.
"""

"""
camtest_v2.py -- Trial-logging ArUco test harness, extended with a
turbidity/low-light image-enhancement pipeline (dehaze, white balance,
CLAHE, gamma) and optional temporal filtering, on top of camtest.py's
original keypress-driven trial logging.

Built for characterizing detection across:
    turbidity : 0-150 NTU   (label each block with --turbidity-ntu --
                              the camera can't sense NTU directly)
    light     : measured automatically every frame via picamera2's Lux
                              metadata -- no separate light meter needed
    depth     : 3-5m, Singapore coastal waters

Everything camtest.py could already do (space/n to log a trial, live
exposure/gain tuning, raw/processed preview toggle, headless stdin mode)
still works the same way. New in this version:

IMAGE PIPELINE (actual stage code lives in underwater_pipeline.py, kept
in a separate module so camFinal.py can later reuse the same tuned
functions instead of duplicating them):
    dehaze -> white balance -> grayscale -> CLAHE -> gamma -> denoise
  Each stage is independently toggleable, both at launch (CLI flags) and
  live while running (keys 1/2/3 below). The ACTUAL state of every
  toggle is logged with every trial row, so an analysis can group by
  exactly which combination was active -- not just "underwater tuning
  on/off" as a single blob like the original camtest.py.

SENSOR WHITE BALANCE (separate from the software gray-world step above):
  --manual-wb-red / --manual-wb-blue lock the sensor's own AWB, the same
  way --manual-exposure / --manual-gain already lock AE. Toggle live
  with 'w'.

CONFIDENCE SIGNALS (new CSV columns, computed every frame):
  contrast_score          -- Laplacian variance of what the detector
                              actually saw this frame (flat/hazy = low)
  reprojection_error_px   -- how self-consistent a successful pose fit
                              was (see underwater_pipeline.reprojection_error) --
                              a bad detection can still return a pose,
                              this flags it even when ids is not None

TEMPORAL FILTER (optional, off by default):
  --temporal-filter turns on a constant-velocity Kalman filter (see
  PoseTracker in underwater_pipeline.py) that coasts through brief
  dropouts. Logged in SEPARATE filtered_x/y/z columns and NEVER changes
  the raw detected/not-detected trial outcome -- that stays the actual
  per-frame ground truth your success-rate numbers need.

ARUCO API COMPATIBILITY:
  cv2.aruco.Dictionary_get() / DetectorParameters_create() -- the old-style
  calls used in aruco_detect.py / cam12cm.py / camFinal.py / camtest.py --
  were removed in recent OpenCV (confirmed removed as of the 4.13 line;
  replaced by getPredefinedDictionary() / DetectorParameters()). This
  script tries the old API first (matching your other four scripts) and
  falls back to the new API automatically, so it keeps working if the
  Pi's OpenCV ever gets upgraded. Your other four scripts don't have this
  fallback -- worth patching if you're not pinning an OpenCV version.

CONTROLS:
  SPACE / n / q / e / d / g / f / a / v  -- unchanged from camtest.py
  w      -- toggle sensor auto-white-balance on/off
  1      -- toggle dehaze
  2      -- toggle software white balance (gray-world)
  3      -- toggle CLAHE

USAGE:
  python3 camtest_v2.py --condition-label "ntu50_daylight" \
      --turbidity-ntu 50 --distance-cm 100 --csv results/turbidity_sweep.csv

  Same trial-logging model as camtest.py: position the marker, tune live
  if needed, SPACE to log (Detected=Y or N, whichever the camera actually
  saw), n to force-log a false positive as N, q to quit.

CSV COLUMNS:
  timestamp, condition_label, distance_cm, angle_deg, lateral_pct,
  turbidity_ntu, detected, marker_id, x_m, y_m, z_m,
  reprojection_error_px, contrast_score,
  exposure_us, analogue_gain, lux, manual_exposure, manual_gain,
  awb_enabled, wb_red_gain, wb_blue_gain,
  dehaze_enabled, white_balance_enabled, clahe_enabled, clahe_clip, gamma,
  underwater_tuning, proc_time_ms,
  filtered_x_m, filtered_y_m, filtered_z_m, filter_state,
  notes

  NOTE: this is a different (larger) schema than camtest.py's CSV --
  don't concatenate old and new CSVs without reconciling columns first.
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

from underwater_pipeline import (
    apply_pipeline,
    contrast_score,
    reprojection_error,
    PoseTracker,
)

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
    "turbidity_ntu",
    "detected", "marker_id", "x_m", "y_m", "z_m",
    "reprojection_error_px", "contrast_score",
    "exposure_us", "analogue_gain", "lux",
    "manual_exposure", "manual_gain",
    "awb_enabled", "wb_red_gain", "wb_blue_gain",
    "dehaze_enabled", "white_balance_enabled", "clahe_enabled", "clahe_clip", "gamma",
    "underwater_tuning", "proc_time_ms", "rolling_latency_p95_ms",
    "filtered_x_m", "filtered_y_m", "filtered_z_m", "filter_state",
    "auto_logged", "notes",
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
            # -c libx264: pin the codec explicitly rather than relying on
            # wf-recorder's own default (libx264 upstream, but this can be
            # rebuilt with a different -Ddefault_codec) -- libx264 is what
            # actually pairs cleanly with the .mp4 container/extension.
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


def get_aruco_dictionary(dict_name):
    """See module docstring: old API first, new API fallback."""
    dict_id = ARUCO_DICTS[dict_name]
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(dict_id)
    return cv2.aruco.getPredefinedDictionary(dict_id)


def get_aruco_base_params():
    """See module docstring: old API first, new API fallback."""
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


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
    Detector-side parameters only (corner refinement, adaptive-threshold
    search range, error-correction tolerance). This is now SEPARATE from
    the image-enhancement pipeline (dehaze/white-balance/CLAHE/gamma),
    which is controlled by its own flags below -- in the original
    camtest.py, a single --underwater-tuning flag covered both; here it
    covers only the detector params so each half can be tested
    independently of the other.
    """
    params = get_aruco_base_params()
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
    Headless (--no-preview) key capture. cv2.waitKey() only reads keys
    when the OpenCV preview window has focus, so headless mode instead
    runs a background thread blocking on input() and pushes commands
    into a thread-safe queue the main loop polls without blocking.
    """
    q = queue.Queue()

    def _reader():
        print("(headless mode) Type a command + Enter: space/s = log trial | n = force NOT-detected | "
              "e+/e- = exposure up/down | g+/g- = gain up/down | a = toggle auto-exposure | "
              "w = toggle auto-white-balance | 1 = toggle dehaze | 2 = toggle white balance | "
              "3 = toggle underwater-tuning bundle (CLAHE+denoise+detector) | q = quit")
        while True:
            try:
                line = input()
            except EOFError:
                q.put("q")
                break
            cmd = line.strip().lower()
            if cmd in ("", "space", "s"):
                q.put(" ")
            elif cmd in ("n", "e+", "e-", "g+", "g-", "a", "w", "1", "2", "3", "q"):
                q.put(cmd)
                if cmd == "q":
                    break
            else:
                print(f"  (unrecognized input '{line}')")

    t = threading.Thread(target=_reader, daemon=True)
    t.start()
    return q


def main():
    parser = argparse.ArgumentParser(description="Trial-logging ArUco test harness "
                                                   "with turbidity/low-light pipeline")
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
                         help="Widen detector adaptive-threshold range + subpixel corner "
                              "refinement + looser error correction. Detector-side only -- "
                              "see --dehaze/--white-balance/--clahe for image enhancement.")
    parser.add_argument("--no-preview", action="store_true")

    # --- image enhancement pipeline ---
    pipeline_group = parser.add_argument_group("image enhancement pipeline")
    pipeline_group.add_argument("--dehaze", action=argparse.BooleanOptionalAction, default=False,
                                 help="Underwater dark-channel-prior dehaze (dark channel from "
                                      "G,B only -- red attenuates within a few meters and biases "
                                      "the standard version). Most expensive stage -- off by "
                                      "default; validate it's actually needed at your worst-case "
                                      "turbidity before enabling for good (Pi 4 has limited headroom).")
    pipeline_group.add_argument("--dehaze-omega", type=float, default=0.85,
                                 help="Haze removal strength, 0-1 (default 0.85).")
    pipeline_group.add_argument("--dehaze-t0", type=float, default=0.15,
                                 help="Transmission floor (default 0.15). Raise if the murkiest "
                                      "frames come out noisy/blown-out after dehaze.")
    pipeline_group.add_argument("--dehaze-downscale", type=int, default=4,
                                 help="Downscale factor for the expensive dehaze estimate "
                                      "(default 4). Raise (e.g. 8) if dehaze is too slow.")
    pipeline_group.add_argument("--white-balance", action=argparse.BooleanOptionalAction, default=False,
                                 help="Software gray-world color correction, on top of locking "
                                      "the sensor's own AWB below.")
    pipeline_group.add_argument("--clahe", action=argparse.BooleanOptionalAction, default=None,
                                 help="Local contrast enhancement. Default: follows "
                                      "--underwater-tuning (this is camtest.py's original "
                                      "proven pairing -- CLAHE was always tested together with "
                                      "the widened detector params, not alone). Pass --clahe or "
                                      "--no-clahe explicitly to decouple them deliberately.")
    pipeline_group.add_argument("--clahe-clip", type=float, default=3.0,
                                 help="CLAHE clipLimit (default 3.0). Higher = more contrast "
                                      "boost, but amplifies noise/turbidity graininess too.")
    pipeline_group.add_argument("--gamma", type=float, default=1.0,
                                 help="Brightness gamma applied after CLAHE (default 1.0 = no-op).")
    pipeline_group.add_argument("--denoise", action=argparse.BooleanOptionalAction, default=None,
                                 help="Bilateral denoise as the pipeline's last step. Default: "
                                      "follows --underwater-tuning, same reasoning as --clahe above.")

    # --- sensor white balance ---
    wb_group = parser.add_argument_group("sensor white balance")
    wb_group.add_argument("--manual-wb-red", type=float, default=None,
                           help="Lock ColourGains red channel (disables sensor AWB). "
                                "Pair with --manual-wb-blue.")
    wb_group.add_argument("--manual-wb-blue", type=float, default=None,
                           help="Lock ColourGains blue channel (disables sensor AWB). "
                                "Pair with --manual-wb-red.")

    # --- temporal filter ---
    filt_group = parser.add_argument_group("temporal filter")
    filt_group.add_argument("--temporal-filter", action=argparse.BooleanOptionalAction, default=False,
                             help="Constant-velocity Kalman filter over (x,y,z), logged into "
                                  "separate filtered_* columns. Does NOT affect the raw "
                                  "detected/not-detected trial outcome.")
    filt_group.add_argument("--coast-frames", type=int, default=15,
                             help="Max consecutive missed frames the filter predicts through "
                                  "before reporting 'lost' (default 15, ~0.5s at 30fps).")

    parser.add_argument("--manual-exposure", type=int, default=None,
                         help="Lock exposure time in microseconds (disables auto-exposure). "
                              "Use with --manual-gain.")
    parser.add_argument("--manual-gain", type=float, default=None,
                         help="Lock analogue gain (disables auto-exposure). Used with --manual-exposure.")

    # --- solo-testing helpers: recording, auto-logging, latency ---
    solo_group = parser.add_argument_group("solo testing")
    solo_group.add_argument("--record", action=argparse.BooleanOptionalAction, default=True,
                             help="Auto-start wf-recorder screen recording at launch, stop "
                                  "cleanly on exit (default ON, matching camtest_v2.py -- "
                                  "--no-record to disable). Run WITHOUT --no-preview for the "
                                  "recording to show the actual annotated camera feed, not "
                                  "just the terminal.")
    solo_group.add_argument("--recording-file", default=None,
                             help="Explicit path for the screen recording. Default: auto-named "
                                  "under --recording-dir from the condition label + timestamp.")
    solo_group.add_argument("--recording-dir", default="recordings",
                             help="Directory for auto-named recordings (default: recordings/).")
    solo_group.add_argument("--auto-log", action=argparse.BooleanOptionalAction, default=False,
                             help="Log trials automatically instead of requiring SPACE -- see "
                                  "module docstring for what this changes about what a 'trial' "
                                  "means. SPACE/n still work on top of this.")
    solo_group.add_argument("--auto-log-interval", type=float, default=0.0,
                             help="Seconds between auto-logged rows (default 0.0 = every frame). "
                                  "Raise this if every-frame logging produces more correlated "
                                  "data than you want for your report.")
    solo_group.add_argument("--latency-warn-ms", type=float, default=100.0,
                             help="Rolling p95 latency threshold (ms) above which the overlay "
                                  "flags red (default 100ms / ~10Hz -- see chat for how this "
                                  "was derived from your SDR's 10cm position-error budget). "
                                  "Doesn't stop anything -- it's a signal to back off dehaze/"
                                  "resolution/downscale, not a hard limit.")
    solo_group.add_argument("--latency-window", type=int, default=90,
                             help="Number of recent frames the rolling latency stats are "
                                  "computed over (default 90, i.e. a few seconds of history).")

    # --- experiment/trial labeling ---
    exp_group = parser.add_argument_group("experiment/trial labeling")
    exp_group.add_argument("--csv", required=True, help="Path to CSV file to append trial rows to")
    exp_group.add_argument("--condition-label", default="",
                            help="Free-text label for this test block, e.g. 'ntu50_daylight'")
    exp_group.add_argument("--distance-cm", type=float, default=None,
                            help="Target/ruler-measured Z distance for this block of trials")
    exp_group.add_argument("--angle-deg", type=float, default=None,
                            help="Marker tilt angle for this block of trials")
    exp_group.add_argument("--lateral-pct", type=float, default=None,
                            help="Marker lateral offset as %% of half-frame-width")
    exp_group.add_argument("--turbidity-ntu", type=float, default=None,
                            help="Target/measured turbidity (NTU) for this block -- the camera "
                                 "can't sense this directly, so log it from your turbidity "
                                 "meter/Secchi disk reading before starting the block.")
    args = parser.parse_args()

    # --clahe/--denoise default to following --underwater-tuning (camtest.py's
    # original single flag bundled CLAHE+denoise+widened detector params
    # together and that combination is what's actually validated to work --
    # only override independently if you're deliberately decoupling them).
    clahe_start = args.underwater_tuning if args.clahe is None else args.clahe
    denoise_start = args.underwater_tuning if args.denoise is None else args.denoise

    # --- session id + recording (started as early as possible, before camera
    # setup, so nothing is missed even if camera init itself hangs/fails) ---
    session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.recording_file:
        recording_file = args.recording_file
        if not recording_file.lower().endswith(".mp4"):
            print(f"NOTE: --recording-file '{recording_file}' doesn't end in .mp4 -- "
                  f"the recorder is pinned to libx264, which is happiest in an .mp4 "
                  f"container. Using your path as given.")
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

    aruco_dict = get_aruco_dictionary(args.dict)
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
          f"turbidity_ntu={args.turbidity_ntu}")
    print(f"Pipeline start state: dehaze={args.dehaze} white_balance={args.white_balance} "
          f"clahe={clahe_start}(clip={args.clahe_clip}) gamma={args.gamma} "
          f"denoise={denoise_start} underwater_tuning={args.underwater_tuning} "
          f"temporal_filter={args.temporal_filter}")
    if args.auto_log:
        interval_desc = "every frame" if args.auto_log_interval <= 0 else f"every {args.auto_log_interval}s"
        print(f"Auto-logging ON ({interval_desc}) -- SPACE/n still work for manual marks. "
              f"Change condition_label/turbidity_ntu/distance_cm etc. between blocks by relaunching.")
    print(f"Latency warning threshold: {args.latency_warn_ms:.0f}ms rolling p95 "
          f"(window={args.latency_window} frames)")
    print("Controls: SPACE = log trial | n = force-log as NOT detected | "
          "e/d = exposure up/down | g/f = gain up/down | a = toggle auto-exposure | "
          "w = toggle auto-white-balance | v = toggle raw/processed view | "
          "1 = toggle dehaze | 2 = toggle white balance | 3 = toggle underwater-tuning bundle (CLAHE+denoise+detector) | q = quit")

    window_name = "ArUco Test Harness v2"
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
    last_auto_log_time = 0.0
    last_latency_warn_print = 0.0

    # --- live exposure/gain state (same pattern as camtest.py) ---
    ae_enabled = not (args.manual_exposure is not None or args.manual_gain is not None)
    current_exposure = args.manual_exposure if args.manual_exposure is not None else 10000
    current_gain = args.manual_gain if args.manual_gain is not None else 4.0
    EXPOSURE_STEP_FACTOR = 1.25
    GAIN_STEP = 1.0
    EXPOSURE_MIN, EXPOSURE_MAX = 100, 1_000_000
    GAIN_MIN, GAIN_MAX = 1.0, 16.0

    # --- live white balance state (mirrors the exposure/gain pattern above) ---
    awb_enabled = not (args.manual_wb_red is not None or args.manual_wb_blue is not None)
    current_wb_red = args.manual_wb_red if args.manual_wb_red is not None else 2.0
    current_wb_blue = args.manual_wb_blue if args.manual_wb_blue is not None else 1.2

    def apply_manual_controls():
        picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": int(current_exposure),
            "AnalogueGain": current_gain,
        })

    def apply_auto_controls():
        picam2.set_controls({"AeEnable": True})

    def apply_manual_wb():
        picam2.set_controls({
            "AwbEnable": False,
            "ColourGains": (current_wb_red, current_wb_blue),
        })

    def apply_auto_wb():
        picam2.set_controls({"AwbEnable": True})

    # --- live pipeline-stage toggles (seeded from CLI, then live-toggleable) ---
    dehaze_enabled = args.dehaze
    wb_sw_enabled = args.white_balance
    clahe_enabled = clahe_start
    denoise_enabled = denoise_start
    underwater_tuning_enabled = args.underwater_tuning

    # --- debug view toggle ---
    view_processed = False

    # --- temporal filter ---
    tracker = PoseTracker(max_coast_frames=args.coast_frames) if args.temporal_filter else None

    if not ae_enabled:
        apply_manual_controls()
        time.sleep(0.2)
        print(f"Manual exposure control ON: ExposureTime={current_exposure}us AnalogueGain={current_gain}")
    if not awb_enabled:
        apply_manual_wb()
        time.sleep(0.2)
        print(f"Manual white balance ON: red={current_wb_red} blue={current_wb_blue}")

    def log_trial(force_not_detected, is_auto=False):
        """
        Builds and writes one CSV row from the CURRENT frame's state (all
        read from main()'s enclosing scope at call time). Shared by the
        manual SPACE/n keypress path and the --auto-log path so the two
        can't silently drift out of sync with each other.
        """
        nonlocal trial_count, flash_text, last_flash_time
        meta = live_meta  # this frame's metadata, matching what was on screen
        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "session_id": session_id,
            "condition_label": args.condition_label,
            "distance_cm": args.distance_cm,
            "angle_deg": args.angle_deg,
            "lateral_pct": args.lateral_pct,
            "turbidity_ntu": args.turbidity_ntu,
            "detected": "N" if force_not_detected else ("Y" if detected else "N"),
            "marker_id": "" if force_not_detected else marker_id,
            "x_m": "" if force_not_detected else x,
            "y_m": "" if force_not_detected else y,
            "z_m": "" if force_not_detected else z,
            "reprojection_error_px": "" if force_not_detected else reproj_err,
            "contrast_score": frame_contrast,
            "exposure_us": meta.get("ExposureTime", ""),
            "analogue_gain": meta.get("AnalogueGain", ""),
            "lux": meta.get("Lux", ""),
            "manual_exposure": "" if ae_enabled else current_exposure,
            "manual_gain": "" if ae_enabled else current_gain,
            "awb_enabled": awb_enabled,
            "wb_red_gain": "" if awb_enabled else current_wb_red,
            "wb_blue_gain": "" if awb_enabled else current_wb_blue,
            "dehaze_enabled": dehaze_enabled,
            "white_balance_enabled": wb_sw_enabled,
            "clahe_enabled": clahe_enabled,
            "clahe_clip": args.clahe_clip,
            "gamma": args.gamma,
            "underwater_tuning": underwater_tuning_enabled,
            "proc_time_ms": round(proc_time_ms, 1),
            "rolling_latency_p95_ms": round(rolling_p95, 1),
            "filtered_x_m": "" if filtered_x is None else filtered_x,
            "filtered_y_m": "" if filtered_y is None else filtered_y,
            "filtered_z_m": "" if filtered_z is None else filtered_z,
            "filter_state": filter_state,
            "auto_logged": is_auto,
            "notes": "forced N (false positive override)" if force_not_detected else "",
        }
        writer.writerow(row)
        csv_file.flush()
        trial_count += 1
        flash_text = f"logged trial {trial_count}: {row['detected']}"
        last_flash_time = time.time()
        # avoid flooding stdout when auto-logging every frame at 20-30fps --
        # manual logs always print, auto-logs print every 25th row
        if not is_auto or trial_count % 25 == 0:
            print(f"  trial {trial_count}: detected={row['detected']} "
                  f"z={row['z_m']} err={row['reprojection_error_px']} auto={is_auto}")

    try:
        while True:
            t_proc_start = time.perf_counter()
            frame = picam2.capture_array()  # "RGB888" -- actually [B,G,R] order, see underwater_pipeline.py docstring

            detect_input = apply_pipeline(
                frame,
                dehaze_enabled=dehaze_enabled,
                wb_enabled=wb_sw_enabled,
                clahe_enabled=clahe_enabled,
                clahe_clip=args.clahe_clip,
                gamma=args.gamma,
                denoise_enabled=denoise_enabled,
                dehaze_omega=args.dehaze_omega,
                dehaze_t0=args.dehaze_t0,
                dehaze_downscale=args.dehaze_downscale,
            )

            corners, ids, _ = cv2.aruco.detectMarkers(detect_input, aruco_dict, parameters=aruco_params)

            detected = ids is not None
            marker_id, x, y, z = None, None, None, None
            reproj_err = None
            frame_contrast = contrast_score(detect_input)

            display = cv2.cvtColor(detect_input, cv2.COLOR_GRAY2BGR) if view_processed else frame.copy()

            if detected:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.marker_size, camera_matrix, dist_coeffs
                )
                marker_id = int(ids.flatten()[0])
                rvec_raw, tvec_raw = rvecs[0], tvecs[0]

                reproj_err = reprojection_error(
                    corners[0][0], rvec_raw, tvec_raw, args.marker_size,
                    camera_matrix, dist_coeffs
                )

                x, y, z = tvec_raw[0]
                x *= args.z_correction
                y *= args.z_correction
                z *= args.z_correction

                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                                   rvec_raw, tvec_raw, args.marker_size * 0.5)
                cv2.putText(display, f"id={marker_id} z={z:.3f}m err={reproj_err:.1f}px", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display, "NOT DETECTED", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            proc_time_ms = (time.perf_counter() - t_proc_start) * 1000
            proc_time_history.append(proc_time_ms)
            rolling_mean, rolling_p95, rolling_max = latency_stats(proc_time_history)
            latency_over_budget = rolling_p95 > args.latency_warn_ms

            # --- temporal filter (does NOT affect `detected` / trial logging) ---
            filtered_x = filtered_y = filtered_z = None
            filter_state = ""
            if tracker is not None:
                meas = (x, y, z) if detected else None
                filtered_x, filtered_y, filtered_z, filter_state = tracker.update(meas, time.time())

            # --- live metadata / overlay ---
            live_meta = picam2.capture_metadata()
            live_exposure = live_meta.get("ExposureTime", "?")
            live_gain = live_meta.get("AnalogueGain", "?")
            live_lux = live_meta.get("Lux", "?")
            meta_text = f"exp={live_exposure}us gain={live_gain} lux={live_lux}"

            # --- auto-log (hands-free trial logging for solo testing) ---
            if args.auto_log:
                now_t = time.time()
                due = (args.auto_log_interval <= 0) or (now_t - last_auto_log_time >= args.auto_log_interval)
                if due:
                    last_auto_log_time = now_t
                    log_trial(force_not_detected=False, is_auto=True)

            if latency_over_budget and (time.time() - last_latency_warn_print) >= 5.0:
                print(f"\nWARNING: rolling p95 latency {rolling_p95:.0f}ms exceeds "
                      f"--latency-warn-ms {args.latency_warn_ms:.0f}ms -- consider "
                      f"--dehaze-downscale higher, dropping dehaze, or lowering --width/--height.")
                last_latency_warn_print = time.time()

            cv2.putText(display, f"trials logged: {trial_count}", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(display, meta_text, (20, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 0), 2)
            ae_label = "AE: AUTO" if ae_enabled else f"AE: MANUAL exp={current_exposure:.0f}us gain={current_gain:.1f}"
            cv2.putText(display, ae_label, (20, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            awb_label = "AWB: AUTO" if awb_enabled else f"AWB: MANUAL r={current_wb_red:.2f} b={current_wb_blue:.2f}"
            cv2.putText(display, awb_label, (20, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            pipe_label = (f"PIPE: dehaze={'ON' if dehaze_enabled else 'off'} "
                          f"wb={'ON' if wb_sw_enabled else 'off'} "
                          f"clahe={'ON' if clahe_enabled else 'off'} "
                          f"denoise={'ON' if denoise_enabled else 'off'} "
                          f"detector-tuning={'ON' if underwater_tuning_enabled else 'off'} "
                          f"gamma={args.gamma:.2f}")
            cv2.putText(display, pipe_label, (20, 195),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 255), 2)
            quality_label = (f"contrast={frame_contrast:.0f}  "
                              f"proc={proc_time_ms:.0f}ms (~{1000 / max(proc_time_ms, 1):.0f}fps)")
            cv2.putText(display, quality_label, (20, 225),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
            latency_color = (0, 0, 255) if latency_over_budget else (100, 255, 100)
            latency_label = (f"LATENCY mean={rolling_mean:.0f} p95={rolling_p95:.0f} "
                              f"max={rolling_max:.0f}ms (warn>{args.latency_warn_ms:.0f}ms)"
                              + ("  HIGH" if latency_over_budget else " OK"))
            cv2.putText(display, latency_label, (20, 255),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, latency_color, 2)
            view_label = "VIEW: PROCESSED (what detector sees)" if view_processed else "VIEW: RAW"
            cv2.putText(display, view_label, (20, 285),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 255), 2)
            if tracker is not None:
                if filter_state == "lost" or filtered_x is None:
                    filt_label = "AUV-view: LOST"
                else:
                    filt_label = (f"AUV-view: {filter_state.upper()} "
                                   f"x={filtered_x:.2f} y={filtered_y:.2f} z={filtered_z:.2f}")
                cv2.putText(display, filt_label, (20, 315),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 2)

            if flash_text and (time.time() - last_flash_time) < 1.0:
                cv2.putText(display, flash_text, (20, args.height - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if not args.no_preview:
                # `display` is already [B,G,R] order (what picamera2's "RGB888"
                # format actually delivers, and what cv2.imshow wants) -- no
                # extra conversion needed here. An earlier version of this
                # script did an extra cv2.cvtColor(..., COLOR_RGB2BGR) at this
                # point, which -- given the data was already BGR-ordered --
                # actually inverted red/blue in the preview window only
                # (didn't affect detection, which never went through this line).
                cv2.imshow(window_name, display)
                raw_key = cv2.waitKey(1) & 0xFF
                key_map = {
                    ord(" "): " ", ord("n"): "n", ord("q"): "q",
                    ord("e"): "e+", ord("d"): "e-",
                    ord("g"): "g+", ord("f"): "g-",
                    ord("a"): "a", ord("v"): "v", ord("w"): "w",
                    ord("1"): "1", ord("2"): "2", ord("3"): "3",
                }
                cmd = key_map.get(raw_key)
            else:
                try:
                    cmd = stdin_queue.get_nowait()
                except queue.Empty:
                    cmd = None
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
                log_trial(force_not_detected=(cmd == "n"), is_auto=False)

            elif cmd == "a":
                ae_enabled = not ae_enabled
                if ae_enabled:
                    apply_auto_controls()
                    print("\nAuto-exposure: ON")
                else:
                    apply_manual_controls()
                    print(f"\nAuto-exposure: OFF -- locked to exp={current_exposure}us gain={current_gain:.1f}")

            elif cmd == "w":
                awb_enabled = not awb_enabled
                if awb_enabled:
                    apply_auto_wb()
                    print("\nAuto-white-balance: ON")
                else:
                    apply_manual_wb()
                    print(f"\nAuto-white-balance: OFF -- locked to red={current_wb_red:.2f} blue={current_wb_blue:.2f}")

            elif cmd == "v":
                view_processed = not view_processed
                print(f"\nView: {'PROCESSED (what the detector sees)' if view_processed else 'RAW'}")

            elif cmd == "1":
                dehaze_enabled = not dehaze_enabled
                print(f"\nDehaze: {'ON' if dehaze_enabled else 'OFF'}")

            elif cmd == "2":
                wb_sw_enabled = not wb_sw_enabled
                print(f"\nSoftware white balance: {'ON' if wb_sw_enabled else 'OFF'}")

            elif cmd == "3":
                # Toggles the FULL bundle that camtest.py's original single
                # --underwater-tuning flag controlled together: CLAHE + denoise
                # + widened detector adaptive-threshold/corner-refinement params.
                # These were always validated as one combination, not CLAHE alone
                # -- see the module docstring for why splitting them apart broke
                # detection when only CLAHE was toggled on its own.
                underwater_tuning_enabled = not underwater_tuning_enabled
                clahe_enabled = underwater_tuning_enabled
                denoise_enabled = underwater_tuning_enabled
                aruco_params = build_aruco_params(underwater_tuning_enabled)
                print(f"\nUnderwater tuning (CLAHE+denoise+detector params): "
                      f"{'ON' if underwater_tuning_enabled else 'OFF'}")

            elif cmd in ("e+", "e-", "g+", "g-"):
                if ae_enabled:
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
