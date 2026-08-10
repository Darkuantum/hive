"""
camtestv5_9cond.py -- camtestv5.py, adapted to run the 9-condition turbidity x
lighting test plan (see camtestplan/9condition.md):

CONDITION/POSITION PRESETS (new this version):
  --condition 1-9   -- looks up the plan's condition table (turbidity label x
                       lighting label) and auto-builds --condition-label from
                       it if you didn't pass one explicitly. Also suggests a
                       starting --led-brightness for that lighting tier
                       (normal=0.30, low=0.12, worstcase=0.0) unless you pass
                       --led-brightness yourself. Deliberately does NOT set
                       --turbidity-ntu -- per the plan's own limitation notes,
                       NTU is a physically-prepared/estimated value each
                       session, not something a preset table should fabricate.
  --position NAME   -- looks up the plan's x-y position table (center/plusx/
                       minusx/plusy/minusy/diagonal) and fills --true-x-cm/
                       --true-y-cm from it unless you passed those yourself.
  --reps N          -- display-only target repeat count for the current
                       block (default 3, matching the plan's 3-repeat
                       blocks). Shown on the overlay as a progress counter;
                       does not auto-stop anything.
  Depth (--distance-cm) is unchanged from v5 -- pass 40 or 80 directly.

LED AUTO-ADJUST (new this version, folded in from the standalone camtestv6.py):
  --led-auto-adjust (default ON) continuously reads the sensor's metered
  Lux value each frame and trims LED brightness DOWN in --led-adjust-step
  increments (rate-limited to one step per --led-adjust-interval seconds,
  default 0.2s, so the strip doesn't visibly flicker) whenever measured
  lux exceeds --max-lux (default 150). When there's headroom (lux comfortably
  below the ceiling) it eases brightness back UP toward whatever level you
  set via --led-brightness / the i/k keys / a --condition preset -- that
  starting/manually-set value acts as the ceiling the auto-loop ramps back
  toward, it never drives the strip brighter than what you asked for. Manual
  i/k presses still work on top of this and re-anchor that ceiling to the
  new value. Toggle live with 'l'. Purely a brightness safety net -- doesn't
  touch color, and doesn't affect detected/not-detected trial logging.

GAP-CLOSING METRICS (new this version, closing two gaps identified against
the plan's required metrics):
  measured_yaw_deg / yaw_error_deg -- extracted from the marker's rvec via
      cv2.Rodrigues (approximate: assumes a fixed marker-mounting axis
      convention, fine for relative comparison against --angle-deg ground
      truth, not a calibrated absolute yaw sensor). yaw_error_deg is blank
      unless --angle-deg was given for this block.
  latency_to_stable_ms -- wall-clock time from the first frame of a
      detection streak (not-detected -> detected transition) to the frame
      where --stable-frames (default 5) consecutive frames have been
      detected in a row. This is the plan's "time from marker in-frame to
      stable position output" metric, which v5's proc_time_ms/rolling
      latency stats (per-frame compute time, not settling time) don't
      capture. Blank until a streak actually reaches --stable-frames.

Everything else -- x/y/z-correction defaults (id0 DICT_4X4_50 50mm, x=1.1
y=1.15 z=2.0), target-ID locking, fixed dehaze/CLAHE/LED/AE, trial logging,
recording -- unchanged from camtestv5.py.

TARGET-ID LOCKING:
  --target-id (default 0) + --id-filter (default ON) -- only the marker
  whose ID matches --target-id counts as "detected"/gets tracked/logged;
  any other visible marker is drawn on-screen (for context -- e.g. you
  can see you're pointed at the wrong tag) but otherwise ignored, so a
  stray second marker in frame can't silently pollute the trial log with
  the wrong marker's pose. Pass --no-id-filter to fall back to v4's
  behavior (tracks whichever marker's corners came back first).

FIXED FOR THIS VERSION (no longer toggleable, no CLI escape hatch,
unchanged from camtestv4.py):
  dehaze        -- underwater dark-channel-prior dehaze, always ON
                   (--dehaze-omega/--dehaze-t0/--dehaze-downscale still
                   tune its strength/cost, they just can't turn it off)
  clahe         -- local contrast enhancement, always ON
                   (--clahe-clip still tunes strength)
  led           -- both DotStar strips, always ON (brightness 'i'/'k' and
                   color 'c' are still live-tunable, just not on/off)
  auto-exposure -- sensor AE, always ON (no manual exposure/gain lock --
                   that's what freed up the e/d/g/f keys, see below)

X/Y/Z TUNING (unchanged mechanism from camtestv4.py, new starting values):
  --x-correction / --y-correction / --z-correction default to 1.1 / 1.15 /
  2.0 -- calibrated for the id0 DICT_4X4_50 50mm marker this version
  targets, not v4's flat 1.6 guess.
  Live keys: e/d = x-correction up/down, g/f = y-correction up/down,
      [ / ] = z-correction up/down (e/d/g/f reuse the old exposure/gain
      keys now that AE is fixed and those keys were sitting idle).
  --true-x-cm / --true-y-cm -- ruler-measured ground-truth X/Y position
      for this block. --distance-cm doubles as ground-truth Z (it already
      existed for that purpose). When set, every trial row logs
      x_error_cm/y_error_cm/z_error_cm = measured - true, so accuracy is
      a number in the CSV instead of something eyeballed off the overlay.

Everything else -- trial logging, auto-recording, auto-logging, latency
tracking, software white balance / denoise / detector-tuning toggles,
temporal filter, sensor AWB lock -- is unchanged from camtestv3_led_dual.py.
See that file's docstring for the full history of those features.

CONTROLS:
  SPACE / n / q       -- log trial / force-log NOT-detected / quit
  v                   -- toggle raw/processed preview
  w                   -- toggle sensor auto-white-balance on/off
  2                   -- toggle software white balance (gray-world)
  3                   -- toggle underwater-tuning bundle (denoise + detector params)
  e / d               -- x-correction up/down
  g / f               -- y-correction up/down
  [ / ]               -- z-correction up/down
  i / k               -- LED brightness up/down (both strips)
  c                   -- cycle LED color (both strips)
  l                   -- toggle LED auto-adjust (keep lux under --max-lux)

USAGE:
  python3 camtestv6.py --condition 4 --position plusx --distance-cm 40 \\
      --turbidity-ntu 40 --csv results/9cond.csv

CSV COLUMNS:
  timestamp, session_id, condition_label, distance_cm, angle_deg, lateral_pct,
  turbidity_ntu, detected, marker_id, x_m, y_m, z_m,
  x_correction, y_correction, z_correction,
  true_x_cm, true_y_cm, x_error_cm, y_error_cm, z_error_cm,
  measured_yaw_deg, yaw_error_deg,
  reprojection_error_px, contrast_score, latency_to_stable_ms,
  exposure_us, analogue_gain, lux,
  awb_enabled, wb_red_gain, wb_blue_gain,
  white_balance_enabled, clahe_clip, gamma,
  underwater_tuning, proc_time_ms, rolling_latency_p95_ms,
  filtered_x_m, filtered_y_m, filtered_z_m, filter_state,
  led_brightness, led_color, led_auto_adjust,
  auto_logged, notes

  NOTE: different schema than camtestv5.py's CSV (measured_yaw_deg/
  yaw_error_deg/latency_to_stable_ms/led_auto_adjust added) -- don't
  concatenate without reconciling columns first.
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

import board
import adafruit_dotstar as dotstar

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

# camtestplan/9condition.md section 4 (test matrix): 3 turbidity levels x 3
# lighting levels. turbidity_ntu is deliberately NOT included here -- it's a
# physically-prepared/estimated value each session (see the plan's section 3.1
# limitation note about not disguising qualitative levels as precise NTU
# numbers), so --turbidity-ntu always has to be passed explicitly.
# led_brightness_hint is only a starting suggestion for that lighting tier,
# overridden by an explicit --led-brightness.
CONDITION_TABLE = {
    1: {"turbidity": "clear", "lighting": "normal", "led_brightness_hint": 0.30},
    2: {"turbidity": "clear", "lighting": "low", "led_brightness_hint": 0.12},
    3: {"turbidity": "clear", "lighting": "worstcase", "led_brightness_hint": 0.0},
    4: {"turbidity": "site_medium", "lighting": "normal", "led_brightness_hint": 0.30},
    5: {"turbidity": "site_medium", "lighting": "low", "led_brightness_hint": 0.12},
    6: {"turbidity": "site_medium", "lighting": "worstcase", "led_brightness_hint": 0.0},
    7: {"turbidity": "high", "lighting": "normal", "led_brightness_hint": 0.30},
    8: {"turbidity": "high", "lighting": "low", "led_brightness_hint": 0.12},
    9: {"turbidity": "high", "lighting": "worstcase", "led_brightness_hint": 0.0},
}

# camtestplan/9condition.md section 5 (spatial test positions), x/y offsets in cm.
POSITION_TABLE = {
    "center": (0.0, 0.0),
    "plusx": (10.0, 0.0),
    "minusx": (-10.0, 0.0),
    "plusy": (0.0, 10.0),
    "minusy": (0.0, -10.0),
    "diagonal": (15.0, 15.0),
}

# Cycle through these live with 'c'. Warm/cool white variants matter
# underwater since water absorbs red wavelengths first with distance --
# a slightly warm source can help color-based contrast at close range
# while straight/cool white may carry further before red drops out.
LED_COLOR_PRESETS = [
    ("white", (255, 255, 255)),
    ("warm_white", (255, 180, 120)),
    ("cool_white", (180, 200, 255)),
    ("red", (255, 0, 0)),
    ("green", (0, 255, 0)),
    ("blue", (0, 0, 255)),
]

CSV_FIELDS = [
    "timestamp", "session_id", "condition_label", "distance_cm", "angle_deg", "lateral_pct",
    "turbidity_ntu",
    "detected", "marker_id", "x_m", "y_m", "z_m",
    "x_correction", "y_correction", "z_correction",
    "true_x_cm", "true_y_cm", "x_error_cm", "y_error_cm", "z_error_cm",
    "measured_yaw_deg", "yaw_error_deg",
    "reprojection_error_px", "contrast_score", "latency_to_stable_ms",
    "exposure_us", "analogue_gain", "lux",
    "awb_enabled", "wb_red_gain", "wb_blue_gain",
    "white_balance_enabled", "clahe_clip", "gamma",
    "underwater_tuning", "proc_time_ms", "rolling_latency_p95_ms",
    "filtered_x_m", "filtered_y_m", "filtered_z_m", "filter_state",
    "led_brightness", "led_color", "led_auto_adjust",
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
    search range, error-correction tolerance). Separate from the
    image-enhancement pipeline (dehaze/CLAHE, both fixed ON in this
    version) -- this only covers the detector params, toggled with '3'.
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


def setup_led_strip(clock_pin_name, data_pin_name, num_pixels, brightness, baudrate):
    """
    SK9822/APA102 strip on its OWN clock+data pins. adafruit_dotstar picks
    hardware SPI automatically when given board.SCK/board.MOSI and falls
    back to bitbang (software SPI, any two GPIO) for any other pin pair --
    that's what lets the second strip run on its own pins concurrently
    with the first instead of sharing the one hardware SPI bus.
    """
    try:
        clock_pin = getattr(board, clock_pin_name)
        data_pin = getattr(board, data_pin_name)
    except AttributeError as e:
        raise SystemExit(
            f"Unknown board pin in {clock_pin_name!r}/{data_pin_name!r} ({e}). "
            f"Use names as they appear on `import board; dir(board)`, e.g. SCK, MOSI, D5, D6."
        )
    return dotstar.DotStar(clock_pin, data_pin, num_pixels,
                            brightness=brightness, auto_write=False, baudrate=baudrate)


def apply_led_state(strips, color, brightness):
    """Applies the SAME state to every strip in `strips` -- this is what keeps
    the two strips in sync. LEDs are always on in this version, so there's no
    on/off argument -- just color/brightness."""
    for pixels in strips:
        pixels.brightness = brightness
        pixels.fill(color)
        pixels.show()


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
              "e+/e- = x-correction up/down | g+/g- = y-correction up/down | "
              "z+/z- = z-correction up/down | "
              "w = toggle auto-white-balance | 2 = toggle white balance | "
              "3 = toggle underwater-tuning bundle (denoise+detector) | "
              "i/k = LED brightness up/down | c = cycle LED color | "
              "l = toggle LED auto-adjust (keep lux under --max-lux) | q = quit")
        while True:
            try:
                line = input()
            except EOFError:
                q.put("q")
                break
            cmd = line.strip().lower()
            if cmd in ("", "space", "s"):
                q.put(" ")
            elif cmd in ("n", "e+", "e-", "g+", "g-", "z+", "z-", "w", "2", "3", "i", "k", "c", "l", "q"):
                q.put(cmd)
                if cmd == "q":
                    break
            else:
                print(f"  (unrecognized input '{line}')")

    t = threading.Thread(target=_reader, daemon=True)
    t.start()
    return q


def main():
    parser = argparse.ArgumentParser(description="ArUco test harness for the 9-condition "
                                                   "turbidity x lighting test plan -- dehaze/"
                                                   "CLAHE/LED/AE fixed ON, X/Y/Z correction "
                                                   "defaults calibrated for id0 DICT_4X4_50 "
                                                   "50mm marker, --condition/--position presets, "
                                                   "LED brightness auto-adjusts to keep measured "
                                                   "lux under --max-lux")
    parser.add_argument("--dict", default="DICT_4X4_50", choices=ARUCO_DICTS.keys())
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--calib", default=None,
                         help="Path to .npz with camera_matrix/dist_coeffs (real checkerboard calib)")
    parser.add_argument("--hfov", type=float, default=100.0)
    parser.add_argument("--vfov", type=float, default=72.0)
    parser.add_argument("--marker-size", type=float, default=0.05,
                         help="Marker BLACK SQUARE side length in meters (default 0.05 = 50mm, "
                              "matching this script's calibrated target marker).")
    parser.add_argument("--x-correction", type=float, default=1.1,
                         help="Empirical multiplier on x_m only (default 1.1, calibrated for the "
                              "id0 DICT_4X4_50 50mm target marker). Live-tunable with e/d.")
    parser.add_argument("--y-correction", type=float, default=1.15,
                         help="Empirical multiplier on y_m only (default 1.15, calibrated for the "
                              "id0 DICT_4X4_50 50mm target marker). Live-tunable with g/f.")
    parser.add_argument("--z-correction", type=float, default=2.0,
                         help="Empirical multiplier on z_m only (default 2.0, calibrated for the "
                              "id0 DICT_4X4_50 50mm target marker). Live-tunable with [ / ].")
    parser.add_argument("--target-id", type=int, default=0,
                         help="Only track this ArUco marker ID (default 0). Ignored if "
                              "--no-id-filter is passed. See --id-filter.")
    parser.add_argument("--id-filter", action=argparse.BooleanOptionalAction, default=True,
                         help="When ON (default), only the marker matching --target-id counts as "
                              "'detected'/gets its pose logged -- other visible IDs are drawn on "
                              "screen for context but otherwise ignored, so a stray second marker "
                              "can't pollute the trial log. --no-id-filter tracks whichever "
                              "marker's corners came back first instead (camtestv4.py behavior).")
    parser.add_argument("--underwater-tuning", action="store_true",
                         help="Widen detector adaptive-threshold range + subpixel corner "
                              "refinement + looser error correction, and enable denoise. "
                              "Detector/denoise only -- dehaze and CLAHE are always on regardless.")
    parser.add_argument("--no-preview", action="store_true")

    # --- image enhancement pipeline (dehaze + CLAHE are fixed ON, see module docstring) ---
    pipeline_group = parser.add_argument_group("image enhancement pipeline")
    pipeline_group.add_argument("--dehaze-omega", type=float, default=0.85,
                                 help="Haze removal strength, 0-1 (default 0.85). Dehaze itself "
                                      "is always on in this version.")
    pipeline_group.add_argument("--dehaze-t0", type=float, default=0.15,
                                 help="Transmission floor (default 0.15). Raise if the murkiest "
                                      "frames come out noisy/blown-out after dehaze.")
    pipeline_group.add_argument("--dehaze-downscale", type=int, default=4,
                                 help="Downscale factor for the expensive dehaze estimate "
                                      "(default 4). Raise (e.g. 8) if dehaze is too slow.")
    pipeline_group.add_argument("--white-balance", action=argparse.BooleanOptionalAction, default=False,
                                 help="Software gray-world color correction, on top of locking "
                                      "the sensor's own AWB below. Still independently toggleable "
                                      "live with '2'.")
    pipeline_group.add_argument("--clahe-clip", type=float, default=3.0,
                                 help="CLAHE clipLimit (default 3.0). Higher = more contrast "
                                      "boost, but amplifies noise/turbidity graininess too. CLAHE "
                                      "itself is always on in this version.")
    pipeline_group.add_argument("--gamma", type=float, default=1.0,
                                 help="Brightness gamma applied after CLAHE (default 1.0 = no-op).")
    pipeline_group.add_argument("--denoise", action=argparse.BooleanOptionalAction, default=None,
                                 help="Bilateral denoise as the pipeline's last step. Default: "
                                      "follows --underwater-tuning.")

    # --- LED lighting (SK9822/APA102, two strips, always ON in this version) ---
    led_group = parser.add_argument_group("LED lighting (SK9822/APA102, two strips in sync, always ON)")
    led_group.add_argument("--led-num-pixels", type=int, default=10,
                            help="Number of LEDs on EACH strip (default: 10). Both strips are "
                                 "assumed the same length since they're always driven in sync.")
    led_group.add_argument("--led-brightness", type=float, default=None,
                            help="Starting LED brightness, 0.0-1.0. Default: 0.3, UNLESS "
                                 "--condition is given, in which case that condition's lighting "
                                 "tier suggests a starting value (normal=0.30, low=0.12, "
                                 "worstcase=0.0) -- pass --led-brightness explicitly to override "
                                 "either default. Start low underwater -- backscatter off "
                                 "suspended particles gets worse fast as this climbs, "
                                 "especially at higher --turbidity-ntu. Live-tunable with i/k.")
    led_group.add_argument("--led-baudrate", type=int, default=4000000,
                            help="SPI clock rate in Hz for both strips (default: 4000000). "
                                 "Lower this (e.g. 400000) if an uncut/long strip or loose "
                                 "jumper wiring is causing signal integrity issues. Only "
                                 "actually matters for whichever strip(s) land on hardware SPI "
                                 "pins -- bitbang strips are paced in software instead.")
    led_group.add_argument("--led-color", default="white",
                            choices=[name for name, _ in LED_COLOR_PRESETS],
                            help="Starting LED color preset (default: white), applied to both "
                                 "strips. Cycle live with 'c'.")
    led_group.add_argument("--led1-clock-pin", default="SCK",
                            help="board.* pin name for strip 1's clock line (default: SCK -- "
                                 "hardware SPI clock). Paired with --led1-data-pin.")
    led_group.add_argument("--led1-data-pin", default="MOSI",
                            help="board.* pin name for strip 1's data line (default: MOSI -- "
                                 "hardware SPI MOSI). Paired with --led1-clock-pin.")
    led_group.add_argument("--led2-clock-pin", default="D5",
                            help="board.* pin name for strip 2's clock line (default: D5). Must "
                                 "differ from strip 1's pins -- the Pi only has one hardware SPI "
                                 "clock/MOSI pair, so this strip runs over bitbang (software) "
                                 "SPI on whatever GPIO you wire it to.")
    led_group.add_argument("--led2-data-pin", default="D6",
                            help="board.* pin name for strip 2's data line (default: D6). Paired "
                                 "with --led2-clock-pin.")
    led_group.add_argument("--max-lux", type=float, default=150.0,
                            help="Lux ceiling the LED auto-adjust loop targets (default: 150). "
                                 "Only takes effect when --led-auto-adjust is ON.")
    led_group.add_argument("--led-auto-adjust", action=argparse.BooleanOptionalAction, default=True,
                            help="Continuously trim LED brightness down when metered lux exceeds "
                                 "--max-lux, and ease it back up toward --led-brightness (or "
                                 "whatever i/k/--condition last set) once there's headroom again "
                                 "(default: ON, --no-led-auto-adjust to disable). Never drives the "
                                 "strip brighter than the current manually-set level. Live-tunable "
                                 "with 'l'.")
    led_group.add_argument("--led-adjust-step", type=float, default=0.02,
                            help="Brightness step per auto-adjust tick (default: 0.02).")
    led_group.add_argument("--led-adjust-interval", type=float, default=0.2,
                            help="Seconds between auto-adjust ticks (default: 0.2s) -- rate-limits "
                                 "changes so the strip doesn't visibly flicker/hunt.")

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

    # --- solo-testing helpers: recording, auto-logging, latency ---
    solo_group = parser.add_argument_group("solo testing")
    solo_group.add_argument("--record", action=argparse.BooleanOptionalAction, default=True,
                             help="Auto-start wf-recorder screen recording at launch, stop "
                                  "cleanly on exit (default ON -- --no-record to disable). Run "
                                  "WITHOUT --no-preview for the recording to show the actual "
                                  "annotated camera feed, not just the terminal.")
    solo_group.add_argument("--recording-file", default=None,
                             help="Explicit path for the screen recording. Default: auto-named "
                                  "under --recording-dir from the condition label + timestamp.")
    solo_group.add_argument("--recording-dir", default="recordings",
                             help="Directory for auto-named recordings (default: recordings/).")
    solo_group.add_argument("--auto-log", action=argparse.BooleanOptionalAction, default=False,
                             help="Log trials automatically instead of requiring SPACE. "
                                  "SPACE/n still work on top of this.")
    solo_group.add_argument("--auto-log-interval", type=float, default=0.0,
                             help="Seconds between auto-logged rows (default 0.0 = every frame). "
                                  "Raise this if every-frame logging produces more correlated "
                                  "data than you want for your report.")
    solo_group.add_argument("--latency-warn-ms", type=float, default=100.0,
                             help="Rolling p95 latency threshold (ms) above which the overlay "
                                  "flags red (default 100ms). Doesn't stop anything -- it's a "
                                  "signal to back off dehaze/resolution/downscale, not a hard limit.")
    solo_group.add_argument("--latency-window", type=int, default=90,
                             help="Number of recent frames the rolling latency stats are "
                                  "computed over (default 90, i.e. a few seconds of history).")

    # --- experiment/trial labeling ---
    exp_group = parser.add_argument_group("experiment/trial labeling")
    exp_group.add_argument("--csv", required=True, help="Path to CSV file to append trial rows to")
    exp_group.add_argument("--condition-label", default="",
                            help="Free-text label for this test block, e.g. 'xy_tune_100cm'")
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
    exp_group.add_argument("--true-x-cm", type=float, default=None,
                            help="Ruler-measured ground-truth X position (cm) for this block, "
                                 "same pattern as --distance-cm for Z. When set, every trial "
                                 "logs x_error_cm = measured x - true x.")
    exp_group.add_argument("--true-y-cm", type=float, default=None,
                            help="Ruler-measured ground-truth Y position (cm) for this block. "
                                 "When set, every trial logs y_error_cm = measured y - true y.")
    exp_group.add_argument("--condition", type=int, choices=range(1, 10), default=None,
                            metavar="1-9",
                            help="9-condition test-plan preset (camtestplan/9condition.md "
                                 "section 4): auto-builds --condition-label from the turbidity/"
                                 "lighting labels if you didn't pass one, and suggests a starting "
                                 "--led-brightness for that lighting tier unless you pass that "
                                 "explicitly. Does NOT set --turbidity-ntu -- pass that yourself.")
    exp_group.add_argument("--position", choices=list(POSITION_TABLE.keys()), default=None,
                            help="Test-plan x-y position preset (section 5): fills --true-x-cm/"
                                 "--true-y-cm from the spec's offsets unless you pass those "
                                 "yourself.")
    exp_group.add_argument("--reps", type=int, default=3,
                            help="Display-only target repeat count for this block (default 3, "
                                 "matching the test plan's 3-repeat blocks). Shown on the overlay "
                                 "as a progress counter -- does not auto-stop the session.")
    exp_group.add_argument("--stable-frames", type=int, default=5,
                            help="Consecutive detected frames required before a detection streak "
                                 "counts as 'stable' (default 5). latency_to_stable_ms measures "
                                 "wall-clock time from the first frame of the streak to this "
                                 "point -- the test plan's 'time from marker in-frame to stable "
                                 "position output' metric.")
    args = parser.parse_args()

    # --- test-plan condition/position presets (fill only what wasn't passed explicitly) ---
    if args.condition is not None:
        preset = CONDITION_TABLE[args.condition]
        if not args.condition_label:
            pos_part = args.position or "unspecified"
            depth_part = f"{args.distance_cm:.0f}cm" if args.distance_cm is not None else "depthNA"
            args.condition_label = (f"cond{args.condition}_{preset['turbidity']}_"
                                     f"{preset['lighting']}_{pos_part}_{depth_part}")
        if args.led_brightness is None:
            args.led_brightness = preset["led_brightness_hint"]
    if args.led_brightness is None:
        args.led_brightness = 0.3
    if args.position is not None:
        pos_x, pos_y = POSITION_TABLE[args.position]
        if args.true_x_cm is None:
            args.true_x_cm = pos_x
        if args.true_y_cm is None:
            args.true_y_cm = pos_y

    # --denoise defaults to following --underwater-tuning, same reasoning
    # as camtestv3_led_dual.py -- only override independently if you're
    # deliberately decoupling them.
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

    # --- LED setup (always on in this version -- both strips are set up
    # unconditionally, each on its own pins, always driven together, see
    # apply_led_state()). ---
    led_color_idx = next((i for i, (name, _) in enumerate(LED_COLOR_PRESETS)
                           if name == args.led_color), 0)
    led_brightness = max(0.0, min(1.0, args.led_brightness))
    LED_BRIGHTNESS_STEP = 0.05
    strip1 = setup_led_strip(args.led1_clock_pin, args.led1_data_pin,
                              args.led_num_pixels, led_brightness, args.led_baudrate)
    strip2 = setup_led_strip(args.led2_clock_pin, args.led2_data_pin,
                              args.led_num_pixels, led_brightness, args.led_baudrate)
    strips = [strip1, strip2]
    apply_led_state(strips, LED_COLOR_PRESETS[led_color_idx][1], led_brightness)
    print(f"LEDs ready (always ON): 2 strips x {args.led_num_pixels}px "
          f"(strip1 clk={args.led1_clock_pin}/data={args.led1_data_pin}, "
          f"strip2 clk={args.led2_clock_pin}/data={args.led2_data_pin}), "
          f"brightness={led_brightness:.2f}, "
          f"color={LED_COLOR_PRESETS[led_color_idx][0]}")

    # --- camera setup ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    picam2.set_controls({"AeEnable": True})  # AE fixed ON for this version, see module docstring

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
          f"turbidity_ntu={args.turbidity_ntu} true_x_cm={args.true_x_cm} true_y_cm={args.true_y_cm}")
    print(f"Fixed ON: dehaze white-balance-sensor-AE clahe(clip={args.clahe_clip}) led -- "
          f"gamma={args.gamma} denoise={denoise_start} underwater_tuning={args.underwater_tuning} "
          f"temporal_filter={args.temporal_filter}")
    print(f"X/Y/Z tuning start: x_correction={args.x_correction} y_correction={args.y_correction} "
          f"z_correction={args.z_correction}")
    print(f"Target marker: dict={args.dict} marker_size={args.marker_size * 1000:.0f}mm "
          f"target_id={args.target_id} id_filter={'ON' if args.id_filter else 'OFF'}")
    if args.condition is not None:
        preset = CONDITION_TABLE[args.condition]
        print(f"Test-plan condition {args.condition}: turbidity={preset['turbidity']} "
              f"lighting={preset['lighting']} position={args.position or 'unspecified'} "
              f"-> condition_label='{args.condition_label}' led_brightness={args.led_brightness}")
    print(f"Target block reps: {args.reps} | latency-to-stable threshold: "
          f"{args.stable_frames} consecutive detected frames")
    print(f"LED auto-adjust: {'ON' if args.led_auto_adjust else 'off'} target<{args.max_lux:.0f} lux "
          f"(step={args.led_adjust_step:.3f} interval={args.led_adjust_interval:.2f}s)")
    if args.auto_log:
        interval_desc = "every frame" if args.auto_log_interval <= 0 else f"every {args.auto_log_interval}s"
        print(f"Auto-logging ON ({interval_desc}) -- SPACE/n still work for manual marks. "
              f"Change condition_label/turbidity_ntu/distance_cm/true_x_cm/true_y_cm etc. "
              f"between blocks by relaunching.")
    print(f"Latency warning threshold: {args.latency_warn_ms:.0f}ms rolling p95 "
          f"(window={args.latency_window} frames)")
    print("Controls: SPACE = log trial | n = force-log as NOT detected | "
          "e/d = x-correction up/down | g/f = y-correction up/down | [/] = z-correction up/down | "
          "w = toggle auto-white-balance | v = toggle raw/processed view | "
          "2 = toggle white balance | 3 = toggle underwater-tuning bundle (denoise+detector) | "
          "i/k = LED brightness up/down | c = cycle LED color | "
          "l = toggle LED auto-adjust (keep lux under --max-lux) | q = quit")

    window_name = ("ArUco Test Harness v6 (9-condition test plan, id0 4x4_50 50mm, "
                    "X/Y/Z tuning, LED auto-adjust)")
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

    # --- live X/Y/Z correction state (this version's main tuning knobs) ---
    x_correction = args.x_correction
    y_correction = args.y_correction
    z_correction = args.z_correction
    CORRECTION_STEP = 0.05

    # --- detection-streak state, for latency_to_stable_ms (new this version,
    # see module docstring) ---
    streak_start_time = None
    streak_consecutive_frames = 0
    streak_stable_latency_ms = None
    was_detected_prev = False

    # --- LED auto-adjust state (folded in from camtestv6.py, see module docstring) ---
    led_auto_adjust_enabled = args.led_auto_adjust
    led_target_brightness = led_brightness  # ceiling the auto-loop eases back up toward
    last_led_adjust_time = 0.0

    # --- live white balance state (sensor AWB, unchanged from v3) ---
    awb_enabled = not (args.manual_wb_red is not None or args.manual_wb_blue is not None)
    current_wb_red = args.manual_wb_red if args.manual_wb_red is not None else 2.0
    current_wb_blue = args.manual_wb_blue if args.manual_wb_blue is not None else 1.2

    def apply_manual_wb():
        picam2.set_controls({
            "AwbEnable": False,
            "ColourGains": (current_wb_red, current_wb_blue),
        })

    def apply_auto_wb():
        picam2.set_controls({"AwbEnable": True})

    # --- live pipeline-stage toggles (dehaze/CLAHE are fixed ON, not toggleable) ---
    wb_sw_enabled = args.white_balance
    denoise_enabled = denoise_start
    underwater_tuning_enabled = args.underwater_tuning

    # --- debug view toggle ---
    view_processed = False

    # --- temporal filter ---
    tracker = PoseTracker(max_coast_frames=args.coast_frames) if args.temporal_filter else None

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
        x_error_cm = ""
        y_error_cm = ""
        z_error_cm = ""
        if not force_not_detected and detected:
            if args.true_x_cm is not None:
                x_error_cm = round((x * 100.0) - args.true_x_cm, 2)
            if args.true_y_cm is not None:
                y_error_cm = round((y * 100.0) - args.true_y_cm, 2)
            if args.distance_cm is not None:
                z_error_cm = round((z * 100.0) - args.distance_cm, 2)
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
            "x_correction": round(x_correction, 3),
            "y_correction": round(y_correction, 3),
            "z_correction": round(z_correction, 3),
            "true_x_cm": args.true_x_cm,
            "true_y_cm": args.true_y_cm,
            "x_error_cm": x_error_cm,
            "y_error_cm": y_error_cm,
            "z_error_cm": z_error_cm,
            "measured_yaw_deg": "" if (force_not_detected or measured_yaw_deg is None) else round(measured_yaw_deg, 2),
            "yaw_error_deg": "" if (force_not_detected or yaw_error_deg is None) else round(yaw_error_deg, 2),
            "reprojection_error_px": "" if force_not_detected else reproj_err,
            "contrast_score": frame_contrast,
            "latency_to_stable_ms": "" if (force_not_detected or streak_stable_latency_ms is None) else round(streak_stable_latency_ms, 1),
            "exposure_us": meta.get("ExposureTime", ""),
            "analogue_gain": meta.get("AnalogueGain", ""),
            "lux": meta.get("Lux", ""),
            "awb_enabled": awb_enabled,
            "wb_red_gain": "" if awb_enabled else current_wb_red,
            "wb_blue_gain": "" if awb_enabled else current_wb_blue,
            "white_balance_enabled": wb_sw_enabled,
            "clahe_clip": args.clahe_clip,
            "gamma": args.gamma,
            "underwater_tuning": underwater_tuning_enabled,
            "proc_time_ms": round(proc_time_ms, 1),
            "rolling_latency_p95_ms": round(rolling_p95, 1),
            "filtered_x_m": "" if filtered_x is None else filtered_x,
            "filtered_y_m": "" if filtered_y is None else filtered_y,
            "filtered_z_m": "" if filtered_z is None else filtered_z,
            "filter_state": filter_state,
            "led_brightness": round(led_brightness, 3),
            "led_color": LED_COLOR_PRESETS[led_color_idx][0],
            "led_auto_adjust": led_auto_adjust_enabled,
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
                  f"xerr={row['x_error_cm']} yerr={row['y_error_cm']} zerr={row['z_error_cm']} auto={is_auto}")

    try:
        while True:
            t_proc_start = time.perf_counter()
            frame = picam2.capture_array()  # "RGB888" -- actually [B,G,R] order, see underwater_pipeline.py docstring

            detect_input = apply_pipeline(
                frame,
                dehaze_enabled=True,
                wb_enabled=wb_sw_enabled,
                clahe_enabled=True,
                clahe_clip=args.clahe_clip,
                gamma=args.gamma,
                denoise_enabled=denoise_enabled,
                dehaze_omega=args.dehaze_omega,
                dehaze_t0=args.dehaze_t0,
                dehaze_downscale=args.dehaze_downscale,
            )

            corners, ids, _ = cv2.aruco.detectMarkers(detect_input, aruco_dict, parameters=aruco_params)

            any_seen = ids is not None
            target_idx = None
            other_ids_seen = []
            if any_seen:
                ids_flat = ids.flatten()
                if args.id_filter:
                    matches = np.flatnonzero(ids_flat == args.target_id)
                    if matches.size > 0:
                        target_idx = int(matches[0])
                    other_ids_seen = [int(i) for i in ids_flat if i != args.target_id]
                else:
                    target_idx = 0

            detected = target_idx is not None
            marker_id, x, y, z = None, None, None, None
            measured_yaw_deg, yaw_error_deg = None, None
            reproj_err = None
            frame_contrast = contrast_score(detect_input)

            # --- detection-streak tracking, for latency_to_stable_ms (see module
            # docstring) -- runs regardless of view mode, ahead of pose estimation
            # so the timing isn't skewed by drawing/overlay work below ---
            now_wall = time.time()
            if detected:
                if not was_detected_prev:
                    streak_start_time = now_wall
                    streak_consecutive_frames = 0
                    streak_stable_latency_ms = None
                streak_consecutive_frames += 1
                if streak_stable_latency_ms is None and streak_consecutive_frames >= args.stable_frames:
                    streak_stable_latency_ms = (now_wall - streak_start_time) * 1000.0
            else:
                streak_start_time = None
                streak_consecutive_frames = 0
                streak_stable_latency_ms = None
            was_detected_prev = detected

            display = cv2.cvtColor(detect_input, cv2.COLOR_GRAY2BGR) if view_processed else frame.copy()

            if any_seen:
                # draws every marker seen (including ignored non-target ones) so a
                # stray tag in frame is visible even when it's not being tracked
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

            if detected:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.marker_size, camera_matrix, dist_coeffs
                )
                marker_id = int(ids_flat[target_idx])
                rvec_raw, tvec_raw = rvecs[target_idx], tvecs[target_idx]

                reproj_err = reprojection_error(
                    corners[target_idx][0], rvec_raw, tvec_raw, args.marker_size,
                    camera_matrix, dist_coeffs
                )

                x, y, z = tvec_raw[0]
                x *= x_correction
                y *= y_correction
                z *= z_correction

                # Approximate yaw from the rotation matrix -- assumes a fixed
                # marker-mounting axis convention, good enough for relative
                # comparison against --angle-deg ground truth, not a
                # calibrated absolute yaw sensor (see module docstring).
                rot_matrix, _ = cv2.Rodrigues(rvec_raw)
                measured_yaw_deg = math.degrees(math.atan2(rot_matrix[1, 0], rot_matrix[0, 0]))
                if args.angle_deg is not None:
                    yaw_error_deg = measured_yaw_deg - args.angle_deg

                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                                   rvec_raw, tvec_raw, args.marker_size * 0.5)
                cv2.putText(display, f"id={marker_id} x={x:.3f}m y={y:.3f}m z={z:.3f}m err={reproj_err:.1f}px",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                not_detected_label = "NOT DETECTED"
                if other_ids_seen:
                    not_detected_label += f" (target id={args.target_id} not seen; other ids: {other_ids_seen})"
                cv2.putText(display, not_detected_label, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7 if other_ids_seen else 0.8, (0, 0, 255), 2)

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

            # --- LED auto-adjust (folded in from camtestv6.py, see module
            # docstring). Metered lux lags the frame it's read for by one
            # capture (same as exposure/gain above), fine for a slow
            # brightness trim loop -- rate-limited to --led-adjust-interval
            # regardless. ---
            lux_over_limit = isinstance(live_lux, (int, float)) and live_lux > args.max_lux
            if led_auto_adjust_enabled and isinstance(live_lux, (int, float)):
                now_t_led = time.time()
                if now_t_led - last_led_adjust_time >= args.led_adjust_interval:
                    last_led_adjust_time = now_t_led
                    new_brightness = led_brightness
                    if live_lux > args.max_lux:
                        new_brightness = max(0.0, led_brightness - args.led_adjust_step)
                    elif live_lux < args.max_lux * 0.9 and led_brightness < led_target_brightness:
                        new_brightness = min(led_target_brightness, led_brightness + args.led_adjust_step)
                    if new_brightness != led_brightness:
                        led_brightness = new_brightness
                        apply_led_state(strips, LED_COLOR_PRESETS[led_color_idx][1], led_brightness)

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
                      f"--dehaze-downscale higher or lowering --width/--height.")
                last_latency_warn_print = time.time()

            cv2.putText(display, f"trials logged: {trial_count}", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(display, meta_text, (20, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 0), 2)
            awb_label = "AWB: AUTO" if awb_enabled else f"AWB: MANUAL r={current_wb_red:.2f} b={current_wb_blue:.2f}"
            cv2.putText(display, awb_label, (20, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            pipe_label = (f"PIPE(fixed): dehaze=ON clahe=ON led=ON ae=ON | "
                          f"wb={'ON' if wb_sw_enabled else 'off'} "
                          f"denoise={'ON' if denoise_enabled else 'off'} "
                          f"detector-tuning={'ON' if underwater_tuning_enabled else 'off'} "
                          f"gamma={args.gamma:.2f} | "
                          f"target=id{args.target_id}(filter={'ON' if args.id_filter else 'OFF'})")
            cv2.putText(display, pipe_label, (20, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 255), 2)
            correction_label = (f"CORRECTION: x={x_correction:.2f} (e/d) y={y_correction:.2f} (g/f) "
                                 f"z={z_correction:.2f} ([/])")
            cv2.putText(display, correction_label, (20, 195),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            if detected and (args.true_x_cm is not None or args.true_y_cm is not None or args.distance_cm is not None):
                xe = f"{(x * 100.0) - args.true_x_cm:+.2f}cm" if args.true_x_cm is not None else "n/a"
                ye = f"{(y * 100.0) - args.true_y_cm:+.2f}cm" if args.true_y_cm is not None else "n/a"
                ze = f"{(z * 100.0) - args.distance_cm:+.2f}cm" if args.distance_cm is not None else "n/a"
                error_label = f"ERROR vs true: x={xe} y={ye} z={ze}"
                cv2.putText(display, error_label, (20, 225),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            quality_label = (f"contrast={frame_contrast:.0f}  "
                              f"proc={proc_time_ms:.0f}ms (~{1000 / max(proc_time_ms, 1):.0f}fps)")
            cv2.putText(display, quality_label, (20, 255),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
            latency_color = (0, 0, 255) if latency_over_budget else (100, 255, 100)
            latency_label = (f"LATENCY mean={rolling_mean:.0f} p95={rolling_p95:.0f} "
                              f"max={rolling_max:.0f}ms (warn>{args.latency_warn_ms:.0f}ms)"
                              + ("  HIGH" if latency_over_budget else " OK"))
            cv2.putText(display, latency_label, (20, 285),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, latency_color, 2)
            view_label = "VIEW: PROCESSED (what detector sees)" if view_processed else "VIEW: RAW"
            cv2.putText(display, view_label, (20, 315),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 255), 2)
            if tracker is not None:
                if filter_state == "lost" or filtered_x is None:
                    filt_label = "AUV-view: LOST"
                else:
                    filt_label = (f"AUV-view: {filter_state.upper()} "
                                   f"x={filtered_x:.2f} y={filtered_y:.2f} z={filtered_z:.2f}")
                cv2.putText(display, filt_label, (20, 345),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 2)

            led_label = (f"LED x2: ON {LED_COLOR_PRESETS[led_color_idx][0]} bri={led_brightness:.2f}")
            cv2.putText(display, led_label, (20, 375),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 120, 255), 2)

            reps_done = trial_count  # this launch == this block, matches --reps' per-block intent
            reps_color = (100, 255, 100) if reps_done >= args.reps else (200, 200, 200)
            reps_label = f"block reps: {reps_done}/{args.reps}"
            cv2.putText(display, reps_label, (20, 405),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, reps_color, 2)

            if detected:
                yaw_label = f"yaw={measured_yaw_deg:+.1f}deg"
                if yaw_error_deg is not None:
                    yaw_label += f" yaw_err={yaw_error_deg:+.1f}deg"
                if streak_stable_latency_ms is not None:
                    yaw_label += f" | stable in {streak_stable_latency_ms:.0f}ms"
                else:
                    yaw_label += f" | stabilizing ({streak_consecutive_frames}/{args.stable_frames})"
            else:
                yaw_label = "yaw=n/a | latency-to-stable=n/a"
            cv2.putText(display, yaw_label, (20, 435),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            auto_adjust_color = (0, 0, 255) if lux_over_limit else (100, 255, 100)
            auto_adjust_label = (f"LED auto-adjust: {'ON' if led_auto_adjust_enabled else 'off'} "
                                  f"(max={args.max_lux:.0f} lux)" + (" OVER LIMIT" if lux_over_limit else ""))
            cv2.putText(display, auto_adjust_label, (20, 465),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, auto_adjust_color, 2)

            if flash_text and (time.time() - last_flash_time) < 1.0:
                cv2.putText(display, flash_text, (20, args.height - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if not args.no_preview:
                # `display` is already [B,G,R] order (what picamera2's "RGB888"
                # format actually delivers, and what cv2.imshow wants) -- no
                # extra conversion needed here.
                cv2.imshow(window_name, display)
                raw_key = cv2.waitKey(1) & 0xFF
                key_map = {
                    ord(" "): " ", ord("n"): "n", ord("q"): "q",
                    ord("e"): "e+", ord("d"): "e-",
                    ord("g"): "g+", ord("f"): "g-",
                    ord("["): "z+", ord("]"): "z-",
                    ord("v"): "v", ord("w"): "w",
                    ord("2"): "2", ord("3"): "3",
                    ord("i"): "i", ord("k"): "k",
                    ord("c"): "c", ord("l"): "l",
                }
                cmd = key_map.get(raw_key)
            else:
                try:
                    cmd = stdin_queue.get_nowait()
                except queue.Empty:
                    cmd = None
                if time.time() - last_status_print >= 0.5:
                    if detected:
                        status = f"id={marker_id} x={x:.3f}m y={y:.3f}m z={z:.3f}m"
                    elif other_ids_seen:
                        status = f"NOT DETECTED (target id={args.target_id} not seen; other ids: {other_ids_seen})"
                    else:
                        status = "NOT DETECTED"
                    auto_flag = "auto" if led_auto_adjust_enabled else "manual"
                    led_status = f"LED(ON,{LED_COLOR_PRESETS[led_color_idx][0]},{led_brightness:.2f},{auto_flag})"
                    lux_flag = " LUX-OVER-LIMIT" if lux_over_limit else ""
                    print(f"\rlive: {status}  {meta_text}  x_corr={x_correction:.2f} y_corr={y_correction:.2f} "
                          f"z_corr={z_correction:.2f}  {led_status}{lux_flag}   (trials logged: {trial_count})   ",
                          end="", flush=True)
                    last_status_print = time.time()

            # --- dispatch ---
            if cmd == "q":
                break

            elif cmd in (" ", "n"):
                log_trial(force_not_detected=(cmd == "n"), is_auto=False)

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

            elif cmd == "2":
                wb_sw_enabled = not wb_sw_enabled
                print(f"\nSoftware white balance: {'ON' if wb_sw_enabled else 'OFF'}")

            elif cmd == "3":
                # Toggles denoise + widened detector adaptive-threshold/corner-
                # refinement params together (CLAHE is fixed ON regardless in
                # this version, so it's no longer part of this bundle).
                underwater_tuning_enabled = not underwater_tuning_enabled
                denoise_enabled = underwater_tuning_enabled
                aruco_params = build_aruco_params(underwater_tuning_enabled)
                print(f"\nUnderwater tuning (denoise+detector params): "
                      f"{'ON' if underwater_tuning_enabled else 'OFF'}")

            elif cmd in ("i", "k"):
                if cmd == "i":
                    led_brightness = min(led_brightness + LED_BRIGHTNESS_STEP, 1.0)
                else:
                    led_brightness = max(led_brightness - LED_BRIGHTNESS_STEP, 0.0)
                led_target_brightness = led_brightness  # manual nudge re-anchors the auto-adjust ceiling
                apply_led_state(strips, LED_COLOR_PRESETS[led_color_idx][1], led_brightness)
                print(f"\nLED brightness={led_brightness:.2f} (both strips)")

            elif cmd == "c":
                led_color_idx = (led_color_idx + 1) % len(LED_COLOR_PRESETS)
                apply_led_state(strips, LED_COLOR_PRESETS[led_color_idx][1], led_brightness)
                print(f"\nLED color={LED_COLOR_PRESETS[led_color_idx][0]} (both strips)")

            elif cmd == "l":
                led_auto_adjust_enabled = not led_auto_adjust_enabled
                if led_auto_adjust_enabled:
                    led_target_brightness = led_brightness  # resume easing back up toward current level
                print(f"\nLED auto-adjust (target <{args.max_lux:.0f} lux): "
                      f"{'ON' if led_auto_adjust_enabled else 'OFF'}")

            elif cmd in ("e+", "e-"):
                if cmd == "e+":
                    x_correction += CORRECTION_STEP
                else:
                    x_correction -= CORRECTION_STEP
                print(f"\nx_correction={x_correction:.3f}")

            elif cmd in ("g+", "g-"):
                if cmd == "g+":
                    y_correction += CORRECTION_STEP
                else:
                    y_correction -= CORRECTION_STEP
                print(f"\ny_correction={y_correction:.3f}")

            elif cmd in ("z+", "z-"):
                if cmd == "z+":
                    z_correction += CORRECTION_STEP
                else:
                    z_correction -= CORRECTION_STEP
                print(f"\nz_correction={z_correction:.3f}")

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        picam2.stop()
        for pixels in strips:
            pixels.fill((0, 0, 0))
            pixels.show()
        if not args.no_preview:
            cv2.destroyAllWindows()
        stop_recording(recorder_proc, recording_file)
        print(f"Done. {trial_count} trials logged to {args.csv}")


if __name__ == "__main__":
    main()
