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
import os
import subprocess
import sys
import time
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls as libcamera_controls

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CAMERA_DIR = os.path.abspath(os.path.join(_THIS_DIR, '..', 'camera'))
if _CAMERA_DIR not in sys.path:
    sys.path.insert(0, _CAMERA_DIR)

from underwater_pipeline import apply_pipeline  # noqa: E402

# The camera's physical mount produces an image flipped relative to what
# the rest of the pipeline (and the mounting calibration in
# pose_controller.py) assumes -- flip it back right at capture so
# detection, pose estimation, and the returned preview/overlay frame are
# all consistent. flipCode=0 is a top/bottom (vertical) flip; if the real
# mismatch turns out to be left/right instead, use flipCode=1.
MIRROR_FRAME_VERTICAL = True

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


def _fallback_identify_candidate(gray, candidate_corners, aruco_dict):
    """Second-pass decode for a candidate quad cv2.aruco itself rejected.

    Found empirically debugging pool-side detection: cv2.aruco's own
    internal per-cell bit sampling can reject a candidate (no ids, but a
    same-sized quad shows up in the `rejected` list) even when the
    candidate's geometry is exactly right -- its sampling is more
    sensitive to the residual blur/noise real captures have than a
    plain coarse-average-per-cell read is. Warping the SAME corners
    cv2 already found and reading each cell as a trimmed mean (instead
    of cv2's stricter per-cell sampling) recovered a clean, consistent
    id=0 decode across every frame in that test where cv2's own decode
    kept failing. Returns (marker_id, corners_reordered_to_rotation) or
    None if this candidate doesn't decode to any dictionary entry.
    """
    n = aruco_dict.markerSize + 2  # + 2 for the black border ring
    src = candidate_corners.reshape(-1, 2).astype(np.float32)
    cell_px = 8
    warp_size = n * cell_px
    dst = np.array([[0, 0], [warp_size, 0], [warp_size, warp_size], [0, warp_size]],
                    dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(gray, matrix, (warp_size, warp_size))

    margin = max(1, cell_px // 4)
    grid = np.zeros((n, n))
    for r in range(n):
        for c in range(n):
            y0, y1 = r * cell_px + margin, (r + 1) * cell_px - margin
            x0, x1 = c * cell_px + margin, (c + 1) * cell_px - margin
            grid[r, c] = warped[y0:y1, x0:x1].mean()
    thresh = (grid.max() + grid.min()) / 2
    bits = (grid > thresh).astype(np.uint8)
    inner_bits = bits[1:n - 1, 1:n - 1]

    ok, marker_id, rotation = aruco_dict.identify(inner_bits, 1.0)
    if not ok:
        return None
    # cv2 rolls the returned corner order by the detected rotation so pose
    # estimation sees a consistent orientation -- replicate that here.
    reordered = np.roll(src, -rotation, axis=0).reshape(1, 4, 2)
    return int(marker_id), reordered


def marker_yaw_from_rvec(rvec):
    """Extract yaw (rotation about the camera's z-axis) from an ArUco
    rotation vector. Duplicated here (also defined in pose_controller.py)
    so camFinal.py has no hard dependency on that file -- keeps this
    module usable standalone."""
    rmat, _ = cv2.Rodrigues(rvec)
    return float(np.arctan2(rmat[1, 0], rmat[0, 0]))


# Dehaze/white-balance/CLAHE/gamma defaults below match the settings the
# camera/camtestv6.py 9-condition turbidity x lighting test plan validated
# (dehaze ON, software white balance OFF -- sensor AWB handles it, CLAHE
# clip 3.0). See camera/underwater_pipeline.py for the pipeline itself.
def enhance_low_light(frame_bgr, dehaze_enabled=True, wb_enabled=False,
                       clahe_clip=3.0, gamma=1.0):
    """Runs the shared underwater enhancement pipeline (dehaze -> white
    balance -> grayscale -> CLAHE -> gamma -> denoise) to help ArUco
    detection in dim/hazy/underwater conditions where the marker has low
    contrast against its surroundings. Returns a grayscale, enhanced
    frame suitable for detectMarkers() -- does not modify the original
    frame, which is still needed in color for the operator video
    feed/overlay. frame_bgr must be BGR-ordered (picamera2's "RGB888"
    format delivers BGR bytes despite the name -- see
    capture_and_detect() below).
    """
    return apply_pipeline(
        frame_bgr,
        dehaze_enabled=dehaze_enabled,
        wb_enabled=wb_enabled,
        clahe_enabled=True,
        clahe_clip=clahe_clip,
        gamma=gamma,
        denoise_enabled=True,
    )


class ArucoDetector:
    """Wraps the camera + ArUco detection loop. Call start() once, then
    get_pose() repeatedly (e.g. once per main-loop iteration or in a
    background thread), then stop() on shutdown."""

    def __init__(self, dict_name="DICT_6X6_50", width=640, height=480,
                 hfov_deg=100.0, vfov_deg=72.0, marker_size=0.10,
                 x_correction=0.95, y_correction=0.9, z_correction=1.8,
                 exposure_us=20000, gain=4.0, auto_exposure=True,
                 autofocus=True, af_mode="continuous",
                 tuning_file="imx708_noir.json",
                 calib_path=None, enhance_low_light=True,
                 dehaze=True, white_balance=False, gamma=1.0, clahe_clip=3.0,
                 target_id=0, id_filter=True):
        self.width = width
        self.height = height
        self.marker_size = marker_size
        # Per-axis empirical correction factors -- from
        # camera/camtestv5_100mm.py's live-tuned calibration for a 100mm
        # marker (default marker_size=0.10 above matches). These come from
        # the physical marker SIZE, not its dictionary/bit encoding, so they
        # stay valid across the DICT_4X4_50 -> DICT_6X6_50 switch (see
        # dict_name default below) as long as the print size stays 100mm.
        # camtestv6.py's 1.1/1.15/2.0 values are NOT applicable here -- that
        # script (and its 9-condition test plan) targets a 50mm marker
        # (marker_size=0.05), a different physical target with different
        # optics/distance error characteristics.
        self.x_correction = x_correction
        self.y_correction = y_correction
        self.z_correction = z_correction
        self.enhance_low_light_enabled = enhance_low_light
        self.dehaze_enabled = dehaze
        self.white_balance_enabled = white_balance
        self.gamma = gamma
        self.clahe_clip = clahe_clip

        # Single-marker ArUco pose estimation (estimatePoseSingleMarkers)
        # has a well-known orientation ambiguity: two near-equally-valid
        # PnP solutions differing by ~180 deg, which can flip between
        # frames purely from small viewing-angle changes -- not real
        # marker/vehicle rotation. A frame-to-frame yaw jump bigger than
        # any plausible real rotation at this frame rate is almost
        # certainly a flip, so it's rejected and the last good yaw is
        # reused instead of feeding a spurious ~180 deg error downstream
        # (confirmed live 2026-08-12: manual-mode translation with no PID
        # running still showed yaw snapping between ~+90 and ~-90 deg).
        self._last_yaw = None
        self._pending_yaw = None
        self._pending_count = 0
        self.max_yaw_jump_rad = math.radians(60.0)
        self.yaw_confirm_frames = 3  # consecutive matching frames to accept a big jump as real

        # Target-ID lock: matches the convention validated across the
        # camera/camtestv5*.py tuning scripts (single physical marker,
        # id0, 100mm, DICT_6X6_50 as of the dictionary switch -- see
        # marker_size/dict_name defaults above). When id_filter is on, a
        # stray second marker in frame is drawn for operator visibility
        # but never reported as the tracked pose -- avoids silently
        # locking onto the wrong tag.
        self.target_id = target_id
        self.id_filter = id_filter

        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[dict_name]) \
            if hasattr(cv2.aruco, 'Dictionary_get') \
            else cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])

        # OpenCV 4.7+ removed the old free-function detectMarkers() API
        # (cv2.aruco.Dictionary_get / DetectorParameters_create) in favor
        # of a class-based ArucoDetector. Support both so this doesn't
        # silently break again on whatever OpenCV version a given
        # machine happens to have installed.
        self._new_aruco_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self._new_aruco_api:
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Sub-pixel corner refinement: slightly slower per frame, but
        # corner positions are noticeably more stable/accurate, which
        # directly improves both detection consistency and pose accuracy.
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1

        # Widen the adaptive thresholding window range -- helps catch
        # markers under uneven lighting (shadows, glare, murky water),
        # a common cause of intermittent/flaky detection.
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 43
        self.aruco_params.adaptiveThreshWinSizeStep = 4

        # New API needs one persistent detector object built from the
        # dict+params; old API just calls the free function each frame
        # with the dict+params passed in directly (nothing to build here).
        if self._new_aruco_api:
            self._detector_obj = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        if calib_path:
            self.camera_matrix, self.dist_coeffs = load_calibration(calib_path)
        else:
            self.camera_matrix, self.dist_coeffs = approximate_camera_matrix(
                width, height, hfov_deg=hfov_deg, vfov_deg=vfov_deg
            )

        self.exposure_us = exposure_us
        self.gain = gain
        # exposure_us/gain above are tuned for dim underwater light -- on a
        # bright bench-test scene (e.g. indoor daylight, Lux in the tens of
        # thousands) they wildly overexpose and the feed reads back as solid
        # white. auto_exposure defaults to True (sensor AE runs instead of
        # the manual override below) so the app works out of the box on the
        # bench; pass auto_exposure=False / --no-auto-exposure for real
        # underwater runs so the tuned manual exposure/gain apply instead.
        self.auto_exposure = auto_exposure
        # This IMX708 module has no IR-cut filter (NoIR variant), confirmed
        # empirically: the stock "imx708.json" tuning drives ColourGains to
        # roughly (R=2.7, B=1.6) here, producing a strong magenta/pink cast
        # outdoors -- its AWB assumes an IR-cut filter is present and
        # under-corrects for the extra near-IR hitting the red channel
        # without one. "imx708_noir.json" gives balanced gains (~1.3/1.4)
        # and correct color. Pass tuning_file=None to fall back to
        # Picamera2's default tuning (e.g. if the module is ever swapped
        # for a standard/IR-cut unit).
        self.tuning_file = tuning_file
        # Nothing was ever driving focus -- AfMode defaults to Manual with
        # whatever fixed LensPosition the sensor happens to power up at, so
        # the lens never actually re-focuses on the target. autofocus=True
        # turns on AF (mode controlled by af_mode below) so it can focus on
        # the target in the first place.
        self.autofocus = autofocus
        # 'continuous': AfMode=Continuous -- keeps re-hunting focus every
        #   frame as the scene changes. Right for a real approach, where
        #   distance to the AUV is genuinely changing. On a bench/pool rig
        #   at roughly fixed distance, water ripples, glare, and marker
        #   motion during step tests are enough scene change to keep
        #   re-triggering full hunts, which shows up as focus loss/blur at
        #   exactly the moments detection needs to be reliable.
        # 'once': runs a single autofocus_cycle() in start() and leaves
        #   AfMode=Auto afterward -- Auto mode only refocuses when
        #   explicitly triggered, so once it converges the lens just stays
        #   put. Right for calibration testing at a fixed distance.
        if af_mode not in ("continuous", "once"):
            raise ValueError(f"af_mode must be 'continuous' or 'once', got {af_mode!r}")
        self.af_mode = af_mode
        self.picam2 = None

    def start(self):
        tuning = Picamera2.load_tuning_file(self.tuning_file) if self.tuning_file else None
        self.picam2 = Picamera2(tuning=tuning)
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)  # let auto-exposure/focus settle before overriding

        camera_controls = {
            # Disable the IPA software denoise stage (SDN) that runs on
            # the CPU on Pi 4 -- it adds per-frame latency. Both Pi 4 and
            # Pi 5 have hardware ISP denoise; this only disables the extra
            # software post-processing stage.
            "NoiseReductionMode": 0,
        }
        if not self.auto_exposure:
            camera_controls["ExposureTime"] = self.exposure_us
            camera_controls["AnalogueGain"] = self.gain
        if self.autofocus and self.af_mode == "continuous":
            camera_controls["AfMode"] = libcamera_controls.AfModeEnum.Continuous
            camera_controls["AfRange"] = libcamera_controls.AfRangeEnum.Full
        self.picam2.set_controls(camera_controls)

        if self.autofocus and self.af_mode == "once":
            self.refocus()

    def refocus(self) -> bool:
        """Run one autofocus scan right now and hold the result.

        Safe to call whenever the lens looks wrong (e.g. the startup
        scan in start() locked onto the wrong thing, or the rig's
        distance from the marker changed) -- not just at start(). In
        'continuous' af_mode this briefly drops to Auto for the scan and
        switches back to Continuous afterward, so an on-demand refocus
        doesn't change the configured steady-state behavior. Returns
        whether the scan converged; the lens is left wherever it ended
        up either way (better than nothing even on a failed convergence).
        """
        if self.picam2 is None or not self.autofocus:
            return False
        # Auto mode is required for a one-shot trigger -- in 'once' mode
        # we're already sitting in Auto (from start() or a prior
        # refocus()), and in 'continuous' mode this temporarily drops out
        # of Continuous for the duration of the scan.
        self.picam2.set_controls({
            "AfMode": libcamera_controls.AfModeEnum.Auto,
            "AfRange": libcamera_controls.AfRangeEnum.Full,
        })
        converged = self.picam2.autofocus_cycle()
        if not converged:
            print("[camFinal] one-shot autofocus did not converge -- "
                  "check marker distance/lighting; lens may be left "
                  "hunting or mis-focused", file=sys.stderr)
        if self.af_mode == "continuous":
            self.picam2.set_controls({
                "AfMode": libcamera_controls.AfModeEnum.Continuous,
                "AfRange": libcamera_controls.AfRangeEnum.Full,
            })
        return bool(converged)

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
        # picamera2's "RGB888" format string is a misnomer -- it actually
        # delivers BGR-ordered bytes, so `frame` is already what cv2
        # display/encode calls want. No RGB2BGR conversion needed (an
        # earlier version did one here, which inverted red/blue on the
        # live feed since the data was already BGR-ordered).
        frame = self.picam2.capture_array()
        if MIRROR_FRAME_VERTICAL:
            frame = cv2.flip(frame, 0)
        if self.enhance_low_light_enabled:
            detect_input = enhance_low_light(
                frame, dehaze_enabled=self.dehaze_enabled,
                wb_enabled=self.white_balance_enabled,
                clahe_clip=self.clahe_clip, gamma=self.gamma,
            )
        else:
            # Pass grayscale directly -- detectMarkers converts internally
            # anyway, but a single-channel image skips ~2ms of color
            # conversion per frame in the detection pipeline.
            detect_input = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._new_aruco_api:
            corners, ids, rejected = self._detector_obj.detectMarkers(detect_input)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                detect_input, self.aruco_dict, parameters=self.aruco_params
            )

        bgr = frame

        if ids is None:
            # cv2's own candidate geometry search still runs even when its
            # bit-decode rejects everything -- try the more tolerant
            # fallback decode on whatever it found before giving up.
            for candidate in rejected:
                found = _fallback_identify_candidate(detect_input, candidate, self.aruco_dict)
                if found is not None:
                    marker_id, fixed_corners = found
                    corners = [fixed_corners]
                    ids = np.array([[marker_id]])
                    break
            else:
                return None, bgr

        # draws every marker seen (including a non-target stray tag) so it's
        # visible to the operator even when it's not being tracked
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        ids_flat = ids.flatten()
        if self.id_filter:
            matches = np.flatnonzero(ids_flat == self.target_id)
            if matches.size == 0:
                return None, bgr
            target_idx = int(matches[0])
        else:
            target_idx = 0

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        marker_id = int(ids_flat[target_idx])
        x, y, z = tvecs[target_idx][0]
        x *= self.x_correction
        y *= self.y_correction
        z *= self.z_correction
        yaw = marker_yaw_from_rvec(rvecs[target_idx])

        # Reject implausible frame-to-frame yaw jumps (ArUco single-marker
        # pose flip, see self.max_yaw_jump_rad above) -- fall back to the
        # last accepted yaw rather than passing a spurious ~180deg error on.
        # A jump that *persists* for yaw_confirm_frames in a row is treated
        # as a real reorientation (not a one-frame flip) and accepted, so a
        # genuine fast turn doesn't get stuck reporting a stale heading.
        if self._last_yaw is None:
            self._last_yaw = yaw
        else:
            jump = abs(math.atan2(math.sin(yaw - self._last_yaw),
                                   math.cos(yaw - self._last_yaw)))
            if jump <= self.max_yaw_jump_rad:
                self._last_yaw = yaw
                self._pending_yaw = None
                self._pending_count = 0
            else:
                if self._pending_yaw is not None and abs(math.atan2(
                        math.sin(yaw - self._pending_yaw),
                        math.cos(yaw - self._pending_yaw))) <= self.max_yaw_jump_rad:
                    self._pending_count += 1
                else:
                    self._pending_yaw = yaw
                    self._pending_count = 1

                if self._pending_count >= self.yaw_confirm_frames:
                    self._last_yaw = yaw
                    self._pending_yaw = None
                    self._pending_count = 0
                else:
                    yaw = self._last_yaw

        cv2.drawFrameAxes(bgr, self.camera_matrix, self.dist_coeffs,
                           rvecs[target_idx], tvecs[target_idx], self.marker_size * 0.5)

        pose = {
            "id": marker_id,
            "x": float(x), "y": float(y), "z": float(z),
            "yaw": yaw,
            "frame": bgr,
        }
        return pose, bgr


# ---------------------------------------------------------------------
def _run_preview(args):
    """Original standalone behaviour: live window, prints on detection.
    Pass --no-preview to skip the window and run headless (prints only)."""
    detector = ArucoDetector(
        dict_name=args.dict, width=args.width, height=args.height,
        marker_size=args.marker_size, x_correction=args.x_correction,
        y_correction=args.y_correction, z_correction=args.z_correction,
        exposure_us=args.exposure_us, gain=args.gain,
        auto_exposure=args.auto_exposure, autofocus=args.autofocus,
        af_mode=args.af_mode,
        tuning_file=args.tuning_file,
        calib_path=args.calib,
        enhance_low_light=not args.no_enhance_low_light,
        dehaze=not args.no_dehaze, white_balance=args.white_balance,
        gamma=args.gamma, clahe_clip=args.clahe_clip,
        target_id=args.target_id, id_filter=not args.no_id_filter,
    )
    detector.start()
    print(f"Camera started ({args.width}x{args.height}), dictionary: {args.dict}")
    if args.no_preview:
        print("Running headless (--no-preview) -- Ctrl+C to quit.")
    else:
        print("Press 'q' in the preview window to quit.")

    window_name = "ArUco Detection - IMX708"
    if not args.no_preview:
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
            if not args.no_preview:
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        if not args.no_preview:
            cv2.destroyAllWindows()


def _run_calibration_check(args):
    """Live camera-frame -> body-frame comparison, for setting
    CAMERA_MOUNT_*_DEG in pose_controller.py against your real mount.
    By default shows the live video feed with the numbers overlaid, plus
    prints them to the console. Pass --no-preview to skip the cv2 window
    entirely and run over SSH with no monitor attached -- prints only.
    Press 'q' in the video window (or Ctrl+C in either mode) to quit."""
    from pose_controller import camera_to_body, camera_to_body_yaw

    detector = ArucoDetector(
        dict_name=args.dict, width=args.width, height=args.height,
        marker_size=args.marker_size, x_correction=args.x_correction,
        y_correction=args.y_correction, z_correction=args.z_correction,
        exposure_us=args.exposure_us, gain=args.gain,
        auto_exposure=args.auto_exposure, autofocus=args.autofocus,
        af_mode=args.af_mode,
        tuning_file=args.tuning_file,
        calib_path=args.calib,
        enhance_low_light=not args.no_enhance_low_light,
        dehaze=not args.no_dehaze, white_balance=args.white_balance,
        gamma=args.gamma, clahe_clip=args.clahe_clip,
        target_id=args.target_id, id_filter=not args.no_id_filter,
    )
    detector.start()
    print("Mounting calibration check -- move the marker to a known")
    print("position (e.g. 'to the platform's right') and confirm the")
    print("body-frame values match what you physically expect.")
    if args.no_preview:
        print("Running headless (--no-preview) -- Ctrl+C to quit.\n")
    else:
        print("Press 'q' in the video window (or Ctrl+C here) to quit.\n")

    window_name = "Mounting Calibration Check"
    if not args.no_preview:
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
                if not args.no_preview:
                    cv2.putText(frame, "no marker detected", (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                x_body, y_body, z_body = camera_to_body(pose["x"], pose["y"], pose["z"])
                yaw_body = camera_to_body_yaw(pose["yaw"])

                cam_line = (f"cam:  x={pose['x']:+.3f} y={pose['y']:+.3f} "
                            f"z={pose['z']:+.3f} yaw={math.degrees(pose['yaw']):+.1f}deg")
                body_line = (f"body: surge={x_body:+.3f} sway={y_body:+.3f} "
                             f"heave={z_body:+.3f} yaw={math.degrees(yaw_body):+.1f}deg")

                if not args.no_preview:
                    cv2.putText(frame, cam_line, (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, body_line, (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

                now = time.time()
                if now - last_print_time >= 0.2:
                    print(f"{cam_line}   |   {body_line}")
                    last_print_time = now

            if not args.no_preview:
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        if not args.no_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArUco detection + pose on IMX708 CSI camera")
    parser.add_argument("--dict", default="DICT_6X6_50", choices=ARUCO_DICTS.keys(),
                         help="ArUco dictionary (default: DICT_6X6_50 -- DICT_4X4_50's "
                              "maxCorrectionBits is only 1, i.e. at most 1 of its 16 data "
                              "bits can be wrong before decode fails outright; a 6x6 grid "
                              "has a much larger error budget, which real-world blur/"
                              "backlit-contrast/noise needs)")
    # Standalone preview keeps 1280x720 for display quality on a monitor.
    # The class default (640x480) is optimized for the headless integration
    # path where detection speed matters more than preview resolution.
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--calib", default=None,
                         help="Path to .npz calibration file (most accurate)")
    parser.add_argument("--marker-size", type=float, default=0.10,
                         help="Marker BLACK SQUARE side length in meters "
                              "(default: 0.10, matching the team's label spec)")
    parser.add_argument("--x-correction", type=float, default=0.95,
                         help="Empirical multiplier on x (default: 0.95, from "
                              "camera/camtestv5_100mm.py's live-tuned calibration "
                              "for the id0, 100mm marker -- size-dependent, not "
                              "dictionary-dependent, see dict_name default)")
    parser.add_argument("--y-correction", type=float, default=0.9,
                         help="Empirical multiplier on y (default: 0.9, same "
                              "100mm-marker calibration as --x-correction)")
    parser.add_argument("--z-correction", type=float, default=1.8,
                         help="Empirical multiplier on z (default: 1.8, same "
                              "100mm-marker calibration as --x-correction)")
    parser.add_argument("--exposure-us", type=int, default=20000,
                         help="Manual exposure time in microseconds "
                              "(default: 20000, a moderate underwater starting point)")
    parser.add_argument("--gain", type=float, default=4.0,
                         help="Manual analogue gain (default: 4.0, moderate "
                              "boost for underwater daytime light)")
    parser.add_argument("--no-auto-exposure", dest="auto_exposure", action="store_false",
                         help="Use the manual --exposure-us/--gain override instead of "
                              "the sensor's own auto-exposure (default: auto-exposure "
                              "is ON). Pass this for real underwater runs, where the "
                              "manual values are tuned for dim light -- auto-exposure "
                              "is the right default for in-air bench testing, where it "
                              "would otherwise badly overexpose to a solid white feed.")
    parser.set_defaults(auto_exposure=True)
    parser.add_argument("--no-autofocus", dest="autofocus", action="store_false",
                         help="Leave the lens at whatever fixed manual position it "
                              "powers up at instead of running AF (default: "
                              "AF is ON, mode set by --af-mode). "
                              "Nothing was ever driving focus before this flag existed, "
                              "so the lens previously just sat wherever it defaulted to.")
    parser.set_defaults(autofocus=True)
    parser.add_argument("--af-mode", choices=["continuous", "once"], default="continuous",
                         help="'continuous' (default) keeps re-hunting focus every frame "
                              "-- right for a real approach where distance keeps changing. "
                              "'once' runs a single autofocus scan at startup then holds "
                              "the lens still -- right for bench/pool calibration at a "
                              "roughly fixed distance, where continuous AF re-hunts on "
                              "ripples/glare/marker motion and shows up as focus loss "
                              "during exactly the moments detection needs to be sharp.")
    parser.add_argument("--tuning-file", default="imx708_noir.json",
                         help="libcamera tuning file name to load (default: "
                              "imx708_noir.json -- this module has no IR-cut "
                              "filter, and the stock imx708.json tuning's AWB "
                              "badly over-reds the image outdoors as a result). "
                              "Pass an empty string to use Picamera2's default "
                              "tuning instead.")
    parser.add_argument("--no-enhance-low-light", action="store_true",
                         help="Disable the whole enhancement pipeline (dehaze/"
                              "white-balance/CLAHE/gamma/denoise) -- enabled "
                              "by default for underwater use")
    parser.add_argument("--no-dehaze", action="store_true",
                         help="Disable underwater dark-channel-prior dehaze "
                              "(enabled by default, per the camtestv6.py "
                              "9-condition test plan's validated settings). "
                              "Has no effect if --no-enhance-low-light is set.")
    parser.add_argument("--white-balance", action="store_true",
                         help="Enable software gray-world white balance on top "
                              "of the sensor's own AWB (default: off, matching "
                              "camtestv6.py's default -- sensor AWB alone was "
                              "sufficient in testing)")
    parser.add_argument("--gamma", type=float, default=1.0,
                         help="Brightness gamma applied after CLAHE "
                              "(default: 1.0 = no-op)")
    parser.add_argument("--clahe-clip", type=float, default=3.0,
                         help="CLAHE clipLimit -- higher = more contrast boost, "
                              "but amplifies noise/turbidity graininess too "
                              "(default: 3.0)")
    parser.add_argument("--target-id", type=int, default=0,
                         help="Only this marker ID counts as the tracked pose "
                              "when --id-filter is on (default: 0, matching "
                              "the team's id0, 100mm, DICT_6X6_50 marker)")
    parser.add_argument("--no-id-filter", action="store_true",
                         help="Track whichever marker is seen first instead of "
                              "requiring --target-id -- a stray second marker "
                              "in frame could get tracked instead of the real "
                              "one. id-filter is ON by default.")
    parser.add_argument("--calibration-check", action="store_true",
                         help="Run the live camera-to-body mounting check "
                              "instead of the preview window")
    parser.add_argument("--no-preview", action="store_true",
                         help="Run headless -- no cv2 window, console output only. "
                              "Needed on a headless Pi with no monitor attached; works "
                              "with both the default preview mode and --calibration-check.")
    args = parser.parse_args()

    if args.calibration_check:
        _run_calibration_check(args)
    else:
        _run_preview(args)
