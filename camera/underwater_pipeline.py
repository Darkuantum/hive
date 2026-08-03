"""
underwater_pipeline.py -- Image enhancement pipeline for ArUco marker
detection under variable turbidity (target range: 0-150 NTU) and light
(measured via picamera2's own Lux metadata) in coastal water, 3-5m depth.

Pipeline order:
    dehaze -> white balance -> grayscale -> CLAHE -> gamma -> denoise

Each stage is a standalone function so a caller (camtest_v2.py, or later
a tuned deployment script) can toggle stages independently and log which
combination produced which detection result. Keeping these functions in
one shared module -- instead of copy-pasted into every camera script --
means a parameter fix only has to happen in one place.

WHY THIS ORDER:
  Dehazing needs the image's original (un-equalized) contrast/color
  relationships to correctly estimate atmospheric light and transmission.
  Running CLAHE first would distort those relationships and cause dehaze
  to amplify backscatter noise instead of removing haze.
  White balance runs on the DEHAZED image, so the color correction
  reflects the corrected scene rather than the hazy one.
  CLAHE and gamma both operate on the grayscale image that's actually
  handed to the ArUco detector, matching camtest_v2.py's 'v' (raw vs.
  processed) preview toggle -- what you see there is what the detector sees.

UNDERWATER-SPECIFIC ADAPTATION (why this isn't stock dark-channel-prior):
  Standard dark-channel-prior assumes the dark channel -> 0 almost
  everywhere except haze/sky. Underwater, that assumption breaks for the
  RED channel specifically: red wavelengths attenuate within the first
  few meters of water -- roughly your 3-5m test depth -- so red reads
  uniformly dark regardless of whether a given patch is hazy or not.
  Left in, this biases the atmospheric-light estimate. Following the
  "Underwater DCP" approach (Drews et al.), the dark channel here is
  computed from G,B only; the recovered image is still full RGB.
"""

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Stage 1: Dehazing (underwater-adapted dark-channel prior)
# ---------------------------------------------------------------------------

def _dark_channel_gb(frame_rgb, patch_size):
    """
    Minimum across G,B only (see module docstring for why R is excluded
    underwater), then minimum-filtered over a patch_size x patch_size
    window. cv2.erode with a square kernel IS a patch-wise minimum
    filter, and is hardware-accelerated -- much faster than a manual
    sliding-window loop in Python.
    """
    min_gb = np.minimum(frame_rgb[:, :, 1], frame_rgb[:, :, 2])
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (patch_size, patch_size))
    return cv2.erode(min_gb, kernel)


def _atmospheric_light(frame_rgb, dark_channel, top_fraction=0.001):
    """
    Estimate atmospheric light A (the color of the haze/backscatter
    itself) as the mean RGB of the brightest `top_fraction` of pixels in
    the dark channel -- these are the pixels most likely to be pure haze
    rather than scene detail.
    """
    h, w = dark_channel.shape
    n_pixels = max(int(h * w * top_fraction), 1)
    flat_dark = dark_channel.reshape(-1)
    flat_rgb = frame_rgb.reshape(-1, 3).astype(np.float32)
    brightest_idx = np.argpartition(flat_dark, -n_pixels)[-n_pixels:]
    return flat_rgb[brightest_idx].mean(axis=0)


def _guided_filter(guide_gray, src, radius=20, eps=1e-3):
    """
    Edge-preserving filter (He et al., 2010) used to refine the coarse
    transmission map so it respects real edges instead of being blocky
    at the downscaled estimation resolution. Implemented with
    cv2.boxFilter only -- no opencv-contrib / cv2.ximgproc dependency,
    since that module isn't guaranteed to be installed on the Pi.
    """
    guide = guide_gray.astype(np.float32) / 255.0
    src = src.astype(np.float32)

    mean_guide = cv2.boxFilter(guide, cv2.CV_32F, (radius, radius))
    mean_src = cv2.boxFilter(src, cv2.CV_32F, (radius, radius))
    mean_guide_src = cv2.boxFilter(guide * src, cv2.CV_32F, (radius, radius))
    cov_guide_src = mean_guide_src - mean_guide * mean_src

    mean_guide_sq = cv2.boxFilter(guide * guide, cv2.CV_32F, (radius, radius))
    var_guide = mean_guide_sq - mean_guide * mean_guide

    a = cov_guide_src / (var_guide + eps)
    b = mean_src - a * mean_guide

    mean_a = cv2.boxFilter(a, cv2.CV_32F, (radius, radius))
    mean_b = cv2.boxFilter(b, cv2.CV_32F, (radius, radius))

    return mean_a * guide + mean_b


def dehaze(frame_rgb, omega=0.85, t0=0.15, patch_size=15, downscale=4):
    """
    Underwater-adapted dark-channel-prior dehaze. Returns a dehazed RGB
    frame at the same size as the input.

    omega: haze-removal strength, 0-1. Underwater backscatter is usually
        less uniform than atmospheric haze, so this defaults lower
        (0.85) than the typical outdoor value (~0.95) to avoid
        over-darkening turbid frames.
    t0: transmission floor -- stops the recovery formula from dividing
        by a near-zero transmission (which blows out noise) in the
        murkiest patches of a frame.
    patch_size: dark-channel min-filter window. Larger = smoother, less
        local detail; 15 is the standard starting point.
    downscale: the dark-channel/transmission/atmospheric-light estimate
        runs on a downscaled copy (then upsamples back), since that's
        the expensive part and doesn't need full resolution to estimate
        correctly. Only the final recovery formula runs at full res.
        Raise this if dehaze is too slow on the Pi 4.
    """
    h, w = frame_rgb.shape[:2]
    small_w, small_h = max(w // downscale, 8), max(h // downscale, 8)
    small = cv2.resize(frame_rgb, (small_w, small_h), interpolation=cv2.INTER_AREA)

    dark = _dark_channel_gb(small, patch_size)
    A = _atmospheric_light(small, dark)
    A_safe = np.maximum(A, 1.0)

    norm = small.astype(np.float32) / A_safe
    t_coarse = 1.0 - omega * _dark_channel_gb(norm, patch_size)

    small_gray = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
    t_refined = _guided_filter(small_gray, t_coarse, radius=max(patch_size, 8))
    t_refined = np.clip(t_refined, t0, 1.0)

    t_full = cv2.resize(t_refined, (w, h), interpolation=cv2.INTER_LINEAR)
    t_full = np.clip(t_full, t0, 1.0)[:, :, np.newaxis]

    recovered = (frame_rgb.astype(np.float32) - A_safe) / t_full + A_safe
    return np.clip(recovered, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Stage 2: White balance (software gray-world, on top of locking sensor AWB)
# ---------------------------------------------------------------------------

def gray_world_white_balance(frame_rgb):
    """
    Gray-world color-constancy: assumes the average color of a normal
    scene is neutral gray, so any consistent per-channel bias in the
    frame average (blue-green cast, red loss with depth, sediment-brown
    cast -- whatever direction it happens to be) is corrected by scaling
    each channel so all three channel means match the overall mean.
    Cheap, and -- unlike a single fixed manual sensor gain -- adapts as
    the cast's direction changes with depth/turbidity through a dive.
    """
    frame_f = frame_rgb.astype(np.float32)
    mean_r = frame_f[:, :, 0].mean()
    mean_g = frame_f[:, :, 1].mean()
    mean_b = frame_f[:, :, 2].mean()
    mean_gray = (mean_r + mean_g + mean_b) / 3.0

    scale_r = mean_gray / max(mean_r, 1e-3)
    scale_g = mean_gray / max(mean_g, 1e-3)
    scale_b = mean_gray / max(mean_b, 1e-3)

    frame_f[:, :, 0] *= scale_r
    frame_f[:, :, 1] *= scale_g
    frame_f[:, :, 2] *= scale_b

    return np.clip(frame_f, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Stage 3 & 4: CLAHE + gamma (cheap, default-on per the recommended build order)
# ---------------------------------------------------------------------------

def clahe_enhance(gray, clip_limit=3.0, tile_grid_size=(8, 8)):
    """Local contrast enhancement -- recovers contrast lost to haze/low
    light without blowing out already-bright regions, unlike a global
    histogram equalization."""
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    return clahe.apply(gray)


_gamma_lut_cache = {}


def gamma_correct(gray, gamma=1.0):
    """
    Power-law brightness correction: output = 255*(input/255)^(1/gamma).
    gamma > 1 brightens; gamma == 1.0 is a no-op (skipped entirely).
    LUTs are cached per distinct gamma value since building a 256-entry
    table from scratch every frame is wasted work at a fixed setting.
    """
    if gamma == 1.0:
        return gray
    if gamma not in _gamma_lut_cache:
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8)
        _gamma_lut_cache[gamma] = table
    return cv2.LUT(gray, _gamma_lut_cache[gamma])


# ---------------------------------------------------------------------------
# Stage 5: Denoise (kept from the original camtest.py preprocessing)
# ---------------------------------------------------------------------------

def denoise(gray, d=5, sigma_color=50, sigma_space=50):
    """Edge-preserving smoothing -- removes sensor/turbidity speckle
    without blurring the sharp black/white edges ArUco detection needs."""
    return cv2.bilateralFilter(gray, d=d, sigmaColor=sigma_color, sigmaSpace=sigma_space)


# ---------------------------------------------------------------------------
# Full pipeline
# ---------------------------------------------------------------------------

def apply_pipeline(frame_rgb, dehaze_enabled=False, wb_enabled=False,
                    clahe_enabled=True, clahe_clip=3.0, gamma=1.0,
                    denoise_enabled=True, dehaze_omega=0.85, dehaze_t0=0.15,
                    dehaze_downscale=4):
    """
    Runs dehaze -> white balance -> grayscale -> CLAHE -> gamma -> denoise,
    with each stage independently toggleable. Returns a single-channel
    (grayscale) frame ready for cv2.aruco.detectMarkers().
    """
    stage = frame_rgb

    if dehaze_enabled:
        stage = dehaze(stage, omega=dehaze_omega, t0=dehaze_t0, downscale=dehaze_downscale)

    if wb_enabled:
        stage = gray_world_white_balance(stage)

    gray = cv2.cvtColor(stage, cv2.COLOR_RGB2GRAY)

    if clahe_enabled:
        gray = clahe_enhance(gray, clip_limit=clahe_clip)

    if gamma != 1.0:
        gray = gamma_correct(gray, gamma=gamma)

    if denoise_enabled:
        gray = denoise(gray)

    return gray


# ---------------------------------------------------------------------------
# Confidence signals
# ---------------------------------------------------------------------------

def contrast_score(gray):
    """
    Variance of the Laplacian -- a cheap, standard focus/contrast proxy.
    Low values indicate a flat, hazy, low-contrast frame. Logged
    alongside detection results so success rate can be checked against
    what the camera actually saw, not just the NTU/lux the frame was
    labeled with.
    """
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def reprojection_error(corners_xy, rvec, tvec, marker_size, camera_matrix, dist_coeffs):
    """
    RMS pixel distance between the marker corners actually detected and
    where those corners WOULD project given the estimated pose. Low
    error = the pose fit is internally consistent (high confidence);
    high error = something's off (motion blur, a bad corner, partial
    occlusion) even though detectMarkers() still returned a result --
    a more useful per-frame confidence signal than just "was ids not
    None", since a bad detection can still produce a pose.

    corners_xy: the (4, 2) corner array for ONE marker, as returned by
        detectMarkers() (i.e. corners[i][0]).
    rvec, tvec: this marker's pose, in the SAME units/frame used to
        estimate it. Pass the raw, un-z-corrected tvec -- z_correction
        is an empirical display/logging fudge, not a real geometric
        correction, so feeding a z-corrected tvec back through
        projectPoints would bias this number.
    """
    half = marker_size / 2.0
    obj_points = np.array([
        [-half, half, 0],
        [half, half, 0],
        [half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    detected = np.asarray(corners_xy, dtype=np.float32).reshape(-1, 2)

    return float(np.sqrt(np.mean(np.sum((projected - detected) ** 2, axis=1))))


# ---------------------------------------------------------------------------
# Temporal filtering
# ---------------------------------------------------------------------------

class PoseTracker:
    """
    Constant-velocity Kalman filter over marker position (x, y, z), for
    smoothing brief single-frame dropouts so a downstream AUV controller
    doesn't see the pose vanish/jump every time a flicker of turbidity or
    light causes one frame to miss detection.

    IMPORTANT: this does NOT change what counts as a logged trial in
    camtest_v2.py. The raw per-frame detected/not-detected result is
    still what gets logged as the trial outcome (that's the number your
    success-rate analysis needs, and temporal smoothing would quietly
    corrupt it if it fed back into "detected"). PoseTracker only produces
    an ADDITIONAL filtered estimate, logged in separate filtered_x/y/z
    columns, so you can compare "did the detector see it this frame"
    against "what would the AUV actually be steering toward" side by side.

    State: [x, y, z, vx, vy, vz]. Measurement: [x, y, z].
    NOTE: tracks a single marker, matching the single-marker assumption
    used throughout camtest.py / camtest_v2.py (ids.flatten()[0]).
    """

    def __init__(self, process_noise=1e-3, measurement_noise=1e-2, max_coast_frames=15):
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.hstack([
            np.eye(3, dtype=np.float32), np.zeros((3, 3), dtype=np.float32)
        ])
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * process_noise
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * measurement_noise

        self.max_coast_frames = max_coast_frames
        self.coast_count = 0
        self.initialized = False
        self._last_time = None

    def _set_dt(self, dt):
        F = np.eye(6, dtype=np.float32)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        self.kf.transitionMatrix = F

    def update(self, measurement_xyz, now):
        """
        Call once per frame. measurement_xyz: (x,y,z) tuple if detected
        this frame, else None. Returns (x, y, z, state) where state is
        "tracking" (fresh measurement this frame), "coasting" (predicting
        through a dropout, still within max_coast_frames), or "lost"
        (dropout too long -- treat position as unknown, don't trust a
        stale prediction). x/y/z are None when state is "lost".
        """
        dt = 0.0 if self._last_time is None else max(now - self._last_time, 1e-3)
        self._last_time = now
        self._set_dt(dt)

        if not self.initialized:
            if measurement_xyz is None:
                return None, None, None, "lost"
            x, y, z = measurement_xyz
            self.kf.statePost = np.array([x, y, z, 0, 0, 0], dtype=np.float32).reshape(6, 1)
            self.kf.errorCovPost = np.eye(6, dtype=np.float32)
            self.initialized = True
            self.coast_count = 0
            return x, y, z, "tracking"

        predicted = self.kf.predict()

        if measurement_xyz is not None:
            measurement = np.array(measurement_xyz, dtype=np.float32).reshape(3, 1)
            corrected = self.kf.correct(measurement)
            self.coast_count = 0
            return (float(corrected[0, 0]), float(corrected[1, 0]),
                    float(corrected[2, 0]), "tracking")

        self.coast_count += 1
        if self.coast_count > self.max_coast_frames:
            return None, None, None, "lost"
        return (float(predicted[0, 0]), float(predicted[1, 0]),
                float(predicted[2, 0]), "coasting")
