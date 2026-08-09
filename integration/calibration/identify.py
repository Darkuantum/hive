"""Offline step-response identification: CSV → first-order velocity model.

Reads a telemetry CSV produced by TelemetryLogger, detects the motor-command
step onset, computes velocity from position via smoothed finite differences,
and fits K / tau / L via linearized regression.  Pure Python — no numpy/scipy.
"""

import csv
import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class StepResponseModel:
    """First-order model fit from a step-response CSV."""

    axis: str                 # 'surge' | 'sway' | 'yaw'
    K: float                  # steady-state gain  (velocity per motor-command unit)
    tau: float                # time constant       (seconds)
    L: float                  # dead time           (seconds)
    v_ss: float               # steady-state velocity (m/s or rad/s)
    F_step: float             # motor command amplitude during step (normalized –1..1)
    R_squared: float          # fit quality  (0..1)
    n_samples: int            # velocity samples used in the fit
    step_onset_ts: float      # timestamp of detected step onset
    fit_method: str           # 'linearized_regression'

    def summary(self) -> str:
        """Human-readable one-line summary."""
        return (
            f"axis={self.axis}  K={self.K:.4f}  tau={self.tau:.3f}s  "
            f"L={self.L:.3f}s  v_ss={self.v_ss:.4f}  F={self.F_step:.3f}  "
            f"R\u00b2={self.R_squared:.4f}  n={self.n_samples}"
        )


# ---------------------------------------------------------------------------
# Column mapping (CSV column names defined in logging.TELEMETRY_COLUMNS)
# ---------------------------------------------------------------------------
_AXIS_CONFIG = {
    "surge": {"motor_col": "motor_x", "pos_col": "surge_measured"},
    "sway":  {"motor_col": "motor_y", "pos_col": "sway_measured"},
    "yaw":   {"motor_col": "motor_r", "pos_col": "yaw_measured"},
}


# ---------------------------------------------------------------------------
# Helper: parsing
# ---------------------------------------------------------------------------

def _read_csv_rows(csv_path: str) -> list:
    """Read a CSV file and return a list of row-dicts.

    Raises ``ValueError`` on any I/O or parse failure.
    """
    try:
        with open(csv_path, "r", newline="") as fh:
            reader = csv.DictReader(fh)
            return list(reader)
    except FileNotFoundError:
        raise ValueError(f"CSV file not found: '{csv_path}'")
    except (OSError, csv.Error) as exc:
        raise ValueError(f"Failed to read CSV '{csv_path}': {exc}")


def _safe_float(value: str, *, nan_ok: bool = True) -> float:
    """Convert a CSV string to float.  Returns ``float('nan')`` for blanks / nan."""
    if value in ("", "nan", "None", "NA"):
        return float("nan")
    try:
        return float(value)
    except (ValueError, TypeError):
        return float("nan")


def _safe_int(value: str, *, default: int = 0) -> int:
    try:
        return int(float(value))
    except (ValueError, TypeError):
        return default


# ---------------------------------------------------------------------------
# Step detection
# ---------------------------------------------------------------------------

def _detect_step_onset(
    motor_values: List[float],
    threshold: float = 0.05,
    sustain: int = 5,
) -> int:
    """Find the index where the motor command first sustains above *threshold*.

    *motor_values* must be in the normalised domain (–1 … +1).
    'Sustained' means the absolute value stays ≥ *threshold* for *sustain*
    consecutive samples.

    Raises ``ValueError`` when no step is found.
    """
    n = len(motor_values)
    if n < sustain + 5:
        raise ValueError(
            f"Too few samples ({n}) to detect a step onset (need ≥ {sustain + 5})"
        )
    for i in range(n - sustain + 1):
        if abs(motor_values[i]) >= threshold:
            ok = True
            for j in range(i, i + sustain):
                if abs(motor_values[j]) < threshold:
                    ok = False
                    break
            if ok:
                return i
    raise ValueError(
        "No step onset detected — motor command never reaches a sustained "
        f"nonzero value (threshold={threshold})"
    )


def _detect_step_end(
    motor_values: List[float],
    onset_idx: int,
    threshold: float = 0.05,
    sustain: int = 5,
) -> int:
    """Return the index of the first sustained-zero after *onset_idx*,
    or ``len(motor_values)`` when the step never ends."""
    n = len(motor_values)
    # Require at least a few samples of step before we look for the end
    start = onset_idx + 10
    if start >= n:
        return n
    for i in range(start, n - sustain + 1):
        if abs(motor_values[i]) < threshold:
            ok = True
            for j in range(i, i + sustain):
                if j < n and abs(motor_values[j]) >= threshold:
                    ok = False
                    break
            if ok:
                return i
    return n


# ---------------------------------------------------------------------------
# Signal processing helpers  (pure Python)
# ---------------------------------------------------------------------------

def _smooth(data: List[float], window: int = 5) -> List[float]:
    """Simple centred moving-average smoother.

    At the edges the window shrinks symmetrically so every input sample
    produces exactly one output sample.
    """
    n = len(data)
    if n < 3 or window < 2:
        return data[:]
    half = window // 2
    out: List[float] = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out.append(sum(data[lo:hi]) / (hi - lo))
    return out


def _central_difference(
    positions: List[float],
    timestamps: List[float],
    window: int = 3,
) -> Tuple[List[float], List[float]]:
    """Velocity via centred finite differences.

    ``v[i] = (x[i+w] – x[i–w]) / (t[i+w] – t[i–w])``

    Returns ``(velocities, timestamps)`` — shorter than input by *window*
    samples at each edge.
    """
    n = len(positions)
    if n < 2 * window + 1:
        return [], []
    vel: List[float] = []
    ts: List[float] = []
    for i in range(window, n - window):
        dt = timestamps[i + window] - timestamps[i - window]
        if abs(dt) > 1e-12:
            vel.append((positions[i + window] - positions[i - window]) / dt)
        else:
            vel.append(0.0)
        ts.append(timestamps[i])
    return ts, vel


def _linear_regression(
    xs: List[float], ys: List[float]
) -> Tuple[float, float, float]:
    """Ordinary-least-squares fit.  Returns ``(slope, intercept, R²)``."""
    n = len(xs)
    if n < 3:
        return 0.0, 0.0, 0.0
    sx  = sum(xs)
    sy  = sum(ys)
    sxy = sum(x * y for x, y in zip(xs, ys))
    sx2 = sum(x * x for x in xs)
    sy2 = sum(y * y for y in ys)
    denom = n * sx2 - sx * sx
    if abs(denom) < 1e-12:
        return 0.0, sy / n, 0.0
    slope     = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n
    y_mean = sy / n
    ss_tot = sum((y - y_mean) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return slope, intercept, r2


# ---------------------------------------------------------------------------
# Internal helpers (not public API)
# ---------------------------------------------------------------------------

def _extract_longest_visible_segment(
    timestamps: List[float],
    positions: List[float],
    visible: List[int],
    lo: int,
    hi: int,
) -> Tuple[List[float], List[float]]:
    """Return the longest contiguous segment where *visible* is true and
    *positions* is not NaN, within ``[lo, hi)``."""
    best_ts:  List[float] = []
    best_pos: List[float] = []
    i = lo
    n = len(timestamps)
    while i < min(hi, n):
        if visible[i] and not math.isnan(positions[i]):
            seg_ts:  List[float] = []
            seg_pos: List[float] = []
            while i < min(hi, n) and visible[i] and not math.isnan(positions[i]):
                seg_ts.append(timestamps[i])
                seg_pos.append(positions[i])
                i += 1
            if len(seg_ts) > len(best_ts):
                best_ts, best_pos = seg_ts, seg_pos
        else:
            i += 1
    return best_ts, best_pos


def _linearize_velocity(
    vel: List[float],
    vel_ts: List[float],
    v_ss: float,
    min_ratio: float = 0.05,
    max_ratio: float = 0.85,
) -> Tuple[List[float], List[float]]:
    r"""Compute ``y = ln(1 – v/v_ss)`` for samples in the rising portion.

    The ratio bounds (0.05–0.85) avoid noise amplification near zero and
    near steady state where the log transform diverges.

    Returns ``(time_offsets, y_values)`` ready for linear regression.
    """
    if abs(v_ss) < 1e-9:
        return [], []
    t0 = vel_ts[0]
    lin_ts: List[float] = []
    lin_y:  List[float] = []
    for j in range(len(vel)):
        ratio = vel[j] / v_ss
        if min_ratio < ratio < max_ratio:
            y = math.log(1.0 - ratio)
            if math.isfinite(y):
                lin_ts.append(vel_ts[j] - t0)
                lin_y.append(y)
    return lin_ts, lin_y


def _estimate_dead_time(
    vel: List[float],
    vel_ts: List[float],
    v_ss: float,
    threshold_frac: float = 0.05,
) -> float:
    """Dead time = delay from step onset until velocity first exceeds a
    fraction *threshold_frac* of |v_ss|."""
    t0 = vel_ts[0]
    thr = threshold_frac * abs(v_ss)
    for j in range(len(vel)):
        if abs(vel[j]) > thr:
            return max(0.0, vel_ts[j] - t0)
    return 0.0


def _compute_velocity_r2(
    vel: List[float],
    vel_ts: List[float],
    v_ss: float,
    tau: float,
    L: float,
) -> float:
    """R² between measured velocity and the first-order model prediction."""
    if len(vel) < 3:
        return 0.0
    t0 = vel_ts[0]
    v_mean = sum(vel) / len(vel)
    ss_tot = 0.0
    ss_res = 0.0
    for j in range(len(vel)):
        t_rel = vel_ts[j] - t0
        if t_rel >= L:
            vp = v_ss * (1.0 - math.exp(-(t_rel - L) / tau))
        else:
            vp = 0.0
        ss_tot += (vel[j] - v_mean) ** 2
        ss_res += (vel[j] - vp) ** 2
    if ss_tot < 1e-12:
        return 0.0
    return max(0.0, min(1.0, 1.0 - ss_res / ss_tot))


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def identify_from_csv(csv_path: str, axis: str = "surge") -> StepResponseModel:
    """Read a step-response CSV and fit a first-order velocity model.

    Algorithm
    ---------
    1. Parse the CSV, extracting *ts*, the motor column, the position
       column, and ``aruco_visible``.
    2. Normalise motor commands from raw (–1000 … +1000) to (–1 … +1).
    3. Detect step onset (first sustained non-zero motor command).
    4. Detect step end (motor returns to near-zero).
    5. Within the step window, extract the longest contiguous
       ``aruco_visible`` segment.
    6. Smooth positions (window=5), compute velocity via centred finite
       differences (window=5), smooth velocity (window=3).
    7. Estimate steady-state velocity from the last 20 % of the step phase.
    8. Estimate τ via the **area method** using position data:
       ``area = v_ss · T − (x(T) − x(0))``, then iteratively solve
       ``area = v_ss · τ · (1 − exp(−T/τ))``.  This avoids the noise
       amplification that the log-linearised velocity approach suffers from.
    9. ``K = v_ss / F_step`` where *F_step* is the signed normalised motor
       amplitude.
    10. Dead time *L* from the first detectable velocity rise.
    11. R² computed on the full velocity trace against the fitted model.

    Raises ``ValueError`` when data is insufficient or no step is found.
    """
    if axis not in _AXIS_CONFIG:
        raise ValueError(
            f"Unknown axis '{axis}'. Must be one of: {list(_AXIS_CONFIG.keys())}"
        )

    cfg = _AXIS_CONFIG[axis]
    motor_col = cfg["motor_col"]
    pos_col   = cfg["pos_col"]

    # ---- 1. Parse CSV ------------------------------------------------
    rows = _read_csv_rows(csv_path)
    if not rows:
        raise ValueError(f"CSV '{csv_path}' is empty or has no data rows")

    timestamps: List[float] = []
    motor_raw:  List[float] = []
    positions:  List[float] = []
    visible:    List[int]   = []

    for row in rows:
        ts_val = _safe_float(row.get("ts", ""))
        m_val  = _safe_float(row.get(motor_col, ""))
        p_val  = _safe_float(row.get(pos_col, ""))
        v_val  = _safe_int(row.get("aruco_visible", "0"))
        if math.isnan(ts_val):
            continue  # skip rows without a valid timestamp
        timestamps.append(ts_val)
        motor_raw.append(0.0 if math.isnan(m_val) else m_val)
        positions.append(p_val)
        visible.append(v_val)

    if len(timestamps) < 20:
        raise ValueError(
            f"Only {len(timestamps)} valid rows in CSV — need at least 20"
        )

    # ---- 2. Normalise motor -----------------------------------------
    motor_norm = [m / 1000.0 for m in motor_raw]

    # ---- 3. Detect step onset ---------------------------------------
    onset_idx = _detect_step_onset(motor_norm, threshold=0.05)
    onset_ts   = timestamps[onset_idx]

    # Signed amplitude: average of first ~1 s after onset (up to 20 samples)
    amp_end = min(onset_idx + 20, len(motor_norm))
    F_step = sum(motor_norm[onset_idx:amp_end]) / (amp_end - onset_idx)
    if abs(F_step) < 0.01:
        raise ValueError(
            f"Step amplitude too small (F_step={F_step:.4f}) — "
            "cannot identify system"
        )

    # ---- 4. Detect step end -----------------------------------------
    step_end_idx = _detect_step_end(motor_norm, onset_idx, threshold=0.05)
    if step_end_idx - onset_idx < 15:
        raise ValueError(
            f"Step phase too short ({step_end_idx - onset_idx} samples) — "
            "need at least 15"
        )

    # ---- 5. Longest continuous visible segment -----------------------
    step_ts, step_pos = _extract_longest_visible_segment(
        timestamps, positions, visible, onset_idx, step_end_idx
    )
    if len(step_ts) < 15:
        raise ValueError(
            f"Only {len(step_ts)} continuous visible samples during step "
            "phase — need at least 15"
        )

    # ---- 6. Velocity ------------------------------------------------
    smoothed_pos = _smooth(step_pos, window=5)
    vel_ts, vel  = _central_difference(smoothed_pos, step_ts, window=5)
    if not vel:
        raise ValueError("Velocity computation yielded no samples")
    vel = _smooth(vel, window=3)

    if len(vel) < 10:
        raise ValueError(
            f"Only {len(vel)} velocity samples after differentiation — "
            "need at least 10"
        )

    # ---- 7. Steady-state velocity -----------------------------------
    n_ss = max(int(len(vel) * 0.2), 5)
    v_ss = sum(vel[-n_ss:]) / n_ss
    if abs(v_ss) < 1e-6:
        raise ValueError(
            "Steady-state velocity is near zero — cannot identify system gain"
        )

    # ---- 8. τ via area method (position-based, noise-robust) ------
    # The "area" between the v_ss asymptote and the velocity curve equals
    # v_ss · τ · (1 − exp(−T/τ)).  Since position is the integral of velocity,
    # this area is v_ss·T − (x(T) − x(0)).  Using position avoids the noise
    # amplification that plagues the log-linearised velocity approach.
    T = step_ts[-1] - step_ts[0]
    x0 = smoothed_pos[0]
    xT = smoothed_pos[-1]
    displacement = xT - x0
    area = v_ss * T - displacement

    # area should have the same sign as v_ss (positive for positive steps,
    # negative for negative steps).  Use absolute values for the iteration.
    abs_area = abs(area)
    abs_vss = abs(v_ss)
    sign_ok = (v_ss > 0 and area > 0) or (v_ss < 0 and area < 0)
    if abs_area < 1e-9 or not sign_ok:
        raise ValueError(
            f"Area method yielded suspect area ({area:.4f}) for v_ss={v_ss:.4f} — "
            "check data quality (step may be too short or position reversed)"
        )

    # Iteratively solve: |area| = |v_ss| · τ · (1 − exp(−T/τ))
    tau = abs_area / abs_vss  # first guess (exact when T >> τ)
    for _ in range(10):
        if tau > 1e-9:
            corr = 1.0 - math.exp(-T / tau)
            if corr > 1e-6:
                tau_new = abs_area / (abs_vss * corr)
                if abs(tau_new - tau) < 1e-6:
                    tau = tau_new
                    break
                tau = tau_new
            else:
                break
        else:
            break

    if tau <= 0 or tau > 120:
        raise ValueError(
            f"Unphysical time constant τ={tau:.3f}s — expected 0 < τ < 120 s"
        )

    # ---- 9. K -------------------------------------------------------
    K = v_ss / F_step if abs(F_step) > 1e-9 else 0.0

    # ---- 10. Dead time L --------------------------------------------
    L = _estimate_dead_time(vel, vel_ts, v_ss)

    # ---- 11. R² on velocity prediction -------------------------------
    R_squared = _compute_velocity_r2(vel, vel_ts, v_ss, tau, L)

    return StepResponseModel(
        axis=axis,
        K=K,
        tau=tau,
        L=L,
        v_ss=v_ss,
        F_step=F_step,
        R_squared=R_squared,
        n_samples=len(vel),
        step_onset_ts=onset_ts,
        fit_method="area_method",
    )
