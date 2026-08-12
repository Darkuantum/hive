"""Closed-loop tracking performance metrics.

Reads a telemetry CSV from a closed-loop step run and computes classic
step-response metrics: overshoot, settling time, rise time, steady-state
error, and tracking RMSE.

Pure Python (csv + math, no numpy dependency).
"""

import csv
import math
from dataclasses import dataclass


@dataclass
class TrackingMetrics:
    """Computed performance metrics for one closed-loop step response."""
    axis: str
    setpoint: float

    overshoot_pct: float       # % overshoot beyond setpoint (0 if none)
    settling_time_s: float     # seconds to settle within ±5% band (-1 if never)
    rise_time_s: float         # seconds to first reach 90% of setpoint
    steady_state_error: float  # |mean(last 20% of hold) - setpoint|
    tracking_rmse: float       # RMSE of (position - setpoint) during hold

    def summary(self) -> str:
        parts = [
            f"axis={self.axis}",
            f"setpoint={self.setpoint:.4f}",
            f"overshoot={self.overshoot_pct:.1f}%",
            f"settling_time={self.settling_time_s:.2f}s",
            f"rise_time={self.rise_time_s:.2f}s",
            f"ss_error={self.steady_state_error:.5f}",
            f"rmse={self.tracking_rmse:.5f}",
        ]
        return "  ".join(parts)


def compute_metrics(csv_path: str, axis: str = 'surge',
                   setpoint: float = None) -> TrackingMetrics:
    """Read a closed-loop tracking CSV and compute performance metrics.

    Parameters
    ----------
    csv_path : path to telemetry.csv from a closed-loop step run
    axis : 'surge' | 'sway' | 'yaw'
    setpoint : target value; auto-detected from data if None
    """
    rows = _read_csv(csv_path)

    if not rows:
        raise ValueError(f"CSV is empty: {csv_path}")

    # Column names for measured position and setpoint
    measured_col = f"{axis}_measured"
    setpoint_col = f"{axis}_setpoint"

    # Build parallel arrays of (timestamp, measured, setpoint_value, phase)
    ts_list = []
    pos_list = []
    sp_list = []
    phase_list = []

    for row in rows:
        try:
            ts = float(row.get('ts', 0))
            pos = float(row.get(measured_col, ''))
            # Setpoint column may be empty string in some modes
            sp_raw = row.get(setpoint_col, '')
            sp_val = float(sp_raw) if sp_raw != '' else None
        except (ValueError, TypeError):
            continue  # skip malformed rows

        ts_list.append(ts)
        pos_list.append(pos)
        sp_list.append(sp_val)
        phase_list.append(row.get('phase', ''))

    if len(ts_list) < 2:
        raise ValueError(f"Not enough data rows in {csv_path}")

    t0 = ts_list[0]

    # --- Step onset / hold-end detection ---
    # Preferred: the "phase" column, logged directly by ClosedLoopRunner
    # ("pre"/"step"/"post"). Falls back to inferring boundaries from the
    # setpoint/position columns for older CSVs that predate the phase
    # column -- that fallback is unreliable when the setpoint column is
    # always 0.0 (true of every closed-loop run: the offset is injected
    # into the *measured* pose upstream of PoseController.compute(), which
    # always logs its own internal target as 0), so prefer phase whenever
    # it's present.
    onset_idx, hold_end_idx = _detect_phase_bounds(phase_list)
    if onset_idx is None:
        onset_idx = _detect_step_onset(ts_list, sp_list, pos_list, t0)
        if onset_idx is None:
            raise ValueError("Could not detect step onset in data")
        hold_end_idx = _detect_step_end(ts_list, sp_list, onset_idx)

    t_onset = ts_list[onset_idx]

    # --- Auto-detect setpoint if not provided ---
    if setpoint is None:
        setpoint = _auto_detect_setpoint(pos_list, onset_idx,
                                         hold_end_idx or len(pos_list))
    if setpoint == 0:
        raise ValueError("Detected setpoint is zero; cannot compute tracking metrics")

    abs_sp = abs(setpoint)

    # --- Isolate hold-phase data (from onset onwards, up to post-phase) ---
    if hold_end_idx is None:
        hold_end_idx = len(ts_list)

    hold_ts = ts_list[onset_idx:hold_end_idx]
    hold_pos = pos_list[onset_idx:hold_end_idx]

    if len(hold_ts) < 2:
        raise ValueError("Hold phase too short to compute metrics")

    # --- Rise time: time from onset to first reaching 90% of setpoint ---
    threshold_90 = 0.9 * setpoint if setpoint > 0 else -0.9 * abs_sp
    rise_time_s = -1.0
    for i, p in enumerate(hold_pos):
        if setpoint > 0 and p >= threshold_90:
            rise_time_s = hold_ts[i] - t_onset
            break
        elif setpoint < 0 and p <= threshold_90:
            rise_time_s = hold_ts[i] - t_onset
            break

    # --- Overshoot: max deviation beyond setpoint (after first crossing) ---
    overshoot_pct = 0.0
    first_cross_idx = None
    for i, p in enumerate(hold_pos):
        if setpoint > 0 and p >= setpoint:
            first_cross_idx = i
            break
        elif setpoint < 0 and p <= setpoint:
            first_cross_idx = i
            break

    if first_cross_idx is not None:
        remaining = hold_pos[first_cross_idx:]
        if setpoint > 0:
            max_excess = max((p - setpoint for p in remaining), default=0)
            overshoot_pct = max(0, (max_excess / abs_sp) * 100) if abs_sp > 0 else 0
        else:
            max_excess = max((setpoint - p for p in remaining), default=0)
            overshoot_pct = max(0, (max_excess / abs_sp) * 100) if abs_sp > 0 else 0

    # --- Settling time: last time position exits ±5% band around setpoint ---
    band = 0.05 * abs_sp
    settling_time_s = -1.0
    last_exit_idx = None
    for i, p in enumerate(hold_pos):
        if abs(p - setpoint) > band:
            last_exit_idx = i

    if last_exit_idx is not None:
        settling_time_s = hold_ts[last_exit_idx] - t_onset
    else:
        # Never left the band — settled instantly
        settling_time_s = 0.0

    # --- Steady-state error: |mean(last 20% of hold) - setpoint| ---
    n20 = max(1, len(hold_pos) // 5)
    tail = hold_pos[-n20:]
    ss_mean = sum(tail) / len(tail)
    steady_state_error = abs(ss_mean - setpoint)

    # --- Tracking RMSE during hold ---
    sq_errors = [(p - setpoint) ** 2 for p in hold_pos]
    tracking_rmse = math.sqrt(sum(sq_errors) / len(sq_errors))

    return TrackingMetrics(
        axis=axis,
        setpoint=setpoint,
        overshoot_pct=overshoot_pct,
        settling_time_s=settling_time_s,
        rise_time_s=rise_time_s,
        steady_state_error=steady_state_error,
        tracking_rmse=tracking_rmse,
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _read_csv(csv_path: str) -> list:
    """Read all rows from a CSV file into a list of dicts."""
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        return list(reader)


def _detect_step_onset(ts_list, sp_list, pos_list, t0):
    """Find the index where the step begins.

    Primary: setpoint column jumps from 0 to nonzero.
    Fallback: detect a large velocity change in position.
    """
    # Primary: use setpoint column
    sp_nonempty = [s for s in sp_list if s is not None]
    if len(sp_nonempty) >= 2:
        for i, s in enumerate(sp_list):
            if s is not None and s != 0.0:
                return i

    # Fallback: detect velocity change in position
    if len(pos_list) >= 4:
        for i in range(2, len(pos_list)):
            vel = (pos_list[i] - pos_list[i - 2]) / max(
                ts_list[i] - ts_list[i - 2], 1e-6
            )
            if abs(vel) > 0.01:  # threshold: 0.01 m/s or rad/s
                return i - 2

    return None


def _detect_step_end(ts_list, sp_list, onset_idx):
    """Find the index where the hold phase ends (setpoint drops back).

    Returns None if the setpoint never drops back (hold continues to end).
    """
    if onset_idx is None:
        return None

    # Find the last index where setpoint is nonzero
    last_nonzero = None
    for i in range(onset_idx, len(sp_list)):
        if sp_list[i] is not None and sp_list[i] != 0.0:
            last_nonzero = i

    if last_nonzero is not None and last_nonzero + 1 < len(ts_list):
        return last_nonzero + 1

    return None


def _detect_phase_bounds(phase_list):
    """Find (onset_idx, hold_end_idx) directly from a logged "phase" column.

    onset_idx is the first "step"-phase row; hold_end_idx is the first row
    after it that's no longer "step" (typically the first "post" row).
    Returns (None, None) if the column is absent/empty (older CSVs, or
    manual-mode runs), so callers can fall back to heuristic detection.
    """
    onset_idx = None
    hold_end_idx = None
    for i, p in enumerate(phase_list):
        if p == 'step':
            if onset_idx is None:
                onset_idx = i
        elif onset_idx is not None:
            hold_end_idx = i
            break
    return onset_idx, hold_end_idx


def _auto_detect_setpoint(pos_list, onset_idx, hold_end_idx=None):
    """Auto-detect setpoint from the mean position during hold.

    hold_end_idx bounds the averaging window to the real hold phase when
    known (from the phase column); without it, falls back to the middle
    50% of onset-to-end-of-data, which risks blending in post-phase data.
    """
    n = hold_end_idx if hold_end_idx is not None else len(pos_list)
    # Use the middle 50% of the hold window for robustness
    start = onset_idx + max(1, (n - onset_idx) // 4)
    end = onset_idx + max(1, 3 * (n - onset_idx) // 4)
    segment = pos_list[start:end]
    if not segment:
        segment = pos_list[onset_idx:min(onset_idx + 10, n)]
    return sum(segment) / len(segment) if segment else 0.0
