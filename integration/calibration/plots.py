"""Optional step-response plots.  Requires matplotlib.

All public functions return the output path on success, or ``None`` when
matplotlib is unavailable (or plotting fails).  They never raise.
"""

import csv
import math
import os
import sys
from typing import List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from .identify import StepResponseModel

# ---------------------------------------------------------------------------
# Lazy matplotlib import
# ---------------------------------------------------------------------------

def _check_matplotlib():
    """Return ``matplotlib.pyplot`` or ``None`` if unavailable."""
    try:
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        print(
            "[plots] matplotlib not available — install with: pip install matplotlib",
            file=sys.stderr,
        )
        return None


# ---------------------------------------------------------------------------
# Minimal CSV reader (mirrors identify.py logic but stays self-contained)
# ---------------------------------------------------------------------------

_AXIS_COLS = {
    "surge": ("motor_x", "surge_measured"),
    "sway":  ("motor_y", "sway_measured"),
    "yaw":   ("motor_r", "yaw_measured"),
}


def _read_step_csv(
    csv_path: str, axis: str = "surge"
) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Return ``(timestamps, motor_norm, positions, aruco_visible)``."""
    if axis not in _AXIS_COLS:
        raise ValueError(f"Unknown axis '{axis}'")
    motor_col, pos_col = _AXIS_COLS[axis]

    timestamps: List[float] = []
    motor_norm: List[float] = []
    positions:  List[float] = []
    visible:    List[float] = []

    with open(csv_path, "r", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            ts_s = row.get("ts", "")
            m_s  = row.get(motor_col, "")
            p_s  = row.get(pos_col, "")
            v_s  = row.get("aruco_visible", "0")
            try:
                ts = float(ts_s) if ts_s not in ("", "nan", "None") else float("nan")
            except (ValueError, TypeError):
                continue
            try:
                motor = float(m_s) if m_s not in ("", "nan", "None") else 0.0
            except (ValueError, TypeError):
                motor = 0.0
            try:
                pos = float(p_s) if p_s not in ("", "nan", "None") else float("nan")
            except (ValueError, TypeError):
                pos = float("nan")
            try:
                vis = int(float(v_s)) if v_s not in ("", "nan", "None") else 0
            except (ValueError, TypeError):
                vis = 0
            if math.isnan(ts):
                continue
            timestamps.append(ts)
            motor_norm.append(motor / 1000.0)
            positions.append(pos)
            visible.append(float(vis))

    return timestamps, motor_norm, positions, visible


def _smooth(data: List[float], window: int = 5) -> List[float]:
    n = len(data)
    if n < 3:
        return list(data)
    half = window // 2
    out = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out.append(sum(data[lo:hi]) / (hi - lo))
    return out


def _central_diff(
    pos: List[float], ts: List[float], w: int = 5
) -> Tuple[List[float], List[float]]:
    n = len(pos)
    if n < 2 * w + 1:
        return [], []
    vel = []
    t = []
    for i in range(w, n - w):
        dt = ts[i + w] - ts[i - w]
        if abs(dt) > 1e-12:
            vel.append((pos[i + w] - pos[i - w]) / dt)
        else:
            vel.append(0.0)
        t.append(ts[i])
    return t, vel


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def plot_step_response(
    csv_path: str,
    model: "StepResponseModel",
    output_path: Optional[str] = None,
) -> Optional[str]:
    """Generate a 3-subplot figure: motor command, position, velocity + fit.

    Returns *output_path* on success, ``None`` on any failure.
    """
    plt = _check_matplotlib()
    if plt is None:
        return None

    try:
        timestamps, motor_norm, positions, visible = _read_step_csv(
            csv_path, model.axis
        )
        if not timestamps:
            print("[plots] no data in CSV", file=sys.stderr)
            return None

        if output_path is None:
            output_path = csv_path.replace(".csv", "_plot.png")

        # Compute velocity (visible-only segment)
        sm_pos = _smooth(positions, window=5)
        vel_ts, vel_raw = _central_diff(sm_pos, timestamps, w=5)
        vel_raw = _smooth(vel_raw, window=3)

        # Model velocity prediction
        t0 = timestamps[0]
        vel_pred = []
        vel_pred_ts = []
        for ti in timestamps:
            t_rel = ti - model.step_onset_ts
            if t_rel >= model.L:
                vp = model.v_ss * (1.0 - math.exp(-(t_rel - model.L) / model.tau))
            else:
                vp = 0.0
            vel_pred.append(vp)
            vel_pred_ts.append(ti)

        # ---- Plot ----
        fig, (ax_m, ax_p, ax_v) = plt.subplots(3, 1, figsize=(10, 8),
                                                  sharex=True,
                                                  constrained_layout=True)

        # Motor command
        ax_m.step(timestamps, motor_norm, where="post", linewidth=0.8,
                  color="steelblue", label="motor command")
        ax_m.axhline(0, color="grey", linewidth=0.5, linestyle="--")
        ax_m.axvline(model.step_onset_ts, color="red", linewidth=0.6,
                     linestyle=":", label="step onset")
        ax_m.set_ylabel("Motor (norm)")
        ax_m.legend(loc="upper right", fontsize=8)
        ax_m.set_title(
            f"Step-response  axis={model.axis}   "
            f"K={model.K:.4f}  τ={model.tau:.3f}s  "
            f"L={model.L:.3f}s  R²={model.R_squared:.3f}",
            fontsize=10,
        )

        # Position
        ax_p.plot(timestamps, positions, linewidth=0.6, color="seagreen",
                  label="measured")
        ax_p.axvline(model.step_onset_ts, color="red", linewidth=0.6,
                     linestyle=":")
        ax_p.set_ylabel("Position (m / rad)")
        ax_p.legend(loc="upper left", fontsize=8)

        # Velocity + model overlay
        if vel_ts:
            ax_v.plot(vel_ts, vel_raw, linewidth=0.6, color="darkorange",
                      alpha=0.7, label="measured velocity")
        ax_v.plot(vel_pred_ts, vel_pred, linewidth=1.0, color="crimson",
                  linestyle="--", label="fitted model")
        ax_v.axhline(model.v_ss, color="grey", linewidth=0.5, linestyle="--")
        ax_v.axvline(model.step_onset_ts, color="red", linewidth=0.6,
                     linestyle=":")
        ax_v.set_ylabel("Velocity (m/s or rad/s)")
        ax_v.set_xlabel("Time (s)")
        ax_v.legend(loc="lower right", fontsize=8)

        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        return output_path

    except Exception as exc:  # pragma: no cover — belt-and-suspenders
        print(f"[plots] failed to generate plot: {exc}", file=sys.stderr)
        return None
