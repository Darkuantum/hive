#!/usr/bin/env python3
"""
calibration_figures.py — Professional ROV calibration figures (Phase 3).

Produces slide-ready PNGs:
  - integration/logs/fig2_closedloop_validation.png   (closed-loop, Fig 2)
  - integration/logs/fig3_openloop_plant.png          (test-vs-real plant, Fig 3)

Conventions from docs/calibration-methodology.md §5/§6 and
docs/calibration-presentation-research.md §A2/§F2/§F4:
  - Rise time: 10-90% of setpoint (computed on smoothed signal)
  - Settling time: +/-5% band
  - Overshoot: (peak - final)/final, first peak
  - SSE: |mean(last 20% of step) - setpoint|
  - RMSE: step-phase RMS of (measured - setpoint)
  - 2-3 significant figures everywhere
  - Every metric tagged (sim) or (measured)

Usage:
    uv run --with matplotlib python3 scripts/calibration_figures.py
"""

import csv
import math
import os

# ---------------------------------------------------------------------------
# Lazy matplotlib import
# ---------------------------------------------------------------------------
_matplotlib_ok = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    _matplotlib_ok = True
except ImportError:
    plt = None


LOGS_DIR = os.path.join(os.path.dirname(__file__), "..", "integration", "logs")
LOGS_DIR = os.path.normpath(LOGS_DIR)

AXES = ["surge", "sway", "yaw"]

# Identified plant params (test frame) vs extrapolated (real frame)
# axis -> frame -> (K, tau)   [K in (m/s)/unit or (rad/s)/unit; tau in s]
PLANT_PARAMS = {
    "surge": {"test frame": (0.054, 1.84), "real frame": (0.054, 2.20)},
    "sway":  {"test frame": (0.079, 2.16), "real frame": (0.079, 2.58)},
    "yaw":   {"test frame": (0.100, 2.00), "real frame": (0.019, 1.11)},
}

# Per-axis output limits (match PoseController)
OUTPUT_LIMITS = {"surge": 0.4, "sway": 0.4, "yaw": 0.6}

# Motor column name per axis
MOTOR_COL = {"surge": "motor_x", "sway": "motor_y", "yaw": "motor_r"}

# Axis labels
POS_LABEL = {"surge": "Position (m)", "sway": "Position (m)", "yaw": "Position (rad)"}
VEL_LABEL = {"surge": "Velocity (m/s)", "sway": "Velocity (m/s)",
             "yaw": "Yaw rate (rad/s)"}

# Color palette — consistent across all figures
C_INITIAL   = "#4477AA"   # muted blue
C_TUNED     = "#EE7733"   # warm orange
C_SETPOINT  = "#999999"   # grey
C_BAND      = "#AAAAAA"   # grey
C_TEST      = "#7744BB"   # purple
C_REAL      = "#228833"   # green
C_ARROW     = "#444444"   # dark grey
C_SAT_LINE  = "#BB3344"   # muted red for saturation

# Smoothing window for moving-median (noise is a sim artifact;
# the real vehicle's ArUco noise profile differs).  Window of 5 at
# dt=0.1 s = 0.5 s, enough to suppress single-sample spikes while
# preserving genuine step dynamics.
SMOOTH_WIN = 5


# =========================================================================
# Data loading & alignment
# =========================================================================

def load_csv(csv_path):
    rows = []
    with open(csv_path, "r") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def align(rows, axis):
    """Extract aligned time series with t=0 at step onset.

    Returns (t_rel, measured, setpoint, motor_int, phase_list).
    """
    t_rel = []
    measured = []
    setpoint = []
    motor_int = []
    phase_list = []

    # Find the timestamp of the first "step" row
    t0 = None
    for r in rows:
        if r["phase"] == "step":
            t0 = float(r["ts"])
            break
    if t0 is None:
        return t_rel, measured, setpoint, motor_int, phase_list

    col_meas = f"{axis}_measured"
    col_sp   = f"{axis}_setpoint"
    col_mot  = MOTOR_COL[axis]

    for r in rows:
        ts = float(r["ts"]) - t0
        try:
            m = float(r[col_meas])
        except (ValueError, KeyError):
            m = 0.0
        try:
            s = float(r[col_sp])
        except (ValueError, KeyError):
            s = 0.0
        try:
            mot = int(float(r[col_mot]))
        except (ValueError, KeyError):
            mot = 0
        t_rel.append(ts)
        measured.append(m)
        setpoint.append(s)
        motor_int.append(mot)
        phase_list.append(r["phase"])

    return t_rel, measured, setpoint, motor_int, phase_list


def moving_median(values, window):
    """Return a list of the same length with a centered moving median.

    Edge samples (closer than half-window to an edge) use a truncated
    window.  For very short lists (len < window) the original values are
    returned unchanged.
    """
    n = len(values)
    if n < window or window < 2:
        return list(values)
    half = window // 2
    out = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out.append(sorted(values[lo:hi])[half])
    return out


# =========================================================================
# Metrics computation  (methodology section 5 definitions)
# =========================================================================

def sigfig(value, n=3):
    """Round value to n significant figures, return a clean string."""
    if value is None or value == float("inf"):
        return "never"
    if value == 0:
        return "0"
    return f"{value:.{n}g}"


def format_settle(settle_s, capture_dur):
    """Format settling time; if it equals the capture duration, note that."""
    if settle_s >= capture_dur - 0.05:
        cap = sigfig(capture_dur)
        return f">{cap} s (no +/-5% settle)"
    return f"{sigfig(settle_s)} s"


def compute_metrics(t_rel, measured, setpoint_list, phase_list):
    """Compute tracking metrics on the step-phase rows only.

    Returns dict with: rise_s, settle_s, overshoot_pct, sse, rmse,
    t_10, t_90, settle_capped.
    """
    # Filter to step-phase rows
    step_idx = [i for i in range(len(phase_list)) if phase_list[i] == "step"]
    if len(step_idx) < 5:
        return {}

    step_t = [t_rel[i] for i in step_idx]
    step_m = [measured[i] for i in step_idx]
    step_sp = [setpoint_list[i] for i in step_idx]

    sp_val = step_sp[0]
    abs_sp = abs(sp_val)
    n = len(step_m)
    if abs_sp < 1e-9:
        return {}

    t0 = step_t[0]  # should be ~0 since we aligned at step onset
    t_end = step_t[-1]  # end of capture
    capture_dur = t_end - t0

    # --- Rise time: time from 10% to 90% crossing ---
    rise_s = float("inf")
    t_10 = None
    t_90 = None
    lv10 = 0.10 * sp_val
    lv90 = 0.90 * sp_val
    if sp_val > 0:
        for i, m in enumerate(step_m):
            if t_10 is None and m >= lv10:
                t_10 = step_t[i] - t0
            if t_90 is None and m >= lv90:
                t_90 = step_t[i] - t0
                break
    elif sp_val < 0:
        for i, m in enumerate(step_m):
            if t_10 is None and m <= lv10:
                t_10 = step_t[i] - t0
            if t_90 is None and m <= lv90:
                t_90 = step_t[i] - t0
                break
    if t_10 is not None and t_90 is not None and t_90 > t_10:
        rise_s = t_90 - t_10

    # --- Overshoot: (peak - final)/final, first peak after crossing ---
    overshoot_pct = 0.0
    first_cross = None
    if sp_val > 0:
        for i, m in enumerate(step_m):
            if m >= sp_val:
                first_cross = i
                break
    elif sp_val < 0:
        for i, m in enumerate(step_m):
            if m <= sp_val:
                first_cross = i
                break
    if first_cross is not None:
        post = step_m[first_cross:]
        if sp_val > 0:
            peak_excess = max(m - sp_val for m in post)
            overshoot_pct = (peak_excess / abs_sp) * 100.0
        elif sp_val < 0:
            peak_excess = max(sp_val - m for m in post)
            overshoot_pct = (peak_excess / abs_sp) * 100.0

    # --- Settling time: last time signal exits +/-5% band ---
    band = 0.05 * abs_sp
    last_exit = None
    for i, m in enumerate(step_m):
        if abs(m - sp_val) > band:
            last_exit = i
    settle_s = (step_t[last_exit] - t0) if last_exit is not None else 0.0
    settle_capped = settle_s >= capture_dur - 0.05

    # --- SSE: |mean(last 20% of step) - setpoint| ---
    k = max(1, n // 5)
    last20 = step_m[-k:]
    sse = abs(sum(last20) / len(last20) - sp_val)

    # --- RMSE: step-phase RMS of (measured - setpoint) ---
    rmse = math.sqrt(sum((m - sp_val) ** 2 for m in step_m) / n)

    return {
        "rise_s": rise_s,
        "settle_s": settle_s,
        "overshoot_pct": overshoot_pct,
        "sse": sse,
        "rmse": rmse,
        "t_10": t_10,
        "t_90": t_90,
        "settle_capped": settle_capped,
        "capture_dur": capture_dur,
    }


def format_metrics(m, label, axis):
    """Format a metrics dict into a monospace block for annotation."""
    unit = "rad" if axis == "yaw" else "m"
    rise_str = sigfig(m["rise_s"])
    settle_str = format_settle(m["settle_s"], m["capture_dur"])
    lines = [label]
    lines.append(f" t_rise   = {rise_str} s (10-90%, smoothed)")
    lines.append(f" t_settle = {settle_str}")
    lines.append(f" OS%      = {sigfig(m['overshoot_pct'])}%")
    lines.append(f" SSE      = {sigfig(m['sse'])} {unit}")
    lines.append(f" RMSE     = {sigfig(m['rmse'])} {unit}")
    return "\n".join(lines)


# =========================================================================
# Figure 2 — Closed-loop validation (headline figure)
# =========================================================================

def make_figure2(out_dir):
    """Closed-loop step response: initial vs tuned, per axis.

    3 columns (surge, sway, yaw).  Each column has a TOP tracking subplot
    and a BOTTOM control-effort subplot (shared x-axis, height ratio ~3:1).
    """
    if not _matplotlib_ok:
        print("[WARN] matplotlib not available -- skipping Figure 2.")
        return None

    # --- Load data ---
    data = {}
    for axis in AXES:
        data[axis] = {}
        for scenario in ["initial", "tuned"]:
            csv_path = os.path.join(out_dir, f"{axis}_{scenario}",
                                   "telemetry.csv")
            if not os.path.exists(csv_path):
                print(f"[WARN] Missing: {csv_path}")
                continue
            rows = load_csv(csv_path)
            result = align(rows, axis)
            if result[0]:
                data[axis][scenario] = result  # (t, m, sp, mot, phase)

    # --- Smooth all measured signals ---
    smoothed = {}
    for axis in AXES:
        smoothed[axis] = {}
        for scenario in ["initial", "tuned"]:
            if scenario not in data.get(axis, {}):
                continue
            t, m, sp, mot, phases = data[axis][scenario]
            m_sm = moving_median(m, SMOOTH_WIN)
            smoothed[axis][scenario] = (t, m_sm, sp, mot, phases)

    # --- Compute metrics on SMOOTHED signals ---
    metrics = {}
    for axis in AXES:
        metrics[axis] = {}
        for scenario in ["initial", "tuned"]:
            if scenario not in smoothed.get(axis, {}):
                continue
            t, m_sm, sp, mot, phases = smoothed[axis][scenario]
            met = compute_metrics(t, m_sm, sp, phases)
            if met:
                metrics[axis][scenario] = met

    # --- Derive per-axis setpoint from the tuned data ---
    sp_vals = {}
    for axis in AXES:
        if "tuned" in data.get(axis, {}):
            _, _, sp_list, _, _ = data[axis]["tuned"]
            step_sp = [s for s, p in zip(sp_list, data[axis]["tuned"][4])
                       if p == "step"]
            if any(abs(s) > 1e-6 for s in step_sp):
                sp_vals[axis] = step_sp[0]
    # Fallback
    for axis in AXES:
        sp_vals.setdefault(axis, 0.1)

    # --- Build figure ---
    fig = plt.figure(figsize=(15, 8))
    gs = fig.add_gridspec(
        nrows=2, ncols=3,
        height_ratios=[3, 1],
        hspace=0.08, wspace=0.32,
        top=0.86, bottom=0.08, left=0.07, right=0.97,
    )
    axes_top = [fig.add_subplot(gs[0, i]) for i in range(3)]
    axes_bot = [fig.add_subplot(gs[1, i], sharex=axes_top[i]) for i in range(3)]
    plt.setp([a.get_xticklabels() for a in axes_top], visible=False)

    for idx, axis in enumerate(AXES):
        ax_t = axes_top[idx]
        ax_b = axes_bot[idx]
        ax_t.set_facecolor("#FAFAFA")
        ax_b.set_facecolor("#FAFAFA")
        sp_val = sp_vals[axis]

        # ------ TOP: tracking ------

        # Setpoint (dashed grey, full width)
        if "tuned" in data.get(axis, {}):
            t_sp, _, sp_vals_arr, _, _ = data[axis]["tuned"]
            ax_t.plot(t_sp, sp_vals_arr, linestyle="--", linewidth=1.0,
                      color=C_SETPOINT, label="Setpoint", zorder=1)

        # Raw data: faint underlay (noise is a sim artifact)
        for scenario, color, lw in [("initial", C_INITIAL, 0.5),
                                   ("tuned", C_TUNED, 0.5)]:
            if scenario not in data.get(axis, {}):
                continue
            t_raw, m_raw, _, _, _ = data[axis][scenario]
            ax_t.plot(t_raw, m_raw, linewidth=lw, color=color,
                      alpha=0.25, zorder=1)

        # Smoothed tracking traces (primary visual)
        if axis == "yaw":
            # Yaw: tuned-only (synthesized plant)
            if "tuned" in smoothed.get(axis, {}):
                t, m_sm, _, _, _ = smoothed[axis]["tuned"]
                ax_t.plot(t, m_sm, linewidth=1.8, color=C_TUNED,
                          label="Tuned (synth. plant)", zorder=3)
        else:
            # Surge/sway: initial (thin) + tuned (bold)
            if "initial" in smoothed.get(axis, {}):
                t, m_sm, _, _, _ = smoothed[axis]["initial"]
                ax_t.plot(t, m_sm, linewidth=1.0, color=C_INITIAL,
                          label="Initial", zorder=4)
            if "tuned" in smoothed.get(axis, {}):
                t, m_sm, _, _, _ = smoothed[axis]["tuned"]
                ax_t.plot(t, m_sm, linewidth=1.8, color=C_TUNED,
                          label="Tuned", zorder=5)

        # +/-5% settling band (axhspan)
        band_lo = sp_val * (1 - 0.05)
        band_hi = sp_val * (1 + 0.05)
        ax_t.axhspan(band_lo, band_hi, alpha=0.12, color=C_BAND,
                     label="+/-5% band", zorder=0)

        # 10% and 90% reference lines
        ax_t.axhline(0.10 * sp_val, color="grey", linewidth=0.5,
                     linestyle=":", zorder=0)
        ax_t.axhline(0.90 * sp_val, color="grey", linewidth=0.5,
                     linestyle=":", zorder=0)

        # Rise-time double-headed arrow (on tuned trace, between 10% and 90%)
        if "tuned" in metrics.get(axis, {}):
            mt = metrics[axis]["tuned"]
            t10 = mt["t_10"]
            t90 = mt["t_90"]
            if (t10 is not None and t90 is not None
                    and t90 > t10 and mt["rise_s"] < float("inf")):
                y_arr = 0.10 * sp_val
                ax_t.annotate(
                    "", xy=(t90, y_arr), xytext=(t10, y_arr),
                    arrowprops=dict(arrowstyle="<->", color=C_ARROW,
                                    lw=1.2, shrinkA=0, shrinkB=0),
                    zorder=5,
                )
                t_mid = (t10 + t90) / 2.0
                ax_t.text(
                    t_mid, y_arr - 0.004 * sp_val,
                    f"t_r = {sigfig(mt['rise_s'])} s (smoothed)",
                    fontsize=7, ha="center", va="top", color=C_ARROW,
                    fontfamily="monospace",
                    bbox=dict(boxstyle="round,pad=0.15",
                              facecolor="white", alpha=0.85,
                              edgecolor="none"),
                    zorder=6,
                )

        # 10% / 90% labels at right margin (after drawing everything)
        def _label_pct(level, label):
            ax_t.text(
                0.99, level * sp_val, f" {label}",
                transform=ax_t.get_yaxis_transform(),
                fontsize=7, va="center", ha="left", color="grey",
                clip_on=True, fontfamily="monospace",
            )
        _label_pct(0.10, "10%")
        _label_pct(0.90, "90%")

        # Metrics text boxes (top-left, avoid overshoot peaks)
        box_kw = dict(boxstyle="round,pad=0.3", facecolor="white",
                     alpha=0.90, edgecolor="#CCCCCC", linewidth=0.5)
        bx, by = 0.03, 0.97

        if axis != "yaw" and "initial" in metrics.get(axis, {}):
            txt = format_metrics(metrics[axis]["initial"], "Initial (sim):",
                                 axis)
            ax_t.text(bx, by, txt, transform=ax_t.transAxes,
                      fontsize=7, va="top", fontfamily="monospace",
                      bbox=box_kw, zorder=7)
            by -= 0.40

        if "tuned" in metrics.get(axis, {}):
            lbl = ("Tuned (sim):" if axis != "yaw"
                   else "Tuned (synth, sim):")
            txt = format_metrics(metrics[axis]["tuned"], lbl, axis)
            ax_t.text(bx, by, txt, transform=ax_t.transAxes,
                      fontsize=7, va="top", fontfamily="monospace",
                      bbox=box_kw, zorder=7)

        # Yaw synthesized-plant note
        if axis == "yaw":
            ax_t.text(
                0.98, 0.02,
                "yaw plant synthesized\n(tau estimated, not identified)",
                transform=ax_t.transAxes, fontsize=6,
                va="bottom", ha="right", color="#888888",
                fontstyle="italic", zorder=7,
            )

        ax_t.set_ylabel(POS_LABEL[axis], fontsize=9)
        ax_t.set_title(axis.capitalize(), fontsize=11, fontweight="bold",
                       pad=6)

        # ------ BOTTOM: control effort (tuned motor command) ------
        if "tuned" in data.get(axis, {}):
            t_mot, _, _, mot_int, mot_phases = data[axis]["tuned"]
            mot_norm = [v / 1000.0 for v in mot_int]

            ax_b.step(t_mot, mot_norm, where="post", linewidth=0.7,
                      color=C_TUNED, zorder=2)

            # +/- output_limit lines (normalized to +/-1)
            ax_b.axhline(1.0, color=C_SAT_LINE, linewidth=0.8,
                         linestyle="--", zorder=1)
            ax_b.axhline(-1.0, color=C_SAT_LINE, linewidth=0.8,
                         linestyle="--", zorder=1)
            raw_lim = OUTPUT_LIMITS[axis]
            ax_b.text(
                0.98, 0.95,
                f"+/- output limit = +/-{raw_lim}",
                transform=ax_b.transAxes, fontsize=6,
                va="top", ha="right", color=C_SAT_LINE,
                fontfamily="monospace", zorder=5,
            )

            # Count saturation & sign-flips during step phase
            step_mot = [v for v, p in zip(mot_norm, mot_phases)
                        if p == "step"]
            if step_mot:
                n_sat = sum(1 for v in step_mot if abs(v) >= 0.999)
                n_flip = sum(1 for i in range(1, len(step_mot))
                              if step_mot[i] * step_mot[i - 1] < 0)
                pct = n_sat / len(step_mot) * 100
                ax_b.text(
                    0.02, 0.95,
                    f"tuned-gains saturation + sign-flipping\n"
                    f"  derivative-thrashing evidence\n"
                    f"  ({sigfig(pct)}% saturated, "
                    f"{n_flip} sign-changes in step)",
                    transform=ax_b.transAxes, fontsize=5.5,
                    va="top", ha="left", color="#555555",
                    fontfamily="monospace",
                    bbox=dict(boxstyle="round,pad=0.2",
                              facecolor="#FFF8F0", alpha=0.9,
                              edgecolor="#DDCCBB", linewidth=0.5),
                    zorder=5,
                )

        ax_b.set_ylim(-1.25, 1.25)
        ax_b.set_xlabel("Time (s)", fontsize=9)
        ax_b.set_ylabel("Motor command\n(+/-1 = full thrust)", fontsize=7.5,
                        labelpad=2)
        ax_b.axhline(0, color="grey", linewidth=0.4, zorder=0)

    # ------ Shared legend (collected from top axes) ------
    handles, labels = [], []
    seen = set()
    for ax in axes_top:
        h, l = ax.get_legend_handles_labels()
        for hi, li in zip(h, l):
            if li not in seen:
                seen.add(li)
                handles.append(hi)
                labels.append(li)
    fig.legend(
        handles, labels, loc="upper center",
        ncol=len(handles), fontsize=8,
        bbox_to_anchor=(0.5, 0.99),
        frameon=False, handlelength=2.5,
    )

    fig.suptitle(
        "Per-axis closed-loop step response (sim)\n"
        "(surge/sway: initial vs tuned; yaw: tuned-only, synth. plant)",
        fontsize=12, fontweight="bold", y=1.06,
    )

    png_path = os.path.join(out_dir, "fig2_closedloop_validation.png")
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] Figure 2 written to: {png_path}")
    return png_path, metrics


# =========================================================================
# Figure 3 — Open-loop plant: test frame vs real frame (uncertainty band)
# =========================================================================

def make_figure3(out_dir):
    """Open-loop plant velocity step response: test (measured) vs real
    (extrapolated with uncertainty band).

    v(t) = K * (1 - exp(-t/tau))
    Real frame band: K +/-35%, tau +/-25%.
    """
    if not _matplotlib_ok:
        print("[WARN] matplotlib not available -- skipping Figure 3.")
        return None

    t = [i * 0.05 for i in range(201)]  # 0..10 s at 0.05 s resolution

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.subplots_adjust(wspace=0.30, top=0.84, bottom=0.16,
                        left=0.06, right=0.97)

    for idx, axis in enumerate(AXES):
        ax = axes[idx]
        ax.set_facecolor("#FAFAFA")

        K_test, tau_test = PLANT_PARAMS[axis]["test frame"]
        K_real, tau_real = PLANT_PARAMS[axis]["real frame"]

        # Test frame: solid line (measured, not extrapolated)
        v_test = [K_test * (1.0 - math.exp(-ti / tau_test)) for ti in t]
        ax.plot(
            t, v_test, linewidth=1.6, color=C_TEST,
            label=(f"Test frame (measured): "
                   f"K={K_test:.3f}, \u03c4={tau_test:.2f} s"),
        )

        # Real frame: uncertainty band + nominal dashed center
        K_lo = K_real * 0.65
        K_hi = K_real * 1.35
        tau_lo = tau_real * 0.75
        tau_hi = tau_real * 1.25

        # Envelope corners: fast response vs slow response
        v_fast = [K_hi * (1.0 - math.exp(-ti / tau_lo)) for ti in t]
        v_slow = [K_lo * (1.0 - math.exp(-ti / tau_hi)) for ti in t]
        v_upper = [max(v_fast[i], v_slow[i]) for i in range(len(t))]
        v_lower = [min(v_fast[i], v_slow[i]) for i in range(len(t))]

        ax.fill_between(t, v_lower, v_upper, alpha=0.20, color=C_REAL,
                        zorder=1)
        v_nom = [K_real * (1.0 - math.exp(-ti / tau_real)) for ti in t]
        ax.plot(
            t, v_nom, linewidth=1.6, color=C_REAL, linestyle="--",
            label=(f"Real frame (extrapolated +/-35% K / "
                   f"+/-25% \u03c4): "
                   f"K={K_real:.3f}, \u03c4={tau_real:.2f} s"),
        )

        ax.axhline(0, color="grey", linewidth=0.4)
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel(VEL_LABEL[axis], fontsize=9)
        ax.set_title(axis.capitalize(), fontsize=11, fontweight="bold",
                     pad=6)
        ax.legend(fontsize=6.5, loc="lower right", framealpha=0.9,
                  edgecolor="#CCCCCC")

        # Yaw: annotate the K collapse (authority loss)
        if axis == "yaw":
            ax.annotate(
                "K collapses ~5x\n(authority loss on\nreal frame)",
                xy=(8.0, K_real * 0.95),
                fontsize=7, color=C_REAL, ha="center", va="bottom",
                fontstyle="italic",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                          alpha=0.85, edgecolor="none"),
            )

    fig.suptitle(
        "Open-loop plant response: test (measured) vs real frame "
        "(extrapolated)",
        fontsize=12, fontweight="bold", y=0.98,
    )
    fig.text(
        0.5, 0.01,
        "Real-frame values are physics-based extrapolations, not "
        "measured.  Uncertainty: K +/-35%, tau +/-25% "
        "(methodology section 7).",
        ha="center", fontsize=7, color="#888888", fontstyle="italic",
    )

    png_path = os.path.join(out_dir, "fig3_openloop_plant.png")
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] Figure 3 written to: {png_path}")
    return png_path


# =========================================================================
# Main
# =========================================================================

def main():
    if not _matplotlib_ok:
        print("[INFO] matplotlib not available -- skipping all figures.")
        return

    print(f"[INFO] Reading data from: {LOGS_DIR}")

    # Figure 2 — closed-loop validation
    result2 = make_figure2(LOGS_DIR)

    # Print metrics for correctness check
    if result2:
        _, mets = result2
        print("\n" + "=" * 60)
        print("  FINAL METRICS (smoothed signal)")
        print("=" * 60)
        for axis in AXES:
            if axis in mets:
                for sc in ["initial", "tuned"]:
                    if sc in mets[axis]:
                        mm = mets[axis][sc]
                        print(f"\n  {axis} {sc}:")
                        print(f"    rise     = {sigfig(mm['rise_s'])} s (smoothed)")
                        stxt = format_settle(mm['settle_s'], mm['capture_dur'])
                        print(f"    settle  = {stxt}")
                        print(f"    OS%       = {sigfig(mm['overshoot_pct'])}%")
                        print(f"    SSE      = {sigfig(mm['sse'])} "
                              f"{'rad' if axis == 'yaw' else 'm'}")
                        print(f"    RMSE     = {sigfig(mm['rmse'])} "
                              f"{'rad' if axis == 'yaw' else 'm'}")

        # Per-axis saturation stats for tuned gains
        print("\n  Tuned-gains motor saturation (step phase):")
        for axis in AXES:
            csv_path = os.path.join(LOGS_DIR, f"{axis}_tuned", "telemetry.csv")
            if not os.path.exists(csv_path):
                continue
            rows = load_csv(csv_path)
            t, _, _, mot_int, phases = align(rows, axis)
            step_mot = [v / 1000.0 for v, p in zip(mot_int, phases) if p == "step"]
            if step_mot:
                n_sat = sum(1 for v in step_mot if abs(v) >= 0.999)
                n_flip = sum(1 for i in range(1, len(step_mot))
                              if step_mot[i] * step_mot[i - 1] < 0)
                pct = n_sat / len(step_mot) * 100
                print(f"    {axis}: {sigfig(pct)}% saturated, "
                      f"{n_flip} sign-changes")

        print("=" * 60 + "\n")

    # Figure 3 — open-loop plant
    result3 = make_figure3(LOGS_DIR)

    # Report file sizes
    for label, result in [("Figure 2", result2), ("Figure 3", result3)]:
        if result:
            path = result if isinstance(result, str) else result[0]
            size = os.path.getsize(path)
            print(f"[INFO] {label}: {path} ({size:,} bytes)")


if __name__ == "__main__":
    main()
