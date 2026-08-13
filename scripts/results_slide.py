#!/usr/bin/env python3
"""Clean slide-ready CLOSED-LOOP TRACKING results graph.

Per axis (surge/sway/yaw): setpoint (step SHAPE: 0->hold->0) + measured tracking,
plotted over the step AND post phases so both the RISE and the FALL are visible.
t=0 at step onset, real seconds. Metrics box (rise/overshoot/settle/SSE) on the
step phase. Yaw is tuned-only (synthesized plant).

Run: uv run --with matplotlib python3 scripts/results_slide.py
"""
import csv, os, statistics
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

LOGS = os.path.join(os.path.dirname(__file__), "..", "integration", "logs")
AXES = ["surge", "sway", "yaw"]
YLABEL = {"surge": "Position (m)", "sway": "Position (m)", "yaw": "Heading (rad)"}
SHOW_INITIAL = {"surge": True, "sway": True, "yaw": True}  # yaw plant synthesized; initial also synthesized


def _med(vals, w=5):
    out = []
    half = w // 2
    for i in range(len(vals)):
        lo, hi = max(0, i - half), min(len(vals), i + half + 1)
        out.append(statistics.median(vals[lo:hi]))
    return out


def load(axis, scenario):
    """Return (t, measured_smoothed, sp_series) over step+post, t=0 at step onset."""
    path = os.path.join(LOGS, f"{axis}_{scenario}", "telemetry.csv")
    if not os.path.exists(path):
        return None
    rows = [r for r in csv.DictReader(open(path)) if r["phase"] in ("step", "post")]
    step_rows = [r for r in rows if r["phase"] == "step"]
    if not step_rows:
        return None
    t0 = float(step_rows[0]["ts"])
    t = [float(r["ts"]) - t0 for r in rows]
    m = _med([float(r[f"{axis}_measured"]) for r in rows])
    sp = [float(r[f"{axis}_setpoint"]) for r in rows]   # step shape: 0.1 then 0
    return t, m, sp


def metrics(t_step, m_step, sp):
    abs_sp = abs(sp)
    thr90 = 0.9 * sp
    ri = next((i for i, v in enumerate(m_step) if v >= thr90), None)
    rise_s = f"{t_step[ri]:.1f} s" if ri is not None else "never"
    peak = max(m_step)
    os_pct = max(0, (peak - sp) / abs_sp * 100) if abs_sp > 0 else 0
    band = 0.05 * abs_sp
    last_out = next((i for i in range(len(m_step) - 1, -1, -1) if abs(m_step[i] - sp) > band), None)
    settle_s = f"{t_step[last_out]:.1f} s" if last_out is not None else "in band"
    sse = abs(statistics.mean(m_step[-max(1, len(m_step) // 5):]) - sp)
    return rise_s, os_pct, settle_s, sse


def step_metrics(t, m, sp_series, sp_hold):
    """Metrics over the step-phase rows (where setpoint == sp_hold)."""
    t_s = [tt for tt, s in zip(t, sp_series) if abs(s - sp_hold) < 1e-9]
    m_s = [v for v, s in zip(m, sp_series) if abs(s - sp_hold) < 1e-9]
    return metrics(t_s, m_s, sp_hold)


def main():
    if plt is None:
        print("matplotlib unavailable"); return
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5), constrained_layout=True)
    legend_handles = None
    for ax, axis in zip(axes, AXES):
        d_t = load(axis, "tuned")
        if d_t is None:
            ax.set_visible(False); continue
        t_t, m_t, sp_t = d_t
        sp_hold = max(sp_t)                      # the held setpoint (0.1)
        step_end = max(tt for tt, s in zip(t_t, sp_t) if abs(s - sp_hold) < 1e-9)
        # +/-5% settling band, only during the step hold
        ax.fill_between([0, step_end], [sp_hold * 0.95, sp_hold * 0.95],
                        [sp_hold * 1.05, sp_hold * 1.05], color="#888888", alpha=0.13)
        # setpoint step shape (dashed)
        ax.plot(t_t, sp_t, color="#555555", linestyle="--", linewidth=1.3, label="setpoint")
        # initial (faint) where applicable
        if SHOW_INITIAL[axis]:
            d_i = load(axis, "initial")
            if d_i:
                ax.plot(d_i[0], d_i[1], color="#1f77b4", linewidth=1.3, alpha=0.7, label="default")
        # tuned (orange)
        ax.plot(t_t, m_t, color="#d55e00", linewidth=2.0, alpha=0.85, label="λ/IMC (λ=τ)")
        # de-rated (green, bold) -- the deployable result
        d_d = load(axis, "derated")
        if d_d:
            ax.plot(d_d[0], d_d[1], color="#009e73", linewidth=2.6, label="λ/IMC (λ=2τ, selected)")
        # metrics: tuned vs de-rated (step phase)
        rise_t, os_t, settle_t, sse_t = step_metrics(t_t, m_t, sp_t, sp_hold)
        if d_d:
            rise_d, os_d, settle_d, sse_d = step_metrics(d_d[0], d_d[1], d_d[2], sp_hold)
            box = (f"λ=τ      : OS {os_t:.0f}%, SSE {sse_t:.3f}\n"
                   f"λ=2τ     : OS {os_d:.0f}%, SSE {sse_d:.3f}")
        else:
            box = (f"tuned: OS {os_t:.0f}%, SSE {sse_t:.3f}")
        ax.text(0.02, 0.97, box, transform=ax.transAxes, fontsize=9, va="top",
                family="monospace", bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#cccccc", alpha=0.9))
        ax.set_xlabel("Time (s)", fontsize=11)
        ax.set_ylabel(YLABEL[axis], fontsize=11)
        ax.set_title(axis, fontsize=12, fontweight="bold")
        ax.set_xlim(left=0)
        ax.tick_params(labelsize=10)
        ax.grid(alpha=0.25)
        if legend_handles is None:
            legend_handles, legend_labels = ax.get_legend_handles_labels()
        print(f"[CHK] {axis}: tuned OS={os_t:.0f}% SSE={sse_t:.3f}"
              + (f" | de-rated OS={os_d:.0f}% SSE={sse_d:.3f}" if d_d else ""))
    # single legend inside the first panel (avoids title/legend overlap)
    axes[0].legend(legend_handles, legend_labels, loc="lower right", fontsize=10, frameon=True)
    fig.suptitle("Closed-loop step response: default vs λ/IMC (λ=τ and 2τ)",
                 fontsize=13)
    out = os.path.join(LOGS, "results_slide.png")
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] {out} ({os.path.getsize(out)} bytes)")


if __name__ == "__main__":
    main()
