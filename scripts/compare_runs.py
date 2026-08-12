#!/usr/bin/env python3
"""
compare_runs.py — Overlay comparison of closed-loop step-response runs.

Reads telemetry CSVs from integration/logs/<axis>_{initial,tuned,actual}/
and produces:
  - integration/logs/comparison_<axis>.png   (one per axis)
  - integration/logs/comparison_master.png   (3 subplots side-by-side)

Usage:
    uv run --with matplotlib python3 scripts/compare_runs.py
"""

import csv
import os

# ---------------------------------------------------------------------------
# Lazy matplotlib import
# ---------------------------------------------------------------------------
_matplotlib_ok = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _matplotlib_ok = True
except ImportError:
    plt = None


LOGS_DIR = os.path.join(os.path.dirname(__file__), "..", "integration", "logs")
LOGS_DIR = os.path.normpath(LOGS_DIR)

SCENARIOS = ["initial", "tuned", "actual"]
AXES = ["surge", "sway", "yaw"]
UNIT_LABEL = {"surge": "Position (m)", "sway": "Position (m)", "yaw": "Position (rad)"}


def load_csv(csv_path):
    """Return list of dicts from a telemetry CSV."""
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def align(rows, axis):
    """Extract (relative_time, measured, setpoint) with t=0 at step onset.

    Returns three lists: t_rel, measured, setpoint.
    """
    t_rel = []
    measured = []
    setpoint = []

    # Find the timestamp of the first "step" row to use as t=0
    t0 = None
    for r in rows:
        if r["phase"] == "step":
            t0 = float(r["ts"])
            break

    if t0 is None:
        return t_rel, measured, setpoint

    col_meas = f"{axis}_measured"
    col_sp = f"{axis}_setpoint"

    for r in rows:
        ts = float(r["ts"]) - t0
        try:
            m = float(r[col_meas])
        except (ValueError, KeyError):
            continue
        try:
            s = float(r[col_sp])
        except (ValueError, KeyError):
            s = 0.0
        t_rel.append(ts)
        measured.append(m)
        setpoint.append(s)

    return t_rel, measured, setpoint


def plot_per_axis(axis, out_dir):
    """Generate comparison_<axis>.png overlaying initial/tuned/actual."""
    fig, ax = plt.subplots(figsize=(9, 6), constrained_layout=True)
    y_label = UNIT_LABEL[axis]
    colors = {"initial": "tab:blue", "tuned": "tab:orange", "actual": "tab:green"}

    for scenario in SCENARIOS:
        csv_path = os.path.join(out_dir, f"{axis}_{scenario}", "telemetry.csv")
        if not os.path.exists(csv_path):
            print(f"[WARN] Missing: {csv_path}", flush=True)
            continue
        rows = load_csv(csv_path)
        t, m, sp = align(rows, axis)
        if not t:
            continue
        ax.plot(t, m, linewidth=1.0, label=scenario, color=colors[scenario])
        # Use the setpoint from the last scenario (they're all 0.1 in this case)
        if scenario == SCENARIOS[-1]:
            ax.axhline(sp[0] if sp else 0.1, color="grey", linestyle="--",
                       linewidth=0.8, label="setpoint")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(y_label)
    ax.set_title(f"{axis} position tracking: initial vs tuned vs actual", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    png_path = os.path.join(out_dir, f"comparison_{axis}.png")
    fig.savefig(png_path, dpi=200)
    plt.close(fig)
    print(f"[INFO] PNG written to: {png_path}")
    return png_path


def plot_master(out_dir):
    """Generate comparison_master.png with 3 side-by-side subplots."""
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True,
                             sharey=False)
    colors = {"initial": "tab:blue", "tuned": "tab:orange", "actual": "tab:green"}

    for idx, axis in enumerate(AXES):
        ax = axes[idx]
        y_label = UNIT_LABEL[axis]
        for scenario in SCENARIOS:
            csv_path = os.path.join(out_dir, f"{axis}_{scenario}", "telemetry.csv")
            if not os.path.exists(csv_path):
                continue
            rows = load_csv(csv_path)
            t, m, sp = align(rows, axis)
            if not t:
                continue
            ax.plot(t, m, linewidth=1.0, label=scenario, color=colors[scenario])
            if scenario == SCENARIOS[-1]:
                ax.axhline(sp[0] if sp else 0.1, color="grey", linestyle="--",
                           linewidth=0.8, label="setpoint")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(y_label)
        ax.set_title(axis, fontsize=10)

    # Single shared legend from the last axis that has data
    handles, labels = axes[-1].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, fontsize=8)
    fig.suptitle("Position tracking: initial vs tuned vs actual", fontsize=11, y=1.02)

    png_path = os.path.join(out_dir, "comparison_master.png")
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] PNG written to: {png_path}")
    return png_path


def main():
    if not _matplotlib_ok:
        print("[INFO] matplotlib not available — skipping plots.", flush=True)
        return

    print(f"[INFO] Reading runs from: {LOGS_DIR}")

    # Per-axis comparison plots
    for axis in AXES:
        plot_per_axis(axis, LOGS_DIR)

    # Master 3-subplot comparison
    plot_master(LOGS_DIR)


if __name__ == "__main__":
    main()
