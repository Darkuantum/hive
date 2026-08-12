#!/usr/bin/env python3
"""
sim_cl_step.py — Closed-loop step-response dataset generator.

Simulates a PID-controlled plant (first-order velocity + integrator + dead time)
responding to a position step, then emits:
  1. Telemetry CSV (27 cols, byte-compatible with the real logger)
  2. Plot PNG (optional; gracefully skipped if matplotlib is absent)
  3. Printed tracking metrics (rise time, overshoot, settling time, SSE, RMSE)

Usage examples:
    python3 scripts/sim_cl_step.py --axis surge --setpoint 0.1
    python3 scripts/sim_cl_step.py --axis surge --setpoint 0.1 \
        --surge-kp 2.83 --surge-ki 0.35 --surge-kd 5.72 --name sim_surge_tuned
    python3 scripts/sim_cl_step.py --axis yaw --setpoint 0.2 --no-plot
"""

import argparse
import csv
import json
import math
import os
import random
import sys
import time as _time
from collections import deque

# ---------------------------------------------------------------------------
# Lazy matplotlib import — mirroring the pattern in integration/calibration/
# plots.py: try/except ImportError, print a helpful message, still emit CSV.
# ---------------------------------------------------------------------------
_matplotlib_ok = False
try:
    import matplotlib
    matplotlib.use("Agg")          # non-interactive backend before pyplot
    import matplotlib.pyplot as plt
    _matplotlib_ok = True
except ImportError:
    matplotlib = None
    plt = None


# ===================================================================
# PID controller — mirrors integration/pose_controller.py exactly
# ===================================================================

class PID:
    """Single-axis PID with anti-windup and derivative-on-error.

    Core algorithm mirrors integration/pose_controller.py exactly.
    An optional first-order low-pass filter on the derivative term
    (d_filter_tau > 0) models the implicit measurement-pipeline
    filtering present in the real system; without it, raw discrete
    derivative amplifies sensor noise at high kd values.
    """

    def __init__(self, kp, ki, kd, output_limit=0.5, integral_limit=1.0,
                 d_filter_tau=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.d_filter_tau = d_filter_tau  # 0.0 = no filter (raw, matches pose_controller.py)

        self._integral = 0.0
        self._prev_error = None
        self._d_filtered = 0.0

        # Logged terms (mirrors what the real controller exposes)
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        p_term = self.kp * error

        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        if self._prev_error is None:
            d_raw = 0.0
        else:
            d_raw = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Optional first-order low-pass on derivative term
        if self.d_filter_tau > 0 and dt > 0:
            alpha = dt / (self.d_filter_tau + dt)
            self._d_filtered = alpha * d_raw + (1.0 - alpha) * self._d_filtered
            d_term = self._d_filtered
        else:
            d_term = d_raw

        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))

        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_output = output
        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._d_filtered = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0


# ===================================================================
# First-order velocity plant with integrator and dead time
# ===================================================================

class Plant:
    """
    Discrete plant: G(s) = K / (s (tau*s + 1))  with dead time L.

    Each tick (dt):
        m_d  = delayed motor command (from FIFO buffer of size round(L/dt))
        dv   = (K * m_d - v) / tau * dt
        v   += dv
        x   += v * dt
    """

    def __init__(self, K, tau, L, dt=0.1, sigma=0.005):
        self.K = K
        self.tau = tau
        self.L = L
        self.dt = dt
        self.sigma = sigma            # measurement noise std-dev

        self.v = 0.0                  # velocity state
        self.x = 0.0                  # position state

        delay_len = max(1, round(L / dt))
        self._delay_buf = deque([0.0] * delay_len, maxlen=delay_len)

    def step(self, motor_norm):
        """Advance one tick with normalised command in [-1, 1].
        Returns the delayed command actually applied this tick."""
        self._delay_buf.append(motor_norm)
        m_d = self._delay_buf[0]

        # First-order lag on velocity
        dv = (self.K * m_d - self.v) / self.tau * self.dt
        self.v += dv

        # Position = integral of velocity
        self.x += self.v * self.dt
        return m_d

    def measured(self):
        """True position + Gaussian sensor noise."""
        return self.x + random.gauss(0, self.sigma)


# ===================================================================
# Plant defaults — REAL identified values from open-loop data
# ===================================================================
#
# Identified from open-loop step-response fits on 2026-08-11 telemetry
# (ol_* runs).  Quality filter: R² ≥ 0.5.  Gains derived via the
# lambda/IMC tuning rule (same rule as calibration/tuning.py).
#
# |K| used as the effective positive gain the PID sees.  Raw log K is
# negative (rig / motor-wiring convention); the live controller uses
# positive gains (empirically validated on the vehicle).
#
#   surge: 4/12 runs qualified.  |K|=0.054, tau=1.84 s, L≈0 (use 0.10).
#   sway:  6/6 runs qualified (cleanest axis).  |K|=0.079, tau=2.16 s,
#          L≈0.01 → rounded to 0.10.
#   yaw:   0/3 runs qualified — UNIDENTIFIABLE (tau fits garbage; R^2≈0;
#          ArUco PnP yaw-flip corrupts yaw telemetry).  |K|≈0.10 rad/s/unit
#          IS, however, consistently detected across the failed fits, so
#          the MAGNITUDE is data-supported.  tau synthesized to ~2.0 s
#          (rotational inertia comparable to the translational timescale).
#          Values are estimates — replace once yaw open-loop data is
#          re-collected.  METHODOLOGY NOTE: with the hive's 4 openings the
#          controller should snap yaw to the nearest 90° rather than target
#          true heading; that also makes the ArUco 180° flip harmless (a
#          180° flip maps onto another valid opening).  Neither main nor the
#          calibration branch implements this snap yet.
#
# ===================================================================

PLANT_DEFAULTS = {
    # Real identified plant (open-loop step fits, R^2>=0.5, on 2026-08-11 data).
    # |K| used: sign in raw logs is negative (rig/motor-wiring convention); the
    # live controller uses positive gains (empirically validated), so the sim
    # models the effective plant the PID sees, with positive K.
    "surge": {"K": 0.054, "tau": 1.84, "L": 0.10, "sigma": 0.005},  # 4/12 runs
    "sway":  {"K": 0.079, "tau": 2.16, "L": 0.10, "sigma": 0.005},  # 6/6 runs (cleanest)
    # yaw: K magnitude from failed fits (consistent ~0.10); tau synthesized.
    # Yaw is unidentifiable from current data (ArUco PnP flip) — these are
    # plausible estimates, not measured. Replace after yaw data re-collection.
    "yaw":   {"K": 0.10,  "tau": 2.00, "L": 0.05, "sigma": 0.010},
}

# PoseController default gains (fallback if gains file missing)
DEFAULT_GAINS = {
    "surge": {"kp": 0.6, "ki": 0.05, "kd": 0.15},
    "sway":  {"kp": 0.6, "ki": 0.05, "kd": 0.15},
    "yaw":   {"kp": 0.8, "ki": 0.0,  "kd": 0.1},
}

# Per-axis output limits (match PoseController)
OUTPUT_LIMITS = {"surge": 0.4, "sway": 0.4, "yaw": 0.6}

# Telemetry CSV columns — 27 columns, byte-compatible with the real logger
COLUMNS = [
    "ts", "frame_idx", "mode", "aruco_visible",
    "yaw_pixhawk_rad", "yaw_aruco_rad",
    "surge_setpoint", "surge_measured", "surge_p", "surge_i", "surge_d", "surge_out",
    "sway_setpoint",  "sway_measured",  "sway_p",  "sway_i",  "sway_d",  "sway_out",
    "yaw_setpoint",   "yaw_measured",   "yaw_p",   "yaw_i",   "yaw_d",   "yaw_out",
    "motor_x", "motor_y", "motor_z", "motor_r",
    "battery_voltage", "phase",
]


# ===================================================================
# Gains file loader
# ===================================================================

def load_gains(path):
    """Load gains from a JSON file (schema v2). Returns dict or None."""
    try:
        with open(path, "r") as f:
            data = json.load(f)
        if data.get("version") != 2:
            print(f"[WARN] gains file {path}: unsupported version "
                  f"{data.get('version')}, expected 2 — using defaults",
                  file=sys.stderr)
            return None
        return data
    except (FileNotFoundError, json.JSONDecodeError, KeyError) as e:
        print(f"[WARN] gains file {path}: {e} — using defaults", file=sys.stderr)
        return None


# ===================================================================
# Metrics computation (pure Python, no numpy)
# ===================================================================

def compute_metrics(rows, setpoint):
    """
    Compute tracking metrics on step-phase rows for the stepped axis.

    *rows*: list of dicts with 'measured' and 'ts' keys (step phase only).
    *setpoint*: the step setpoint value.
    """
    if not rows:
        return {}

    abs_sp = abs(setpoint)
    n = len(rows)

    measured_vals = [r["measured"] for r in rows]
    times = [r["ts"] for r in rows]
    t0 = times[0]  # step onset time

    # Rise time: first time measured reaches 90% of setpoint
    rise_time = None
    if setpoint > 0:
        threshold_90 = 0.9 * setpoint
        for i, m in enumerate(measured_vals):
            if m >= threshold_90:
                rise_time = times[i] - t0
                break
    elif setpoint < 0:
        threshold_90 = 0.9 * setpoint  # more negative
        for i, m in enumerate(measured_vals):
            if m <= threshold_90:
                rise_time = times[i] - t0
                break
    else:
        rise_time = 0.0
    if rise_time is None:
        rise_time = float("inf")

    # Overshoot: max deviation beyond setpoint after first crossing
    overshoot_pct = 0.0
    first_cross_idx = None
    for i, m in enumerate(measured_vals):
        if setpoint > 0 and m >= setpoint:
            first_cross_idx = i
            break
        elif setpoint < 0 and m <= setpoint:
            first_cross_idx = i
            break
    if first_cross_idx is not None:
        post_cross = measured_vals[first_cross_idx:]
        if setpoint > 0:
            max_excess = max(m - setpoint for m in post_cross)
            overshoot_pct = (max_excess / abs_sp) * 100.0 if abs_sp > 0 else 0.0
        elif setpoint < 0:
            max_excess = max(setpoint - m for m in post_cross)
            overshoot_pct = (max_excess / abs_sp) * 100.0 if abs_sp > 0 else 0.0

    # Settling time: last time measured exits +/-5% band around setpoint
    settling_time = 0.0
    band = 0.05 * abs_sp if abs_sp > 0 else 0.001
    last_exit = None
    for i, m in enumerate(measured_vals):
        if abs(m - setpoint) > band:
            last_exit = i
    if last_exit is not None:
        settling_time = times[last_exit] - t0

    # Steady-state error: abs(mean of last 20%) - setpoint
    last_20_count = max(1, n // 5)
    last_20 = measured_vals[-last_20_count:]
    ss_mean = sum(last_20) / len(last_20)
    steady_state_error = abs(ss_mean - setpoint)

    # Tracking RMSE over step phase
    sq_errors = [(m - setpoint) ** 2 for m in measured_vals]
    rmse = math.sqrt(sum(sq_errors) / n)

    return {
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "steady_state_error": steady_state_error,
        "tracking_rmse": rmse,
    }


# ===================================================================
# Plotting
# ===================================================================

def make_plot(out_dir, name, axis, setpoint, gains, plant_params, records):
    """Generate 3 separate PNG files if matplotlib is available."""
    if not _matplotlib_ok:
        print("[INFO] matplotlib not available — skipping plot. "
              "Install it or use --no-plot.", file=sys.stderr)
        return []

    ts_all = [r["ts"] for r in records]
    motor_norms = [r["motor_norm"] for r in records]
    measured = [r["measured"] for r in records]
    setpoints = [r["setpoint"] for r in records]
    p_terms = [r["p"] for r in records]
    i_terms = [r["i"] for r in records]
    d_terms = [r["d"] for r in records]
    outs = [r["out"] for r in records]

    # Phase boundary times
    pre_end = None
    step_end = None
    for r in records:
        if r["phase"] == "step" and pre_end is None:
            pre_end = r["ts"]
        if r["phase"] == "post" and step_end is not None:
            break
        if r["phase"] == "step":
            step_end = r["ts"] + 0.1  # approximate; last step tick + dt

    kp, ki, kd = gains["kp"], gains["ki"], gains["kd"]
    written = []

    # --- 1. Motor command ---
    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax.step(ts_all, motor_norms, where="post", linewidth=1.0)
    ax.axhline(0, color="grey", linewidth=0.5, linestyle="--")
    if pre_end is not None:
        ax.axvline(pre_end, color="grey", linestyle=":", linewidth=0.8)
    if step_end is not None:
        ax.axvline(step_end, color="grey", linestyle=":", linewidth=0.8)
    ax.set_ylabel("Motor command (normalized)")
    ax.set_xlabel("Time (s)")
    ax.set_ylim(-1.1, 1.1)
    ax.set_title(f"{axis} motor command — kp={kp:.2f} ki={ki:.2f} kd={kd:.2f}",
                 fontsize=10)
    path = os.path.join(out_dir, f"{name}_motor.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # --- 2. Position tracking ---
    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax.plot(ts_all, measured, linewidth=1.0, label="measured")
    ax.plot(ts_all, setpoints, "--", linewidth=0.8, label="setpoint")
    if pre_end is not None:
        ax.axvline(pre_end, color="grey", linestyle=":", linewidth=0.8)
    if step_end is not None:
        ax.axvline(step_end, color="grey", linestyle=":", linewidth=0.8)
    pos_label = "Position (rad)" if axis == "yaw" else "Position (m)"
    ax.set_ylabel(pos_label)
    ax.set_xlabel("Time (s)")
    ax.set_title(f"{axis} position tracking — setpoint={setpoint}", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    path = os.path.join(out_dir, f"{name}_position.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # --- 3. PID terms ---
    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax.plot(ts_all, p_terms, linewidth=0.8, label="p")
    ax.plot(ts_all, i_terms, linewidth=0.8, label="i")
    ax.plot(ts_all, d_terms, linewidth=0.8, label="d")
    ax.plot(ts_all, outs, linewidth=1.0, label="out")
    if pre_end is not None:
        ax.axvline(pre_end, color="grey", linestyle=":", linewidth=0.8)
    if step_end is not None:
        ax.axvline(step_end, color="grey", linestyle=":", linewidth=0.8)
    ax.set_ylabel("PID term output")
    ax.set_xlabel("Time (s)")
    ax.set_title(f"{axis} PID terms — kp={kp:.2f} ki={ki:.2f} kd={kd:.2f}",
                 fontsize=10)
    ax.legend(loc="upper right", fontsize=8)
    path = os.path.join(out_dir, f"{name}_pid.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    return written


# ===================================================================
# Main simulation
# ===================================================================

def simulate(args):
    dt = 0.1  # UPDATE_PERIOD, 10 Hz

    # --- Seed for reproducibility ---
    random.seed(args.seed)

    # --- Load gains ---
    gains_data = load_gains(args.gains_file)
    gains = {}
    for axis_name in ("surge", "sway", "yaw"):
        if gains_data and axis_name in gains_data:
            g = dict(gains_data[axis_name])
        else:
            g = dict(DEFAULT_GAINS[axis_name])

        # CLI per-axis overrides take precedence
        for suffix in ("kp", "ki", "kd"):
            cli_val = getattr(args, f"{axis_name}_{suffix}", None)
            if cli_val is not None:
                g[suffix] = cli_val
        gains[axis_name] = g

    stepped_axis = args.axis
    setpoint = args.setpoint

    # --- Build plant parameters ---
    plant_params = {}
    for axis_name in ("surge", "sway", "yaw"):
        pp = dict(PLANT_DEFAULTS[axis_name])
        # Stepped axis overrides from CLI --K/--tau/--L
        if axis_name == stepped_axis:
            if args.K is not None:
                pp["K"] = args.K
            if args.tau is not None:
                pp["tau"] = args.tau
            if args.L is not None:
                pp["L"] = args.L
        plant_params[axis_name] = pp

    # --- Create PID and plant for each axis ---
    pids = {}
    plants = {}
    for axis_name in ("surge", "sway", "yaw"):
        g = gains[axis_name]
        pp = plant_params[axis_name]
        # Derivative filter tau: models implicit measurement-pipeline filtering.
        # Without it, raw discrete derivative amplifies sensor noise at high kd.
        # Set to 0 to match pose_controller.py exactly (noisy but faithful).
        d_filter = 0.15  # 1.5 ticks @ 10 Hz — gentle smoothing
        pids[axis_name] = PID(g["kp"], g["ki"], g["kd"],
                              output_limit=OUTPUT_LIMITS[axis_name],
                              d_filter_tau=d_filter)
        plants[axis_name] = Plant(pp["K"], pp["tau"], pp["L"],
                                  dt=dt, sigma=pp["sigma"])

    # --- Timing ---
    pre_dur = args.pre_duration
    step_dur = args.hold_duration
    post_dur = args.post_duration
    total_ticks = int(round((pre_dur + step_dur + post_dur) / dt))

    # --- Output paths ---
    out_dir = os.path.join(args.out_dir, args.name)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "telemetry.csv")

    # --- Simulate ---
    ts_base = _time.time()  # epoch base
    records = []  # full list of dicts for plotting
    last_motor_norm = {"surge": 0.0, "sway": 0.0, "yaw": 0.0}

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()

        for tick in range(total_ticks):
            t = ts_base + tick * dt

            # Determine phase
            elapsed = tick * dt
            if elapsed < pre_dur:
                phase = "pre"
            elif elapsed < pre_dur + step_dur:
                phase = "step"
            else:
                phase = "post"

            # Setpoints
            sp = {}
            for axis_name in ("surge", "sway", "yaw"):
                if phase == "step" and axis_name == stepped_axis:
                    sp[axis_name] = setpoint
                else:
                    sp[axis_name] = 0.0

            # --- Step 1: Step all three plants with the last motor command ---
            for axis_name in ("surge", "sway", "yaw"):
                plants[axis_name].step(last_motor_norm[axis_name])

            # --- Step 2: Measure + PID update for all axes ---
            meas = {}
            motor_norms = {}
            for axis_name in ("surge", "sway", "yaw"):
                m = plants[axis_name].measured()
                meas[axis_name] = m
                error = sp[axis_name] - m
                pid_out = pids[axis_name].update(error, dt)

                # Normalised motor command for next tick
                ol = OUTPUT_LIMITS[axis_name]
                mn = max(-1.0, min(1.0, pid_out / ol))
                motor_norms[axis_name] = mn
                last_motor_norm[axis_name] = mn

            # Motor integers
            motor_x = int(round(motor_norms["surge"] * 1000))
            motor_y = int(round(motor_norms["sway"] * 1000))
            motor_r = int(round(motor_norms["yaw"] * 1000))
            motor_z = 0  # heave, unused

            # Build CSV row
            row = {
                "ts": f"{t:.3f}",
                "frame_idx": -1,
                "mode": "calibration",
                "aruco_visible": 1,
                "yaw_pixhawk_rad": "",       # nan → empty string
                "yaw_aruco_rad": f"{meas['yaw']:.6f}",

                "surge_setpoint": f"{sp['surge']:.6f}",
                "surge_measured":  f"{meas['surge']:.6f}",
                "surge_p": f"{pids['surge'].last_p:.6f}",
                "surge_i": f"{pids['surge'].last_i:.6f}",
                "surge_d": f"{pids['surge'].last_d:.6f}",
                "surge_out": f"{pids['surge'].last_output:.6f}",

                "sway_setpoint": f"{sp['sway']:.6f}",
                "sway_measured":  f"{meas['sway']:.6f}",
                "sway_p": f"{pids['sway'].last_p:.6f}",
                "sway_i": f"{pids['sway'].last_i:.6f}",
                "sway_d": f"{pids['sway'].last_d:.6f}",
                "sway_out": f"{pids['sway'].last_output:.6f}",

                "yaw_setpoint": f"{sp['yaw']:.6f}",
                "yaw_measured":  f"{meas['yaw']:.6f}",
                "yaw_p": f"{pids['yaw'].last_p:.6f}",
                "yaw_i": f"{pids['yaw'].last_i:.6f}",
                "yaw_d": f"{pids['yaw'].last_d:.6f}",
                "yaw_out": f"{pids['yaw'].last_output:.6f}",

                "motor_x": motor_x,
                "motor_y": motor_y,
                "motor_z": motor_z,
                "motor_r": motor_r,
                "battery_voltage": "12.0",
                "phase": phase,
            }
            writer.writerow(row)

            # Record for plotting (stepped axis)
            records.append({
                "ts": t,
                "phase": phase,
                "measured": meas[stepped_axis],
                "setpoint": sp[stepped_axis],
                "motor_norm": motor_norms[stepped_axis],
                "p": pids[stepped_axis].last_p,
                "i": pids[stepped_axis].last_i,
                "d": pids[stepped_axis].last_d,
                "out": pids[stepped_axis].last_output,
            })

    # --- Extract step-phase rows for metrics ---
    step_records = [r for r in records if r["phase"] == "step"]
    metrics = compute_metrics(step_records, setpoint)

    # --- Print metrics ---
    print(f"\n{'='*60}")
    print(f"  Step-response metrics — axis={stepped_axis}  setpoint={setpoint}")
    print(f"  Gains: kp={gains[stepped_axis]['kp']:.3f}  "
          f"ki={gains[stepped_axis]['ki']:.3f}  "
          f"kd={gains[stepped_axis]['kd']:.3f}")
    print(f"  Plant: K={plant_params[stepped_axis]['K']:.3f}  "
          f"tau={plant_params[stepped_axis]['tau']:.2f}s  "
          f"L={plant_params[stepped_axis]['L']:.2f}s")
    print(f"{'='*60}")
    if metrics:
        rt = metrics["rise_time_s"]
        rt_str = f"{rt:.2f}s" if rt != float("inf") else ">total"
        print(f"  rise_time_s       : {rt_str}")
        print(f"  overshoot_pct     : {metrics['overshoot_pct']:.1f}%")
        print(f"  settling_time_s   : {metrics['settling_time_s']:.2f}s")
        print(f"  steady_state_error: {metrics['steady_state_error']:.6f}")
        print(f"  tracking_rmse     : {metrics['tracking_rmse']:.6f}")
    print(f"{'='*60}\n")

    # --- Plot ---
    png_paths = []
    if not args.no_plot:
        png_paths = make_plot(out_dir, args.name, stepped_axis, setpoint,
                              gains[stepped_axis], plant_params[stepped_axis],
                              records)

    print(f"[INFO] CSV written to: {csv_path}")
    for p in png_paths:
        print(f"[INFO] PNG written to: {p}")

    return csv_path, png_paths, metrics


# ===================================================================
# CLI
# ===================================================================

def build_parser():
    p = argparse.ArgumentParser(
        description="Closed-loop step-response dataset generator")

    p.add_argument("--axis", choices=["surge", "sway", "yaw"],
                    default="surge",
                    help="Axis to step (default: surge)")
    p.add_argument("--setpoint", type=float, default=0.1,
                    help="Step setpoint (m for surge/sway, rad for yaw; "
                         "default: 0.1)")
    p.add_argument("--hold-duration", type=float, default=5.0,
                    help="Step phase duration in seconds (default: 5.0)")
    p.add_argument("--pre-duration", type=float, default=2.0,
                    help="Pre-step phase duration in seconds (default: 2.0)")
    p.add_argument("--post-duration", type=float, default=3.0,
                    help="Post-step phase duration in seconds (default: 3.0)")
    p.add_argument("--gains-file", type=str,
                    default="integration/gains.json",
                    help="Path to gains JSON file (default: integration/gains.json)")
    p.add_argument("--surge-kp", type=float, default=None)
    p.add_argument("--surge-ki", type=float, default=None)
    p.add_argument("--surge-kd", type=float, default=None)
    p.add_argument("--sway-kp", type=float, default=None)
    p.add_argument("--sway-ki", type=float, default=None)
    p.add_argument("--sway-kd", type=float, default=None)
    p.add_argument("--yaw-kp", type=float, default=None)
    p.add_argument("--yaw-ki", type=float, default=None)
    p.add_argument("--yaw-kd", type=float, default=None)
    p.add_argument("--K", type=float, default=None,
                    help="Override stepped-axis plant velocity gain")
    p.add_argument("--tau", type=float, default=None,
                    help="Override stepped-axis plant time constant (s)")
    p.add_argument("--L", type=float, default=None,
                    help="Override stepped-axis plant dead time (s)")
    p.add_argument("--seed", type=int, default=1,
                    help="Random seed for reproducibility (default: 1)")
    p.add_argument("--out-dir", type=str, default="integration/logs",
                    help="Output root directory (default: integration/logs)")
    p.add_argument("--name", type=str, default=None,
                    help="Run name (default: sim_<axis>_<setpoint>)")
    p.add_argument("--no-plot", action="store_true",
                    help="Skip plotting (even if matplotlib is available)")
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Default name
    if args.name is None:
        args.name = f"sim_{args.axis}_{args.setpoint}"

    # Validate setpoint magnitude
    if args.axis in ("surge", "sway"):
        if abs(args.setpoint) > 0.3:
            parser.error(f"--setpoint must be <= 0.3 m for {args.axis}, "
                         f"got {args.setpoint}")
    elif args.axis == "yaw":
        if abs(args.setpoint) > 0.5:
            parser.error(f"--setpoint must be <= 0.5 rad for yaw, "
                         f"got {args.setpoint}")

    simulate(args)


if __name__ == "__main__":
    main()
