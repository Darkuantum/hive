# ROV Calibration Methodology (slide-ready)

> Companion to `docs/calibration-presentation-research.md` (presentation conventions).
> Numbers are from the test-frame identification + real-frame extrapolation done in this work;
> replace with live re-identification on the full-scale vehicle once available. Tag every
> closed-loop metric **(sim)** or **(measured)** — our current closed-loop datasets are
> simulations (`scripts/sim_cl_step.py`).

---

## 1. System & control architecture

- **Vehicle:** underwater ROV, BlueROV2-vectored style — **4 horizontal thrusters mixed for surge/sway/yaw** (no heave/roll/pitch actuation). Mass ~15 kg (test frame), ~125 kg (full-scale target).
- **Pose sensor:** ArUco single-marker PnP at 10 Hz → body-frame position (surge/sway) + heading (yaw). Position noise σ ≈ 5 mm (test, in air) / ~10 mm (underwater).
- **Controller:** position-hold PID per axis (`integration/pose_controller.py`). Output is a **velocity command**, normalized by a per-axis `output_limit` (±0.4 m/s surge/sway, ±0.6 rad/s yaw) → ArduSub stick ±1000. **Setpoint is implicitly 0** (drive marker-relative pose to zero); closed-loop validation injects a setpoint offset.
- **Key implication of vectored mixing:** surge/sway/yaw share the same 4 thrusters, so a saturated yaw PID starves translation of thrust. This was the root cause of the earlier "commanded surge, got yaw" failure (§8).

**Block diagram (methodology slide):**
```
ArUco pose → [camera_to_body] → body pose (x,y,yaw)
                                    ↓
 setpoint offset ──→ error ──→ [PID per axis] ──→ velocity cmd ──→ /output_limit ──→ motors ──→ [Plant]
                                                                                              ↑
                                                              open-loop step test → identify K,τ,L → λ/IMC gains
```

## 2. Plant model

From the PID's perspective the plant is a **first-order velocity lag + integrator + dead time**:

```
G(s) = K · e^(−Ls) / [ s (τs + 1) ]
```

| param | meaning | unit |
|---|---|---|
| **K** | velocity gain = steady-state velocity per unit thrust | (m/s)/unit (surge/sway), (rad/s)/unit (yaw) |
| **τ** | time constant (inertia/drag) | s |
| **L** | dead time (actuator + comms + thruster spool-up) | s |
| **K_eff = K / output_limit** | effective gain the PID actually sees | — |

**Physical motivation:** a thruster command produces thrust; velocity rises with a first-order lag (thruster/mass inertia τ); position is the integral of velocity; a small hardware delay L precedes motion.

**Why FOPDT (not higher-order / frequency-domain):** the plant is low-order, the vehicle operates at low speed in a calm tank (no wave excitation, no flexible modes), and the model structure is fixed by physics. Step-response ID is the standard for such plants — minimal excitation (a single step), directly estimates the three parameters (Fossen, *Handbook of Marine Craft Hydrodynamics*, Ch. 9; Åström & Hägglund, *Advanced PID Control*, §2.1). Frequency sweeps (Bode/Nyquist ID) offer no advantage for a 3-parameter model and take far longer.

## 3. Open-loop identification

**Procedure (`integration/calibration/identify.py`):**
1. Drive an open-loop step in motor command (manual mode, amplitude ≤ 0.5).
2. Detect step onset from the sustained motor jump.
3. Smooth position; compute velocity via central differences.
4. **τ via the area method** (position-based, noise-robust): `area = v_ss·T − (x(T)−x(0))`; iteratively solve `area = v_ss·τ·(1−e^(−T/τ))`.
5. **K = v_ss / F_step** (steady-state velocity ÷ normalized step amplitude).
6. **L** = time to first detectable motion.
7. **Fit quality:** NRMSE fit% on **velocity** (primary) = `100×(1 − RMSE_v / σ_v)`.

> **Why the area method, not log-linearized regression:** `ln(1 − v/v_ss)` diverges as v→v_ss, amplifying noise on near-steady-state samples (>60% τ errors in testing). The area method uses position (high SNR, no differentiation) and converges robustly (100/100 random seeds, median K error 1.8%, τ error 6.6%).

**Parameter uncertainty:** reported as **mean ± σ across runs** (NOT Fisher-information CIs — single-step tests don't justify them). Quality-gate fits at R² ≥ 0.5; outliers (>2σ) flagged, not silently dropped.

**Residual analysis:** the model-fit residual subplot (on the open-loop figure) is the lightweight residual check — a systematic trend flags structure problems. Formal autocorrelation/cross-correlation is deferred as scope (model structure is fixed by physics, not chosen from candidates).

## 4. Tuning — λ/IMC rule (`integration/calibration/tuning.py`)

For an integrating + first-order plant with PID, the λ/IMC (Internal Model Control) rule:

```
Kp = 1 / (K_eff · τ_cl)
Ki = Kp / (4·τ_cl)
Kd = τ · Kp          ← cancels the plant's first-order lag
τ_cl = max(τ, 0.5)   ← desired closed-loop time constant (don't be faster than the actuator)
```

The **Kd = τ·Kp term provides pole-zero cancellation** so the closed loop is **dominated by** a single pole at −1/τ_cl (the SIMC/IMC design intent), with faster auxiliary poles from the integrating plant. Response is effectively overdamped → 10–90% rise and ~0% overshoot are the **design targets**; the small actual deviations (rise inflated ~30% vs the ideal τ_cl·ln9, occasional overshoot from dead time + the derivative filter) are **reported in §6**.

## 5. Closed-loop validation

Inject a position setpoint step (e.g., 0.1 m surge) and measure tracking. **Metric definitions** (state these on every results slide):

| metric | definition |
|---|---|
| Rise time | 10–90% of setpoint (overdamped 1st-order closed loop; python-control/MATLAB default) |
| Settling time | ±5% band (process-industry convention; also more robust to extract from noisy ArUco telemetry than ±2%) |
| Overshoot | (peak − final)/final, first peak |
| SSE | \|mean(last 20% of step) − setpoint\| |
| RMSE | step-phase RMS of (measured − setpoint) |

**Data source:** all closed-loop numbers below are **(sim)** — from `scripts/sim_cl_step.py`. Once the vehicle logs closed-loop telemetry, present **(measured)** alongside as the validation, with sim as a dashed "predicted" overlay.

## 6. Results — test frame (0.82 m, ~15 kg)

**Identified plant** (open-loop step ID, quality-gated R² ≥ 0.5):

| axis | K | τ (s) | L (s) | qualifying runs | fit signal |
|---|---|---|---|---|---|
| surge | 0.054 (m/s)/unit | 1.84 | 0.10 | 4/12 | velocity NRMSE |
| sway | 0.079 (m/s)/unit | 2.16 | 0.10 | 6/6 (cleanest) | velocity NRMSE |
| yaw | 0.10 (rad/s)/unit **(synth)** | 2.0 **(synth)** | 0.05 | 0/3 | **no fit — ArUco-corrupted** |

**Initial vs λ/IMC-tuned (sim), 10 s hold, 0.1 setpoint:**

| axis | scenario | Kp/Ki/Kd | rise (s) | OS% | SSE | RMSE |
|---|---|---|---|---|---|---|
| surge | initial | 0.6/0.05/0.15 | — (slow) | 0 | 0.038 | high |
| surge | tuned | 4.0/0.54/7.36 | 5.3 | ~0 | 0.017 | low |
| sway | initial | 0.6/0.05/0.15 | — (slow) | 0 | 0.020 | high |
| sway | tuned | 2.34/0.27/5.07 | 5.7 | ~0 | 0.011 | low |
| yaw | tuned (synth plant) | 3.0/0.375/6.0 | 4.8 | 6 | 0.029 | — |

> **2–3 significant figures only** — our pipeline prints 6 decimals elsewhere; that implies false precision on noisy data.

> *Yaw initial-vs-tuned omitted — the synthesized yaw plant makes the before/after comparison uninformative.*

**Derivative-thrashing finding (load-bearing caveat):** the tuned gains' high **Kd** (7.36 surge, 6.0 yaw) amplifies ArUco position noise through the raw derivative → the motor saturates and oscillates frame-to-frame even with a light derivative filter (d_filter=0.15s). Measured: tuned surge motor saturates ~20% of the step, ~20 sign-changes; initial gains (Kd=0.15) saturate 0%. **The λ/IMC gains are not directly deployable** — they need either a stronger derivative filter or de-rating. The original deployed gains (0.6/0.8) are smooth precisely because their Kd is tiny.

## 7. Real-frame extrapolation (2 m, ~125 kg, fully submerged)

Physics scaling test→target, anchored on τ = m_eff/b and K = F_max/b:

| ratio | value | basis |
|---|---|---|
| length | ×2.44 | 0.82→2 m |
| dry mass | ×8.3 | 15→125 kg (sub-geometric; λ³=14.5) |
| wetted area | ×6 | λ² |
| m_eff | ×~10 | dry mass + added mass (full submersion) |
| damping b | ×~8.6 | area × full-vs-partial submersion × wave-drag removal |

**Result** (translation: mass tracks damping → τ grows only modestly; K preserved under "thrust scales with drag" — the only self-consistent assumption without a thruster spec):

| axis | K_target | τ_target (s) | L_target (s) | vs test |
|---|---|---|---|---|
| surge | 0.054 | 2.20 | 0.12 | K≈same, τ×1.19 |
| sway | 0.079 | 2.58 | 0.12 | K≈same, τ×1.19 |
| yaw | **0.019** | 1.11 | 0.08 | **K×0.19 — authority collapse** |

**Uncertainty band: K ±35%, τ ±30%** — plot the real-frame response as a `fill_between` band, NOT a point trajectory. **Yaw is the binding constraint** on the 2 m frame: rotational damping (∝ L⁵, ×86) outpaces both rotational inertia (∝ m·L², ×50) and thrust-torque (∝ F·L, ×15–21) — so τ_yaw actually drops (faster settling) while K_yaw = torque/damping collapses ~5× to ~1°/s at full thrust. **Yaw authority is a thruster-sizing decision (up-size or dedicated yaw thrusters), not a tuning output.** Real-frame values are physics-based extrapolations, not measured — re-identify on the real frame once built.

## 8. Yaw control methodology — the 4-axis snap

**Problem:** OpenCV `estimatePoseSingleMarkers` has a real two-solution PnP ambiguity for near-head-on markers (~180° apart). On the live system this caused the detected yaw to flip ~180° frame-to-frame, **saturating the yaw PID** → on the vectored frame that starved surge/sway of thrust → the "commanded surge, got yaw" failure.

**Solution (shipped on `main`, commit `9f538a2`):** because the hive has **4 symmetric openings**, the vessel only needs to be parallel/antiparallel to the nearest opening axis — not aligned to the marker's true heading. The yaw error is snapped to the **nearest of 4 equivalent headings (every π/2)**, computed flip-invariantly in doubled-angle space. A 180° ArUco flip then maps onto another valid opening → **identical yaw error** → can no longer saturate the PID. The `--yaw-snap-axes` flag (default 4; <2 disables) allows A/B testing.

> This is why targeting the true heading (the old behavior) was the root cause, and why fighting the ArUco filter was the wrong fix. Snap neutralizes the ambiguity by changing the control objective to match the task's 4-fold symmetry.

## 9. Limitations & caveats (on every results slide)

1. Plant identified from open-loop step tests on a **test frame (~15 kg, in air)**; full-scale ROV parameters are **physics-extrapolated, not water-validated**.
2. **Yaw τ is synthesized** (ArUco PnP 180° ambiguity corrupted yaw telemetry — 0/3 qualifying ID runs); K magnitude is data-supported, τ is an estimate. Re-collect yaw data after the snap fix; re-identify.
3. Sensor: ArUco single-marker PnP @ 10 Hz; position σ ≈ 5 mm (test) / ~10 mm (underwater).
4. Model assumes **decoupled axes** (no surge/sway/yaw cross-coupling); minor thruster-wake coupling unmodeled.
5. FOPDT captures dominant dynamics; higher-order effects (thruster lag, structural vibration, prop ventilation) unmodeled.
6. Results show **setpoint tracking**; disturbance rejection (currents, tugs) is provided by integral action (Ki) and λ-robustness, not separately tested.

## 10. Reproducibility

- Identification + tuning: custom pure-Python (`integration/calibration/identify.py`, `tuning.py`) — no numpy needed on the vehicle. Methodology follows MATLAB System ID Toolbox conventions (Ljung 1999; Seborg et al. 2016).
- Closed-loop dataset generator: `scripts/sim_cl_step.py`. Comparison overlays: `scripts/compare_runs.py`.
- Datasets local under `integration/logs/` (gitignored, regenerated on demand).
- Per-figure repro: plant params + PID gains + sim params (dt, duration, noise σ) + metric definitions + code commit.

---

# Presentation spec (for slides)

**3-slide structure (+ appendix):**

**Slide 1 — Methodology:** block diagram (§1) + model G(s) (§2) + λ/IMC rule (§4) + 1-line caveat (test-frame). 20 s.

**Slide 2 — Open-loop results:** Figure 1 (1×3 measured-vs-model fit + residual subplot + NRMSE fit%; **yaw subplot = raw data + "τ synthesized, no fit" watermark, no fit%**) + plant table (§6, K/τ/L ± σ). Key callout: "Sway cleanest (6/6); yaw unidentifiable (ArUco)." 30 s.

**Slide 3 — Closed-loop results:** Figure 2 (1×3 setpoint + tracking (surge/sway: initial-vs-tuned; **yaw: tuned-only, synthesized-plant note**) + ±5% band + **control-effort subplot below with ±output_limit lines, annotated "tuned-gains saturation + sign-flipping — derivative-thrashing evidence"** (this subplot earns its place by exposing the deployability problem) + per-scenario metrics boxes tagged (sim)) + before/after metrics table (§6) + improvement callout ("tuned: SSE ~4–7× lower") + derivative-thrashing caveat (§6). 30 s.

**Appendix — Real-frame extrapolation:** open-loop test-vs-real plant response (Figure 3, with **uncertainty band**, not point) + the yaw-authority-collapse finding (§7). Label "physics-based, not measured."

**Figure inventory** (full specs in `docs/calibration-presentation-research.md` §F2):
1. Open-loop identification (measured vs model fit + residual + NRMSE; yaw raw-only).
2. Closed-loop validation (setpoint + tracking [surge/sway initial-vs-tuned; yaw tuned-only] + ±5% band + control-effort subplot; metrics (sim)).
3. Test-vs-real-frame open-loop plant (uncertainty band).
4. (optional) Ensemble open-loop (spaghetti + mean±σ).

**Metrics table format:** axes × scenario × (Kp/Ki/Kd + rise + settle + OS% + RMSE), units in header, 2–3 sig figs, convention footnote (rise 10–90, settle ±5, SSE last-20%, RMSE step-phase, fit NRMSE-on-velocity), all tagged (sim)/(measured). Tuned row bold/green on slides.
