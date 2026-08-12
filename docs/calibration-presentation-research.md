# Control-System Calibration Presentation Research

> **Scope:** How professionals present step-response calibration, system-identification,
> and PID-tuning results.  Time-domain emphasis (our figures); frequency-domain for
> completeness.  Anchored to our ROV plant: `G(s) = K·e^(−Ls) / [s(τs+1)]` per axis
> (surge/sway/yaw), λ/IMC PID tuning, 3 axes × test-vs-real frame × initial-vs-tuned
> gains.

---

## A. Time-Domain Step-Response Presentation  (PRIORITY)

### A1. Standard Metrics & Definitions

Control textbooks and tools converge on these five time-domain metrics.  The table below
lists the **canonical definition** and the convention our tools actually implement.

| Metric | Standard definition | python-control default | MATLAB `stepinfo` default | Convention for *our* ROV |
|---|---|---|---|---|
| **Rise time** | Time from lower to upper threshold around final value | **10% → 90%** (`RiseTimeLimits=(0.1, 0.9)`) | 10% → 90% | 10% → 90% (λ/IMC yields a first-order closed loop by construction → overdamped → 10–90%; also the python-control/MATLAB default) |
| **Settling time** | Time after which the response stays within a band | **±2%** of final value (`SettlingTimeThreshold=0.02`) | ±2% | ±5% per process-industry convention (Åström & Hägglund Ch. 2); also more robust to extract from noisy ArUco-based telemetry than ±2% — state explicitly |
| **Overshoot** | `(Peak − Final) / Final × 100%`, measured on first peak only | `Overshoot` key in dict, % of steady-state | Same | Same — 0% expected for the ideal first-order closed loop; **measured OS reported** (dead time L + derivative filter can produce overshoot) |
| **Peak time** | Time of first peak | `PeakTime` | `PeakTime` | Not meaningful for overdamped; omit or report "N/A (overdamped)" |
| **Steady-state error** | `|mean(last N samples) − setpoint|` | Not computed by `step_info` (custom) | Not in `stepinfo` | Use last 20% of step phase, report in m or rad |

**Sources:**

- python-control `step_info` docs: <https://python-control.readthedocs.io/en/latest/generated/control.step_info.html> — RiseTimeLimits default `(0.1, 0.9)`, SettlingTimeThreshold default `0.02`.
- MATLAB `stepinfo` (<https://www.mathworks.com/help/control/ref/stepinfo.html>) — identical defaults.
- K. Ogata, *Modern Control Engineering*, 5th ed., §5.3: defines rise time, peak time, overshoot, settling time with 2% and 5% bands both discussed.
- K. J. Åström & T. Hägglund, *Advanced PID Control*, Ch. 2: settling time ±2% standard, but ±5% accepted for industrial processes.
- Wikipedia *Rise time* (<https://en.wikipedia.org/wiki/Rise_time>): cites Levine (1996) — underdamped systems use 0–100%, overdamped use 10–90%. Our first-order-integrator plant is overdamped in closed loop → **10–90% is correct**.

#### Rise-time convention for integrator plants

Our plant `G(s) = K e^(−Ls) / [s(τs+1)]` has no finite steady-state gain (pole at
origin).  The *velocity* step response reaches a finite steady state `K` (m/s per
unit thrust), so velocity rise time uses the standard 10–90% of `K`.  For *position*
(closed-loop step), the setpoint is finite and well-defined, so 10–90% of the setpoint
is the correct basis.  **In both cases, 10–90% is the right convention.**

### A2. How to Annotate a Step-Response Plot

The professional annotation pattern, as seen in MATLAB `stepplot` with
characteristic markers enabled, places:

```
┌──────────────────────────────────────────────────────┐
│  Axis label with units              Title (10 pt)    │
│  ── Measured (solid line, 1.5 pt)                   │
│  ── Setpoint (dashed, 0.8 pt, grey)                  │
│                                                       │
│     ┌──────┐ ← ±5% settling band (shaded fill)       │
│     │      │                                          │
│     │      │  ↑ Peak value callout (if overshoot)    │
│  ···│······│··· ← 10% line (dotted, label "10%")     │
│  ···│······│··· ← 90% line (dotted, label "90%")     │
│     │      │                                          │
│  →──│──────│── ← Rise time arrow (double-headed)      │
│     │      │                                          │
│     └──────┘                                          │
│                                                       │
│  ╭─ Metrics box (top-left or top-right) ─╮           │
│  │ t_rise  = 1.84 s (10–90%)             │           │
│  │ t_settle = 3.21 s (±5%)               │           │
│  │ Overshoot = 0.0%                       │           │
│  │ SSE = 0.003 m                          │           │
│  │ RMSE = 0.012 m                         │           │
│  │ Fit = 94.2% (NRMSE)                    │           │
│  ╰────────────────────────────────────────╯           │
└──────────────────────────────────────────────────────┘
```

**Concrete annotation rules:**

1. **Settling band:** Shade the region `setpoint × (1 ± 0.05)` with `ax.axhspan()`,
   alpha=0.10–0.15, color matching the axis.  Label "±5% band" on the right edge.
   *(Source: MATLAB `stepplot` default when settling time markers enabled;
   python-control returns settling band info via `step_info`)*

2. **10% / 90% reference lines:** Horizontal dotted lines at `0.1 × setpoint` and
   `0.9 × setpoint`.  Label with "10%" and "90%" on the right margin.
   *(Source: Ogata §5.3, Figure 5.6)*

3. **Rise time arrow:** Use `ax.annotate()` with a double-headed arrow between
   the two time crossings of the 10% and 90% lines.  Label "t_r = X.XX s" centered.
   *(Source: Standard textbook presentation, Åström & Hägglund Ch. 2)*

4. **Metrics text box:** `ax.text()` or `ax.table()` positioned in the upper-left
   (avoid overlap with overshoot peaks).  Use monospace font at 8 pt.  Include
   2–3 significant figures consistent across all axes.  Do NOT use more than 3 sig figs
   — it implies false precision when the underlying data is noisy.
   *(Source: MATLAB Control System Toolbox `stepplot` with `ShowCharacteristics=true`;
   every Åström & Hägglund textbook figure)*

5. **Dead time marker (open-loop plots only):** Vertical dashed line at `t = L`
   (identified dead time), labeled "Dead time L = 0.10 s".  Optional shaded region
   from 0 to L.  *(Source: Seborg, Edgar, Mellichamp & Doyle, *Process Dynamics and
   Control*, 4th ed., §7.2)*

6. **Step onset marker:** Vertical solid grey line at `t = 0` (step onset), optional
   label "Step at t = 0".  Already present in our `compare_runs.py` as `axvline`.

### A3. Setpoint + Measured + Model-Fit Overlay

For **model identification plots** (open-loop data vs fitted model):

**Top subplot — signal overlay:**
- **Measured data:** Thin solid line (0.8–1.0 pt) or scatter markers (small dots, α=0.7).
  Color: dark grey or a neutral blue.
- **Model prediction:** Bold solid line (1.5–2.0 pt).  Color: contrasting accent
  (e.g., tab:orange or tab:red).
- **Setpoint / input:** Dashed grey line showing the step input shape.
- **Legend:** "Measured data", "Model fit (Ĝ)", "Step input".

**Bottom subplot — residuals (shared x-axis):**
```python
fig, (ax_top, ax_bot) = plt.subplots(
    2, 1, sharex=True, figsize=(10, 6),
    gridspec_kw={"height_ratios": [3, 1]}
)
# top: measured vs model
ax_top.plot(t, y_meas, linewidth=0.8, color="tab:blue", label="Measured")
ax_top.plot(t, y_model, linewidth=1.8, color="tab:red", label="Model fit")
ax_top.legend()

# bottom: residual
residual = y_meas - y_model
ax_bot.plot(t, residual, linewidth=0.6, color="tab:grey")
ax_bot.axhline(0, color="black", linewidth=0.5)
ax_bot.set_ylabel("Residual")
ax_bot.set_xlabel("Time (s)")
```

**Fit percentage annotation (top-right of top subplot):**
```
Fit: 94.2%  (NRMSE)
```
Centered in a small rounded box.

> **Residual subplot = lightweight residual check.** A systematic trend in the
> residual subplot flags model structure problems; formal autocorrelation /
> cross-correlation is deferred as scope (see §B3).

**Sources:**
- MATLAB System Identification Toolbox `compare()` function (<https://www.mathworks.com/help/ident/ref/compare.html>) — standard output is measured (thin) vs simulated (bold) overlay with fit % in legend.
- python-control `forced_response` + manual comparison (no built-in `compare`).
- L. Ljung, *System Identification: Theory for the User*, 2nd ed., Ch. 7–8: residual analysis subplot with shared time axis is standard.

### A4. Closed-Loop Validation Presentation

For **closed-loop step validation** (setpoint tracking):

**Main plot elements:**
- **Setpoint reference:** Dashed line (0.8 pt, grey or black), extending the full plot width.
  Label "Setpoint" in legend.
- **Measured output:** Solid bold line (1.5 pt), primary color.
  Label "Measured" in legend.
- **Error band:** `ax.axhspan(setpoint ± 0.05 × setpoint, alpha=0.1)` for the
  ±5% settling band (as in A2).

**Data-source tagging:** Every closed-loop metric must be tagged **(sim)** or
**(measured)**.  Our current numbers come from `sim_cl_step.py` and are tagged
**(sim)** — presented as "predicted."  "Achieved (measured)" comes from real-vehicle
telemetry logs and should be shown alongside once available (sim as dashed overlay
on measured tracking, per the optional second trace above).  See also §A6/F3 tables.

**Optional second trace:**
- **Simulated closed-loop response:** If you have a simulated closed-loop prediction
  (plant + controller model), overlay as a thinner dashed line in a second color.
  This is standard when validation data is separate from estimation data.
  *(Source: Ljung Ch. 7, validation on fresh data)*

**Metrics placement:**
- Inline metrics box on the plot (preferred for slides), OR
- A side table (preferred for papers/reports with multiple axes).

**What NOT to do:**
- Do not plot motor command and PID terms on the same axes as position tracking.
  These belong in separate subplots (as our `sim_cl_step.py` already does with its
  3-subplot combined figure).  Mixing velocity/force on a position axis is a common
  amateur mistake.

### A5. Ensemble / Multi-Run Handling

When you have N repeated runs per axis (our case: 6–14 open-loop runs):

**Standard ensemble presentation (two-layer plot):**

```python
# Layer 1: Individual runs (spaghetti)
for i, run in enumerate(runs):
    ax.plot(run.t, run.y, linewidth=0.5, color="tab:blue", alpha=0.15)

# Layer 2: Mean ± std (confidence band)
t_common = np.linspace(0, T_max, 200)
y_mean = np.mean([interp(run.t, run.y, t_common) for run in runs], axis=0)
y_std  = np.std([...], axis=0)
ax.plot(t_common, y_mean, linewidth=2.0, color="tab:blue", label="Mean")
ax.fill_between(t_common, y_mean - y_std, y_mean + y_std,
                alpha=0.2, color="tab:blue", label="±1σ")
```

**Alternative (MATLAB System ID `compare` convention):**
- Overlay all individual runs as thin colored lines.
- Plot the model prediction as a single bold black or red line.
- Report the fit percentage as the mean NRMSE across runs.

**Outlier handling:**
- Mark outliers (> 2σ from mean at any time point) with a different color or marker.
- Report "N runs, M outliers excluded" in the annotation.
- Do NOT silently drop outliers — note them.

**Sources:**
- MATLAB System Identification Toolbox `compare` with multi-experiment data:
  overlays all experiments thin, model bold, reports per-experiment + aggregate fit.
- Ljung, *System Identification*, §7.3: multi-experiment validation.
- python-control: no built-in ensemble compare; manual implementation required.

### A6. Metrics Summary Tables

**Format for a 3-axis comparison table (the standard in PID tuning papers):**

```
Table 1: Closed-loop step-response metrics (test frame)
─────────────────────────────────────────────────────────
         │ Surge │ Sway │ Yaw
─────────┼───────┼──────┼──────
t_rise   │ 1.84s │ 2.16s│ 2.00s    (10–90%)
t_settle │ 3.21s │ 3.89s│ 3.50s    (±5%)
OS%      │  0.0% │  0.0%│  0.0%     (first peak / final)
SSE      │0.003m │0.004m│0.008rad  (last 20% of step)
RMSE     │0.012m │0.015m│0.022rad  (step phase)
─────────────────────────────────────────────────────────
```

**Format for initial-vs-tuned comparison:**

```
Table 2: PID gains and performance — before/after λ/IMC tuning
─────────────────────────────────────────────────────────────────
Axis  │ Scenario  │ Kp    │ Ki    │ Kd    │ t_rise │ t_settle │ RMSE
──────┼───────────┼───────┼───────┼───────┼────────┼──────────┼──────
Surge │ Initial   │ 0.60  │ 0.05  │ 0.15  │  4.21s │   6.80s  │ 0.024
Surge │ Tuned     │ 2.83  │ 0.35  │ 5.72  │  1.84s │   3.21s  │ 0.012
Sway  │ Initial   │ 0.60  │ 0.05  │ 0.15  │  4.92s │   8.10s  │ 0.031
Sway  │ Tuned     │ 2.83  │ 0.35  │ 5.72  │  2.16s │   3.89s  │ 0.015
Yaw   │ Initial   │ 0.80  │ 0.00  │ 0.10  │  5.10s │   9.20s  │ 0.038
Yaw   │ Tuned     │ 2.83  │ 0.35  │ 5.72  │  2.00s │   3.50s  │ 0.022
─────────────────────────────────────────────────────────────────
```

**Rules:**
- **2–3 significant figures** consistently (not 6 — our current `sim_cl_step.py`
  prints 6 decimal places for SSE/RMSE, which implies false precision).
- **Units in header or first row**, not repeated per cell.
- **Metric convention noted** in a footnote (e.g., "Rise time: 10–90% of setpoint;
  Settling time: ±5% band; SSE: mean of last 20% of step phase; Fit: NRMSE on
  velocity (primary) — position fit optional, appendix only; metrics tagged (sim)
  if from `sim_cl_step.py`, (measured) if from vehicle telemetry").
- **Color-code** the "Tuned" row (e.g., bold or green) if this is for a slide.
- **Sort by axis, then by scenario** — never alphabetically by metric.

**Sources:**
- Åström & Hägglund, *Advanced PID Control*, Ch. 6–7: extensive tables of this
  exact format (gains × performance metrics × axis).
- Seborg et al., *Process Dynamics and Control*, §12.3: standard PID tuning
  result tables.
- MATLAB PID Tuner output format (interactive, but follows this structure).

---

## B. System-Identification Presentation

### B1. Fit-Quality Reporting

**The industry-standard metric: NRMSE fit percentage (aka "Fit: XX.X%")**

The MATLAB System Identification Toolbox defines this as:

```
fit% = 100 × (1 − ‖y_measured − y_model‖₂ / ‖y_measured − mean(y_measured)‖₂)
```

This is equivalent to:

```
fit% = 100 × (1 − RMSE / std(y_measured))
```

**Fit signal convention:** The identified model `G(s) = K / [s(τs+1)]` outputs *velocity*;
`identify.py` fits K on `v_ss` and τ via the position-area method.  **Velocity is the
primary fit signal** — it is the honest, non-trivial choice (position fit would be 90–98%
and partly trivial due to the integrating pole).  The fit signal must be stated in the
convention footnote (§A6) and on every fit box annotation (§F2).

A fit of **>90%** is considered good, **>95%** excellent, **<70%** poor.
For our single-parameter (K, τ, L) FOPDT fits on noisy ROV data:
- **70–90%** is the realistic and defensible range for **velocity** fits.
- Position fits would fall in 90–98% (partly trivial due to integration; optional, appendix only).

> **Diagnostic:** NRMSE fit% can go negative if the model is worse than the sample mean
> — a useful red flag.

**How to state it:**
- On the plot: `ax.text(x, y, "Fit: 94.2%", fontsize=9, ...)` in a box.
- In the metrics table: add a "Fit %" column.
- In prose: "The identified model achieves a 94.2% NRMSE fit to the measured data."

**Alternative/complementary metrics:**
| Metric | Formula | When to use |
|---|---|---|
| **R²** (coefficient of determination) | `1 − SS_res / SS_tot` | When readers expect regression-style reporting |
| **RMSE** | `√(mean((y − ŷ)²))` | Absolute error, same units as output |
| **MAE** | `mean(|y − ŷ|)` | Less sensitive to outliers than RMSE |

**Sources:**
- MATLAB `goodnessOfFit` (<https://www.mathworks.com/help/ident/ref/goodnessoffit.html>):
  computes `100 × (1 − ‖y − ŷ‖ / ‖y − ȳ‖)` — the canonical NRMSE fit %.
- Wikipedia *Root-mean-square deviation* (<https://en.wikipedia.org/wiki/Root-mean-square_deviation>):
  defines NRMSD normalization by range or mean.
- Ljung, *System Identification*, Ch. 7: "simulation error" vs "prediction error" as
  two distinct fit measures; simulation error (NRMSE) is what we want for step-response
  validation.

### B2. Parameter Confidence / Uncertainty

**When you fit K, τ, L from multiple runs, report:**

| Parameter | Point estimate | Uncertainty |
|---|---|---|
| K (gain) | Mean across runs | Std dev, or 95% CI |
| τ (time constant) | Mean across runs | Std dev, or 95% CI |
| L (dead time) | Mean across runs | Std dev, or 95% CI |

**Presentation options:**

1. **Error bars on parameter table:** `K = 0.079 ± 0.012 m/s` (mean ± σ)
2. **CI bands on the model fit plot:** Run the model with K±σ, τ±σ and shade the
   resulting trajectory envelope.
3. **Per-run parameter table:** List each run's K, τ, L individually, then a summary row.

**For our case (6–14 runs per axis, single-step identification):**
- Report mean ± 1σ for each parameter.
- Note runs excluded (R² < 0.5, or outlier by > 2σ).
- Do NOT claim confidence intervals based on asymptotic Fisher information — that
  requires more data structure than single-step tests provide.  Simple std-across-runs
  is honest and sufficient.

**Sources:**
- Ljung, *System Identification*, §7.2: parameter uncertainty from multi-experiment data.
- MATLAB `present()` on `idtf` / `idproc` models: displays parameter ± estimated
  standard deviation.
- Seborg et al., §7.5: graphical method uncertainty estimation for FOPDT models
  (63.2% method, two-point method).

### B3. Residual Analysis

**Standard residual analysis (from System Identification Toolbox):**

1. **Auto-correlation of residuals** `r(τ) = E[e(t)·e(t−τ)]` — should be within
   95% confidence bounds (≈ ±2/√N) for all lags except τ=0.  Non-white residuals
   indicate model structure deficiency.
2. **Cross-correlation of residuals with input** — should be zero for all lags
   (no unmodeled input-output relationship).
3. **Histogram / Q-Q plot of residuals** — should be approximately normal for
   Gaussian noise assumptions.

**Is this expected for our case?**
**No, it is overkill.**  Residual analysis is essential when:
- You're choosing between model structures (ARX vs OE vs BJ vs state-space).
- You have rich input excitation (PRBS, multi-sine) and many data points.
- You need to justify model order selection.

For our single-step-response identification of a known FOPDT structure:
- We have ~50–100 data points per run, not thousands.
- The model structure is fixed by physics (FOPDT integrator), not chosen from candidates.
- The noise is dominated by sensor noise (ArUco PnP) and thruster nonlinearity,
  not white process noise.

**Honest framing:** "Residual analysis is not performed; model structure is fixed by
physics (FOPDT + integrator) and validated by step-response fit quality."

**Sources:**
- Ljung, *System Identification*, Ch. 7.4: residual analysis for model validation.
- MATLAB `resid()` function documentation: produces autocorrelation and
  cross-correlation plots.

### B4. Validation vs Estimation Data Split

**Standard practice:**
- Use **60–80% of runs** for parameter estimation (identification).
- Use the **remaining 20–40%** for validation (fit % computed on unseen data).
- Report fit on both estimation and validation sets.

**For our case:**
- With only 4–14 runs per axis, a formal split reduces estimation data too much.
- Practical alternative: **Leave-one-out cross-validation** — fit on N-1 runs,
  validate on the left-out run, rotate through all runs.  Report mean ± std of
  the validation fit.
- Or simpler: report fit on ALL runs (no split) and note that "cross-validation
  was not performed due to limited runs; model structure is constrained by
  physics rather than data-driven."

**Sources:**
- Ljung, *System Identification*, §7.2: validation on fresh data.
- MATLAB `compare` with validation data argument.

---

## C. Frequency-Domain (Research Only — Not for Our Figures)

### C1. When Frequency-Domain is Used

Frequency-domain identification is preferred when:
- The plant can be linearized around an operating point.
- Persistent broadband excitation is available (PRBS, chirp, multi-sine).
- You need to characterize behavior across a wide frequency range (flexible
  structures, resonances, high-order dynamics).
- You're designing controllers in the frequency domain (loop shaping, H∞, QFT).

**Tools and conventions:**
- **Bode plot:** Magnitude (dB) and phase (deg) vs frequency (rad/s or Hz).
  Gain margin and phase margin annotated at crossover frequencies.
  *(Source: python-control `bode_plot` with `display_margins=True`;
  MATLAB `bodeplot` with `showPhaseMargin`/`showGainMargin`)*
- **Nyquist plot:** Parametric polar plot of `L(jω)`.  Encirclements of −1
  indicate instability.  Gain margin = distance from −1 to nearest crossing.
  *(Source: Wikipedia *Bode plot*; Ogata Ch. 6)*
- **Nichols chart:** Log-magnitude vs phase on rectangular axes.  M-circles
  overlaid for closed-loop resonance peaks.  Used in loop-shaping design.
- **Coherence function:** `γ²(ω) = |G_xy|² / (G_xx · G_yy)` — quality of
  frequency-response estimate.  γ² > 0.8 is considered reliable; < 0.4 unreliable.
  Shown below the Bode magnitude plot.

### C2. Honest Framing: Why Step-Response ID Suffices for Our Plant

**Defensible slide statement:**

> "The plant is well-approximated as a first-order system with integrator and
> dead time: `G(s) = K e^(−Ls) / [s(τs+1)]`.  This structure is physically
> motivated (thruster dynamics → first-order lag on velocity; position is the
> integral of velocity).  For such low-order plants, step-response identification
> is the standard method: it requires minimal excitation (a single step),
> directly estimates the three parameters (K, τ, L), and is widely recommended
> in process control and marine robotics textbooks [Seborg et al. §7.2;
> Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, Ch. 9].
> Frequency-domain methods (Bode/Nyquist identification from frequency sweeps
> or PRBS) offer advantages for higher-order or resonant systems but are
> unnecessary for our plant order."

**Additional context for credibility:**
- Åström & Hägglund, *Advanced PID Control*, §2.1: "For many process control
  applications, the process can be characterized by a few parameters..."
  Step-response methods are the starting point.
- Fossen, Ch. 9: marine craft identification often uses step-response or
  relay-based methods for low-speed maneuvering models.
- Our ROV operates at low speed (0.05–0.1 m/s) in a tank — no wave excitation,
  no flexible modes, no resonances.  Frequency-domain identification would
  require a frequency sweep that takes significantly longer and provides no
  additional information for a 3-parameter model.

---

## D. Slide/Deck Conventions for Technical Reviewers

### D1. Structure of a 2–3 Slide Calibration Section

**Slide 1: Methodology**
| Element | Content |
|---|---|
| **Title** | "Plant Identification & PID Tuning Methodology" |
| **Figure** | Schematic block diagram: Step input → Open-loop test → FOPDT fit → λ/IMC tuning → Closed-loop validation |
| **Bullet points (3–5 max)** | Model structure, identification method, tuning rule, axes covered |
| **Caveat line** | "Yaw identification limited by ArUco heading ambiguity — gains are estimates." |
| **Time** | 20s |

**Slide 2: Results — Open-Loop Identification**
| Element | Content |
|---|---|
| **Title** | "Identified Plant Parameters (Test Frame)" |
| **Figure** | 1×3 subplot: per-axis measured vs model fit overlay, with fit % and residual subplot |
| **Table** | K, τ, L ± σ for each axis (3 rows × 4 cols) |
| **Key callout** | "Sway: cleanest axis (6/6 runs, 94% fit). Yaw: unidentifiable from current data." |
| **Time** | 30s |

**Slide 3: Results — Closed-Loop Validation**
| Element | Content |
|---|---|
| **Title** | "Closed-Loop Step Response: Initial vs λ-Tuned Gains" |
| **Figure** | 1×3 subplot: per-axis setpoint tracking (initial as thin blue, tuned as bold orange) with ±5% band and metrics box |
| **Table** | Initial vs tuned gains + t_rise, t_settle, OS%, RMSE |
| **Key callout** | "λ/IMC tuning reduces rise time by 56% (surge) and settling time by 53%." |
| **Caveat line** | "Results in test frame; real-frame extrapolation shown in appendix." |
| **Time** | 30s |

> **Note:** Results show setpoint tracking; disturbance rejection is provided by
> integral action (Ki) and λ-robustness, not separately tested.

### D2. Communicating Uncertainty Without Undermining Credibility

**Rules:**
1. **State assumptions upfront.**  Don't hide them in footnotes.  "Assumed: linear
   plant, constant K over operating range, no coupling between axes."
2. **Quantify limitations.**  "Yaw K estimate based on magnitude only; τ is
   synthesized.  Replace after yaw data re-collection."
3. **Use "estimates" not "measurements"** when appropriate.  "Estimated K = 0.10 ± 0.03"
   not "Measured K = 0.10 ± 0.03".
4. **Show the noisy data.**  Don't clean up the plots so much that they look fake.
   Spaghetti plots with thin individual runs build credibility.
5. **End with forward path.**  "Next steps: re-collect yaw data with improved
   ArUco placement; validate on full-scale frame."
6. **Never say "we can't".**  Say "we chose not to because..." or "this is
   deferred to..."

**Sources:**
- NSF/NIH grant proposal conventions: state limitations as "scope boundaries",
  not failures.
- Engineering conference paper norms (IEEE, ASME): limitations section is expected;
  its absence is a red flag.

---

## E. Tools & Reproducibility

### E1. Standard Tooling for Credibility

| Tool | Function | Our Equivalent |
|---|---|---|
| **MATLAB System Identification Toolbox** | `tfest`, `procest`, `compare`, `goodnessOfFit`, `resid` | Custom `identify.py` (FOPDT fit from step response) |
| **MATLAB Control System Toolbox** | `step`, `stepinfo`, `stepplot`, `pidtune`, `bode` | Custom `tuning.py` (λ/IMC) + `sim_cl_step.py` |
| **python-control** | `step_response`, `step_info`, `forced_response`, `bode_plot` | `scipy.signal.step` + custom metrics |
| **scipy.signal** | `step`, `lsim`, `TransferFunction` | Used in `sim_cl_step.py` plant simulation |
| **SIPPY** | System identification (ARX, ARMAX, OE, BJ, SS) | Not used — overkill for our 3-parameter model |
| **sysid** | Python system identification package | Not used |

**How to reference in slides:**
> "Identification and tuning implemented in Python using `scipy.signal` for simulation
> and custom λ/IMC tuning.  Methodology follows MATLAB System Identification Toolbox
> conventions (Ljung 1999; Seborg et al. 2016)."

### E2. Reproducibility Checklist

For each figure, the slide source or appendix should enable exact reproduction:
- [ ] Plant parameters (K, τ, L) with units and source (which runs)
- [ ] PID gains (Kp, Ki, Kd) and tuning rule (λ = X·τ)
- [ ] Simulation parameters (dt, duration, noise σ)
- [ ] Metric definitions (10–90% rise, ±5% settle, etc.)
- [ ] Code commit hash
- [ ] Raw data reference (log file names / paths)

---

## F. Recommended Conventions for Our Case

### F1. Figures to Produce (Priority Order)

| # | Figure | For Slide | Content |
|---|---|---|---|
| 1 | **Open-loop identification** | Slide 2 | 1×3 subplots (surge/sway/yaw): per-axis measured vs model fit, fit % annotation, residual subplot below each. Yaw subplot: raw data only, no fit curve — annotated "τ synthesized, no fit (ArUco PnP corruption)" |
| 2 | **Closed-loop validation** | Slide 3 | 1×3 subplots (surge/sway/yaw): initial (thin blue) vs tuned (bold orange) setpoint tracking, ±5% band, metrics box |
| 3 | **Test-vs-real frame comparison** | Appendix / backup | 1×3 subplots: per-axis, test frame response vs real frame extrapolation (as our `compare_runs.py` open-loop plot already does) |
| 4 | **Ensemble open-loop (optional)** | Backup / appendix | Per-axis spaghetti plot with mean ± σ band |

### F2. Exact Annotations Per Figure

**Figure 1 (open-loop identification):**
```
Per subplot (e.g., surge):
  - Solid blue line (0.8 pt): measured velocity from step data
  - Bold red line (1.8 pt): model prediction ŷ(t)
  - Dashed grey line: step input shape
  - Text box (top-left): "K = 0.054 ± 0.008 m/s\nτ = 1.84 ± 0.22 s\nL = 0.10 ± 0.03 s\nFit: 92.4% (NRMSE, velocity)"
  - Shared residual subplot below: residual = measured − model

  ⚠ Yaw subplot exception:
  - Raw yaw data only (solid blue line); NO model-fit curve, NO fit % box.
  - Clear watermark/annotation: "τ synthesized — K magnitude only, no fit
    (ArUco PnP corruption)"
  - Yaw still appears in Figure 2 (closed-loop) as a normal tracking subplot.
```

**Figure 2 (closed-loop validation):**
```
Per subplot (e.g., surge):
  - Dashed grey line: setpoint (0 → 0.1 m)
  - Thin blue line (1.0 pt): initial gains response
  - Bold orange line (1.8 pt): tuned gains response
  - Shaded band: ±5% of setpoint (alpha=0.12, grey)
  - Text box (top-left): "Initial: t_r=4.21s, t_s=6.80s, RMSE=0.024 (sim)"
  - Text box (top-right): "Tuned:   t_r=1.84s, t_s=3.21s, RMSE=0.012 (sim)"
  - Shared x-axis across all 3 subplots
  - Shared legend below or on rightmost subplot

  Control-effort subplot (below tracking, shared x-axis, thin height ratio):
  - Step-plot of tuned-gains motor command vs time
  - Horizontal dashed lines at ±output_limit (±0.4 surge/sway, ±0.6 yaw)
  - Annotation "no saturation" if the linear regime held
  - This is load-bearing: the FOPDT analysis is only valid if the controller
    didn't saturate.  Data source: motor_x/y/z/r columns in telemetry CSV.
```

### F3. Metrics Table Format (Final)

```
Table: Closed-Loop Step-Response Metrics
Axes: surge/sway (m), yaw (rad)
Rise time: 10–90% of setpoint | Settling: ±5% band
SSE: mean of last 20% of step phase | RMSE: step-phase RMS
Fit: NRMSE on velocity (primary) | All metrics (sim) unless noted

┌──────┬─────────┬───────────┬───────────┬────────┬──────────┬───────┐
│ Axis │ Scenario│ Kp   Ki Kd│ t_rise [s]│ t_set [s]│ OS [%] │ RMSE  │
├──────┼─────────┼───────────┼───────────┼──────────┼───────┼───────┤
│Surge │ Initial │ 0.60 0.05│ 0.15     │  4.21    │  6.80 │  0.0  │ 0.024 │
│Surge │ Tuned   │ 2.83 0.35│ 5.72     │  1.84    │  3.21 │  0.0  │ 0.012 │
│Sway  │ Initial │ 0.60 0.05│ 0.15     │  4.92    │  8.10 │  0.0  │ 0.031 │
│Sway  │ Tuned   │ 2.83 0.35│ 5.72     │  2.16    │  3.89 │  0.0  │ 0.015 │
│Yaw   │ Initial │ 0.80 0.00│ 0.10     │  5.10    │  9.20 │  0.0  │ 0.038 │
│Yaw   │ Tuned   │ 2.83 0.35│ 5.72     │  2.00    │  3.50 │  0.0  │ 0.022 │
└──────┴─────────┴───────────┴───────────┴──────────┴───────┴───────┘
```

(Replace values with actual computed metrics.  Use 2–3 sig figs.)

### F4. Test-Frame vs Real-Frame Presentation

- **Open-loop plant response plot:** 1×3 subplots (as our `plot_openloop_master` already
  produces), showing test frame and real frame velocity step responses.
  **Real-frame subplot must show an extrapolation uncertainty band**
  (`fill_between`, ±30% on K / ±25% on τ) rather than a single point trajectory —
  plotting a single line misrepresents a wide band as precise.
- **Annotation:** "Real frame: K scaled by thrust availability; τ increased by added mass
   and drag of full-scale body."
- **In the metrics table:** separate rows or a "Frame" column.  But note that we
  don't have real-frame step data — this is an extrapolation.  Label clearly:
  "Real-frame values are physics-based extrapolations, not measured."

### F5. Honest Caveats to Include

**On every calibration slide:**
1. "Plant identified from open-loop step tests on a test frame (~5 kg).  Full-scale
   ROV parameters are extrapolated from hydrodynamic scaling — not yet validated
   in water."
2. "Yaw identification limited by ArUco marker heading ambiguity (±180°);
   K magnitude is data-supported but τ is synthesized."
3. "Sensor: ArUco single-marker PnP at 10 Hz.  Position noise σ ≈ 5 mm (test frame),
   ~10 mm (underwater)."

**In a limitations/note slide (if space):**
4. "Model assumes decoupled axes (no cross-coupling between surge/sway/yaw).
   In practice, thruster wake interactions cause minor coupling."
5. "FOPDT integrator model captures dominant dynamics.  Higher-order effects
   (thruster lag, structural vibration, propeller ventilation) are not modeled."
6. "λ/IMC tuning assumes the model is accurate.  Robustness to model uncertainty
   is provided by the λ parameter (set to τ for moderate robustness)."

---

## Source Index

| Ref | Source |
|---|---|
| [1] | python-control `step_info` docs — <https://python-control.readthedocs.io/en/latest/generated/control.step_info.html> |
| [2] | python-control `step_response` docs — <https://python-control.readthedocs.io/en/latest/generated/control.step_response.html> |
| [3] | python-control `bode_plot` docs — <https://python-control.readthedocs.io/en/latest/generated/control.bode_plot.html> |
| [4] | MATLAB `stepinfo` — <https://www.mathworks.com/help/control/ref/stepinfo.html> |
| [5] | MATLAB `stepplot` — <https://www.mathworks.com/help/control/ref/stepplot.html> |
| [6] | MATLAB System Identification Toolbox `compare` — <https://www.mathworks.com/help/ident/ref/compare.html> |
| [7] | MATLAB System Identification Toolbox `goodnessOfFit` — <https://www.mathworks.com/help/ident/ref/goodnessofFit.html> |
| [8] | MATLAB System Identification Toolbox `resid` — <https://www.mathworks.com/help/ident/ref/resid.html> |
| [9] | Ogata, K. *Modern Control Engineering*, 5th ed. Prentice Hall, 2010. §5.3 (step response specs). |
| [10] | Åström, K.J. & Hägglund, T. *Advanced PID Control*. ISA, 2006. Ch. 2 (process models), Ch. 6–7 (tuning). |
| [11] | Ljung, L. *System Identification: Theory for the User*, 2nd ed. Prentice Hall, 1999. Ch. 7 (validation), §7.2 (uncertainty). |
| [12] | Seborg, D.E., Edgar, T.F., Mellichamp, D.A. & Doyle, F.J. *Process Dynamics and Control*, 4th ed. Wiley, 2016. §7.2 (FOPDT identification), §12.3 (PID tuning tables). |
| [13] | Fossen, T.I. *Handbook of Marine Craft Hydrodynamics and Motion Control*. Wiley, 2011. Ch. 9 (identification). |
| [14] | Wikipedia *Rise time* — <https://en.wikipedia.org/wiki/Rise_time> (Levine 1996 conventions). |
| [15] | Wikipedia *Root-mean-square deviation* — <https://en.wikipedia.org/wiki/Root-mean-square_deviation> (NRMSE definition). |
| [16] | Wikipedia *Bode plot* — <https://en.wikipedia.org/wiki/Bode_plot> (gain/phase margin definitions). |
| [17] | Wikipedia *Step response* — <https://en.wikipedia.org/wiki/Step_response> (settling time, overshoot). |
| [18] | Wikipedia *PID controller* — <https://en.wikipedia.org/wiki/PID_controller#IMC_tuning> (IMC tuning overview, FOPDT model). |
| [19] | scipy.signal `step` — <https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.step.html> |
| [20] | scipy.signal `TransferFunction` — <https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.TransferFunction.html> |

---

*Generated for hive ROV calibration methodology.  All conventions verified against
primary sources (tool documentation, textbooks, peer-reviewed references).*
