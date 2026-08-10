# Vision Subsystem Test Plan: Detection Under Degraded Visibility (Turbidity × Lighting)
**(1st draft — full 9-condition factorial)**

## 1. Objective

This test plan extends the validated Camera Detection Limit characterisation (Test 2) to degraded-visibility conditions. The objective is to determine whether HIVE's vision subsystem continues to meet its detection and positioning requirements as water turbidity and ambient lighting degrade from ideal to worst-case conditions representative of the operating site, and to identify the failure threshold if performance drops below requirement.

Unlike Test 2, which characterised z-axis (depth) estimation accuracy, this test evaluates the metric that actually matters for guidance: marker detection reliability and lateral (x-y) position accuracy, since these are what drive HIVE's thruster corrections during terminal approach.

## 2. Reference Functional Requirements

| Requirement | Quantitative Target |
|---|---|
| Detect and track the AUV | ≥95% true-positive rate; ≤5% false-positive rate |
| Estimate horizontal (x-y) position | Absolute x/y error ≤10 cm for ≥95% of samples |
| Maintain AUV within capture envelope | ±10 cm lateral, ±15° relative yaw, sustained ≥10 s |

Operating envelope (from SDR): recovery depth 2–5 m submerged; disturbance robustness target ≥85% nominal, ≥70% under the validated degraded-condition envelope. This test plan treats these as the ground truth for defining a "failure threshold" under turbidity/lighting degradation.

## 3. Test Variables

### 3.1 Turbidity Levels

Absolute NTU is not independently measurable with the equipment available (no calibrated turbidimeter). Turbidity conditions are therefore prepared as targeted approximations, anchored to one real field measurement, and confirmed only via qualitative visual comparison — consistent with the historical visual-extinction principle (Jackson Candle / JTU method), which is itself only reliable down to ~25 JTU. Below this, values are qualitative by necessity, which is stated explicitly rather than disguised with false-precision numbers.

| Level | Target NTU | Basis / Preparation |
|---|---|---|
| Clear (control) | ~0 NTU | Tap / filtered water |
| Site-representative | ~10 NTU | Field-collected sea sample, ~2 m depth, test site |
| Medium | ~20–60 NTU (target) | Tap water diluted with milk; literature-informed moderate/post-rain range |
| High | ~100–150 NTU (target) | Tap water + additional milk; literature-informed high-sediment-event range |

**Preparation and verification method:**
1. Collect a reference sea-water sample from the test site (weighted-bottle method, ~2 m depth).
2. Seal a portion of each prepared condition in a labelled ziplock bag with a Secchi/high-contrast pattern card behind it.
3. Visually compare each bag against the site reference sample under consistent lighting to confirm relative ordering (clear < site < medium < high).
4. Adjust milk concentration until each condition is qualitatively distinguishable from its neighbours; record the milk-to-water ratio used, so conditions are reproducible across test sessions even without an absolute NTU reading.

**Limitation:** reported NTU values for Medium and High are design targets based on literature for Singapore coastal waters, not instrument-verified measurements. This is stated explicitly in results reporting rather than presented as measured data.

### 3.2 Lighting Levels

| Level | Target Illuminance | Rationale |
|---|---|---|
| Normal | 100–300 lux | Daytime operation, shallow-to-mid depth, moderately clear water |
| Low | 5–20 lux | Deeper / turbid daytime operation, or overcast-day conditions |
| Worst-case (stretch) | <1 lux | Near-total attenuation; tests sensor noise floor, not routine operating condition |

Measured at the camera position, through the water column (not ambient room light), using a lux meter (dedicated meter or phone light-sensor app). An adjustable lamp positioned consistently above the test tank provides Normal/Low; room lighting is fully removed for the Worst-case condition.

## 4. Test Matrix

Core design: 3 (turbidity) × 3 (lighting) full factorial = 9 conditions.

| Condition # | Turbidity | Lighting |
|---|---|---|
| 1 | Clear | Normal (100–300 lux) |
| 2 | Clear | Low (5–20 lux) |
| 3 | Clear | Worst-case (<1 lux) |
| 4 | Site-representative / Medium | Normal (100–300 lux) |
| 5 | Site-representative / Medium | Low (5–20 lux) |
| 6 | Site-representative / Medium | Worst-case (<1 lux) |
| 7 | High | Normal (100–300 lux) |
| 8 | High | Low (5–20 lux) |
| 9 | High | Worst-case (<1 lux) |

Run order: Condition 1 first (reproduces known-good baseline as a rig sanity check), then progress in increasing difficulty. This surfaces equipment/setup issues before they contaminate worst-case data.

## 5. Spatial Test Positions (x-y × depth)

Test 2 already characterised z-axis accuracy across 20–100 cm. This test instead evaluates lateral (x-y) detection accuracy, sized directly around the ±10 cm capture-envelope requirement.

| Position | Offset (x, y) | Purpose |
|---|---|---|
| Center | (0, 0) | Baseline — marker directly ahead |
| +X | (+10 cm, 0) | Edge of lateral tolerance, right |
| −X | (−10 cm, 0) | Edge of lateral tolerance, left |
| +Y | (0, +10 cm) | Edge of tolerance, vertical/up |
| −Y | (0, −10 cm) | Edge of tolerance, vertical/down |
| Diagonal (optional) | (+15 cm, +15 cm) | Beyond-spec stress point, baseline condition only |

Depths tested: 40 cm (typical approach distance) and 80 cm (near outer working limit), both within the validated ≤110 cm required detection range.

## 6. Test Protocol

### Test Setup

A full 9-condition × 5-position × 2-depth × 3-repeat matrix (270 trials) is not feasible within the allotted test window. Testing is tiered accordingly:

**Tier 1 — Full characterisation (baseline condition only)**
Condition 1 (Clear / Normal) only: all 5 x-y positions × 2 depths × 3 repeats = 30 trials. Establishes the nominal x-y accuracy curve as the reference dataset.

**Tier 2 — Degradation check (remaining 8 conditions)**
Conditions 2–9: Center (0,0) + one edge position (+X) × 1 depth (40 cm) × 3 repeats = 6 trials per condition × 8 conditions = 48 trials. Confirms whether detection/x-y accuracy still holds at the edge of tolerance as conditions degrade, without re-mapping the full envelope each time.

**Total: ~78 trials.**

**Pre-check** before running the full matrix: confirm the ArUco/AprilTag marker remains within the camera's field of view at ±10–15 cm offset at both 40 cm and 80 cm, under Condition 1. A framing/FOV limitation would otherwise be misread as a detection failure.

**<1 lux conditions (3, 6, 9):** treated as a binary pass/fail check at the single most critical operational distance/position, rather than a full sweep — at this illuminance, sensor noise dominates and the priority is establishing whether detection succeeds at all, not fine-grained accuracy.

## 7. Metrics and Success Criteria

**Logged per trial:**
- Detection outcome — true positive / false positive / miss
- X-Y position error — absolute error (cm) between reported and known marker position
- Detection latency — time from marker in-frame to stable position output
- Yaw error (if available from pose estimation)

**Reporting format per condition:**
> "At [turbidity]/[lighting], detection succeeded in X% of trials, with mean x-y error of Y cm, and mean latency of Z ms."

The failure threshold is defined as the first condition (in increasing difficulty order) at which detection rate falls below 95%, or mean x-y error exceeds 10 cm — directly answering the Gantt-chart task of identifying the turbidity/lighting failure threshold.

## 8. Equipment

- Test tank with adjustable lighting (existing rig)
- Camera + housing (existing detection code / setup)
- ArUco/AprilTag markers, mounted for known x-y positioning at 40 cm and 80 cm
- Secchi/high-contrast pattern card, ziplock bags for sample comparison
- Full-fat milk (turbidity simulant), measuring cylinder for dilution ratios
- Lux meter (dedicated or phone app)
- Adjustable lamp, positioned consistently above tank
- Field-collected sea-water reference sample (~2 m depth, test site)

## 9. Limitations and Assumptions

- Turbidity values for Medium and High conditions are design targets based on literature for Singapore coastal waters, not instrument-measured NTU.
- Milk is an optical proxy for suspended sediment: color (white vs. brown/green) and settling behaviour differ from real sediment, which may affect any colour-based detection thresholds.
- Site reference sample was collected at ~2 m depth, the shallow end of HIVE's 2–5 m operating envelope; turbidity nearer the seabed may be higher, which the High condition is intended to bound.
- Reduced repeat counts (Tier 2) trade statistical depth for coverage across all 9 conditions within the available test window; Tier 1 provides the higher-confidence reference dataset.
- <1 lux conditions are evaluated as pass/fail checks rather than full accuracy sweeps.
