# Vision Subsystem Test Plan (Simplified): Detection Under Degraded Visibility
**(2nd / latest draft — reduced 3-condition)**

## 1. Objective

Determine whether HIVE's vision subsystem continues to meet its detection and positioning requirements as water turbidity and lighting degrade from ideal to worst-case conditions representative of the operating site, and identify the approximate condition at which performance drops below requirement.

This is a reduced-scope version of the original 9-condition test plan, adapted for limited test time and manpower. It trades some diagnostic granularity (isolating turbidity vs. lighting separately) for a defensible "works / degrades / fails" story that still satisfies FR traceability.

## 2. Reference Functional Requirements

| Requirement | Quantitative Target |
|---|---|
| Detect and track the AUV | ≥95% true-positive rate; ≤5% false-positive rate |
| Estimate horizontal (x-y) position | Absolute x/y error ≤10 cm for ≥95% of samples |
| Maintain AUV within capture envelope | ±10 cm lateral, ±15° relative yaw, sustained ≥10 s |

## 3. Test Conditions

Reduced from 9 to 3 combined conditions (turbidity and lighting varied together, not independently):

| Condition | Turbidity | Lighting | Purpose |
|---|---|---|---|
| 1 — Baseline | Clear (~0 NTU, tap water) | Normal (100–300 lux) | Known-good reference / rig sanity check |
| 2 — Site-representative | ~10 NTU (field-collected sea sample, ~2 m depth) | Normal (100–300 lux) | Realistic operating case |
| 3 — Worst-case combined | High (~100–150 NTU, tap water + milk) | Low (5–20 lux) | Combined stress test — bounds the failure threshold |

Turbidity preparation and verification method (visual comparison against site reference sample, milk-to-water ratio recorded) is unchanged from the original plan. See Section 8, Limitations.

## 4. Spatial Test Position

Center only (0, 0), single depth of 40 cm, for all 3 conditions.

The ±10 cm lateral tolerance envelope is not re-tested at each condition — it was already validated at baseline in prior testing (Test 2 / Tier 1 characterisation). This test plan checks whether detection itself survives degradation, not whether spatial accuracy holds at the edge of tolerance under every condition.

## 5. Test Protocol

Automated continuous logging, not discrete manual trials.

For each of the 3 conditions:
1. Set up the condition (swap water / turbidity level, set lighting level, confirm marker at center position, 40 cm).
2. Run the detection script continuously for 60 seconds (~150–300 frames depending on frame rate).
3. Script logs every frame automatically — no per-frame manual recording.
4. Move to next condition.

Total hands-on test time: ~3 setup changes, roughly 30–45 minutes including setup, instead of manually executing a discrete trial matrix.

Run order: Condition 1 first (baseline sanity check), then 2, then 3 (increasing difficulty), so equipment/setup issues surface before they contaminate worst-case data.

## 6. Metrics and Success Criteria

**Logged per frame (via detection script):**
- Detection outcome (detected / not detected this frame)
- X-Y position (from tvec, pose estimate vs. known physical position)

Dropped from the original plan to reduce scope: yaw error, reprojection error, per-frame latency tracking, brightness/blur diagnostics. These are useful extras but not required to satisfy the FR table — can be added back later if time allows.

**Per-condition summary (computed from the frame log):**
- Detection rate = % of frames with successful detection
- Mean x-y error = average absolute error (cm) vs. known marker position, across detected frames

**Failure threshold:** the first condition (in increasing difficulty order) at which detection rate falls below 95%, or mean x-y error exceeds 10 cm.

**Reporting format:**
> "At [condition], detection succeeded in X% of frames, with mean x-y error of Y cm."

## 7. Equipment

- Test tank with adjustable lighting (existing rig)
- Camera + housing (existing detection code/setup)
- ArUco/AprilTag marker, mounted at known center position, 40 cm from camera
- Full-fat milk (turbidity simulant), measuring cylinder for dilution ratio
- Lux meter (dedicated or phone app), adjustable lamp
- Field-collected sea-water reference sample (~2m depth, test site) for Condition 2
- Automated logging script (frame-by-frame CSV output)

## 8. Limitations and Assumptions

- Turbidity values are design targets based on literature and visual comparison to a field-collected reference sample, not instrument-measured NTU (no calibrated turbidimeter available).
- Milk is an optical proxy for suspended sediment; colour and settling behaviour differ from real sediment.
- Reduced condition count (3 instead of 9) and single spatial position trade diagnostic separation of turbidity vs. lighting effects, and edge-of-envelope accuracy under degradation, for feasibility within available test time and manpower. This plan establishes whether detection survives degradation and roughly where it fails, not a fully separated turbidity/lighting sensitivity map.
- If time permits after this reduced plan, Condition 3 can be split back into separate turbidity-only and lighting-only runs to isolate which factor drives any observed failure.
