# Improvement Plan: hive

*Catalog of improvements possible within camera/, integration/, and Pixhawk/ArduSub
configuration as separate domains. No merging of camera and integration code.
Generated from a deepwork research session (Pixhawk/ArduSub deep dive + integration
code audit + Oracle architectural review).*

> **Implementation status (branch `improvements`):** All code-implementable items
> are done. Phase 1 (MAVLink param management + startup verification + web UI),
> Phase 2 (10 integration safety/robustness items), Phase 3 (camera perf +
> consolidation). Each phase passed an Oracle architectural gate review. Items
> requiring physical testing (I4 camera-to-body transform, I5 MANUAL stick-freeze,
> P1/P2 frame config bench verification) are documented but cannot be code-tested
> without the rig. See `.slim/deepwork/implementation.md` for the full log.

---

## Confirmed architecture context

| Fact | Source |
|---|---|
| Pixhawk + ArduSub over MAVLink is the canonical architecture | User-confirmed |
| 4× T500 horizontal thrusters, NO vertical thrust | User-confirmed |
| Depth set by ballast/crane, not actively controlled | Physical constraint |
| Pi PID at 10 Hz sends MANUAL_CONTROL body-frame sticks | `integration/hardware.py:48` |
| ArduSub runs in MANUAL mode (zero PID loops active) | Code + ArduSub docs |
| Gate 4 target: closed-loop horizontal station-keeping over ArUco marker | README |

**What this means:** the rig can control surge, sway, and yaw. It cannot control
heave/depth or roll/pitch (no vertical thrusters). ArduSub is currently used as a
PWM pass-through and motor mixer only, with no closed-loop control on the Pixhawk
side. All closed-loop control runs on the Pi in Python at 10 Hz.

---

## Domain 1: Camera (standalone)

These improvements live entirely in `camera/` and do not touch `integration/`.

### Performance

| ID | Improvement | Effort | Why |
|---|---|---|---|
| C1 | **Profile detection latency first** | 1h | Wrap `capture_array()` and `detectMarkers()` with `time.perf_counter()`. Print rolling 1s average. Cannot optimize what you have not measured. The `tuningv2` branch reportedly has `camtest_record.py` with latency logging, verify and use it. |
| C2 | **Capture YUV420, use Y plane as grayscale** | 2h | Picamera2 can deliver the luma plane directly. Eliminates the RGB888 deep copy and the `cvtColor` call. ArUco wants grayscale anyway. Roughly 3-4× detection speedup on a Pi 4. |
| C3 | **Downsample detection to 640×480** | 1h | 4× fewer pixels = ~4× faster detection. A 5cm marker at 20-50cm still occupies well above the 20-30px minimum ArUco needs. Verify worst-case pixel count at maximum acquisition range before committing. |
| C4 | **Set `NoiseReductionMode=0` in picamera2** | 15min | The default runs a software denoise on the Pi 4 CPU. Free performance, no detection quality loss for marker tracking. |
| C5 | **Evaluate ArUco Nano (`nanofractal`)** | 4h + testing | Peer-reviewed 5-6× faster detector with Python bindings (`pip install nanofractal`, aarch64 wheels available). Trades multi-window adaptive thresholding for a single fixed pass, which cuts directly against the team's dark/underwater CLAHE tuning. Prototype on a side branch, benchmark on actual underwater footage, switch only if it maintains 90%+ of the current detection rate. Not a rip-and-replace. |

### Code quality

| ID | Improvement | Effort | Why |
|---|---|---|---|
| C6 | ~~**Consolidate camera scripts**~~ ✅ Done | — | **Resolved** on `calibration` branch via `camera/README.md`: production module is `integration/camFinal.py`; standalone benchmark scripts documented with purpose/LED/features table and "which script to use" guide. Scripts kept accessible (each tests a different config) rather than archived. Earlier iterations already in `camera/archive/`. |
| C7 | **Stop committing `camera/results/` scratch data** | 15min | 33+ scratch test files are tracked in git. Add `camera/results/` to `.gitignore`. Test data should not live in the repo long-term. |

### Calibration

| ID | Improvement | Effort | Why |
|---|---|---|---|
| C8 | **Real checkerboard calibration** | 2h + shoot | Replace the approximate camera matrix (HFOV/VFOV estimate) with real intrinsics from a checkerboard shoot. Improves pose estimation accuracy, especially at close docking range. |

---

## Domain 2: Integration (standalone)

These improvements live entirely in `integration/` and do not touch `camera/`.

### Safety (P0, do before any wet closed-loop demo)

| ID | Improvement | Effort | Why |
|---|---|---|---|
| I1 | **Configure `FS_GCS_ENABLE` on the Pixhawk** | 15min QGC param | If the Pi crashes, ArduSub currently has no automatic response. In MANUAL mode, missing `MANUAL_CONTROL` messages cause sticks to freeze at their last value (uncontrolled thrusters). Set `FS_GCS_ENABLE=2` (ALT_HOLD) or `FS_GCS_ENABLE=3` (Disarm). Note: with no vertical thrust, ALT_HOLD's depth-hold is a no-op, but the mode switch still interrupts stick pass-through. **Verify behavior against actual firmware** (see I9). |
| I2 | **Wire leak sensor to failsafe** | 1h code | Currently display-only (`external_sensors.py:65` reads GPIO17, web UI shows it, nothing reacts). Add: if leak detected, disarm. Strictly safer than zeroing sticks, since disarm cuts all motor output at the ESC level. |
| I3 | **Fix `MavlinkInterface._latest` thread safety** | 1h | The `_latest` dict is written by `update()` in the mavlink thread and read by `get_telemetry_deg()` from the Flask thread with no lock. Formally a data race. Benign in practice (monotonically updating values) but should be locked. All `HardwareManager` state is correctly locked; this is the one gap. |

### Correctness and robustness

| ID | Improvement | Effort | Why |
|---|---|---|---|
| I4 | **Verify camera-to-body transform** | 1h + testing | `CAMERA_MOUNT_YAW_DEG=90.0` in `pose_controller.py:50` is explicitly flagged as a placeholder ("set from the live right/left test, verify sign"). A wrong mount rotation silently rotates the error vector: the rig station-keeps on the wrong axis. Needs a real verification procedure: command pure surge, confirm ArUco x-axis (not y-axis) responds. |
| I5 | **Verify MANUAL stick-freeze behavior** | 30min bench | The claim that ArduSub MANUAL mode freezes sticks at last value on `MANUAL_CONTROL` timeout is unverified against the actual firmware version on the Pixhawk. Behavior depends on `FS_PILOT_TIMEOUT` and pilot-input source configuration. Safety-critical: the entire failsafe strategy assumes this. Test by stopping MANUAL_CONTROL mid-run and observing what the motors do. |
| I6 | **Add ACK checking to arm/disarm** | 2h | Currently fire-and-forget (`command_long_send` with no ACK check). If the Pixhawk refuses to arm (pre-arm check fails: EKF not healthy, safety switch, low battery), the operator gets zero feedback. `hardware.py:321` returns `{'ok': True}` regardless. |
| I7 | **Surface STATUSTEXT messages** | 2h | ArduSub sends `STATUSTEXT` messages for pre-arm failures, EKF warnings, mode change confirmations. Currently silently dropped in the `update()` loop (`mavlink_interface.py:101-164`). Parse and display in the web UI + log to console. Without this, operators have no visibility into why ArduSub refuses commands. |
| I8 | **Remove dead code: `send_velocity()`** | 15min | `mavlink_interface.py:289-303` implements `SET_POSITION_TARGET_LOCAL_NED` for GUIDED mode. No caller anywhere in the repo. Either commit to GUIDED (not recommended for Gate 4) or delete it to reduce confusion. The `set_fake_ekf_origin()` method at line 211 corroborates a prior GUIDED prototyping attempt that was abandoned. |
| I9 | **Fix daemon thread cleanup** | 1h | All 3 background threads are `daemon=True` (`hardware.py:120-126`). On Ctrl+C, there is a race where `HardwareManager.stop()` may not run, leaving the camera and I2C bus not cleaned up. Consider non-daemon threads with explicit signal handling, or a signal handler that calls `stop()` before exit. |

### Control quality

| ID | Improvement | Effort | Why |
|---|---|---|---|
| I10 | **Increase control rate from 10 Hz to 20+ Hz** | 1h + testing | `CONTROL_RATE_HZ=10` (`hardware.py:48`) is 5× below the README target of ~50 Hz. Limits PID bandwidth and response time. The PID uses wall-clock dt, so it is stable at any rate. The camera runs at 15-30 fps independently. Could decouple the PID computation from the MAVLink send rate if needed. |
| I11 | **Make PID gains and tolerances configurable** | 2h | PID gains (`pose_controller.py:170-171`), decision engine tolerances (`decision_engine.py:28-35`), and camera mount angles (`pose_controller.py:48-50`) are all hardcoded. Should be runtime-configurable (argparse, config file, or web UI sliders) to enable tuning without code edits and restarts. |

### Monitoring

| ID | Improvement | Effort | Why |
|---|---|---|---|
| I12 | **Add SYS_STATUS monitoring** | 1h | `SYS_STATUS` carries battery voltage, remaining capacity, CPU load, and sensor health flags. Not requested or parsed. Would give the operator battery awareness before ArduSub's own failsafe triggers. |
| I13 | **Send leak state to Pixhawk via SYS_STATUS** | 1h | The leak sensor is on Pi GPIO17. If the Pi dies, leak protection is gone. Send leak state to the Pixhawk via the MAVLink `SYS_STATUS` extended fields so ArduSub's own leak failsafe (`FS_LEAK_ENABLE`) can trigger independently of the Pi. |

---

## Domain 3: Pixhawk/ArduSub Configuration

These are QGroundControl parameter changes and bench tests, not code changes.

### Frame verification (P0)

| ID | Improvement | Effort | Why |
|---|---|---|---|
| P1 | **Verify `FRAME_TYPE` matches the 4-motor layout** | 1h bench | The Vectored frame (`FRAME_TYPE=1`) assumes 6 motors (4 horizontal + 2 vertical). The rig has 4 horizontal thrusters. May need a custom motor matrix or `SimpleROV-4` (`FRAME_TYPE=5`). If the frame config is wrong, thrusters fight each other or respond on the wrong axis. |
| P2 | **Run QGC motor tests** | 30min | Verify each axis command (surge, sway, yaw) spins the correct thrusters in the correct direction. Critical: do this before any wet closed-loop test. |

### Failsafe configuration

| ID | Improvement | Effort | Why |
|---|---|---|---|
| P3 | **Set `FS_GCS_ENABLE`** | 15min QGC | See I1. The Pixhawk-side failsafe is the ultimate backstop if the Pi crashes. |
| P4 | **Set `FENCE_ALT_MAX` / depth limit** | 5min QGC | Cheap insurance for tank testing. Prevents the rig from exceeding a safe depth. |
| P5 | **Configure `FS_BATT_ENABLE`** | 10min QGC | Low battery action (warn, land, or surface). Important for in-water sessions. |

### Mode strategy

| ID | Improvement | Effort | Why |
|---|---|---|---|
| P6 | **Test STABILIZE vs MANUAL** | 1h tank test | STABILIZE adds ArduSub attitude stabilization (roll/pitch angle PID). With 4 horizontal thrusters, roll/pitch authority may be minimal, but any stabilization helps in disturbed water. Test: does the rig stay more level in STABILIZE? Watch for double-loop conflict with Pi yaw PID (should not conflict since STABILIZE only does R/P, not yaw). |
| P7 | **Do NOT pursue GUIDED mode for Gate 4** | Decision (0 work) | GUIDED mode + vision position injection (`VISION_POSITION_ESTIMATE` to EKF3 + `SET_POSITION_TARGET_LOCAL_NED`) is the "proper" architecture, but the complexity exceeds the capstone budget: EKF3 source configuration, covariance tuning, 25+ Hz command rate requirement, `position_ok()` health check fragility, and the fact that ArduSub position-enabled modes are still maturing. Note: GUIDED is functional in production BlueROV2 setups. The rejection is about complexity vs timeline for this capstone, not capability. Revisit post-capstone. |

---

## Domain 4: Documentation

| ID | Improvement | Effort | Why |
|---|---|---|---|
| D1 | **Remove heave from README controlled axes** | 10min | `README.md:84` lists "surge / sway / heave / yaw" but the rig has no vertical thrust. Incorrect. |
| D2 | **Update README architecture to reflect Pixhawk** | 20min | README describes a Pi-direct autopilot (PCA9685 PWM expander to ESCs, no Pixhawk). The canonical architecture is Pixhawk + ArduSub over MAVLink. |
| D3 | **Document the confirmed frame layout** | 15min | Record the physical thruster positions, `FRAME_TYPE`, and motor matrix for future reference. |

---

## Priority ranking for Gate 4

### Phase 1: Must-do before any wet closed-loop demo (safety + correctness)

| # | ID | What | Why first |
|---|---|---|---|
| 1 | P1/P2 | Verify frame config + motor tests | If wrong, nothing works correctly |
| 2 | I1/P3 | Configure `FS_GCS_ENABLE` on Pixhawk | Without it, Pi crash = uncontrolled thrusters |
| 3 | I5 | Verify MANUAL stick-freeze behavior | The safety plan assumes this, and it is unverified |
| 4 | I4 | Verify camera-to-body transform | Wrong rotation = station-keeping on the wrong axis |
| 5 | I2 | Wire leak sensor to failsafe | Water + electronics, no automatic response currently |
| 6 | I3 | Fix `_latest` dict thread safety | Data race in shared telemetry state |
| 7 | C1 | Profile detection latency | Measure before optimizing |

### Phase 2: Should-do for demo quality

| # | ID | What | Why |
|---|---|---|---|
| 8 | C2/C3/C4 | YUV420 + 640×480 + NoiseReduction off | Biggest camera performance wins, low risk |
| 9 | I10 | Increase control rate to 20+ Hz | Improves PID bandwidth and station-keeping precision |
| 10 | I7 | Surface STATEXT messages | Operator visibility into ArduSub state |
| 11 | I11 | Make PID gains configurable | Enables tuning without code restarts |
| 12 | P4/P5 | Depth limit + battery failsafe | Tank insurance |
| 13 | I13/P3 | Leak via MAVLink SYS_STATUS | Independent failsafe path if Pi dies |
| 14 | D1/D2 | Fix README (heave, architecture) | Correct the project record |
| 15 | ~~C6~~ ✅ | Consolidate camera scripts | **Done** — documented via `camera/README.md` |
| 16 | C8 | Real checkerboard calibration | Better pose accuracy at docking range |

### Phase 3: Nice-to-have (if time permits)

| # | ID | What |
|---|---|---|
| 17 | C5 | Evaluate ArUco Nano (side branch benchmark) |
| 18 | I6 | Arm/disarm ACK checking |
| 19 | I8 | Remove dead code (`send_velocity`) |
| 20 | I12 | SYS_STATUS battery monitoring |
| 21 | I9 | Fix daemon thread cleanup |
| 22 | P6 | Test STABILIZE vs MANUAL |
| 23 | C7 | Gitignore camera/results/ scratch data |
| 24 | D3 | Document frame layout |

---

## Pixhawk 1 / ArduSub reference (key learnings from research)

### ArduSub main loop
- Runs at **400 Hz** on the Pixhawk's STM32F427 (168 MHz Cortex-M4F).
- Rate PID and motor output have highest priority (always execute first).
- At 10 Hz Pi control rate, ArduSub runs 40 motor updates between each Pi command.

### Flight modes and active PID loops

| Mode | Rate PID | Angle PID | Position PID | Use case |
|---|---|---|---|---|
| MANUAL | None | None | None | Current setup. Pi PID is the only closed loop. |
| STABILIZE | Yes | R/P only | None | Adds roll/pitch stabilization. Yaw still rate-controlled. |
| ALT_HOLD | Yes | R/P only | Z (depth) | Not useful (no vertical thrust). |
| GUIDED | Yes | Yes | Full cascade | Not recommended for Gate 4 (complexity). |

### MANUAL_CONTROL message
- Body-frame stick command: x=surge, y=sway, r=yaw, z=throttle (0-1000, 500=neutral).
- In MANUAL mode, goes directly to motor mixer with zero PID loops.
- Missing commands: behavior depends on `FS_PILOT_TIMEOUT` param. **Verify experimentally** (I5).

### Bandwidth at 57600 baud
- ~5,760 bytes/second capacity.
- Current usage: ~570 B/s (170 out + 400 in) at 10 Hz. Well within budget.
- Even adding VISION_POSITION_ESTIMATE (~120 bytes/msg at 10 Hz) stays within capacity.
- **Bandwidth is not the bottleneck.** Camera detection latency and Python GIL jitter are.

### EKF3 vision fusion (for future reference, not Gate 4)
- To inject ArUco position into ArduSub's EKF3: send `VISION_POSITION_ESTIMATE` at 20-30 Hz.
- Configure: `EK3_SRC1_POSXY=6, EK3_SRC1_VELXY=6, EK3_SRC1_POSZ=1` (baro for depth), `EK3_SRC1_YAW=6`.
- Covariance matters critically: NaN causes lane switching, 0.00001 = "trust completely", ~0.01 is practical.
- Requires GUIDED mode + `SET_POSITION_TARGET_LOCAL_NED` at 25+ Hz.
- Community reports of `position_ok()` failures and EKF instability are common. Revisit post-capstone.

---

*Research sources: ArduSub source code (v4.5.x), ArduPilot documentation, MAVLink specification,
Blue Robotics community forums, published papers. Full citations in the deepwork research files.*
