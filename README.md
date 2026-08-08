# hive

**Onboard autonomy software for the AUV recovery rig.**

`hive` is the compute, sensing, and control stack that lets a recovery rig find
an AUV, settle over it, and capture it with no human on the joystick. It is the
software half of a hardware capstone built with [BeeX](https://www.beex.sg/)
(Singapore AUV manufacturer) for SUTD's *30.007 Engineering Design Innovation*,
Term 5 2026.

> Repository status: **alpha.** Vision (ArUco on the CSI camera), depth, IMU,
> leak sensing, and a MAVLink-driven control loop are committed and
> bench-tested. The full vision-to-thruster pipeline (camera, frame transform,
> per-axis PID, recovery state machine) is coded in `integration/` and has run
> against a Pixhawk on the bench. Wet closed-loop testing toward Gate 4 is the
> current frontier. Active camera tuning for dark/underwater conditions lives
> on the `tuningv2` branch.

---

## The problem, in one paragraph

BeeX launches and recovers its hovering AUVs (the 55-65 kg **A.IKANBILIS** and
the ~300 kg **BETTA**) by hand: a human drives a crane and joystick to mate a
self-locking spring-jaw catcher onto the vehicle, and the whole thing stops at a
significant wave height of about 1.5 m. BeeX already owns the latch and the
vehicle already carries the navigation. What is missing is the **autonomous
alignment and station-keeping** that gets the catcher onto the vehicle
reliably, in higher sea states, without a human. That is what `hive` provides.

The strategic move that shapes the software: do not fight the rough surface.
Wave energy decays with depth, so the rig captures the vehicle at a calm fixed
pickup depth rather than in the splash zone. Most of the violent disturbance the
control loop would otherwise have to reject is removed by physics before the
software ever runs.

---

## System architecture

A single Raspberry Pi 4 runs the entire loop as a DIY autopilot. The team
A single Raspberry Pi 4 runs the vision processing and upper-level control
loop, sending MANUAL_CONTROL body-frame stick commands to a Pixhawk 1 running
ArduSub over MAVLink serial. ArduSub's vectored frame mixer handles thrust
allocation to the four ESCs.

```
                       sense                 decide                     act
  IMX708 CSI camera -- ArUco pose error ---+
  ICM-20948 IMU     -- attitude/heading ---+--> Pi PID --> MANUAL_CONTROL --> Pixhawk/ArduSub --> 4x AM32 --> 4x T500
  (I2C 0x69)          rate, mag fusion     |   (surge,    (body-frame      (vectored frame     ESC        thruster
                                          |    sway,      stick commands    mixer: motor
  Bar02 (MS5837-02BA) depth -- depth -----+    yaw)       over MAVLink      allocation)
  (via Pixhawk I2C)                                                       |
                                                                         v
                                                      fail-to-stop watchdog:
                                                      leak detected / vision lost / Pi crash
                                                      -> disarm or ArduSub GCS failsafe
```

### Compute and power
| Role | Part | Note |
|---|---|---|
| Companion computer | Raspberry Pi 4 4GB | Headless. Runs ArUco vision processing and the PID control loop at 20 Hz. Sends MANUAL_CONTROL over MAVLink serial to the Pixhawk. |
| Flight controller | Pixhawk 1 (FMUv2) | Runs ArduSub. Receives MANUAL_CONTROL at 57600 baud. Vectored frame mixer allocates surge/sway/yaw to the 4 ESC outputs. |
| Thrusters | 4x Blue Robotics T500 | Loaned from BeeX. 7-24 V, sensorless BLDC, ~16 kgf peak each. Horizontal only, no vertical thrust. |
| ESCs | 4x Skystars Jupiter 50A (AM32) | FPV-grade, reversible ("3D") mode, low-voltage cutoff disabled. *Verify shipped firmware before buying.* |
| Battery | BeeX 24 V pack (loaned) | Matches the T500 ceiling; confirm full-charge stays at or below 24 V (ESC voltage gate ~25.2 V). |
| Pi power | 5 V buck off the 24 V bus | Cytron 5 A buck. |

### Sensor bus (I2C, address map)
| Device | Address | Function |
|---|---|---|
| PCA9685 | `0x40` | PWM to the four ESCs |
| ICM-20948 | `0x68` or `0x69` | 9-DoF attitude and heading |
| Bar02 (MS5837-02BA) | `0x76` | Depth (sealed unit from Blue Robotics) |

Bring-up check: `i2cdetect -y 1` must show `0x40`, `0x68`/`0x69`, and `0x76`.

### The control loop
- **Rate:** 20 Hz control loop on the Pi. ArduSub's inner loop runs at 400 Hz on the Pixhawk.
- **Estimation:** attitude from the Pixhawk's EKF-fused IMU (via MAVLink ATTITUDE); depth from the Bar02 (via MAVLink); relative pose to the target from ArUco on the Pi's CSI camera.
- **Control:** one PID per controlled axis: surge, sway, yaw. No heave control (no vertical thrusters; depth is set by ballast and crane).
- **Allocation:** the Pi sends body-frame stick commands (MANUAL_CONTROL) to ArduSub in MANUAL mode. ArduSub's vectored frame mixer maps surge/sway/yaw onto the four horizontal thrusters. Zero ArduSub PID loops are active in MANUAL mode, so the Pi PID is the only closed loop.
- **Failsafe:** leak sensor triggers immediate disarm (sticks zeroed first, then disarm command). Vision loss zeroes sticks via the DecisionEngine state machine. ArduSub's FS_GCS_ENABLE failsafe is the backstop if the Pi crashes entirely.
- **Magnetometer caveat:** the IMU sits on a mild-steel frame, so the heading solution needs hard/soft-iron calibration after final assembly and a yaw correction for the steel bias.

---

## Repository contents

| Path | Status | Description |
|---|---|---|
| `camera/` | active | ArUco detection and pose estimation on the CSI camera (IMX708). Multiple script variants for different marker sizes and lighting; `camtest.py` is the current tuning harness for dark/underwater work. `results/` holds trial data. |
| `integration/` | active | Canonical integrated stack: MAVLink interface, ArUco vision node, per-axis PID (`pose_controller.py`), recovery state machine (`decision_engine.py`), external sensors, and a Flask web UI with manual and auto (camera-following) modes. Runs headless on the Pi. |
| `webui/` | active | Lightweight LAN web UI for headless monitoring during runs. Manual control only; imports modules from `integration/`. |
| `positioning/` | bench | Blue Robotics SOS leak-detector bench test (`leak_test.py`). The external IMU this used to also cover was removed by design decision — the project relies solely on the Pixhawk's IMU now. |
| `led/` | driver | APA102/DotStar status indicator strip over SPI; `led_test.py` is a standalone bench chase-test, `integration/led_controller.py` is the driver the live app uses. |

---

## The autonomy mission

The guaranteed deliverable (Gate 4) is **closed-loop station-keeping on the
marker in the tank**: the rig holds itself over an ArUco-marked funnel using
vision and IMU, demonstrates the capture sequence, and fails safe. The capture
itself is the "claw machine": a downward camera centres the catcher head over
the vehicle funnel, a linear actuator makes the final insertion stroke, and
BeeX's spring-jaw self-locks. `hive` owns the centring and station-keeping; the
jaw and actuator are hardware.

The autonomy is layered, not monolithic, so each layer can be tested and can
fail independently:

1. **Sense** the relative pose (ArUco) and own state (IMU, depth).
2. **Decide** the body forces needed to hold station over the target (PID +
   allocation).
3. **Act** on the thrusters, then hand off to the linear-actuator insertion and
   the mechanical jaw.
4. **Confirm** the catch took load (load cell), and **retry** on any failed
   gate. Abort-and-retry is designed in from day one; the docking literature
   shows single-attempt capture near 70% compounds above 95% within a few
   attempts.

The **stretch** (Phase 2, Gate 5) adds bearing-only acoustic homing so a
free-swimming catcher can reach a vehicle the crane cannot, using only the
direction of the vehicle's pinger rather than absolute position. That removes
the expensive USBL from the critical path.

---

## Running

All Python runs on a Raspberry Pi 4 under `uv run`. Install per-directory
dependencies as needed.

### Camera scripts (`camera/`)

`aruco_detect.py` is the minimal detector. It uses the legacy OpenCV ArUco API
(`cv2.aruco.Dictionary_get`, `cv2.aruco.detectMarkers`), which requires
**opencv-contrib-python < 4.7**:

```bash
uv pip install opencv-contrib-python numpy picamera2

uv run python camera/aruco_detect.py               # live preview
uv run python camera/aruco_detect.py --no-preview   # headless over SSH
uv run python camera/aruco_detect.py --dict DICT_5X5_50 --width 1920 --height 1080
```

`camFinal.py` adds 6-DOF pose estimation. `camtest.py` is the trial-logging
harness with live exposure/gain tuning and underwater preprocessing; run with
`--help` for the experiment workflow.

### Integrated stack (`integration/`)

The full autonomy stack with a Flask web UI for headless Pi operation:

```bash
cd integration
uv pip install -r requirements.txt     # flask
uv pip install pymavlink picamera2 opencv-contrib-python numpy

# on the Pi, against a Pixhawk on /dev/serial0:
uv run python app.py --mavlink-conn /dev/serial0

# off-Pi bench test (no camera, no external sensors, SITL):
uv run python app.py --no-camera --no-external --mavlink-conn udp:127.0.0.1:14550
```

Open `http://<pi-ip>:8000` from a browser on the same LAN. Manual D-pad
control and auto (camera-following) mode are both available. The simpler
`webui/app.py` serves a monitoring-only variant with the same pattern.

### Sensor tests (`positioning/`, `led/`)

```bash
uv pip install adafruit-circuitpython-icm20x adafruit-blinka RPi.GPIO
uv run python positioning/leak_imu_test.py    # ICM20948 + leak sensor

uv pip install adafruit-circuitpython-dotstar
uv run python led/led_test.py --num-pixels 8   # DotStar chase test
uv run python led/led_test.py --mode on        # solid white
```

Known TODO: the legacy ArUco API is removed in modern OpenCV. Migrate to
`cv2.aruco.ArucoDetector` (OpenCV >= 4.7) before depending on this in the full
stack. For the production rig this node will move to the low-light UVC camera
(Arducam IMX291) rather than the CSI module, with locked auto-exposure so
detection stays stable in dark water.

---

## Roadmap and gates

The software lands against the project's Gate 0-5 ladder. The guaranteed
deliverable is Gate 4; open water (Gate 5) is stretch and pre-committed to
descoping to Gate 4 if it slips.

| Gate | Software milestone |
|---|---|
| 0 dry bench | Pi boots headless; `i2cdetect` sees all three devices; PCA9685 pulses verified; ESCs flashed and configured. |
| 1 wet bench | ESCs tuned (smooth forward/reverse from a stop, no low-RPM desync, thermals stable) via the bench Pixhawk before Pi code exists. |
| 3 sensors | Depth calibrated; IMU calibrated after final assembly; camera focused and ArUco detected in water. |
| **4 integrated tank** | **Closed-loop station-keeping on the marker; failsafe verified.** (guaranteed) |
| 5 open water | Bearing-only homing and full capture sequence at sea. (stretch) |

Full program roadmap (every workstream, 262 tasks, risk register, Gantt) lives
in the parent project as `Recovery_Rig_Roadmap.pdf`.

---

## Hardware context

This repo is software-only. The mechanical, electrical, and procurement detail
lives in the parent project deliverables: the design and budget BOMs, the build
guide (ESC flashing, enclosure and window, sensor potting and calibration, the
power chain), and the learning-resources reading list. The headline hardware
facts the software assumes:

- 4x T500 thrusters on a 24 V bus, ~4.2 kW peak, ~7 min at full throttle,
  station-keeping at 10-25% throttle.
- ESCs flashed to AM32, reversible mode, LVC off, low-KV desync tuned per
  thruster, heat-bedded to an aluminium enclosure wall.
- Capture at a calm fixed pickup depth, not in the splash zone.

---

## Project context

- **Course:** SUTD EPD 30.007 Engineering Design Innovation, Term 5 2026
  (instructors Wai Lee Chan, Bradley Camburn).
- **Industry partner:** BeeX Pte. Ltd., Singapore.
- **Team:** Nathan Ly, Guo Yao, and the EDI cohort team.
- **Related deliverables:** the concept brief, AUV crash course, build guide,
  BOM, and roadmap are in the parent project workspace, not in this repo.

## Contributing

Internal team repo. Use `uv run` for all Python invocations. Keep documents free
of em dashes (project style: use commas, colons, or hyphens instead). Prototypes
and purchased parts are returned to the Pillar at the end of the course.
