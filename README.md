# hive

**Onboard autonomy software for the AUV recovery rig.**

`hive` is the compute, sensing, and control stack that lets a recovery rig find
an AUV, settle over it, and capture it with no human on the joystick. It is the
software half of a hardware capstone built with [BeeX](https://www.beex.sg/)
(Singapore AUV manufacturer) for SUTD's *30.007 Engineering Design Innovation*,
Term 5 2026.

> Repository status: **pre-alpha.** Only the vision node (`aruco_detect.py`) is
> committed. The rest of this README describes the target architecture the team
> is building toward on the Gate 0-5 ladder in [Roadmap](#roadmap--gates).
> Sections marked *planned* are design intent, not shipped code.

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
deliberately chose to write the control loop itself rather than run a ready
autopilot (Blue Robotics Navigator / ArduSub on a Pixhawk), because the
sensor suite is small, the loop is legible, and it saves the cost of a marine
controller board.

```
                       sense                 decide                     act
  IMX291 USB camera -- ArUco pose error ---+
  ICM-20948 IMU     -- attitude/heading ---+--> state --> PID per axis --> thrust --> PCA9685 --> 4x AM32 --> 4x T500
  (I2C 0x68/0x69)      rate, mag fusion     |   estimate  (surge, sway,   allocation  (PWM)       ESC        thruster
                                            |             heave, yaw)     matrix
  MS5837-30BA depth -- depth ---------------+                                          |
  (I2C 0x76)                                                                              |
                                                                                         v
                                                                          fail-to-stop watchdog:
                                                                          vision lost / NaN / stall
                                                                          -> all thrusters neutral
```

### Compute and power
| Role | Part | Note |
|---|---|---|
| Autopilot | Raspberry Pi 4 4GB | Headless. Owns sensing, control, and ESC PWM. Already in hand. |
| PWM expander | PCA9685 (`0x40`) | Generates the 50-60 Hz servo PWM the ESCs expect. |
| Thrusters | 4x Blue Robotics T500 | Loaned from BeeX. 7-24 V, sensorless BLDC, ~16 kgf peak each. |
| ESCs | 4x Skystars Jupiter 50A (AM32) | FPV-grade, reversible ("3D") mode, low-voltage cutoff disabled. *Verify shipped firmware before buying.* |
| Battery | BeeX 24 V pack (loaned) | Matches the T500 ceiling; confirm full-charge stays at or below 24 V (ESC voltage gate ~25.2 V). |
| Pi power | 5 V buck off the 24 V bus | Cytron 5 A buck. |

### Sensor bus (I2C, address map)
| Device | Address | Function |
|---|---|---|
| PCA9685 | `0x40` | PWM to the four ESCs |
| ICM-20948 | `0x68` or `0x69` | 9-DoF attitude and heading |
| MS5837-30BA | `0x76` | Depth (bare board, team-potted and calibrated) |

Bring-up check: `i2cdetect -y 1` must show `0x40`, `0x68`/`0x69`, and `0x76`.

### The control loop *(planned)*
- **Rate:** ~50 Hz, with a watchdog.
- **Estimation:** fuse the IMU and magnetometer for attitude/heading; depth from
  MS5837; relative pose to the target from ArUco.
- **Control:** one PID per controlled axis, surge / sway / heave / yaw.
- **Allocation:** a thrust-allocation matrix maps desired body forces onto the
  four thruster commands, then out to the PCA9685. Thrust commands are mapped
  from `-1..+1` onto `1000..2000 us` with `1500 us` as stop, matching the ESC
  3D mode.
- **Failsafe:** fail-to-stop. If vision is lost, the IMU returns NaN, or the
  loop stalls, every thruster is commanded neutral.
- **Magnetometer caveat:** the IMU sits on a mild-steel frame, so the heading
  solution needs hard/soft-iron calibration after final assembly and a yaw
  correction for the steel bias.

---

## Repository contents

| Path | Status | Description |
|---|---|---|
| `aruco_detect.py` | **shipped** | ArUco marker detection on the Raspberry Pi CSI camera (IMX708). The first vision node; see [Running](#running). |
| `vision/` | planned | Pose estimation from the marker, pixel-to-body error, target handoff to control. |
| `control/` | planned | State estimator, PID loops, thrust-allocation matrix, failsafe watchdog. |
| `drivers/` | planned | PCA9685 PWM, ICM-20948, MS5837, UVC camera wrappers. |
| `mission/` | planned | Capture state machine: home, gate, settle, capture-confirm, retry. |
| `homing/` | planned *(stretch)* | Bearing-only acoustic homing for the self-navigating catcher (Phase 2). |

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

`aruco_detect.py` runs on a Raspberry Pi with a CSI camera (IMX708). It uses
the legacy OpenCV ArUco API (`cv2.aruco.Dictionary_get`,
`cv2.aruco.detectMarkers`), which is provided by **opencv-contrib-python <
4.7**.

```bash
# install deps (prefer uv)
uv pip install opencv-contrib-python numpy picamera2

# live preview with detection overlay
uv run python aruco_detect.py

# headless (e.g. over SSH), default 1280x720
uv run python aruco_detect.py --no-preview

# pick a different ArUco dictionary and resolution
uv run python aruco_detect.py --dict DICT_5X5_50 --width 1920 --height 1080

# throttle repeated "detected" prints
uv run python aruco_detect.py --notify-cooldown 2.0
```

Flags:
- `--dict` ArUco dictionary (default `DICT_4X4_50`).
- `--width` / `--height` capture resolution (default 1280x720).
- `--no-preview` run headless, print detections only.
- `--notify-cooldown` seconds between repeated detection prints (default 1.0).

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
