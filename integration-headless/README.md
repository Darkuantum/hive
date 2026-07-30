# webui

LAN web control panel for the recovery rig: manual thruster control,
**autonomous ArUco-following control**, live Pixhawk telemetry, and the
camera feed -- for driving the Pi while it runs fully headless (no
monitor/keyboard attached, since the electronics are going in the water).

This wraps `integration/mavlink_interface.py`, `integration/camFinal.py`,
`integration/pose_controller.py`, and `integration/decision_engine.py`
from background threads (see `hardware.py`) -- nothing in `integration/`
is modified by this UI beyond what's needed to keep them consistent
(see "Fixed" below).

## Run

```bash
# one-off new dependency vs. the rest of the repo
uv pip install flask

uv run python webui/app.py --mavlink-conn /dev/serial0
```

Then open `http://<pi-ip>:8000` from a browser on any other device on
the same network -- this is the point of the whole UI: the Pi itself
needs no display once this is running.

Useful flags:
- `--mavlink-conn` connection string (`/dev/serial0` for the real Pixhawk,
  `udp:127.0.0.1:14550` for SITL).
- `--no-camera` skip camera startup, e.g. bench-testing the mavlink side
  without the CSI camera attached. Auto mode has nothing to react to
  without a camera, so it'll sit in `SEARCHING` indefinitely -- expected.
- `--host` / `--port` (defaults `0.0.0.0:8000`).

## What it does

- **Manual mode**: on-screen D-pad + yaw buttons, or keyboard
  `W`/`A`/`S`/`D` (surge/sway) and `Q`/`E` (yaw). Sends `MANUAL_CONTROL`
  sticks at 10 Hz via `MavlinkInterface.send_manual_control()`. Heave is
  always neutral -- this platform has no vertical thrusters.
- **Auto mode**: the net follows the AUV's ArUco marker autonomously.
  Runs the exact same logic as `integration/pixhawk_camera_test.py`'s
  loop -- camera pose &rarr; `DecisionEngine` (gates whether to correct,
  tracks `SEARCHING`/`DETECTED`/`ALIGNING`/`READY`/`RECOVERING`) &rarr;
  `PoseController` (the PID) &rarr; stick command -- just running headless
  in a background thread instead of a `cv2` preview window. The current
  state and stick output are shown live in the UI.
- **Switching modes**: click Manual or Auto at the top of the control
  card, from any browser on the LAN. Switching *into* Auto always
  creates a fresh `DecisionEngine` and resets the PID -- `RECOVERING` is
  a terminal state by design (see `decision_engine.py`'s docstring), so
  without this, a stale `RECOVERING` from an earlier auto session would
  silently block all further control with no obvious symptom besides
  "nothing moves." Switch to Manual and back to Auto any time you need
  to force this reset (e.g. during bench testing, same idea as the `r`
  key in `pixhawk_camera_test.py`).
- **Watchdog**: in Manual mode, if the page stops sending control updates
  (closed tab, dropped network) for 0.5s, the sticks are zeroed
  automatically, server-side, independent of the browser. Auto mode has
  its own equivalent safety instead -- `DecisionEngine.is_controlling()`
  gates whether anything nonzero is ever sent, so losing the marker (or
  the browser tab) simply returns the platform to holding still rather
  than continuing on stale input.
- **Telemetry**: roll/pitch/yaw/tilt, depth, both barometers (external
  Bar02 + Pixhawk's internal MS5611), ArduSub flight mode, and raw servo
  PWM, polled from `MavlinkInterface.get_telemetry_deg()`.
- **Camera feed**: MJPEG stream of `ArucoDetector`'s annotated frames
  (now with low-light CLAHE enhancement baked in, ported from the
  underwater-tuned `cam12cm.py`), plus the latest marker pose.

## Two different "modes" -- don't confuse them

- **Control mode** (`manual` / `auto`) -- *who* is driving: the web page
  or the autonomy stack. Set via `/api/control_mode`.
- **ArduSub flight mode** (`STABILIZE`, etc.) -- what the *flight
  controller* is doing internally (attitude auto-level, thruster mixing).
  Set via `/api/mode`. This project uses `STABILIZE` + `MANUAL_CONTROL`
  throughout, not `GUIDED` -- see `integration/pose_controller.py` and
  the project summary for why (`GUIDED` requires a position estimate
  this GPS-less, DVL-less vehicle doesn't have).

## Fixed

- `mavlink_interface.py` previously had two `send_manual_control`
  definitions (Python silently keeps the second one) -- this has been
  cleaned up to a single definition. If you have an older local copy,
  replace it with the one in this delivery rather than patching in place.

## Caveats

- No authentication, no TLS. Trusted LAN or point-to-point link only --
  do not expose this to the open internet.
- Camera and Pixhawk are each driven by a single background thread
  reusing one `ArucoDetector`/`MavlinkInterface` instance; this UI and
  a second process (e.g. a separate autonomy script) should not open
  the camera or serial port at the same time -- use Auto mode from this
  UI instead of running `pixhawk_camera_test.py` alongside it.
- Auto mode inherits the same open item flagged in the project
  summary: three stacked, not-yet-independently-verified sign
  conventions (camera mount yaw, PID error sign, ESC rotation
  direction). Now that thrusters are physically mounted, this is the
  first real opportunity to confirm all three at once -- watch whether
  the net moves *toward* or *away from* the marker the first time you
  switch into Auto, restrained and at low gain/short duration until
  confirmed.
