# webui

LAN web control panel for the recovery rig: manual thruster control, live
Pixhawk telemetry, and the camera feed, for driving the Pi while it runs
headless (no monitor/keyboard attached).

This is side tooling, not part of the autonomy stack. It only *imports*
`integration/mavlink_interface.py` and `integration/camFinal.py` from
background threads (see `hardware.py`) -- nothing in `integration/` is
modified.

## Run

```bash
# one-off new dependency vs. the rest of the repo
uv pip install flask

uv run python webui/app.py --mavlink-conn /dev/serial0
```

Then open `http://<pi-ip>:8000` from a browser on the same network.

Useful flags:
- `--mavlink-conn` connection string (`/dev/serial0` for the real Pixhawk,
  `udp:127.0.0.1:14550` for SITL).
- `--no-camera` skip camera startup, e.g. bench-testing the mavlink side
  without the CSI camera attached.
- `--host` / `--port` (defaults `0.0.0.0:8000`).

## What it does

- **Manual control**: on-screen D-pad + yaw buttons, or keyboard
  `W`/`A`/`S`/`D` (surge/sway) and `Q`/`E` (yaw). Sends `MANUAL_CONTROL`
  sticks at 10 Hz via `MavlinkInterface.send_manual_control()`. Heave is
  always neutral -- this platform has no vertical thrusters.
- **Watchdog**: if the page stops sending control updates (closed tab,
  dropped network) for 0.5s, the sticks are zeroed automatically. This
  runs server-side in `hardware.py`, independent of the browser.
- **Telemetry**: roll/pitch/yaw/tilt, depth, both barometers (external
  Bar02 + Pixhawk's internal MS5611), mode, and raw servo PWM, polled
  from `MavlinkInterface.get_telemetry_deg()`.
- **Camera feed**: MJPEG stream of `ArucoDetector`'s annotated frames,
  plus the latest marker pose.

## Caveats

- No authentication, no TLS. Trusted LAN or point-to-point link only --
  do not expose this to the open internet.
- Camera and Pixhawk are each driven by a single background thread
  reusing one `ArucoDetector`/`MavlinkInterface` instance; this UI and
  a second process (e.g. the eventual autonomy main loop) should not
  open the camera or serial port at the same time.
- `mavlink_interface.py` currently has two `send_manual_control`
  definitions -- Python keeps the second (`x, y, z, r, buttons`), which
  is the one this UI calls. Known, not fixed here per the "audit later"
  plan.
