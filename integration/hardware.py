"""
hardware.py

Thin, thread-safe wrapper around the local integration modules
(MavlinkInterface, ArucoDetector, DecisionEngine, PoseController) for
the web UI. This module does not modify those files -- it only imports
and drives them from background threads, and exposes plain get/set
methods that app.py's Flask routes call. Safe to import and exercise
without Flask at all.

Two control modes, switchable live from the UI:
  - 'manual': sticks come from the web page (D-pad/WASD), same as before.
  - 'auto':   sticks are computed every cycle from the camera pose,
              through DecisionEngine (gates whether to correct) and
              PoseController (the actual PID), exactly like
              pixhawk_camera_test.py's loop -- just running headless in
              a background thread instead of a cv2 preview window.

Switching INTO auto mode always creates a fresh DecisionEngine and
resets the PID -- RECOVERING is a terminal state by design (see
decision_engine.py), so without this a stale RECOVERING from a
previous auto session would silently block control forever.

Three background threads:
  - mavlink thread: connects, drains telemetry, and re-sends the
    current stick command (manual or auto, depending on mode) at a
    steady rate -- this is also what lets a MANUAL_CONTROL-driven
    vehicle keep moving between HTTP posts, and what runs the auto
    control loop.
  - watchdog: implemented inline in the mavlink thread -- in MANUAL
    mode only, if the web client stops posting new control values for
    CONTROL_TIMEOUT_S, the sticks are zeroed automatically. Auto mode
    has its own equivalent safety instead: DecisionEngine.is_controlling()
    gates whether anything nonzero is ever sent.
  - camera thread: continuously grabs frames + ArUco pose so the video
    feed and telemetry API are never blocked on a slow capture, and so
    auto mode always has a fresh pose to react to.
"""
import math
import os
import sys
import threading
import time
import traceback

from pymavlink import mavutil
from mavlink_interface import MavlinkInterface
from pose_controller import PoseController, camera_to_body, camera_to_body_yaw
from decision_engine import DecisionEngine

CONTROL_TIMEOUT_S = 0.5    # manual mode only: zero sticks if nothing posted for this long
CONTROL_RATE_HZ = 20
CAMERA_JPEG_QUALITY = 80
DEFAULT_MANUAL_POWER = 1.0  # 100% -- manual-mode thruster scale, resets here on every connect
NEUTRAL_Z = 500             # neutral z in ArduSub manual_control (3-DOF vehicle, no heave actuator)

VALID_MODES = ('manual', 'auto')


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(v)))


class HardwareManager:
    def __init__(self, mavlink_conn='/dev/serial0', mavlink_baud=57600,
                 enable_camera=True, camera_kwargs=None, enable_external=True,
                 enable_led=True, num_leds=8,
                 pose_controller_kw=None, engine_kw=None,
                 gains_file=None):
        self.enable_camera = enable_camera
        self.enable_external = enable_external

        self.veh = MavlinkInterface(mavlink_conn, baud=mavlink_baud)

        self.detector = None
        if enable_camera:
            # Imported lazily so --no-camera bench-testing works on a
            # machine without picamera2/cv2 installed (e.g. off-Pi).
            from camFinal import ArucoDetector
            self.detector = ArucoDetector(**(camera_kwargs or {}))

        self.external = None
        if enable_external:
            # Imported lazily, same reasoning -- RPi.GPIO/board/busio
            # aren't installable off-Pi.
            from external_sensors import ExternalSensors
            self.external = ExternalSensors()

        self.led = None
        if enable_led:
            try:
                from led_controller import LEDController
                self.led = LEDController(num_pixels=num_leds)
            except Exception as exc:
                print(f"LED controller unavailable: {exc}")

        self._lock = threading.Lock()
        self._mavlink_status = {'connected': False, 'error': None}
        self._camera_status = {'connected': False, 'error': None}
        self._external_status = {'connected': False, 'error': None}
        self._latest_external = None

        self._param_status = None
        self._param_status_lock = threading.Lock()

        # ---- manual mode state ----
        self._control = {'x': 0.0, 'y': 0.0, 'r': 0.0}
        self._control_updated_at = 0.0
        # 0.0-1.0 scale applied to x/y/r before they're sent, manual mode
        # only -- lets you cap thruster output (e.g. 0.5 = 50% power) for
        # fine positioning near the marker or safer bench testing.
        # Enforced here server-side, not just in the UI, so a stale page
        # or a bypassed slider can't push more power than intended.
        self._manual_power = DEFAULT_MANUAL_POWER
        self._led_manual_brightness = 0.5
        self._last_led_engine_state = None  # for backward-transition detection

        # ---- mode + auto mode state ----
        self._mode = 'manual'

        # ---- gain loading: file gains fill in only what CLI didn't set ----
        self._gains_file = gains_file
        _pose_kw = dict(pose_controller_kw or {})
        file_gains = None  # type: ignore[assignment]
        if gains_file is not None:
            from calibration.io import Gains
            file_gains = Gains.from_file(gains_file)
            file_kw = file_gains.to_pose_controller_kwargs()
            # Only fill keys not already set by CLI args (CLI takes precedence)
            for key, val in file_kw.items():
                if key not in _pose_kw:
                    _pose_kw[key] = val

        self.controller = PoseController(**_pose_kw)
        self.engine = DecisionEngine(**(engine_kw or {}))

        # Create velocity damper instances if enabled in gains
        if gains_file is not None and file_gains is not None and file_gains.damper_enabled:
            from velocity_damper import VelocityDamper
            self._damper_x = VelocityDamper(
                kv=file_gains.damper_kv,
                vel_leak=file_gains.damper_vel_leak,
                accel_lpf_hz=file_gains.damper_accel_lpf_hz,
                accel_deadband=file_gains.damper_accel_deadband,
                out_limit=self.controller.pid_surge.output_limit,
            )
            self._damper_y = VelocityDamper(
                kv=file_gains.damper_kv,
                vel_leak=file_gains.damper_vel_leak,
                accel_lpf_hz=file_gains.damper_accel_lpf_hz,
                accel_deadband=file_gains.damper_accel_deadband,
                out_limit=self.controller.pid_sway.output_limit,
            )
            print(f"[damper] enabled (kv={file_gains.damper_kv}, "
                  f"vel_leak={file_gains.damper_vel_leak})")

        self._auto_status = {
            'state': self.engine.state.name,
            'controlling': False,
            'stick': {'x': 0.0, 'y': 0.0, 'r': 0.0},
            'yaw_debug': None,
        }
        self._last_auto_time = None  # for dt -- set on first auto compute

        self._latest_pose = None
        self._latest_jpeg = None

        self._stop = threading.Event()
        self._threads = []

        # ---- calibration run state ----
        self._run_lock = threading.Lock()  # protects run lifecycle transitions
        self._run_handle = None           # RunHandle or None
        self._run_logger = None           # TelemetryLogger or None
        self._run_recorder = None         # VideoRecorder or None
        self._run_active = False          # single boolean, GIL-atomic read
        self._latest_frame_idx = -1      # written by camera thread, read by mavlink thread

        # ---- async step execution state ----
        self._step_thread = None
        self._step_result = None
        self._step_error = None
        self._step_partial = None
        self._last_step_summary = None
        self._step_lock = threading.Lock()
        self._step_abort = threading.Event()
        # While a step is running, ignore /api/control posts from OTHER
        # clients -- manual mode obeys whoever posts last regardless of
        # source, so a stray browser tab left open (still sending its
        # idle-zero heartbeat) races the step's own commands and corrupts
        # the logged motor_x/response data. Confirmed live: motor_x
        # flickered 150/0/0/150 every tick during a step instead of
        # holding steady, because another connected client's heartbeat
        # kept zeroing it out between the step's own sends.
        self._step_owns_control = False

        # ---- closed-loop setpoint offset (for validation step runs) ----
        self._cl_setpoint_x = 0.0
        self._cl_setpoint_y = 0.0
        self._cl_setpoint_yaw = 0.0

        # ---- velocity damper (None when disabled) ----
        self._damper_x = None
        self._damper_y = None
        self._accel_bias_x = 0.0
        self._accel_bias_y = 0.0

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self):
        self._threads.append(threading.Thread(
            target=self._mavlink_thread, name='mavlink', daemon=True))
        if self.enable_camera:
            self._threads.append(threading.Thread(
                target=self._camera_thread, name='camera', daemon=True))
        if self.enable_external:
            self._threads.append(threading.Thread(
                target=self._external_thread, name='external', daemon=True))
        for t in self._threads:
            t.start()

        # Handle SIGTERM for clean shutdown (SIGINT is handled by
        # Flask's KeyboardInterrupt -> finally: manager.stop())
        import signal
        def _on_signal(signum, frame):
            self.stop()
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, _on_signal)

    def stop(self):
        # Finalize any active calibration run before stopping threads
        if self._run_active:
            try:
                self.stop_logging_run()
            except Exception as exc:
                print(f"[run] error finalizing run on stop: {exc}", file=sys.stderr)
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        if self.detector is not None:
            self.detector.stop()
        if self.external is not None:
            self.external.stop()
        if self.led is not None:
            self.led.off()

    # ------------------------------------------------------------------
    # mode switching
    # ------------------------------------------------------------------
    def set_control_mode(self, mode):
        if mode not in VALID_MODES:
            raise ValueError(f"mode must be one of {VALID_MODES}, got {mode!r}")

        with self._lock:
            self._mode = mode
            # Always zero the manual stick state on any mode switch --
            # don't want a stale manual command to linger if we're
            # switching OUT of manual, and it's a harmless no-op if
            # we're switching into manual (page will start posting
            # fresh values immediately).
            self._control = {'x': 0.0, 'y': 0.0, 'r': 0.0}
            self._control_updated_at = 0.0

        if mode == 'auto':
            # Fresh engine + controller every time auto is (re)entered.
            # Without this, a DecisionEngine left over from a previous
            # auto session that ended in RECOVERING (terminal state,
            # see decision_engine.py) would silently refuse to control
            # ever again, with no obvious symptom besides "nothing moves".
            self.engine = DecisionEngine()
            self.controller.reset()
            self._last_auto_time = None
            with self._lock:
                self._auto_status = {
                    'state': self.engine.state.name,
                    'controlling': False,
                    'stick': {'x': 0.0, 'y': 0.0, 'r': 0.0},
                    'yaw_debug': None,
                }

    def get_control_mode(self):
        with self._lock:
            return self._mode

    # ------------------------------------------------------------------
    # parameter verification
    # ------------------------------------------------------------------
    def _verify_startup_params(self):
        """Check safety-critical ArduSub parameters and store results.
        Called once per successful MAVLink connection (including reconnects)."""
        CHECKS = [
            {'name': 'FRAME_CONFIG', 'expected': 1, 'check': 'eq',
             'description': 'BlueROV2 Vectored (4 horizontal thrusters; motors 5-6 vertical unconnected)'},
            {'name': 'FS_GCS_ENABLE', 'expected': 1, 'check': 'gte',
             'description': 'GCS failsafe enabled (1=warn, 2=ALT_HOLD, 3=disarm)'},
            {'name': 'ARMING_CHECK', 'expected': 0, 'check': 'neq',
             'description': 'Pre-arm checks enabled (0=disabled is unsafe)'},
            {'name': 'BATT_MONITOR', 'expected': 0, 'check': 'neq',
             'description': 'Battery monitor configured (0=disabled, no battery failsafe possible)'},
            {'name': 'FENCE_ALT_MAX', 'expected': 0, 'check': 'gt',
             'description': 'Depth fence configured (max depth limit)'},
        ]
        try:
            results = self.veh.verify_params(CHECKS)
            with self._param_status_lock:
                self._param_status = results
        except Exception as exc:
            with self._param_status_lock:
                self._param_status = [
                    {'name': 'verification', 'expected': None, 'actual': None,
                     'ok': False, 'description': 'Parameter verification',
                     'error': f'exception: {exc}'},
                ]
        # Print a summary to the console
        with self._param_status_lock:
            status = self._param_status
        if status:
            print("=== Parameter verification ===")
            for r in status:
                tag = "OK" if r['ok'] else "FAIL"
                print(f"  [{tag}] {r['name']}: "
                      f"expected={r['expected']}, actual={r['actual']}"
                      + (f" ({r.get('error', '')})" if r.get('error') else ""))
            all_ok = all(r['ok'] for r in status)
            print(f"=== {len(status)} checks, {'ALL PASS' if all_ok else 'SOME FAILED'} ===")

    def get_param_status(self):
        """Return the latest parameter verification results (list of dicts),
        or None if verification hasn't run yet."""
        with self._param_status_lock:
            return self._param_status

    # ------------------------------------------------------------------
    # mavlink: telemetry in, sticks out (manual or auto), watchdog inline
    # ------------------------------------------------------------------
    def _mavlink_thread(self):
        RECONNECT_DELAY_S = 3.0

        while not self._stop.is_set():
            try:
                self.veh.connect()
                with self._lock:
                    self._mavlink_status = {'connected': True, 'error': None}

                # Verify safety-critical parameters once per successful connect
                self._verify_startup_params()
            except Exception as exc:
                with self._lock:
                    self._mavlink_status = {'connected': False, 'error': str(exc)}
                # Don't give up permanently -- the Pixhawk may still be
                # booting when this service starts, or the link may come
                # back after a transient USB/serial glitch. Retry rather
                # than requiring someone to SSH in and restart the
                # service, which defeats the point of running headless.
                time.sleep(RECONNECT_DELAY_S)
                continue

            period = 1.0 / CONTROL_RATE_HZ
            try:
                while not self._stop.is_set():
                    loop_start = time.time()
                    self.veh.update(blocking=False)

                    # Leak failsafe: disarm immediately if the hull is wet
                    if self.enable_external:
                        ext = self.get_external_telemetry()
                        if ext and ext.get('leak'):
                            try:
                                # Zero sticks FIRST, then disarm — defense in depth.
                                # If disarm is rejected or its ACK is lost, the motors
                                # still get a neutral command instead of running on
                                # the operator's last nonzero stick input.
                                self.veh.send_manual_control(x=0.0, y=0.0, z=0.5, r=0.0)
                                self.veh.disarm()
                                self.veh.send_statustext(
                                    "LEAK DETECTED - auto-disarmed",
                                    severity=mavutil.mavlink.MAV_SEVERITY_EMERGENCY,
                                )
                            except Exception:
                                pass
                            if self.led is not None:
                                self.led.set_state('leak')
                                self.led.update()
                            with self._lock:
                                self._mavlink_status['error'] = 'LEAK DETECTED - disarmed'
                            time.sleep(1.0)
                            continue

                    mode = self.get_control_mode()
                    if mode == 'auto':
                        x, y, r = self._compute_auto_control()
                    else:
                        x, y, r = self._current_manual_control()

                    # Log telemetry if a calibration run is active
                    if self._run_active:
                        self._log_telemetry_tick(mode, x, y, r)

                    # Update LED status indicator
                    if self.led is not None:
                        error_state = None
                        if self._latest_external and self._latest_external.get('leak'):
                            error_state = 'leak'
                        elif not self._mavlink_status.get('connected'):
                            error_state = 'disconnected'

                        if error_state:
                            self.led.set_state(error_state)
                            self._last_led_engine_state = None
                        elif mode == 'auto':
                            engine_state = self.engine.state.name
                            # Detect backward transition (marker lost, etc.)
                            _ORDER = ['SEARCHING', 'DETECTED', 'ALIGNING', 'READY', 'RECOVERING']
                            if (self._last_led_engine_state in _ORDER
                                    and engine_state in _ORDER
                                    and _ORDER.index(engine_state) < _ORDER.index(self._last_led_engine_state)):
                                self.led.flash_failure(1.5)
                            self._last_led_engine_state = engine_state
                            self.led.set_state(engine_state)
                        else:
                            self.led.set_state('manual')
                            self._last_led_engine_state = None

                        self.led.update()

                    self.veh.send_manual_control(x=x, y=y, z=0.5, r=r)
                    with self._lock:
                        self._mavlink_status['error'] = None
                    time.sleep(max(0.0, period - (time.time() - loop_start)))
            except Exception as exc:
                # Something went wrong mid-flight (dropped serial port,
                # etc.) -- fall back to the outer loop and try to
                # reconnect from scratch rather than dying silently.
                with self._lock:
                    self._mavlink_status = {'connected': False, 'error': str(exc)}
                time.sleep(RECONNECT_DELAY_S)

    def _log_telemetry_tick(self, mode, x, y, r):
        """Build and log a telemetry row if a run is active.

        Called from the MAVLink thread. Catches all errors to avoid
        crashing the control loop.
        """
        logger = self._run_logger
        if logger is None:
            return

        try:
            telem = self.veh.get_telemetry_deg()
            pose = self.get_pose()
            aruco_visible = 1 if pose is not None else 0

            # Yaw sources
            yaw_pixhawk = telem.get('yaw')
            yaw_aruco = pose['yaw'] if pose is not None else float('nan')

            # PID state from controller (auto mode only; manual mode leaves empty)
            pid_state = self.controller.last_state

            # Motor commands: x/y/r are normalized -1..1, convert to -1000..1000
            motor_x = int(x * 1000)
            motor_y = int(y * 1000)
            motor_z = NEUTRAL_Z  # neutral (3-DOF vehicle, no heave actuator)
            motor_r = int(r * 1000)

            # Battery voltage
            battery_voltage = telem.get('battery_voltage', float('nan'))

            # Build row
            row = {
                "ts": time.time(),
                "frame_idx": self._latest_frame_idx,
                "mode": mode,
                "aruco_visible": aruco_visible,
                "yaw_pixhawk_rad": yaw_pixhawk if yaw_pixhawk is not None else float('nan'),
                "yaw_aruco_rad": yaw_aruco,
            }

            if pid_state is not None:
                for axis in ('surge', 'sway', 'yaw'):
                    s = pid_state[axis]
                    row[f"{axis}_setpoint"] = s['setpoint']
                    row[f"{axis}_measured"] = s['measured']
                    row[f"{axis}_p"] = s['p']
                    row[f"{axis}_i"] = s['i']
                    row[f"{axis}_d"] = s['d']
                    row[f"{axis}_out"] = s['out']
            else:
                # Manual mode (incl. open-loop step calibration) or non-controlling auto:
                # no PID state, but position IS available from the live ArUco pose.
                for axis in ('surge', 'sway', 'yaw'):
                    row[f"{axis}_setpoint"] = ""
                    row[f"{axis}_p"] = ""
                    row[f"{axis}_i"] = ""
                    row[f"{axis}_d"] = ""
                    row[f"{axis}_out"] = ""

                if pose is not None:
                    x_b, y_b, _z_b = camera_to_body(pose['x'], pose['y'], pose['z'])
                    yaw_b = camera_to_body_yaw(pose['yaw'])
                    row["surge_measured"] = x_b
                    row["sway_measured"] = y_b
                    row["yaw_measured"] = yaw_b
                else:
                    row["surge_measured"] = ""
                    row["sway_measured"] = ""
                    row["yaw_measured"] = ""

            row["motor_x"] = motor_x
            row["motor_y"] = motor_y
            row["motor_z"] = motor_z
            row["motor_r"] = motor_r
            row["battery_voltage"] = battery_voltage

            logger.log(row)
            self._run_handle.ticks_logged = logger.ticks_logged
        except Exception as exc:
            # Never crash the MAVLink thread
            print(f"[telemetry] tick log error: {exc}", file=sys.stderr)

    def _current_manual_control(self):
        with self._lock:
            age = time.time() - self._control_updated_at
            if age > CONTROL_TIMEOUT_S:
                return 0.0, 0.0, 0.0
            power = self._manual_power
            return (
                self._control['x'] * power,
                self._control['y'] * power,
                self._control['r'] * power,
            )

    def _compute_auto_control(self):
        """Runs the same logic as pixhawk_camera_test.py's main loop --
        camera pose -> DecisionEngine -> PoseController -> normalized
        stick command -- but headless, on whatever pose the camera
        thread most recently produced, with no cv2 window involved."""
        now = time.time()
        dt = 0.1 if self._last_auto_time is None else max(now - self._last_auto_time, 1e-3)
        self._last_auto_time = now

        pose = self.get_pose()  # already strips the raw frame, thread-safe
        telem = self.veh.get_telemetry_deg()  # 'roll'/'pitch' keys are radians (see mavlink_interface)

        marker_detected = pose is not None
        if marker_detected:
            state = self.engine.update(
                True, pose['x'], pose['y'], pose['z'], pose['yaw'],
                telem['roll'], telem['pitch'],
            )
        else:
            state = self.engine.update(False)

        # Calibration debug: camera-frame yaw vs. what CAMERA_MOUNT_YAW_DEG
        # turns it into, regardless of whether the engine is currently
        # controlling. This is what lets CAMERA_MOUNT_YAW_DEG in
        # pose_controller.py be verified live from the web UI on a
        # headless rig, instead of needing `camFinal.py --calibration-check`
        # (which needs a monitor attached to the Pi for its cv2 window).
        yaw_debug = None
        if marker_detected:
            yaw_body_now = camera_to_body_yaw(pose['yaw'])
            yaw_debug = {
                'yaw_cam_deg': math.degrees(pose['yaw']),
                'yaw_body_deg': math.degrees(yaw_body_now),
            }

        if self.engine.is_controlling() and marker_detected:
            # Apply CL setpoint offset for validation runs.  The
            # DecisionEngine has already seen the true pose above;
            # we subtract the offset so PoseController.compute() drives
            # to the offset position instead of zero.
            x_eff = pose['x'] - self._cl_setpoint_x
            y_eff = pose['y'] - self._cl_setpoint_y
            yaw_eff = pose['yaw'] - self._cl_setpoint_yaw
            vx, vy, yaw_rate = self.controller.compute(
                x_eff, y_eff, pose['z'], yaw_eff, dt
            )
            x = vx / self.controller.pid_surge.output_limit
            y = vy / self.controller.pid_sway.output_limit
            r = yaw_rate / self.controller.pid_yaw.output_limit
            # Report saturation directly -- if r sits at the yaw limit
            # regardless of the marker's actual orientation, that's the
            # signature of CAMERA_MOUNT_YAW_DEG being wrong (a bad mount
            # offset adds a constant error big enough to clip the PID
            # before the real orientation error is even factored in).
            if yaw_debug is not None:
                yaw_debug['yaw_saturated'] = abs(r) >= 0.999
        else:
            # Marker lost: apply velocity damper if configured
            if self._damper_x is not None and self._damper_y is not None:
                telem_raw = self.veh.get_telemetry()
                damp_x = self._damper_x.update(
                    (telem_raw.get('accel_x', 0.0) or 0.0) - self._accel_bias_x
                )
                damp_y = self._damper_y.update(
                    (telem_raw.get('accel_y', 0.0) or 0.0) - self._accel_bias_y
                )
                x = damp_x / self.controller.pid_surge.output_limit
                y = damp_y / self.controller.pid_sway.output_limit
                r = 0.0  # no yaw damper (gyro-derived angular accel is too noisy)
            else:
                x = y = r = 0.0
            self.controller.reset()

        with self._lock:
            self._auto_status = {
                'state': state.name,
                'controlling': self.engine.is_controlling(),
                'stick': {'x': x, 'y': y, 'r': r},
                'yaw_debug': yaw_debug,
            }
        return x, y, r

    def set_control(self, x, y, r, _from_step=False):
        """Manual-mode stick input from the web UI. Silently has no
        effect while in auto mode -- the mavlink thread only reads
        this in 'manual' mode -- but we still store it, so switching
        back to manual doesn't require an extra click before sticks
        respond.

        While a calibration step owns control (_step_owns_control),
        ignores calls from anywhere except the step runner itself
        (_from_step=True) -- otherwise another connected client's stray
        heartbeat can zero out the step's command between sends. See
        _step_owns_control's definition for how this was found."""
        if self._step_owns_control and not _from_step:
            return
        with self._lock:
            self._control = {'x': _clamp(x), 'y': _clamp(y), 'r': _clamp(r)}
            self._control_updated_at = time.time()

    def set_manual_power(self, power):
        """Set the manual-mode thruster power scale, 0.0-1.0 (e.g. 0.5 =
        50%). Applied to x/y/r in _current_manual_control() before
        anything is sent -- affects manual mode only, not auto (auto's
        output is already bounded by the PID output_limit values in
        pose_controller.py, so scaling it again here would just fight
        the tuned gains)."""
        with self._lock:
            self._manual_power = _clamp(power, 0.0, 1.0)

    def get_manual_power(self):
        with self._lock:
            return self._manual_power

    def set_led_brightness(self, brightness):
        """Manual-mode LED brightness (0.0-1.0). Called from web UI."""
        if self.led is not None:
            self.led.set_manual_brightness(brightness)

    def get_led_brightness(self):
        if self.led is not None:
            return self.led._manual_brightness
        return 0.5

    def arm(self):
        self.veh.arm()

    def disarm(self):
        self.set_control(0.0, 0.0, 0.0)
        self.veh.disarm()

    def set_mode(self, mode_name):
        """ArduSub flight mode (e.g. 'STABILIZE') -- NOT the same thing
        as control_mode ('manual'/'auto') above. Named to match the
        underlying MavlinkInterface method; kept distinct in the API
        (see app.py) to avoid confusing the two."""
        return self.veh.set_mode(mode_name)

    def get_telemetry(self):
        with self._lock:
            status = dict(self._mavlink_status)
            control = dict(self._control)
            never_sent = self._control_updated_at == 0.0
            control_age = None if never_sent else time.time() - self._control_updated_at
            control_mode = self._mode
            auto_status = dict(self._auto_status)
            manual_power = self._manual_power
        telem = self.veh.get_telemetry_deg()
        telem['mode'] = self.veh.get_mode_name()
        telem['output_bank'] = self.veh.get_output_bank()
        telem['control_mode'] = control_mode
        telem['control'] = control
        telem['control_age_s'] = control_age
        telem['manual_power'] = manual_power
        telem['watchdog_tripped'] = (
            control_mode == 'manual' and (never_sent or control_age > CONTROL_TIMEOUT_S)
        )
        telem['auto'] = auto_status
        telem.update(status)
        return telem

    # ------------------------------------------------------------------
    # calibration: run lifecycle, gains, telemetry logging
    # ------------------------------------------------------------------
    def start_logging_run(self, name=None):
        """Start a calibration run. Returns {"run_id", "tmpfs_dir", "final_dir"}.
        Idempotent: if a run is active, returns info for the existing run.
        Initializes TelemetryLogger + VideoRecorder."""
        with self._run_lock:
            if self._run_active:
                return {
                    "run_id": self._run_handle.run_id,
                    "tmpfs_dir": self._run_handle.tmpfs_dir,
                    "final_dir": self._run_handle.final_dir,
                }

            from calibration.logging import start_run as _start_run
            from calibration.video import VideoRecorder

            handle = _start_run(name)
            logger = None
            recorder = None
            video_path = None

            try:
                from calibration.logging import TelemetryLogger
                logger = TelemetryLogger(handle.csv_path)

                video_path = os.path.join(handle.tmpfs_dir, "video.mp4")
                recorder = VideoRecorder(video_path)
            except Exception as exc:
                # Clean up on failure
                if logger:
                    try:
                        logger.close()
                    except Exception:
                        pass
                print(f"[run] failed to start run: {exc}", file=sys.stderr)
                raise

            self._run_handle = handle
            self._run_logger = logger
            self._run_recorder = recorder
            self._run_active = True
            self._latest_frame_idx = -1

        return {
            "run_id": handle.run_id,
            "tmpfs_dir": handle.tmpfs_dir,
            "final_dir": handle.final_dir,
        }

    def stop_logging_run(self):
        """Stop the active run. Closes logger + recorder, syncs tmpfs -> final_dir.
        Returns finalize_run() summary dict. If no active run, returns {"active": False}."""
        with self._run_lock:
            if not self._run_active:
                return {"active": False}

            # Close recorder first (stop writing frames)
            if self._run_recorder is not None:
                try:
                    self._run_recorder.close()
                    self._run_handle.frames_written = self._run_recorder.frame_idx
                except Exception as exc:
                    print(f"[run] error closing video recorder: {exc}", file=sys.stderr)

            # Sync frame count from recorder to handle
            if self._run_recorder is not None:
                self._run_handle.frames_written = self._run_recorder.frame_idx

            # Sync tick count from logger to handle
            if self._run_logger is not None:
                self._run_handle.ticks_logged = self._run_logger.ticks_logged

            # Close logger
            if self._run_logger is not None:
                try:
                    self._run_logger.close()
                except Exception as exc:
                    print(f"[run] error closing telemetry logger: {exc}", file=sys.stderr)

            # Sync tmpfs -> final dir
            from calibration.logging import finalize_run as _finalize_run
            try:
                summary = _finalize_run(self._run_handle)
            except Exception as exc:
                print(f"[run] error finalizing run: {exc}", file=sys.stderr)
                summary = {
                    "run_id": self._run_handle.run_id,
                    "error": str(exc),
                }

            # Reset state
            self._run_handle = None
            self._run_logger = None
            self._run_recorder = None
            self._run_active = False
            self._latest_frame_idx = -1

        return summary

    def get_active_run(self):
        """Returns {"run_id", "frames_written", "ticks_logged", "duration_s"} or None."""
        with self._run_lock:
            if not self._run_active or self._run_handle is None:
                return None
            handle = self._run_handle
            # Get latest counters
            frames = handle.frames_written
            ticks = handle.ticks_logged
            if self._run_recorder is not None:
                frames = self._run_recorder.frame_idx
            if self._run_logger is not None:
                ticks = self._run_logger.ticks_logged
            return {
                "run_id": handle.run_id,
                "frames_written": frames,
                "ticks_logged": ticks,
                "duration_s": round(time.time() - handle.started_at, 2),
            }

    def reload_gains(self, path=None):
        """Load gains from self._gains_file (or override path), apply to PoseController.
        Returns the loaded gains dict. Raises if no gains_file configured."""
        load_path = path or self._gains_file
        if load_path is None:
            raise ValueError("No gains file configured. Set --gains-file at startup.")

        from calibration.io import Gains
        gains = Gains.from_file(load_path)
        kw = gains.to_pose_controller_kwargs()
        self.controller.update_gains(
            kp=kw['kp'], ki=kw['ki'], kd=kw['kd'],
            yaw_kp=kw['yaw_kp'], yaw_ki=kw['yaw_ki'], yaw_kd=kw['yaw_kd'],
        )
        return gains.to_dict()

    def save_gains(self, path=None):
        """Snapshot current PoseController gains to file (default: self._gains_file or gains.json).
        Returns the saved gains dict."""
        save_path = path or self._gains_file or "gains.json"

        from calibration.io import Gains
        pid_s = self.controller.pid_surge
        pid_w = self.controller.pid_sway
        pid_y = self.controller.pid_yaw
        gains = Gains(
            surge_kp=pid_s.kp, surge_ki=pid_s.ki, surge_kd=pid_s.kd,
            sway_kp=pid_w.kp, sway_ki=pid_w.ki, sway_kd=pid_w.kd,
            yaw_kp=pid_y.kp, yaw_ki=pid_y.ki, yaw_kd=pid_y.kd,
        )
        gains.to_file(save_path)
        return gains.to_dict()

    @property
    def is_shutting_down(self) -> bool:
        """True if stop() has been called (shutdown in progress)."""
        return self._stop.is_set()

    def run_open_loop_step(self, axis: str, amplitude: float,
                           pre_duration: float = 2.0, step_duration: float = 5.0,
                           post_duration: float = 3.0, name: str = None) -> dict:
        """Execute an open-loop step response for system identification.

        Blocking call. Switches to manual mode, applies the step, returns to original mode.
        Returns the stop_logging_run() summary dict (run_id, duration, csv_path, video_path, etc.).

        Raises StepAborted if marker lost or mode changed during execution.
        """
        from calibration.trajectories import StepInput
        from calibration.step_runner import StepRunner, StepAborted

        step = StepInput(
            axis=axis, amplitude=amplitude,
            pre_duration=pre_duration, step_duration=step_duration,
            post_duration=post_duration,
        )
        runner = StepRunner(self)
        self._step_owns_control = True
        try:
            return runner.run(step, run_name=name)
        except StepAborted:
            self._last_step_summary = getattr(runner, 'last_summary', None)
            raise
        finally:
            self._step_owns_control = False

    def start_step_async(self, axis, amplitude, pre_duration=2.0,
                         step_duration=5.0, post_duration=3.0, name=None) -> dict:
        """Start an open-loop step in a background thread. Non-blocking.
        Returns dict with status ('running' or 'already_running') + parameters.
        Thread runs the blocking run_open_loop_step() and stores result/error."""
        with self._step_lock:
            if self._step_thread is not None and self._step_thread.is_alive():
                return {"status": "already_running",
                        "message": "a step is already running; check /step/status"}

            self._step_result = None
            self._step_error = None
            self._step_abort.clear()

            from calibration.trajectories import StepInput
            step = StepInput(
                axis=axis, amplitude=amplitude,
                pre_duration=pre_duration, step_duration=step_duration,
                post_duration=post_duration,
            )
            step.validate()  # raises ValueError on bad axis; clamps amplitude/durations

            estimated = step.total_duration()

            self._step_thread = threading.Thread(
                target=self._step_worker,
                args=(axis, amplitude, pre_duration, step_duration, post_duration, name),
                daemon=True,
            )
            self._step_thread.start()

        return {
            "status": "running",
            "axis": axis,
            "amplitude": amplitude,
            "step_duration": step_duration,
            "estimated_duration": round(estimated, 1),
        }

    def _step_worker(self, axis, amplitude, pre_duration, step_duration, post_duration, name):
        """Background worker — runs the blocking step, stores result or error."""
        try:
            self._step_result = self.run_open_loop_step(
                axis=axis, amplitude=amplitude,
                pre_duration=pre_duration, step_duration=step_duration,
                post_duration=post_duration, name=name,
            )
        except Exception as exc:
            self._step_error = str(exc)
            self._step_partial = self._last_step_summary
            print(f"[step] background step failed: {exc}", file=sys.stderr)
            traceback.print_exc(file=sys.stderr)

    def get_step_status(self) -> dict:
        """Returns current step status: idle | running | done | error."""
        with self._step_lock:
            if self._step_thread is None:
                return {"status": "idle"}
            if self._step_thread.is_alive():
                return {"status": "running"}
            if self._step_error is not None:
                resp = {"status": "error", "error": self._step_error}
                if self._step_partial is not None:
                    resp["partial_summary"] = self._step_partial
                return resp
            if self._step_result is not None:
                return {"status": "done", "summary": self._step_result}
            return {"status": "unknown"}

    def abort_step(self) -> dict:
        """Request abort of the currently running step (if any).
        The step thread will detect this within one loop iteration (≤0.2s)
        and raise StepAborted, triggering motor zeroing + run finalization."""
        if self._step_thread is not None and self._step_thread.is_alive():
            self._step_abort.set()
            return {"abort_requested": True}
        return {"abort_requested": False, "message": "no step is currently running"}

    # ------------------------------------------------------------------
    # closed-loop validation step
    # ------------------------------------------------------------------
    def set_cl_setpoint(self, dx: float, dy: float, dyaw: float):
        """Set closed-loop setpoint offset for validation step runs."""
        self._cl_setpoint_x = dx
        self._cl_setpoint_y = dy
        self._cl_setpoint_yaw = dyaw

    def clear_cl_setpoint(self):
        """Clear closed-loop setpoint offset."""
        self._cl_setpoint_x = 0.0
        self._cl_setpoint_y = 0.0
        self._cl_setpoint_yaw = 0.0

    def run_closed_loop_step(self, axis: str, setpoint: float,
                            hold_duration: float = 5.0,
                            pre_duration: float = 2.0,
                            post_duration: float = 3.0,
                            name: str = None) -> dict:
        """Execute a closed-loop step response for PID validation.

        Switches to AUTO mode, injects a setpoint offset so the PID
        drives to a known target, then clears the offset and restores
        the original mode.  Returns the logging summary.

        Raises StepAborted if marker lost or mode changed during execution.
        """
        from calibration.closed_loop_runner import ClosedLoopStep, ClosedLoopRunner, StepAborted

        step = ClosedLoopStep(
            axis=axis, setpoint=setpoint,
            hold_duration=hold_duration,
            pre_duration=pre_duration,
            post_duration=post_duration,
        )
        runner = ClosedLoopRunner(self)
        try:
            return runner.run(step, run_name=name)
        except StepAborted:
            self._last_step_summary = getattr(runner, 'last_summary', None)
            raise

    def start_cl_step_async(self, axis, setpoint, hold_duration=5.0,
                            pre_duration=2.0, post_duration=3.0, name=None) -> dict:
        """Start a closed-loop validation step in a background thread. Non-blocking.
        Shares the same _step_thread/_step_lock as open-loop so only one step
        can run at a time.  Status polled via get_step_status()."""
        with self._step_lock:
            if self._step_thread is not None and self._step_thread.is_alive():
                return {"status": "already_running",
                        "message": "a step is already running; check /step/status"}

            self._step_result = None
            self._step_error = None
            self._step_abort.clear()

            from calibration.closed_loop_runner import ClosedLoopStep
            step = ClosedLoopStep(
                axis=axis, setpoint=setpoint,
                hold_duration=hold_duration,
                pre_duration=pre_duration, post_duration=post_duration,
            )
            step.validate()

            estimated = step.total_duration()

            self._step_thread = threading.Thread(
                target=self._cl_step_worker,
                args=(axis, setpoint, hold_duration, pre_duration, post_duration, name),
                daemon=True,
            )
            self._step_thread.start()

        return {
            "status": "running",
            "axis": axis,
            "setpoint": setpoint,
            "hold_duration": hold_duration,
            "estimated_duration": round(estimated, 1),
        }

    def _cl_step_worker(self, axis, setpoint, hold_duration, pre_duration, post_duration, name):
        """Background worker — runs the blocking closed-loop step, stores result/error."""
        try:
            self._step_result = self.run_closed_loop_step(
                axis=axis, setpoint=setpoint, hold_duration=hold_duration,
                pre_duration=pre_duration, post_duration=post_duration, name=name,
            )
        except Exception as exc:
            self._step_error = str(exc)
            self._step_partial = getattr(self, '_last_step_summary', None)
            print(f"[cl-step] background closed-loop step failed: {exc}", file=sys.stderr)
            traceback.print_exc(file=sys.stderr)

    # ------------------------------------------------------------------
    # velocity damper: accel bias calibration
    # ------------------------------------------------------------------
    def calibrate_accel_bias(self, duration_s: float = 3.0) -> dict:
        """Sample stationary accel for N seconds to capture mounting tilt + sensor offset.

        Must be called while the vehicle is still (on deck, in water, etc.).
        Resets damper velocity estimates after calibration.
        Returns {"bias_x": ..., "bias_y": ..., "n_samples": ...}.
        """
        print(f"[damper] Sampling accel bias for {duration_s:.1f}s — keep the vehicle still...")
        sum_x = sum_y = 0.0
        n = 0
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            telem = self.veh.get_telemetry()
            ax = telem.get('accel_x', 0.0) or 0.0
            ay = telem.get('accel_y', 0.0) or 0.0
            sum_x += ax
            sum_y += ay
            n += 1
            time.sleep(0.05)  # 20 Hz sampling

        if n > 0:
            self._accel_bias_x = sum_x / n
            self._accel_bias_y = sum_y / n

        # Reset dampers to start fresh
        if self._damper_x is not None:
            self._damper_x.reset()
        if self._damper_y is not None:
            self._damper_y.reset()

        result = {"bias_x": self._accel_bias_x, "bias_y": self._accel_bias_y,
                  "n_samples": n}
        print(f"[damper] Bias captured: ax={self._accel_bias_x:+.4f} "
              f"ay={self._accel_bias_y:+.4f} m/s² ({n} samples)")
        return result

    # ------------------------------------------------------------------
    # camera: continuous capture + pose, latest-frame-wins
    # ------------------------------------------------------------------
    def _camera_thread(self):
        import cv2

        RECONNECT_DELAY_S = 3.0

        while not self._stop.is_set():
            try:
                self.detector.start()
                with self._lock:
                    self._camera_status = {'connected': True, 'error': None}
            except Exception as exc:
                with self._lock:
                    self._camera_status = {'connected': False, 'error': str(exc)}
                time.sleep(RECONNECT_DELAY_S)
                continue

            try:
                while not self._stop.is_set():
                    pose, frame = self.detector.capture_and_detect()
                    ok, jpeg = cv2.imencode(
                        '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_QUALITY]
                    )
                    if ok:
                        with self._lock:
                            self._latest_pose = pose
                            self._latest_jpeg = jpeg.tobytes()
                            self._camera_status['error'] = None

                    # Record frame if a calibration run is active
                    if self._run_active and self._run_recorder is not None:
                        try:
                            idx = self._run_recorder.write(frame)
                            self._latest_frame_idx = idx
                        except Exception as exc:
                            print(f"[video] write error: {exc}", file=sys.stderr)
            except Exception as exc:
                with self._lock:
                    self._camera_status = {'connected': False, 'error': str(exc)}
                try:
                    self.detector.stop()
                except Exception:
                    pass
                time.sleep(RECONNECT_DELAY_S)

    def get_pose(self):
        with self._lock:
            if self._latest_pose is None:
                return None
            # drop the raw frame array -- not JSON-serializable and the
            # video feed already carries the image separately
            return {k: v for k, v in self._latest_pose.items() if k != 'frame'}

    def get_camera_status(self):
        with self._lock:
            status = dict(self._camera_status)
            status['marker_detected'] = self._latest_pose is not None
        return status

    def get_jpeg_frame(self):
        with self._lock:
            return self._latest_jpeg

    # ------------------------------------------------------------------
    # external sensors: SOS leak sensor, independent of the Pixhawk
    # entirely -- same reconnect-on-failure pattern as the other two
    # threads. (Used to also read an external ICM20948 IMU as a
    # cross-check; removed by design decision -- Pixhawk IMU only now.)
    # ------------------------------------------------------------------
    def _external_thread(self):
        RECONNECT_DELAY_S = 3.0
        POLL_INTERVAL_S = 0.2

        while not self._stop.is_set():
            try:
                self.external.start()
                with self._lock:
                    self._external_status = {'connected': True, 'error': None}
            except Exception as exc:
                with self._lock:
                    self._external_status = {'connected': False, 'error': str(exc)}
                time.sleep(RECONNECT_DELAY_S)
                continue

            try:
                while not self._stop.is_set():
                    reading = self.external.read()
                    with self._lock:
                        self._latest_external = reading
                        self._external_status['error'] = None
                    time.sleep(POLL_INTERVAL_S)
            except Exception as exc:
                with self._lock:
                    self._external_status = {'connected': False, 'error': str(exc)}
                try:
                    self.external.stop()
                except Exception:
                    pass
                time.sleep(RECONNECT_DELAY_S)

    def get_external_telemetry(self):
        with self._lock:
            status = dict(self._external_status)
            reading = dict(self._latest_external) if self._latest_external else {}
        reading.update(status)
        return reading
