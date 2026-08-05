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
import threading
import time

from pymavlink import mavutil
from mavlink_interface import MavlinkInterface
from pose_controller import PoseController, camera_to_body_yaw
from decision_engine import DecisionEngine

CONTROL_TIMEOUT_S = 0.5    # manual mode only: zero sticks if nothing posted for this long
CONTROL_RATE_HZ = 20
CAMERA_JPEG_QUALITY = 80
DEFAULT_MANUAL_POWER = 1.0  # 100% -- manual-mode thruster scale, resets here on every connect

VALID_MODES = ('manual', 'auto')


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(v)))


class HardwareManager:
    def __init__(self, mavlink_conn='/dev/serial0', mavlink_baud=57600,
                 enable_camera=True, camera_kwargs=None, enable_external=True,
                 enable_led=True, num_leds=8,
                 pose_controller_kw=None, engine_kw=None):
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
        self.controller = PoseController(**(pose_controller_kw or {}))
        self.engine = DecisionEngine(**(engine_kw or {}))
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
            vx, vy, yaw_rate = self.controller.compute(
                pose['x'], pose['y'], pose['z'], pose['yaw'], dt
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

    def set_control(self, x, y, r):
        """Manual-mode stick input from the web UI. Silently has no
        effect while in auto mode -- the mavlink thread only reads
        this in 'manual' mode -- but we still store it, so switching
        back to manual doesn't require an extra click before sticks
        respond."""
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
    # external sensors: ICM20948 + SOS leak sensor, independent of the
    # Pixhawk entirely -- same reconnect-on-failure pattern as the
    # other two threads.
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
