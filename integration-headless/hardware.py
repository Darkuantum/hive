"""
hardware.py

Thin, thread-safe wrapper around the existing integration/ modules
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
import os
import sys
import threading
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION_DIR = os.path.abspath(os.path.join(_THIS_DIR, '..', 'integration'))
if _INTEGRATION_DIR not in sys.path:
    sys.path.insert(0, _INTEGRATION_DIR)

from mavlink_interface import MavlinkInterface  # noqa: E402
from pose_controller import PoseController       # noqa: E402
from decision_engine import DecisionEngine       # noqa: E402

CONTROL_TIMEOUT_S = 0.5    # manual mode only: zero sticks if nothing posted for this long
CONTROL_RATE_HZ = 10
CAMERA_JPEG_QUALITY = 80

VALID_MODES = ('manual', 'auto')


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(v)))


class HardwareManager:
    def __init__(self, mavlink_conn='/dev/serial0', mavlink_baud=57600,
                 enable_camera=True, camera_kwargs=None):
        self.enable_camera = enable_camera

        self.veh = MavlinkInterface(mavlink_conn, baud=mavlink_baud)

        self.detector = None
        if enable_camera:
            # Imported lazily so --no-camera bench-testing works on a
            # machine without picamera2/cv2 installed (e.g. off-Pi).
            from camFinal import ArucoDetector
            self.detector = ArucoDetector(**(camera_kwargs or {}))

        self._lock = threading.Lock()
        self._mavlink_status = {'connected': False, 'error': None}
        self._camera_status = {'connected': False, 'error': None}

        # ---- manual mode state ----
        self._control = {'x': 0.0, 'y': 0.0, 'r': 0.0}
        self._control_updated_at = 0.0

        # ---- mode + auto mode state ----
        self._mode = 'manual'
        self.controller = PoseController()
        self.engine = DecisionEngine()
        self._auto_status = {
            'state': self.engine.state.name,
            'controlling': False,
            'stick': {'x': 0.0, 'y': 0.0, 'r': 0.0},
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
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        if self.detector is not None:
            self.detector.stop()

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
                }

    def get_control_mode(self):
        with self._lock:
            return self._mode

    # ------------------------------------------------------------------
    # mavlink: telemetry in, sticks out (manual or auto), watchdog inline
    # ------------------------------------------------------------------
    def _mavlink_thread(self):
        try:
            self.veh.connect()
            with self._lock:
                self._mavlink_status = {'connected': True, 'error': None}
        except Exception as exc:
            with self._lock:
                self._mavlink_status = {'connected': False, 'error': str(exc)}
            return  # no link -- nothing more this thread can usefully do

        period = 1.0 / CONTROL_RATE_HZ
        while not self._stop.is_set():
            loop_start = time.time()
            try:
                self.veh.update(blocking=False)

                mode = self.get_control_mode()
                if mode == 'auto':
                    x, y, r = self._compute_auto_control()
                else:
                    x, y, r = self._current_manual_control()

                self.veh.send_manual_control(x=x, y=y, z=0.5, r=r)
                with self._lock:
                    self._mavlink_status['error'] = None
            except Exception as exc:
                with self._lock:
                    self._mavlink_status['error'] = str(exc)
            time.sleep(max(0.0, period - (time.time() - loop_start)))

    def _current_manual_control(self):
        with self._lock:
            age = time.time() - self._control_updated_at
            if age > CONTROL_TIMEOUT_S:
                return 0.0, 0.0, 0.0
            return self._control['x'], self._control['y'], self._control['r']

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

        if self.engine.is_controlling() and marker_detected:
            vx, vy, yaw_rate = self.controller.compute(
                pose['x'], pose['y'], pose['z'], pose['yaw'], dt
            )
            x = vx / self.controller.pid_surge.output_limit
            y = vy / self.controller.pid_sway.output_limit
            r = yaw_rate / self.controller.pid_yaw.output_limit
        else:
            x = y = r = 0.0
            self.controller.reset()

        with self._lock:
            self._auto_status = {
                'state': state.name,
                'controlling': self.engine.is_controlling(),
                'stick': {'x': x, 'y': y, 'r': r},
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
        telem = self.veh.get_telemetry_deg()
        telem['mode'] = self.veh.get_mode_name()
        telem['control_mode'] = control_mode
        telem['control'] = control
        telem['control_age_s'] = control_age
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

        try:
            self.detector.start()
            with self._lock:
                self._camera_status = {'connected': True, 'error': None}
        except Exception as exc:
            with self._lock:
                self._camera_status = {'connected': False, 'error': str(exc)}
            return

        while not self._stop.is_set():
            try:
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
                    self._camera_status['error'] = str(exc)
                time.sleep(0.5)

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
