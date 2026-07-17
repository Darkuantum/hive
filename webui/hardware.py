"""
hardware.py

Thin, thread-safe wrapper around the existing integration/ modules
(MavlinkInterface, ArucoDetector) for the web UI. This module does not
modify those files -- it only imports and drives them from background
threads, and exposes plain get/set methods that app.py's Flask routes
call. Safe to import and exercise without Flask at all.

Three background threads:
  - mavlink thread: connects, drains telemetry, and re-sends the latest
    manual-control stick command at a steady rate (this is also what
    lets a MANUAL_CONTROL-driven vehicle keep moving between HTTP posts).
  - watchdog: implemented inline in the mavlink thread -- if the web
    client stops posting new control values for CONTROL_TIMEOUT_S, the
    sticks are zeroed automatically, same fail-to-stop idea as the rest
    of the stack (see README "control loop").
  - camera thread: continuously grabs frames + ArUco pose so the video
    feed and telemetry API are never blocked on a slow capture.
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

CONTROL_TIMEOUT_S = 0.5    # zero the sticks if nothing posted for this long
CONTROL_RATE_HZ = 10
CAMERA_JPEG_QUALITY = 80


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

        self._control = {'x': 0.0, 'y': 0.0, 'r': 0.0}
        self._control_updated_at = 0.0

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
    # mavlink: telemetry in, sticks out, watchdog inline
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
                x, y, r = self._current_control()
                # z is fixed at neutral (0.5): this platform has no
                # vertical thrusters (see mavlink_interface.send_manual_control).
                self.veh.send_manual_control(x=x, y=y, z=0.5, r=r)
                with self._lock:
                    self._mavlink_status['error'] = None
            except Exception as exc:
                with self._lock:
                    self._mavlink_status['error'] = str(exc)
            time.sleep(max(0.0, period - (time.time() - loop_start)))

    def _current_control(self):
        with self._lock:
            age = time.time() - self._control_updated_at
            if age > CONTROL_TIMEOUT_S:
                return 0.0, 0.0, 0.0
            return self._control['x'], self._control['y'], self._control['r']

    def set_control(self, x, y, r):
        with self._lock:
            self._control = {'x': _clamp(x), 'y': _clamp(y), 'r': _clamp(r)}
            self._control_updated_at = time.time()

    def arm(self):
        self.veh.arm()

    def disarm(self):
        self.set_control(0.0, 0.0, 0.0)
        self.veh.disarm()

    def set_mode(self, mode_name):
        return self.veh.set_mode(mode_name)

    def get_telemetry(self):
        with self._lock:
            status = dict(self._mavlink_status)
            control = dict(self._control)
            never_sent = self._control_updated_at == 0.0
            control_age = None if never_sent else time.time() - self._control_updated_at
        telem = self.veh.get_telemetry_deg()
        telem['mode'] = self.veh.get_mode_name()
        telem['control'] = control
        telem['control_age_s'] = control_age
        telem['watchdog_tripped'] = never_sent or control_age > CONTROL_TIMEOUT_S
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
