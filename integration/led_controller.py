"""LEDController -- drives an APA102/DotStar LED strip as a unified status
indicator for the AUV recovery rig. All LEDs show the same color and pattern
simultaneously (the rig is underwater, viewed from far away).

Owns the SPI bus exclusively. Only one process should instantiate this at a
time -- the integration stack owns it during normal operation; led/led_test.py
is for bench testing when the integration stack is NOT running.

State mapping (3 colors, behavior-driven):
  Red   = error (fast blink = leak, slow pulse = MAVLink lost)
  White = camera working (solid=searching, slow pulse=detected, blink=aligning)
  Green = success (solid=ready, fast blink=recovered, lift crane)
"""

import math
import threading
import time

try:
    import board
    import adafruit_dotstar as dotstar
    _HAS_HARDWARE = True
except (ImportError, NotImplementedError, ValueError):
    _HAS_HARDWARE = False


# Pattern definitions (all in Hz or seconds)
SOLID = 'solid'
SLOW_PULSE = 'slow_pulse'    # brightness oscillates 30%-100% at 1 Hz
BLINK = 'blink'              # hard on/off at 1 Hz
FAST_BLINK = 'fast_blink'    # hard on/off at 4 Hz

# State -> (color, pattern, max_brightness) mapping
# Order of precedence: errors override everything, then recovery states
LED_STATES = {
    'leak':          ((255, 0, 0),    FAST_BLINK, 1.0),
    'disconnected':  ((255, 0, 0),    SLOW_PULSE, 0.5),
    'manual':        ((255, 255, 255), SOLID,      None),   # brightness from slider
    'SEARCHING':     ((255, 255, 255), SOLID,      1.0),
    # DETECTED and ALIGNING share the same LED stage: the moment the marker
    # is detected, the PID starts correcting. No operationally meaningful
    # difference for the operator.
    'DETECTED':      ((255, 255, 255), BLINK,      0.5),
    'ALIGNING':      ((255, 255, 255), BLINK,      0.5),
    'READY':         ((0, 255, 0),    FAST_BLINK, 1.0),
    'RECOVERING':    ((0, 255, 0),    SOLID,      0.5),
}

ERROR_STATES = {'leak', 'disconnected'}  # these override recovery states


class LEDController:
    def __init__(self, num_pixels=8, default_brightness=0.5):
        self.num_pixels = num_pixels
        self.default_brightness = default_brightness
        self._state = 'disconnected'
        self._manual_brightness = default_brightness
        self._failure_until = 0.0  # timestamp; failure flash active until this
        self._lock = threading.Lock()

        if _HAS_HARDWARE:
            self.pixels = dotstar.DotStar(
                board.SCK, board.MOSI, num_pixels,
                brightness=default_brightness, auto_write=False,
                baudrate=4000000,
            )
        else:
            self.pixels = None  # off-Pi testing

    def set_state(self, state_name, error_state=None):
        """Set the LED state. If error_state is provided (e.g. 'leak',
        'disconnected'), it overrides state_name. If state_name is 'manual',
        brightness comes from the manual slider (set_manual_brightness)."""
        with self._lock:
            if error_state and error_state in ERROR_STATES:
                self._state = error_state
            elif state_name in LED_STATES:
                self._state = state_name
            else:
                self._state = 'disconnected'

    def set_manual_brightness(self, brightness):
        """Set LED brightness for manual mode (0.0 to 1.0). Called from
        the web UI slider via HardwareManager."""
        with self._lock:
            self._manual_brightness = max(0.0, min(1.0, brightness))

    def get_state(self):
        with self._lock:
            return self._state

    def flash_failure(self, duration_s=1.5):
        """Brief red solid flash to indicate a backward state transition
        (e.g. marker lost during alignment). Distinct from persistent
        error patterns (leak = fast blink, disconnected = slow pulse)."""
        with self._lock:
            self._failure_until = time.time() + duration_s

    def update(self):
        """Called from the mavlink thread at 20 Hz. Computes the current
        color and brightness based on state, pattern, and elapsed time,
        then writes to the LED strip."""
        if self.pixels is None:
            return

        t = time.time()

        # Failure flash: brief red solid, overrides normal state.
        with self._lock:
            failure_active = t < self._failure_until
        if failure_active:
            self.pixels.fill((255, 0, 0))
            self.pixels.show()
            return

        with self._lock:
            state = self._state
            manual_b = self._manual_brightness

        if state not in LED_STATES:
            state = 'disconnected'

        color, pattern, max_brightness = LED_STATES[state]

        # Compute effective brightness based on pattern and time
        if pattern == SOLID:
            brightness = max_brightness if max_brightness is not None else manual_b
        elif pattern == SLOW_PULSE:
            # Oscillate between 30% and 100% of max_brightness at 1 Hz
            base = max_brightness if max_brightness is not None else manual_b
            phase = 0.5 + 0.5 * (0.5 + 0.5 * math.sin(2 * math.pi * t))
            brightness = base * (0.3 + 0.7 * phase)
        elif pattern == BLINK:
            # Hard on/off at 1 Hz
            base = max_brightness if max_brightness is not None else manual_b
            brightness = base if int(t * 2) % 2 == 0 else 0.0
        elif pattern == FAST_BLINK:
            # Hard on/off at 4 Hz
            brightness = max_brightness if int(t * 8) % 2 == 0 else 0.0
        else:
            brightness = 0.0

        brightness = max(0.0, min(1.0, brightness))
        scaled_color = tuple(int(c * brightness) for c in color)

        self.pixels.fill(scaled_color)
        self.pixels.show()

    def off(self):
        if self.pixels is not None:
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
