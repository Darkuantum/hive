"""
external_sensors.py

Wraps the RPi-direct SOS Leak Sensor (GPIO) in the same class pattern as
ArucoDetector/MavlinkInterface, so hardware.py can drive it from its own
background thread the same way as everything else.

NOTE: this used to also read an external ICM20948 IMU (I2C) as a
cross-check against the Pixhawk's own IMU. That's been removed by
design decision -- the project now relies solely on the Pixhawk's IMU,
so this module is leak-sensor-only.

Wiring recap:
  SOS Leak Sensor:  VCC -> Pi pin 1 (3.3V), GND -> Pi pin 9, SIG -> GPIO17 (pin 11)
"""

import RPi.GPIO as GPIO

LEAK_PIN = 17

print(f"[external_sensors] loaded from {__file__} -- leak-only, no I2C/IMU code (v2)")


class ExternalSensors:
    """Call start() once, then read() repeatedly (e.g. once per
    background-thread loop iteration), then stop() on shutdown."""

    def __init__(self, leak_pin=LEAK_PIN):
        self.leak_pin = leak_pin
        self._gpio_ready = False

    def start(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.leak_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self._gpio_ready = True

    def stop(self):
        if self._gpio_ready:
            GPIO.cleanup()
            self._gpio_ready = False

    def read(self):
        """Returns a dict: {leak}. Raises if the GPIO read fails --
        caller (hardware.py) handles reconnect/retry, same pattern as
        the camera and mavlink threads."""
        leak = bool(GPIO.input(self.leak_pin))
        return {'leak': leak}
