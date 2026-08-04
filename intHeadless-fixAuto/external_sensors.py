"""
external_sensors.py

Wraps the RPi-direct sensors -- ICM20948 IMU (I2C) and SOS Leak Sensor
(GPIO) -- in the same class pattern as ArucoDetector/MavlinkInterface,
so hardware.py can drive it from its own background thread the same
way as everything else. This is entirely independent of the Pixhawk;
it exists to give an external, cross-check reading against the
Pixhawk's own IMU (see the webui's "External Sensors (RPi)" panel).

Wiring recap:
  SOS Leak Sensor:  VCC -> Pi pin 1 (3.3V), GND -> Pi pin 9, SIG -> GPIO17 (pin 11)
  ICM20948:         VIN -> Pi pin 1 (3.3V), GND -> Pi pin 6, SDA -> pin 3, SCL -> pin 5
"""

import math

import RPi.GPIO as GPIO
import board
import busio
from adafruit_icm20x import ICM20948

LEAK_PIN = 17
ICM_ADDRESS = 0x69  # confirmed via i2cdetect -- not the 0x68 default


def _accel_to_tilt_deg(ax, ay, az):
    """Rough roll/pitch estimate from raw accelerometer alone -- see
    leak_imu_test.py for the fuller explanation. Not a fused estimate
    like the Pixhawk's, so treat as a rough cross-check, not a precise
    match."""
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))
    return roll, pitch


class ExternalSensors:
    """Call start() once, then read() repeatedly (e.g. once per
    background-thread loop iteration), then stop() on shutdown."""

    def __init__(self, leak_pin=LEAK_PIN, icm_address=ICM_ADDRESS):
        self.leak_pin = leak_pin
        self.icm_address = icm_address
        self.icm = None
        self._gpio_ready = False

    def start(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.leak_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self._gpio_ready = True

        i2c = busio.I2C(board.SCL, board.SDA)
        self.icm = ICM20948(i2c, address=self.icm_address)

    def stop(self):
        if self._gpio_ready:
            GPIO.cleanup()
            self._gpio_ready = False

    def read(self):
        """Returns a dict: {leak, accel_x/y/z, gyro_x/y/z, roll_deg,
        pitch_deg, heading_deg}. Raises if a sensor read fails --
        caller (hardware.py) handles reconnect/retry, same pattern as
        the camera and mavlink threads."""
        leak = bool(GPIO.input(self.leak_pin))

        ax, ay, az = self.icm.acceleration   # m/s^2
        gx, gy, gz = self.icm.gyro           # rad/s
        mx, my, mz = self.icm.magnetic       # microtesla

        roll_deg, pitch_deg = _accel_to_tilt_deg(ax, ay, az)

        # Simple (non-tilt-compensated) heading -- accurate only when
        # roughly level, and sensitive to nearby ferrous metal/motor
        # interference, same caveat as any compass. Good for a rough
        # sanity check against the Pixhawk's yaw_deg, not a precision
        # match.
        heading_deg = math.degrees(math.atan2(my, mx))
        if heading_deg < 0:
            heading_deg += 360

        return {
            'leak': leak,
            'accel_x': ax, 'accel_y': ay, 'accel_z': az,
            'gyro_x': gx, 'gyro_y': gy, 'gyro_z': gz,
            'roll_deg': roll_deg, 'pitch_deg': pitch_deg,
            'yaw_deg': heading_deg,
        }
