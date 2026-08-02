"""
leak_imu_test.py

Combined bench test for the two new sensors:
  - SOS Leak Sensor (Blue Robotics) -- simple digital GPIO input
  - ICM20948 IMU (Adafruit) -- I2C, address 0x69 on this specific board

Run standalone to confirm both are wired correctly and reading sensibly
before building anything more permanent around them.

Wiring recap:
  SOS Leak Sensor:  VCC -> Pi pin 1 (3.3V), GND -> Pi pin 9, SIG -> GPIO17 (pin 11)
  ICM20948:         VIN -> Pi pin 1 (3.3V), GND -> Pi pin 6, SDA -> pin 3, SCL -> pin 5

Install (if not already):
    python3 -m pip install RPi.GPIO adafruit-circuitpython-icm20x adafruit-blinka --break-system-packages
"""

import math
import time

import RPi.GPIO as GPIO
import board
import busio
from adafruit_icm20x import ICM20948

LEAK_PIN = 17          # GPIO17, physical pin 11
ICM_ADDRESS = 0x69     # confirmed via i2cdetect -- this board is NOT the 0x68 default
POLL_INTERVAL_S = 0.5


def accel_to_tilt_deg(ax, ay, az):
    """Rough roll/pitch estimate from raw accelerometer alone (no gyro
    fusion) -- good enough for a side-by-side sanity check against the
    Pixhawk's roll_deg/pitch_deg, not a precision measurement. The
    Pixhawk's numbers come from a proper EKF fusing accel+gyro over
    time, so expect this to be noisier/jumpier, especially while moving."""
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))
    return roll, pitch


def main():
    # ---- Leak sensor setup ----
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LEAK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # ---- IMU setup ----
    i2c = busio.I2C(board.SCL, board.SDA)
    icm = ICM20948(i2c, address=ICM_ADDRESS)

    print("Leak sensor + ICM20948 test running. Press Ctrl+C to stop.\n")
    print("Touch the leak probe with a wet finger to test LEAK detection.")
    print("Move/tilt the IMU board to confirm accel/gyro respond.\n")

    try:
        while True:
            leak = bool(GPIO.input(LEAK_PIN))

            ax, ay, az = icm.acceleration   # m/s^2
            gx, gy, gz = icm.gyro           # rad/s
            roll_deg, pitch_deg = accel_to_tilt_deg(ax, ay, az)

            print(
                f"leak={'LEAK!' if leak else 'dry '}   "
                f"accel(x={ax:+.2f} y={ay:+.2f} z={az:+.2f})  "
                f"gyro(x={gx:+.2f} y={gy:+.2f} z={gz:+.2f})  "
                f"~roll={roll_deg:+.1f}deg ~pitch={pitch_deg:+.1f}deg"
            )
            time.sleep(POLL_INTERVAL_S)

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("\nDone.")


if __name__ == "__main__":
    main()
