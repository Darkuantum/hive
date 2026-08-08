"""
leak_test.py

Standalone bench test for just the SOS Leak Sensor (Blue Robotics) --
simple digital GPIO input. Split out of leak_imu_test.py for testing
the leak sensor in isolation, without needing the IMU wired up.

Wiring recap:
  SOS Leak Sensor:  VCC -> Pi pin 1 (3.3V), GND -> Pi pin 9, SIG -> GPIO17 (pin 11)

Install (if not already):
    python3 -m pip install RPi.GPIO --break-system-packages
"""

import time

import RPi.GPIO as GPIO

LEAK_PIN = 17          # GPIO17, physical pin 11
POLL_INTERVAL_S = 0.5


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LEAK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    print("Leak sensor test running. Press Ctrl+C to stop.\n")
    print("Touch the leak probe with a wet finger to test LEAK detection.\n")

    try:
        while True:
            leak = bool(GPIO.input(LEAK_PIN))
            print(f"leak={'LEAK!' if leak else 'dry '}")
            time.sleep(POLL_INTERVAL_S)

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("\nDone.")


if __name__ == "__main__":
    main()
