"""Drive an APA102/SK9822 (DotStar) LED strip over the Pi's hardware SPI pins.

Wiring:
  5V -> Pi 5V (pin 2/4)   G -> Pi GND (pin 6)
  Di -> Pi GPIO10 / MOSI (pin 19)   Ci -> Pi GPIO11 / SCLK (pin 23)

Requires SPI enabled (raspi-config > Interface Options > SPI, then reboot).
"""
import argparse
import time

import board
import adafruit_dotstar as dotstar


def color_chase(pixels, num_pixels, color, wait):
    for i in range(num_pixels):
        pixels[i] = color
        pixels.show()
        time.sleep(wait)


def run_test(pixels, num_pixels):
    print("Running LED test... Press Ctrl+C to stop.")
    while True:
        color_chase(pixels, num_pixels, (255, 0, 0), 0.1)
        color_chase(pixels, num_pixels, (0, 255, 0), 0.1)
        color_chase(pixels, num_pixels, (0, 0, 255), 0.1)

        pixels.fill((255, 255, 255))
        pixels.show()
        time.sleep(0.5)

        pixels.fill((0, 0, 0))
        pixels.show()
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="APA102/DotStar LED strip control (SPI)")
    parser.add_argument("--num-pixels", type=int, default=8, help="Number of LEDs on the strip (default: 8)")
    parser.add_argument("--brightness", type=float, default=0.5, help="0.0-1.0 (default: 0.5)")
    parser.add_argument("--mode", choices=["test", "on", "off"], default="test",
                         help="'test' runs a chase/pulse pattern, 'on' fills white and holds, 'off' clears the strip")
    parser.add_argument("--baudrate", type=int, default=4000000,
                         help="SPI clock rate in Hz (default: 4000000). Lower this (e.g. 400000) if an "
                              "uncut/long strip or loose jumper wiring is causing signal integrity issues.")
    args = parser.parse_args()

    pixels = dotstar.DotStar(board.SCK, board.MOSI, args.num_pixels,
                              brightness=args.brightness, auto_write=False, baudrate=args.baudrate)

    try:
        if args.mode == "test":
            run_test(pixels, args.num_pixels)
        elif args.mode == "on":
            pixels.fill((255, 255, 255))
            pixels.show()
            print(f"{args.num_pixels} LEDs on at brightness {args.brightness}. Press Ctrl+C to turn off.")
            while True:
                time.sleep(1)
        elif args.mode == "off":
            pixels.fill((0, 0, 0))
            pixels.show()
    except KeyboardInterrupt:
        pixels.fill((0, 0, 0))
        pixels.show()
        print("\nLEDs turned off.")


if __name__ == "__main__":
    main()
