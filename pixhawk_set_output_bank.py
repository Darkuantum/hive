#!/usr/bin/env python3
"""
pixhawk_set_output_bank.py

Move ArduSub's Motor1-4 output assignment between the Pixhawk's MAIN
bank (SERVO1-4, IOMCU-driven, plain PWM only) and AUX bank (SERVO11-14,
i.e. AUX3-6, FMU-direct, DShot + bidirectional-DShot telemetry capable).

Background: this rig's thrusters are normally wired to MAIN1-4. AUX3-6
was chosen over AUX1/AUX2 because this frame's stock BlueROV2-default
params already claim those for Lights1/camera-tilt, and AUX1 doesn't
support bidirectional DShot at all regardless.

This script only writes the FC-side parameters (SERVOx_FUNCTION,
MOT_PWM_TYPE, SERVO_BLH_MASK/3DMASK) for whichever bank you choose --
it does NOT move any wiring. You still have to physically swap the 4
signal+ground connectors between MAIN1-4 and AUX3-6 yourself to match,
and reboot the FC after applying a bank for the change to take effect.

Usage:
    python3 pixhawk_set_output_bank.py aux      # thrusters on AUX3-6, DShot600 + telemetry
    python3 pixhawk_set_output_bank.py main     # thrusters back on MAIN1-4, plain PWM
    python3 pixhawk_set_output_bank.py --check  # report current bank only, no changes
"""
import argparse
import sys
import time

from pymavlink import mavutil

MAVLINK_PORT = "/dev/ttyACM0"
MAVLINK_BAUD = 115200

# physical output channel -> ArduSub motor number, per bank
MAIN_CHANNELS = {1: 1, 2: 2, 3: 3, 4: 4}      # SERVO1-4
AUX_CHANNELS = {11: 1, 12: 2, 13: 3, 14: 4}   # SERVO11-14 (AUX3-6)

MOTOR_FUNCTION_IDS = {1: 33, 2: 34, 3: 35, 4: 36}  # ArduPilot SERVOx_FUNCTION for Motor1-4


def connect():
    print(f"[MAV] Connecting on {MAVLINK_PORT} @ {MAVLINK_BAUD}...")
    m = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD, robust_parsing=True)
    m.wait_heartbeat()
    print(f"[MAV] Heartbeat: sys {m.target_system} comp {m.target_component}")
    return m


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if p is not None and p.param_id.rstrip("\x00") == name:
            return p.param_value
    return None


def set_param(m, name, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), param_type)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if p is not None and p.param_id.rstrip("\x00") == name:
            return p.param_value
    return None


def detect_bank(m):
    """Read live SERVOx_FUNCTION to find where Motor1-4 currently are."""
    found = {}
    for chan in range(1, 17):
        val = get_param(m, f"SERVO{chan}_FUNCTION", timeout=1.5)
        if val is not None and int(val) in (33, 34, 35, 36):
            found[int(val) - 32] = chan  # motor number -> channel
        if len(found) == 4:
            break
    if len(found) < 4:
        return "UNKNOWN", found
    banks = {"MAIN" if c <= 8 else "AUX" for c in found.values()}
    return (banks.pop() if len(banks) == 1 else "MIXED"), found


def apply_bank(m, bank):
    if bank == "aux":
        active, inactive = AUX_CHANNELS, MAIN_CHANNELS
        mot_pwm_type, blh_mask = 6, 15360  # DShot600; bits 10-13 = SERVO11-14
    else:
        active, inactive = MAIN_CHANNELS, AUX_CHANNELS
        mot_pwm_type, blh_mask = 0, 0  # plain PWM; MAIN can't do DShot/passthrough at all

    changes = [(f"SERVO{ch}_FUNCTION", 0) for ch in inactive]
    changes += [(f"SERVO{ch}_FUNCTION", MOTOR_FUNCTION_IDS[motor]) for ch, motor in active.items()]
    changes += [("MOT_PWM_TYPE", mot_pwm_type), ("SERVO_BLH_MASK", blh_mask), ("SERVO_BLH_3DMASK", blh_mask)]

    print(f"\n{'param':<20}{'requested':<12}{'confirmed':<12}ok")
    all_ok = True
    for name, val in changes:
        confirmed = set_param(m, name, val)
        ok = confirmed is not None and abs(confirmed - val) < 0.5
        all_ok &= ok
        print(f"{name:<20}{val!s:<12}{confirmed!s:<12}{'OK' if ok else 'MISMATCH/FAILED'}")
    print(
        "\nALL OK -- reboot the FC for this to take effect."
        if all_ok else
        "\nSOME FAILED -- review above before rebooting."
    )
    return all_ok


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bank", nargs="?", choices=["main", "aux"],
                         help="output bank to switch Motor1-4 to")
    parser.add_argument("--check", action="store_true",
                         help="only report the current bank, make no changes")
    args = parser.parse_args()

    if not args.bank and not args.check:
        parser.print_help()
        sys.exit(1)

    m = connect()

    if m.motors_armed():
        print("[ABORT] Vehicle is armed -- disarm before changing output config.")
        sys.exit(1)

    bank, mapping = detect_bank(m)
    print(f"\nCurrent bank: {bank}  (motor->channel: {mapping})")

    if args.check:
        return

    if bank.lower() == args.bank:
        print(f"Already on {args.bank.upper()} -- no changes needed.")
        return

    apply_bank(m, args.bank)


if __name__ == "__main__":
    main()
