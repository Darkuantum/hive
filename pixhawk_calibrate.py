#!/usr/bin/env python3
"""
Pixhawk / ArduSub Calibration -- motor test + compass
======================================================
ArduSub has NO throttle-learning "ESC calibration" (verified against a live
960-param dump on this Pixhawk: no ESC_CALIBRATION param exists at all). The
FC always drives a fixed PWM range (MOT_PWM_MIN/MOT_PWM_MAX, currently
1100/1900us). Whatever range the AM32 ESCs expect is set on the ESCs
themselves via the AM32 passthrough configurator -- a separate tool, not
this script and not QGC.

What IS available and what this script does:
  1. motor-test  Spin each of the 4 thrusters briefly, one at a time
                 (MAV_CMD_DO_MOTOR_TEST, board output order == MOT_n).
                 You confirm the direction is correct; if reversed, the
                 script flips the matching MOT_<n>_DIRECTION param for you.
  2. compass     Drives the standard onboard mag calibration
                 (MAV_CMD_DO_START_MAG_CAL / MAG_CAL_PROGRESS /
                 MAG_CAL_REPORT). You physically rotate the frame through
                 orientations while it samples; autosaves on success.

Run:  python3 pixhawk_calibrate.py motor-test
      python3 pixhawk_calibrate.py compass
"""

import sys
import time

from pymavlink import mavutil

from frame_station_keep_mavlink import set_mode_manual, arm, disarm

MAVLINK_PORT = "/dev/ttyACM0"
MAVLINK_BAUD = 115200

NUM_THRUSTERS = 4
MOTOR_TEST_THROTTLE_PCT = 15
MOTOR_TEST_DURATION_S = 1.5

COMPASS_CAL_TIMEOUT_S = 120


def connect():
    print(f"[MAV] Connecting on {MAVLINK_PORT} @ {MAVLINK_BAUD}...")
    m = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD)
    m.wait_heartbeat()
    print(f"[MAV] Heartbeat: sys {m.target_system} comp {m.target_component}")
    return m


def recv_match_safe(m, **kwargs):
    """pymavlink chokes on some unrelated instanced messages mid-stream;
    skip those rather than let them kill calibration."""
    while True:
        try:
            return m.recv_match(**kwargs)
        except TypeError:
            continue


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        p = recv_match_safe(m, type="PARAM_VALUE", blocking=True, timeout=0.3)
        if p is not None and p.param_id.rstrip("\x00") == name:
            return p
    return None


def set_param(m, name, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), param_type)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        p = recv_match_safe(m, type="PARAM_VALUE", blocking=True, timeout=0.3)
        if p is not None and p.param_id.rstrip("\x00") == name:
            return p.param_value
    return None


# =====================================================================
# MOTOR TEST
# =====================================================================

def motor_test_one(m, n, pct=MOTOR_TEST_THROTTLE_PCT, duration=MOTOR_TEST_DURATION_S):
    """Non-interactive: arm, spin a single thruster once, disarm, exit. Caller
    (a human physically present at the frame) reports what they observed
    out-of-band. ArduSub refuses MOTOR_TEST while disarmed (verified: it
    returns COMMAND_ACK FAILED with STATUSTEXT 'Arm motors before testing
    motors.'), so arming here is required, not optional."""
    direction_param = f"MOT_{n}_DIRECTION"
    current = get_param(m, direction_param)
    cur_val = current.param_value if current is not None else None
    print(f"[MOTOR TEST] Thruster {n} (output {n}, {direction_param}="
          f"{cur_val:+.0f})..." if cur_val is not None else
          f"[MOTOR TEST] Thruster {n} (output {n})...")

    set_mode_manual(m)
    if not arm(m):
        print("[MOTOR TEST] Arm failed -- aborting test.")
        return

    try:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            n,
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
            pct,
            duration,
            1,
            mavutil.mavlink.MOTOR_TEST_ORDER_BOARD, 0)

        time.sleep(duration + 0.3)
        print(f"[MOTOR TEST] Thruster {n} test pulse sent ({pct}% for {duration}s).")
    finally:
        disarm(m)


def motor_direction_set(m, n, value):
    direction_param = f"MOT_{n}_DIRECTION"
    current = get_param(m, direction_param)
    cur_val = current.param_value if current is not None else None
    result = set_param(m, direction_param, value)
    print(f"[MOTOR TEST] {direction_param}: {cur_val} -> {result:+.0f}")


def motor_test(m):
    print(
        "\n[MOTOR TEST] Spins each of the 4 thrusters briefly at "
        f"{MOTOR_TEST_THROTTLE_PCT}% throttle, one at a time.\n"
        "Confirm props/thrusters are clear before continuing.\n"
        "If your Pixhawk has a physical safety switch, press it in first.\n"
    )
    ans = input("Type 'go' to proceed: ").strip().lower()
    if ans != "go":
        print("[MOTOR TEST] Aborted.")
        return

    set_mode_manual(m)
    if not arm(m):
        print("[MOTOR TEST] Arm failed -- aborting (ArduSub refuses MOTOR_TEST while disarmed).")
        return

    try:
        _motor_test_loop(m)
    finally:
        disarm(m)


def _motor_test_loop(m):
    for n in range(1, NUM_THRUSTERS + 1):
        direction_param = f"MOT_{n}_DIRECTION"
        current = get_param(m, direction_param)
        cur_val = current.param_value if current is not None else None
        print(f"\n[MOTOR TEST] Thruster {n} (output {n}, {direction_param}="
              f"{cur_val:+.0f})..." if cur_val is not None else
              f"\n[MOTOR TEST] Thruster {n} (output {n})...")

        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            n,                                                  # motor number (board/output order)
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,        # throttle type
            MOTOR_TEST_THROTTLE_PCT,                            # throttle value
            MOTOR_TEST_DURATION_S,                              # timeout (s)
            1,                                                  # motor count
            mavutil.mavlink.MOTOR_TEST_ORDER_BOARD, 0)

        time.sleep(MOTOR_TEST_DURATION_S + 0.3)

        resp = input(
            f"  Did thruster {n} spin, and in the expected direction? "
            "[y=ok / r=reversed / s=skip]: "
        ).strip().lower()

        if resp == "r":
            if cur_val is None:
                print(f"  [WARN] Could not read {direction_param}; skipping flip.")
                continue
            new_val = -cur_val
            result = set_param(m, direction_param, new_val)
            print(f"  [MOTOR TEST] {direction_param} set to {result:+.0f}")
        elif resp == "y":
            print(f"  [MOTOR TEST] Thruster {n} OK.")
        else:
            print(f"  [MOTOR TEST] Thruster {n} skipped.")

    print("\n[MOTOR TEST] Done. Re-run this test after any direction flips to confirm.")


# =====================================================================
# COMPASS CALIBRATION
# =====================================================================

MAG_CAL_STATUS_NAMES = {
    mavutil.mavlink.MAG_CAL_NOT_STARTED: "NOT_STARTED",
    mavutil.mavlink.MAG_CAL_WAITING_TO_START: "WAITING_TO_START",
    mavutil.mavlink.MAG_CAL_RUNNING_STEP_ONE: "RUNNING_STEP_ONE",
    mavutil.mavlink.MAG_CAL_RUNNING_STEP_TWO: "RUNNING_STEP_TWO",
    mavutil.mavlink.MAG_CAL_SUCCESS: "SUCCESS",
    mavutil.mavlink.MAG_CAL_FAILED: "FAILED",
    mavutil.mavlink.MAG_CAL_BAD_ORIENTATION: "BAD_ORIENTATION",
    mavutil.mavlink.MAG_CAL_BAD_RADIUS: "BAD_RADIUS",
}


def compass_cal(m, skip_prompt=False):
    print(
        "\n[COMPASS CAL] Standard ArduPilot onboard mag calibration.\n"
        "Once started, slowly rotate the frame through all orientations\n"
        "(nose up/down, roll left/right, spin on each axis) until progress\n"
        "reaches 100% on every compass. No motors will move for this.\n"
    )
    if not skip_prompt:
        ans = input("Type 'go' to start: ").strip().lower()
        if ans != "go":
            print("[COMPASS CAL] Aborted.")
            return

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL, 0,
        0,      # compass bitmask, 0 = all
        1,      # retry on failure
        1,      # autosave on success
        0, 0, 0, 0)

    print("[COMPASS CAL] Started. Rotate the frame now...")
    deadline = time.monotonic() + COMPASS_CAL_TIMEOUT_S
    reports = {}
    last_print = 0.0

    while time.monotonic() < deadline:
        msg = recv_match_safe(
            m, type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT"],
            blocking=True, timeout=1.0)
        if msg is None:
            continue

        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(f"[FC] {msg.text}")
            continue

        if mtype == "MAG_CAL_PROGRESS":
            now = time.monotonic()
            if now - last_print > 0.5:
                status = MAG_CAL_STATUS_NAMES.get(msg.cal_status, msg.cal_status)
                print(f"  compass {msg.compass_id}: {msg.completion_pct:3d}%  ({status})",
                      end="\r")
                last_print = now

        elif mtype == "MAG_CAL_REPORT":
            status = MAG_CAL_STATUS_NAMES.get(msg.cal_status, msg.cal_status)
            reports[msg.compass_id] = status
            print(f"\n[COMPASS CAL] compass {msg.compass_id} report: {status} "
                  f"(fitness={msg.fitness:.2f})")

            if all(s == "SUCCESS" for s in reports.values()) and len(reports) >= 1:
                print("[COMPASS CAL] All reporting compasses succeeded.")
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, 0,
                    0, 0, 0, 0, 0, 0, 0)
                print("[COMPASS CAL] Accepted + saved.")
                return
            if any(s not in ("SUCCESS",) and s in
                   ("FAILED", "BAD_ORIENTATION", "BAD_RADIUS") for s in reports.values()):
                print("[COMPASS CAL] One or more compasses failed. Re-run to retry.")
                return

    print("\n[COMPASS CAL] Timed out waiting for calibration to complete.")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0,
        0, 0, 0, 0, 0, 0, 0)


# =====================================================================
# MAIN
# =====================================================================

def main():
    args = sys.argv[1:]
    usage = (f"Usage: {sys.argv[0]} motor-test\n"
             f"       {sys.argv[0]} motor-test-one <n> [pct] [duration_s]\n"
             f"       {sys.argv[0]} motor-direction <n> <1|-1>\n"
             f"       {sys.argv[0]} compass [--yes]")
    if not args:
        print(usage)
        sys.exit(1)

    cmd = args[0]
    m = connect()

    if cmd == "motor-test":
        motor_test(m)
    elif cmd == "motor-test-one":
        n = int(args[1])
        pct = float(args[2]) if len(args) > 2 else MOTOR_TEST_THROTTLE_PCT
        duration = float(args[3]) if len(args) > 3 else MOTOR_TEST_DURATION_S
        motor_test_one(m, n, pct, duration)
    elif cmd == "motor-direction":
        n = int(args[1])
        value = float(args[2])
        motor_direction_set(m, n, value)
    elif cmd == "compass":
        compass_cal(m, skip_prompt=("--yes" in args))
    else:
        print(usage)
        sys.exit(1)


if __name__ == "__main__":
    main()
