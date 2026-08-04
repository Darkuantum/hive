#!/usr/bin/env python3
"""
Reactive per-motor mixer test -- bypasses ArduSub's automatic vectored
mixer entirely and instead fires specific thruster pairs directly based
on live attitude/accel readings, per this mapping:

  YAW right (heading increasing) -> motors 1, 4
  YAW left  (heading decreasing) -> motors 2, 3
  X right   (+ body accel X)     -> motors 1, 3
  X left    (- body accel X)     -> motors 2, 4
  Y up      (+ body accel Y)     -> motors 1, 2
  Y down    (- body accel Y)     -> motors 3, 4

Roll/pitch/Z are ignored -- this frame has no vertical thrusters and no
roll/pitch control.

Each active motor is driven with MAV_CMD_DO_MOTOR_TEST (the same
mechanism already verified working on this rig), re-issued on a refresh
interval shorter than the pulse duration while its trigger condition
holds. Stop moving the frame -> the trigger clears -> the pulse isn't
refreshed -> it auto-times-out on its own.

Run: python3 reactive_motor_mapper.py
"""

import math
import signal
import sys
import time

from pymavlink import mavutil

_orig_add_message = mavutil.add_message
def _safe_add_message(messages, mtype, msg):
    if msg._instance_field is not None and getattr(msg, msg._instance_field, None) is not None:
        existing = messages.get(mtype)
        if existing is not None and getattr(existing, "_instances", None) is None:
            existing._instances = {}
    return _orig_add_message(messages, mtype, msg)
mavutil.add_message = _safe_add_message

MAVLINK_PORT = "/dev/ttyACM0"
MAVLINK_BAUD = 115200
ARDUSUB_MODE_MANUAL = 19

MOTOR_TEST_PCT = 25.0         # throttle %, modest but visible
MOTOR_TEST_DURATION_S = 0.4   # each DO_MOTOR_TEST pulse auto-stops after this
REFRESH_S = 0.2               # re-issue while trigger holds (< duration so it never gaps)

YAW_THRESHOLD_DEG = 12.0
ACCEL_LPF_HZ = 2.0
ACCEL_DEADBAND = 0.15
ACCEL_THRESHOLD = 0.6         # m/s^2 filtered, beyond deadband -> "moving" trigger

ACCEL_CAL_SECONDS = 2.0

PAIR_YAW_RIGHT = (1, 4)
PAIR_YAW_LEFT = (2, 3)
PAIR_X_RIGHT = (1, 3)
PAIR_X_LEFT = (2, 4)
PAIR_Y_UP = (1, 2)
PAIR_Y_DOWN = (3, 4)


def connect():
    print(f"[MAV] Connecting on {MAVLINK_PORT} @ {MAVLINK_BAUD}...")
    m = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD, robust_parsing=True)
    m.wait_heartbeat()
    print(f"[MAV] Heartbeat: sys {m.target_system} comp {m.target_component}")
    interval_us = int(1e6 / 50)
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                   mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
                   mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0)
    return m


def set_mode_manual(m):
    print("[MAV] Setting MANUAL mode...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        ARDUSUB_MODE_MANUAL, 0, 0, 0, 0, 0)
    time.sleep(1)


def arm(m, timeout_s=10.0):
    print("[MAV] Arming...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"[FC] {msg.text}")
        elif msg.get_type() == "HEARTBEAT" and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("[MAV] Armed.")
            return True
    print("[MAV] Arm timed out -- see [FC] messages above.")
    return False


def disarm(m):
    try:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
    except Exception:
        pass


def motor_test(m, n, pct=MOTOR_TEST_PCT, duration=MOTOR_TEST_DURATION_S):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        n, mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT, pct, duration, 1,
        mavutil.mavlink.MOTOR_TEST_ORDER_BOARD, 0)


def calibrate_accel_bias(m, duration_s=ACCEL_CAL_SECONDS):
    print(f"[CAL] Sampling accel bias for {duration_s:.1f}s -- keep the frame still...")
    sum_ax = sum_ay = 0.0
    n = 0
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        msg = m.recv_match(type=["SCALED_IMU2", "RAW_IMU"], blocking=True, timeout=0.2)
        if msg is None:
            continue
        sum_ax += msg.xacc * 9.80665 / 1000.0
        sum_ay += msg.yacc * 9.80665 / 1000.0
        n += 1
    if n == 0:
        print("[CAL] No IMU samples -- skipping bias correction (bias=0).")
        return 0.0, 0.0
    ax_bias, ay_bias = sum_ax / n, sum_ay / n
    print(f"[CAL] Bias: ax={ax_bias:+.4f} ay={ay_bias:+.4f} m/s^2")
    return ax_bias, ay_bias


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    m = connect()

    def shutdown(sig=None, frame=None):
        print("\n[SYS] Stopping - disarm.")
        disarm(m)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    reorigin = {"requested": False}

    def request_reorigin(sig=None, frame=None):
        reorigin["requested"] = True

    signal.signal(signal.SIGUSR1, request_reorigin)

    ax_bias, ay_bias = calibrate_accel_bias(m)

    set_mode_manual(m)
    if not arm(m):
        sys.exit(1)

    yaw_baseline = None
    last_yaw_deg = None
    accel_f_x = accel_f_y = 0.0
    origin_ax = origin_ay = 0.0
    dt_lpf = 1.0 / 50
    rc = 1.0 / (2 * math.pi * ACCEL_LPF_HZ)
    lpf_alpha = dt_lpf / (rc + dt_lpf)

    last_sent = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
    is_armed = True

    print("[SYS] Reactive mapping active. Ctrl+C to stop.")
    print("  YAW right -> 1,4   YAW left -> 2,3")
    print("  X right   -> 1,3   X left   -> 2,4")
    print("  Y up      -> 1,2   Y down   -> 3,4")
    print("[SYS] Waiting for origin: hold frame in neutral pose, then set origin.")

    while True:
        msg = m.recv_match(type=["ATTITUDE", "SCALED_IMU2", "RAW_IMU",
                                  "STATUSTEXT", "HEARTBEAT"],
                            blocking=True, timeout=0.1)
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "STATUSTEXT":
            print(f"\n[FC] {msg.text}")
            continue

        if mtype == "HEARTBEAT":
            now_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed and not now_armed:
                print("\n[WARN] Vehicle disarmed unexpectedly -- re-arming...")
                set_mode_manual(m)
                arm(m)
            is_armed = now_armed
            continue

        # Keep the manual-control link "alive" so ArduSub doesn't treat this
        # companion as disconnected (silent disarm) between motor pulses --
        # neutral values so it never fights the DO_MOTOR_TEST overrides.
        m.mav.manual_control_send(m.target_system, 0, 0, 500, 0, 0)

        if mtype == "ATTITUDE":
            last_yaw_deg = math.degrees(msg.yaw)
            if yaw_baseline is None:
                yaw_baseline = last_yaw_deg
                print(f"[YAW] Initial heading: {yaw_baseline:.1f} deg (send SIGUSR1 to re-set origin)")
        else:
            ax = msg.xacc * 9.80665 / 1000.0 - ax_bias
            ay = msg.yacc * 9.80665 / 1000.0 - ay_bias
            accel_f_x += lpf_alpha * (ax - accel_f_x)
            accel_f_y += lpf_alpha * (ay - accel_f_y)

        if reorigin["requested"]:
            reorigin["requested"] = False
            if last_yaw_deg is not None:
                yaw_baseline = last_yaw_deg
            origin_ax, origin_ay = accel_f_x, accel_f_y
            print(f"\n[ORIGIN] Set: heading={yaw_baseline:.1f} deg  "
                  f"ax_ref={origin_ax:+.2f} ay_ref={origin_ay:+.2f}")

        if mtype != "ATTITUDE" or last_yaw_deg is None or yaw_baseline is None:
            continue

        yaw_err = wrap180(last_yaw_deg - yaw_baseline)

        if not is_armed:
            continue

        # Winner-take-all: only the single most-over-threshold axis fires,
        # so a hand-held move that nudges more than one axis at once
        # doesn't light up multiple pairs simultaneously. Accel is measured
        # relative to whatever origin was last set (SIGUSR1), not just the
        # static pre-arm bias, so re-setting origin at a new held pose works.
        ax_rel = accel_f_x - origin_ax
        ay_rel = accel_f_y - origin_ay
        ax_use = ax_rel if abs(ax_rel) > ACCEL_DEADBAND else 0.0
        ay_use = ay_rel if abs(ay_rel) > ACCEL_DEADBAND else 0.0

        candidates = [
            (abs(yaw_err) / YAW_THRESHOLD_DEG, "YAW-R" if yaw_err > 0 else "YAW-L",
             PAIR_YAW_RIGHT if yaw_err > 0 else PAIR_YAW_LEFT),
            (abs(ax_use) / ACCEL_THRESHOLD, "X-R" if ax_use > 0 else "X-L",
             PAIR_X_RIGHT if ax_use > 0 else PAIR_X_LEFT),
            (abs(ay_use) / ACCEL_THRESHOLD, "Y-UP" if ay_use > 0 else "Y-DN",
             PAIR_Y_UP if ay_use > 0 else PAIR_Y_DOWN),
        ]
        margin, name, pair = max(candidates, key=lambda c: c[0])

        active = set()
        label = []
        if margin >= 1.0:
            active.update(pair)
            label.append(name)

        now = time.monotonic()
        for n in (1, 2, 3, 4):
            if n in active and now - last_sent[n] > REFRESH_S:
                motor_test(m, n)
                last_sent[n] = now

        print(f"YAW:{yaw_err:+6.1f} AX:{ax_rel:+5.2f} AY:{ay_rel:+5.2f}  "
              f"active:{sorted(active) if active else '-'} {' '.join(label):12s}",
              end="\r")


if __name__ == "__main__":
    main()
