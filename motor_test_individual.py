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

m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200, robust_parsing=True)
m.wait_heartbeat()
print(f"Connected: sys {m.target_system} comp {m.target_component}")

print("[MAV] Setting MANUAL mode...")
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 19, 0,0,0,0,0)
time.sleep(1)

print("[MAV] Arming...")
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
armed = False
deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    msg = m.recv_match(type=["HEARTBEAT","STATUSTEXT"], blocking=True, timeout=0.5)
    if msg is None: continue
    if msg.get_type() == "STATUSTEXT":
        print(f"[FC] {msg.text}")
    elif msg.get_type() == "HEARTBEAT" and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        armed = True
        print("[MAV] Armed.")
        break
if not armed:
    print("[MAV] Never armed, aborting.")
    raise SystemExit(1)


def drain_statustext(duration=0.3):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.1)
        if msg:
            print(f"[FC] {msg.text}")

def motor_test(motor_num, pwm, timeout_s):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        motor_num,
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM,
        pwm,
        timeout_s,
        0,  # motor count (0 = just this one)
        mavutil.mavlink.MOTOR_TEST_ORDER_BOARD,
        0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
    if ack:
        print(f"  motor {motor_num} pwm={pwm} -> ACK result={ack.result}")
    else:
        print(f"  motor {motor_num} pwm={pwm} -> no ACK")
    drain_statustext(0.2)

for motor in (1, 2, 3, 4):
    print(f"\n=== Motor {motor} ===")
    print("  neutral hold (1500) 1.5s")
    motor_test(motor, 1500, 1.5)
    time.sleep(1.6)

    for pwm in (1550, 1600, 1650, 1700, 1750, 1800):
        print(f"  ramp -> {pwm} for 0.8s -- WATCH/LISTEN")
        motor_test(motor, pwm, 0.8)
        time.sleep(0.9)

    print("  back to neutral (1500) 1.0s")
    motor_test(motor, 1500, 1.0)
    time.sleep(1.2)

print("\nDone. All motors should now be stopped (DO_MOTOR_TEST auto-times-out).")

print("[MAV] Disarming.")
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0)
