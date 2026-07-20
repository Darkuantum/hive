#!/usr/bin/env python3
"""
Recovery Frame Station-Keeping (YAW + X + Y)  -- ESCs on the Pixhawk
====================================================================
Architecture:
  Pixhawk 1 (running ArduSub) owns the ESCs on its MAIN OUT rail and does
  ALL PWM generation + motor mixing. The Raspberry Pi 4 is the "brain":
  it reads attitude/IMU over MAVLink, runs the yaw-hold + X/Y-damping
  control loop, and sends the *result* to the Pixhawk as MANUAL_CONTROL
  (surge / sway / yaw effort). ArduSub's vectored-frame mixer converts
  that into the four thruster PWMs and outputs them to the ESCs.

  Pi (this script) --MAVLink MANUAL_CONTROL--> Pixhawk --PWM--> 4x ESC
                    <---ATTITUDE / IMU--------

  Because ArduSub's "Vectored" frame already models 4 corner thrusters
  angled 45 deg, YOU DO NOT hand-write the thruster mixer or the 45 deg
  compensation anymore -- the Pixhawk handles the geometry. You just
  command body-frame surge (X), sway (Y), and yaw (R).

  QGroundControl can stay connected in parallel (via mavlink-router on the
  Pi) for monitoring / e-stop.

--------------------------------------------------------------------
ONE-TIME PIXHAWK / QGC SETUP (do this before running):
  1. Firmware: ArduSub.
  2. Frame:   set to a "Vectored" frame (4 horizontal thrusters at the
              corners, 45 deg). QGC -> Vehicle Setup -> Frame.
  3. Motor directions: QGC -> Motors. Spin each thruster and reverse any
     that push the wrong way (this replaces the old T*_DIR trims).
  4. ESC calibration + compass calibration in QGC.
  5. Confirm MANUAL mode works from a joystick first, THEN hand over to
     this script (which sets MANUAL mode itself).
--------------------------------------------------------------------

NOTE ON FIRMWARE: this assumes ArduSub (custom flight-mode number 19 =
MANUAL). If you are on plain ArduPilot Rover/Copter or PX4, the mode
number and axis conventions differ -- tell me and I'll adjust. If instead
you want the Pi to compute each thruster's PWM itself and push raw values
(bypassing ArduSub's mixer via MAV_CMD_DO_SET_SERVO), I can provide that
variant, but MANUAL_CONTROL below is the robust, standard path.

Dependencies (Pi):  pip3 install pymavlink
Run:                python3 frame_station_keep_mavlink.py
"""

import time
import math
import signal
import sys

from pymavlink import mavutil

# =====================================================================
# CONFIGURATION
# =====================================================================

MAVLINK_PORT = "/dev/ttyACM0"     # USB. Use "/dev/serial0" for TELEM2 -> UART
MAVLINK_BAUD = 115200

ARDUSUB_MODE_MANUAL = 19          # ArduSub custom mode number for MANUAL

LOOP_HZ = 50

# --- Command authority (MANUAL_CONTROL units, full scale = 1000) ---
# Keep these modest at first; raise once behaviour is confirmed in water.
MAX_YAW_EFFORT = 350   # of 1000
MAX_X_EFFORT   = 350
MAX_Y_EFFORT   = 350

# --- YAW heading-hold PID (deg error -> effort units) ---
YAW_KP, YAW_KI, YAW_KD = 8.0, 0.5, 2.5
YAW_DEADBAND_DEG = 1.0
# Heading target: None = capture current heading at startup, or a fixed
# compass heading 0-360 (e.g. 90.0) to hold a specific bearing.
YAW_TARGET_DEG = None

# --- X/Y active damping (resists motion; does NOT hold absolute position) ---
XY_KV        = 600.0   # effort per (m/s) of estimated velocity
VEL_LEAK     = 0.5     # 1/s leak (bounds accelerometer drift)
ACCEL_LPF_HZ = 5.0     # low-pass on accelerometer (kills vibration)
ACCEL_DEADBAND = 0.05  # m/s^2 noise floor ignored

# =====================================================================
# CONTROLLERS
# =====================================================================

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class PID:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error):
        now = time.monotonic()
        dt = 1.0 / LOOP_HZ if self.prev_time is None \
            else max(now - self.prev_time, 1e-4)
        self.prev_time = now

        self.integral += error * dt
        i_lim = self.out_limit / max(self.ki, 1e-6)
        self.integral = clamp(self.integral, -i_lim, i_lim)

        deriv = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return clamp(out, -self.out_limit, self.out_limit)

class VelocityDamper:
    """
    Short-term pseudo-velocity from body accel via a leaky integrator, then
    an opposing effort. Makes the frame 'heavy' against shoves/waves. Cannot
    correct slow steady drift (that needs a DVL / USBL / camera reference).
    """
    def __init__(self, out_limit):
        self.out_limit = out_limit
        self.vel = 0.0
        self.accel_f = 0.0
        self.prev_time = None
        dt = 1.0 / LOOP_HZ
        rc = 1.0 / (2 * math.pi * ACCEL_LPF_HZ)
        self.lpf_alpha = dt / (rc + dt)

    def reset(self):
        self.vel = 0.0
        self.accel_f = 0.0
        self.prev_time = None

    def update(self, accel):
        now = time.monotonic()
        dt = 1.0 / LOOP_HZ if self.prev_time is None \
            else max(now - self.prev_time, 1e-4)
        self.prev_time = now

        self.accel_f += self.lpf_alpha * (accel - self.accel_f)
        a = self.accel_f if abs(self.accel_f) > ACCEL_DEADBAND else 0.0

        self.vel += a * dt
        self.vel -= VEL_LEAK * self.vel * dt

        return clamp(-XY_KV * self.vel, -self.out_limit, self.out_limit)

# =====================================================================
# MAVLINK
# =====================================================================

def connect_pixhawk():
    print(f"[MAV] Connecting on {MAVLINK_PORT} @ {MAVLINK_BAUD}...")
    m = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD)
    m.wait_heartbeat()
    print(f"[MAV] Heartbeat: sys {m.target_system} comp {m.target_component}")

    interval_us = int(1e6 / LOOP_HZ)
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

def arm(m):
    print("[MAV] Arming...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    m.motors_armed_wait()
    print("[MAV] Armed.")

def disarm(m):
    try:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
    except Exception:
        pass

def send_manual(m, x, y, r, z=500):
    """
    x = surge (forward+),  y = sway (starboard+),  r = yaw (right+):  -1000..1000
    z = heave/throttle:    0..1000, 500 = neutral (no vertical thrusters -> leave 500)
    buttons = 0
    """
    m.mav.manual_control_send(
        m.target_system,
        int(clamp(x, -1000, 1000)),
        int(clamp(y, -1000, 1000)),
        int(clamp(z, 0, 1000)),
        int(clamp(r, -1000, 1000)),
        0)

# =====================================================================
# MAIN
# =====================================================================

def main():
    master = connect_pixhawk()

    def shutdown(sig=None, frame=None):
        print("\n[SYS] Stopping - neutral command + disarm.")
        try:
            send_manual(master, 0, 0, 0, 500)
        except Exception:
            pass
        disarm(master)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    set_mode_manual(master)
    arm(master)

    yaw_pid  = PID(YAW_KP, YAW_KI, YAW_KD, MAX_YAW_EFFORT)
    x_damper = VelocityDamper(MAX_X_EFFORT)
    y_damper = VelocityDamper(MAX_Y_EFFORT)

    yaw_target = YAW_TARGET_DEG
    yaw_deg = None
    ax = ay = 0.0
    last_data_time = time.monotonic()

    print("[SYS] Station-keeping active (YAW hold + X/Y damping). Ctrl+C stops.")

    while True:
        msg = master.recv_match(
            type=["ATTITUDE", "SCALED_IMU2", "RAW_IMU"],
            blocking=True, timeout=0.1)
        now = time.monotonic()

        if msg is None:
            if now - last_data_time > 0.5:            # failsafe
                print("\n[FAILSAFE] IMU data lost - neutral command.")
                send_manual(master, 0, 0, 0, 500)
                yaw_pid.reset(); x_damper.reset(); y_damper.reset()
            continue
        last_data_time = now

        mtype = msg.get_type()
        if mtype == "ATTITUDE":
            yaw_deg = math.degrees(msg.yaw)
            if yaw_target is None:
                yaw_target = yaw_deg
                print(f"[YAW] Heading captured: {yaw_target:.1f} deg")
        elif mtype in ("SCALED_IMU2", "RAW_IMU"):
            ax = msg.xacc * 9.80665 / 1000.0   # mG -> m/s^2, body forward
            ay = msg.yacc * 9.80665 / 1000.0   # body starboard
            continue                            # run control only on ATTITUDE

        if yaw_deg is None or yaw_target is None:
            continue

        # ---- YAW heading hold ----
        yaw_err = wrap180(yaw_target - yaw_deg)
        if abs(yaw_err) < YAW_DEADBAND_DEG:
            yaw_err = 0.0
        r = yaw_pid.update(yaw_err)

        # ---- X / Y damping ----
        x = x_damper.update(ax)
        y = y_damper.update(ay)

        # Send to Pixhawk; ArduSub's vectored mixer -> 4 ESC PWMs
        send_manual(master, x, y, r, z=500)

        print(f"YAW:{yaw_deg:+7.1f} (tgt {yaw_target:+6.1f})  "
              f"X:{int(x):+5d} Y:{int(y):+5d} R:{int(r):+5d}   ",
              end="\r")

if __name__ == "__main__":
    main()
