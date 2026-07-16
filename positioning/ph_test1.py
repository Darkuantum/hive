"""
mavlink_interface.py

Reusable connection layer between the Raspberry Pi and the Pixhawk4 (ArduSub).
This is the "MAVLink" box from our architecture diagram -- it does NOT decide
anything, it just gives the decision engine a clean way to:
  - connect and confirm the link is alive (heartbeat)
  - read attitude (roll/pitch/yaw) and depth
  - arm / disarm
  - send a velocity setpoint (surge, sway, yaw rate) in Guided mode

Usage:
    from mavlink_interface import MavlinkInterface

    veh = MavlinkInterface('/dev/serial0', baud=57600)   # real Pixhawk
    # veh = MavlinkInterface('udp:127.0.0.1:14550')       # SITL

    veh.connect()
    veh.set_mode('GUIDED')
    veh.arm()
    veh.send_velocity(vx=0.2, vy=0.0, yaw_rate=0.0)

    telem = veh.get_telemetry()
    print(telem)  # {'roll':..., 'pitch':..., 'yaw':..., 'depth':...}
"""

import time
from pymavlink import mavutil


class MavlinkInterface:
    def __init__(self, connection_string, baud=57600, timeout=10):
        self.connection_string = connection_string
        self.baud = baud
        self.timeout = timeout
        self.master = None

        # Latest known telemetry values, updated as messages arrive
        self._latest = {
            'roll': None, 'pitch': None, 'yaw': None,
            'rollspeed': None, 'pitchspeed': None, 'yawspeed': None,
            'depth': None,
            'armed': False,
            # Raw PWM per MAIN OUT channel (1-4 = your thrusters, 5-6 unused)
            'servo1': None, 'servo2': None, 'servo3': None,
            'servo4': None, 'servo5': None, 'servo6': None,
        }

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def connect(self):
        """Open the connection and block until the first heartbeat arrives."""
        print(f"Connecting to {self.connection_string} ...")
        self.master = mavutil.mavlink_connection(
            self.connection_string, baud=self.baud
        )

        print("Waiting for heartbeat...")
        msg = self.master.wait_heartbeat(timeout=self.timeout)
        if msg is None:
            raise TimeoutError(
                f"No heartbeat received within {self.timeout}s. "
                "Check wiring, baud rate, and that ArduSub is running."
            )
        print(
            f"Heartbeat received (system {self.master.target_system}, "
            f"component {self.master.target_component})"
        )
        self._request_streams()

    def _request_streams(self, rate_hz=10):
        """Ask ArduSub to stream ATTITUDE and VFR_HUD at a known rate.
        Most ArduSub builds stream these by default, but requesting
        explicitly avoids silently getting nothing."""
        for msg_id in [
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        ]:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0,
            )

    # ------------------------------------------------------------------
    # Reading telemetry
    # ------------------------------------------------------------------
    def update(self, blocking=False):
        """Pull any pending messages and update the internal telemetry
        cache. Call this once per loop iteration in the decision engine."""
        while True:
            msg = self.master.recv_match(blocking=blocking)
            if msg is None:
                break
            msg_type = msg.get_type()

            if msg_type == 'ATTITUDE':
                self._latest['roll'] = msg.roll
                self._latest['pitch'] = msg.pitch
                self._latest['yaw'] = msg.yaw
                self._latest['rollspeed'] = msg.rollspeed
                self._latest['pitchspeed'] = msg.pitchspeed
                self._latest['yawspeed'] = msg.yawspeed

            elif msg_type == 'VFR_HUD':
                # alt is negative below the surface (NED-style depth)
                self._latest['depth'] = msg.alt

            elif msg_type == 'SERVO_OUTPUT_RAW':
                # Raw PWM (microseconds) currently being sent to each
                # MAIN OUT channel. 1500 = neutral/no thrust, values above
                # or below that indicate thrust direction and magnitude.
                self._latest['servo1'] = msg.servo1_raw
                self._latest['servo2'] = msg.servo2_raw
                self._latest['servo3'] = msg.servo3_raw
                self._latest['servo4'] = msg.servo4_raw
                self._latest['servo5'] = msg.servo5_raw
                self._latest['servo6'] = msg.servo6_raw

            elif msg_type == 'HEARTBEAT':
                self._latest['armed'] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

            if not blocking:
                # In non-blocking mode recv_match returns None once the
                # queue is drained, so the while loop above exits naturally.
                continue

    def get_telemetry(self):
        """Return the latest known telemetry snapshot."""
        return dict(self._latest)

    # ------------------------------------------------------------------
    # Mode / arming
    # ------------------------------------------------------------------
    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            raise ValueError(f"Unknown mode '{mode_name}'")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    # ------------------------------------------------------------------
    # Commanding motion (the PID output from the decision engine lands here)
    # ------------------------------------------------------------------
    def send_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """Send a body-frame velocity setpoint. vx = surge (forward+),
        vy = sway (right+), vz = heave (down+, leave 0 -- no vertical
        thrusters), yaw_rate = rotation in rad/s. Requires GUIDED mode."""
        type_mask = 0b0000011111000111  # ignore position, accel, yaw(abs)
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,          # position (ignored)
            vx, vy, vz,       # velocity
            0, 0, 0,          # acceleration (ignored)
            0, yaw_rate,      # yaw, yaw_rate
        )


# ---------------------------------------------------------------------
# Simple standalone test: connect, print telemetry for a while
# ---------------------------------------------------------------------
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="MAVLink connection smoke test")
    parser.add_argument(
        '--conn', default='/dev/serial0',
        help="Connection string. Use '/dev/serial0' for real Pixhawk over "
             "UART, or 'udp:127.0.0.1:14550' for SITL."
    )
    parser.add_argument('--baud', type=int, default=57600)
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()

    veh = MavlinkInterface(args.conn, baud=args.baud)
    veh.connect()

    print(f"Streaming telemetry for {args.seconds:.0f}s (Ctrl+C to stop)...")
    start = time.time()
    try:
        while time.time() - start < args.seconds:
            veh.update(blocking=False)
            t = veh.get_telemetry()
            print(
                f"roll={t['roll']} pitch={t['pitch']} yaw={t['yaw']}  "
                f"depth={t['depth']}  armed={t['armed']}"
            )
            print(
                f"  PWM  1={t['servo1']} 2={t['servo2']} 3={t['servo3']} "
                f"4={t['servo4']}   (unused: 5={t['servo5']} 6={t['servo6']})"
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    print("Done.")
