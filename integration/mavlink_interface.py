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
import math
import threading
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
            'pressure_abs': None,   # mbar/hPa, from the Bar02 (external baro)
            'baro_temp': None,      # deg C, Bar02's onboard temperature sensor
            'pressure_int': None,   # mbar/hPa, from the Pixhawk's internal baro (MS5611)
            'baro_temp_int': None,  # deg C, internal baro's temperature sensor
            'armed': False,
            'custom_mode': None,
            'accel_x': None, 'accel_y': None, 'accel_z': None,  # m/s^2, from SCALED_IMU2
            # Raw PWM per MAIN OUT channel (1-4 = your thrusters, 5-6 unused)
            'servo1': None, 'servo2': None, 'servo3': None,
            'servo4': None, 'servo5': None, 'servo6': None,
            'statustext': None, 'statustext_severity': None,
            'battery_voltage': None, 'battery_current': None,
            'battery_remaining': None,
        }
        self._latest_lock = threading.Lock()

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
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
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

            elif msg_type == 'SCALED_PRESSURE2':
                # press_abs is in hPa (equivalent to mbar). temperature is
                # in centi-degrees C, so divide by 100 for actual deg C.
                self._latest['pressure_abs'] = msg.press_abs
                self._latest['baro_temp'] = msg.temperature / 100.0

            elif msg_type == 'SCALED_PRESSURE':
                # Same fields, but this is the Pixhawk's own INTERNAL baro
                # (MS5611) -- reads local air pressure inside the
                # enclosure, not water depth. Useful as a side-by-side
                # sanity check against the external Bar02.
                self._latest['pressure_int'] = msg.press_abs
                self._latest['baro_temp_int'] = msg.temperature / 100.0

            elif msg_type == 'SCALED_IMU2':
                # xacc/yacc/zacc are in mg (milli-g) per the MAVLink
                # spec -- convert to m/s^2 to match the ICM20948's units
                # for an apples-to-apples comparison.
                self._latest['accel_x'] = msg.xacc / 1000.0 * 9.80665
                self._latest['accel_y'] = msg.yacc / 1000.0 * 9.80665
                self._latest['accel_z'] = msg.zacc / 1000.0 * 9.80665

            elif msg_type == 'STATUSTEXT':
                text = msg.text.rstrip(b'\x00').decode('utf-8', errors='replace')
                self._latest['statustext'] = text
                self._latest['statustext_severity'] = msg.severity
                print(f"[ArduSub:{msg.severity}] {text}")

            elif msg_type == 'SYS_STATUS':
                self._latest['battery_voltage'] = msg.voltage_battery / 1000.0
                self._latest['battery_current'] = msg.current_battery / 100.0
                self._latest['battery_remaining'] = msg.battery_remaining

            elif msg_type == 'HEARTBEAT':
                self._latest['armed'] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._latest['custom_mode'] = msg.custom_mode

            if not blocking:
                # In non-blocking mode recv_match returns None once the
                # queue is drained, so the while loop above exits naturally.
                continue

    def get_mode_name(self):
        """Return the current flight mode as a string (e.g. 'GUIDED'),
        decoded from the last HEARTBEAT. Returns None if no heartbeat
        has been received yet."""
        if self._latest.get('custom_mode') is None:
            return None
        mapping = self.master.mode_mapping()
        for name, mode_id in mapping.items():
            if mode_id == self._latest['custom_mode']:
                return name
        return f"unknown({self._latest['custom_mode']})"

    def get_telemetry(self):
        """Return the latest known telemetry snapshot (raw units --
        radians for angles, as MAVLink and your PID math expect).

        Thread safety: we rely on CPython's GIL here rather than an
        explicit lock.  Each ``self._latest[key] = value`` in update()
        is a single bytecode op (atomic under the GIL), and the key set
        is fixed at __init__ (never grows/shrinks at runtime), so
        ``dict(self._latest)`` cannot hit 'dictionary changed size
        during iteration'.  The worst case is a snapshot that mixes
        values from two adjacent telemetry messages, which is benign
        for monotonically-updating sensor data."""
        return dict(self._latest)

    def get_telemetry_deg(self):
        """Same as get_telemetry(), but with angles converted to degrees
        for human-readable display, plus a combined tilt_deg value --
        how far the platform is from level overall, regardless of which
        direction. Useful for a single-glance stability check, since
        roll and pitch separately do not always make the overall tilt
        obvious at a glance."""
        t = self.get_telemetry()

        def to_deg(rad):
            return math.degrees(rad) if rad is not None else None

        t['roll_deg'] = to_deg(t['roll'])
        t['pitch_deg'] = to_deg(t['pitch'])
        t['yaw_deg'] = to_deg(t['yaw'])

        if t['roll'] is not None and t['pitch'] is not None:
            t['tilt_deg'] = math.degrees(
                math.sqrt(t['roll'] ** 2 + t['pitch'] ** 2)
            )
        else:
            t['tilt_deg'] = None

        return t

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------
    def read_param(self, param_name, timeout=2.0):
        """Request a single parameter value from ArduSub and return it.

        Sends PARAM_REQUEST_READ, waits for the matching PARAM_VALUE
        response, and returns the float value.  Returns None if the
        response doesn't arrive within *timeout* seconds or the
        returned param_id doesn't match.
        """
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode('utf-8'),
            -1,
        )
        msg = self.master.recv_match(
            type='PARAM_VALUE', blocking=True, timeout=timeout,
        )
        if msg is None:
            return None
        # param_id is a 16-byte fixed-length field padded with nulls
        if msg.param_id.rstrip(b'\x00').decode('utf-8', errors='replace') != param_name:
            return None
        return msg.param_value

    def set_param(self, param_name, value, param_type=None):
        """Set a parameter on the vehicle and wait for confirmation.

        Sends PARAM_SET and waits for the PARAM_VALUE echo that
        ArduSub sends back to confirm the write.

        The vehicle should be disarmed when changing most parameters;
        some params also require a reboot before they take effect.
        Returns the confirmed value (float) on success, or None on
        timeout.
        """
        if param_type is None:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode('utf-8'),
            float(value),
            param_type,
        )
        msg = self.master.recv_match(
            type='PARAM_VALUE', blocking=True, timeout=2.0,
        )
        if msg is None:
            return None
        if msg.param_id.rstrip(b'\x00').decode('utf-8', errors='replace') != param_name:
            return None
        return msg.param_value

    def verify_params(self, checks):
        """Verify a list of ArduSub parameters against expected values.

        *checks* is a list of dicts, each with keys:
            name        – parameter name (str)
            expected    – expected value (numeric)
            check       – comparison operator: 'eq', 'gte', 'gt', 'neq'
            description – human-readable label

        Returns a list of result dicts with keys:
            name, expected, actual (float or None), ok (bool),
            description, error (str, present only on failure).
        """
        results = []
        for chk in checks:
            name = chk['name']
            expected = chk['expected']
            op = chk.get('check', 'eq')
            actual = self.read_param(name)
            entry = {
                'name': name,
                'expected': expected,
                'actual': actual,
                'ok': False,
                'description': chk.get('description', ''),
            }
            if actual is None:
                entry['error'] = 'read failed'
            else:
                if op == 'eq':
                    entry['ok'] = (actual == expected)
                elif op == 'gte':
                    entry['ok'] = (actual >= expected)
                elif op == 'gt':
                    entry['ok'] = (actual > expected)
                elif op == 'neq':
                    entry['ok'] = (actual != expected)
                else:
                    entry['error'] = f"unknown check operator: {op!r}"
                if not entry['ok'] and 'error' not in entry:
                    entry['error'] = (
                        f"expected {op} {expected}, got {actual}"
                    )
            results.append(entry)
        return results

    # ------------------------------------------------------------------
    # Mode / arming
    # ------------------------------------------------------------------
    def set_fake_ekf_origin(self, lat=0.0, lon=0.0, alt=0.0):
        """Manually set the EKF origin. Required before GUIDED mode will
        engage on a vehicle with no GPS -- ArduSub needs SOME origin
        point to measure local position/velocity relative to, even
        though the actual lat/lon values are meaningless for a
        vision-controlled platform like this one. Call this once,
        right after connect(), before requesting GUIDED mode."""
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
        )
        time.sleep(0.5)  # give the EKF a moment to accept it

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            raise ValueError(f"Unknown mode '{mode_name}'")

        # Use MAV_CMD_DO_SET_MODE via command_long_send rather than the
        # older SET_MODE message -- some ArduPilot configurations don't
        # act on SET_MODE, which was silently swallowing this request.
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )

        # Wait briefly for the acknowledgment so callers get a real
        # answer instead of just hoping the mode change landed.
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if ack is None:
            return None  # no ack received -- caller should check get_mode_name()
        return ack.result  # 0 = MAV_RESULT_ACCEPTED

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if ack is None:
            return None
        return ack.result  # 0 = MAV_RESULT_ACCEPTED

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if ack is None:
            return None
        return ack.result  # 0 = MAV_RESULT_ACCEPTED

    # ------------------------------------------------------------------
    # Commanding motion (the PID output from the decision engine lands here)
    # ------------------------------------------------------------------
    def send_manual_control(self, x=0.0, y=0.0, z=0.5, r=0.0, buttons=0):
        """Send a MANUAL_CONTROL message -- the standard way to drive
        an ArduSub vehicle that has no position source, in MANUAL or
        STABILIZE mode. This does NOT ask ArduSub to achieve a velocity
        target itself (that's what GUIDED + send_velocity() tried to
        do, and it requires a position estimate ArduSub doesn't have
        here) -- it's a direct, open-loop stick command, same as a
        human pushing a joystick. Your own PID loop is the only closed
        loop in the system when using this.

        x, y, r: normalized -1.0 (full reverse/left) to +1.0 (full
                 forward/right), same convention as controller output
                 after dividing by its output_limit.
        z: normalized 0.0 (full down) to 1.0 (full up), 0.5 = neutral
           throttle/no vertical thrust -- you have no vertical
           thrusters, so always leave this at 0.5.
        """
        def to_1000(v, lo=-1000, hi=1000):
            return int(max(lo, min(hi, v * 1000)))

        self.master.mav.manual_control_send(
            self.master.target_system,
            to_1000(x), to_1000(y), int(max(0, min(1000, z * 1000))),
            to_1000(r), buttons,
        )


    def send_statustext(self, text, severity=None):
        """Send a STATUSTEXT message to the Pixhawk (appears in its log
        and QGroundControl).  severity is a MAV_SEVERITY constant; defaults
        to INFO if None."""
        if severity is None:
            severity = mavutil.mavlink.MAV_SEVERITY_INFO
        self.master.mav.statustext_send(
            severity, text.encode('utf-8')[:50],
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
    parser.add_argument(
        '--seconds', type=float, default=None,
        help="Stop after this many seconds. Omit to run until Ctrl+C."
    )
    args = parser.parse_args()

    veh = MavlinkInterface(args.conn, baud=args.baud)
    veh.connect()

    def fmt(value, spec):
        """Format a number, or show a placeholder if data hasn't
        arrived yet (e.g. on the very first loop iteration)."""
        return format(value, spec) if value is not None else "  N/A"

    print(
        f"Streaming telemetry{' for ' + str(args.seconds) + 's' if args.seconds else ''} "
        "(Ctrl+C to stop)..."
    )
    start = time.time()
    try:
        while args.seconds is None or time.time() - start < args.seconds:
            veh.update(blocking=False)
            t = veh.get_telemetry_deg()
            print(
                f"roll={fmt(t['roll_deg'], '+6.1f')} deg  "
                f"pitch={fmt(t['pitch_deg'], '+6.1f')} deg  "
                f"yaw={fmt(t['yaw_deg'], '+6.1f')} deg  "
                f"tilt={fmt(t['tilt_deg'], '5.1f')} deg  "
                f"depth={t['depth']}  "
                f"armed={t['armed']}  "
                f"PWM 1={t['servo1']} 2={t['servo2']} 3={t['servo3']} 4={t['servo4']}"
            )
            print(
                f"  ext (Bar02): {fmt(t['pressure_abs'], '7.2f')} hPa  "
                f"{fmt(t['baro_temp'], '4.1f')} C    "
                f"int (MS5611): {fmt(t['pressure_int'], '7.2f')} hPa  "
                f"{fmt(t['baro_temp_int'], '4.1f')} C"
            )
            time.sleep(0.5)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    print("\nDone.")
