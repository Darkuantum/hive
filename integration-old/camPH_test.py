"""
pixhawk_camera_test.py

Phase 4 of the integration plan: the full software chain running against
the REAL Pixhawk, with a REAL camera and marker -- but no thrusters
connected yet. Nothing physically moves; this validates that the whole
pipeline (camera -> decision engine -> PID -> MAVLink -> PWM output)
works correctly end to end, by watching the PWM values respond and the
telemetry stream in, live, in one window.

IMPORTANT: this vehicle has no GPS (expected -- it's underwater), so
ArduSub's GUIDED mode will refuse to engage until the EKF has SOME
origin to measure position relative to. set_fake_ekf_origin() sets an
arbitrary one -- the actual lat/lon values don't matter for a
vision-controlled platform like this, only that an origin exists at all.

SAFETY: this arms the vehicle. No thrusters are connected, so nothing
physically spins, but treat it with the same care as the Motor Test
screen -- don't run this with anything plugged into MAIN OUT that you
don't want to risk.

Run:
    python3 pixhawk_camera_test.py --conn /dev/ttyACM0 --baud 115200

Press 'q' in the video window to quit, 'r' to reset the decision engine.
"""

import argparse
import math
import time
import cv2

from camFinal import ArucoDetector, get_screen_resolution, half_area_window_size
from pose_controller import PoseController
from decision_engine import DecisionEngine
from mavlink_interface import MavlinkInterface


def draw_line(frame, text, y, color):
    cv2.putText(frame, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def main():
    parser = argparse.ArgumentParser(description="Full pipeline test against real Pixhawk, no thrusters")
    parser.add_argument("--conn", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dict", default="DICT_4X4_50")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--marker-size", type=float, default=0.05)
    args = parser.parse_args()

    # ---- Bring up the camera ----
    detector = ArucoDetector(
        dict_name=args.dict, width=args.width, height=args.height,
        marker_size=args.marker_size,
    )
    detector.start()

    # ---- Bring up the Pixhawk link ----
    veh = MavlinkInterface(args.conn, baud=args.baud)
    veh.connect()

    ack_result = veh.set_mode("STABILIZE")
    print(f"Mode change ack result: {ack_result} (0 = accepted, None = no ack received)")

    actual_mode = None
    for _ in range(20):
        veh.update(blocking=False)
        actual_mode = veh.get_mode_name()
        if actual_mode == "STABILIZE":
            break
        time.sleep(0.2)

    print(f"Mode after polling: {actual_mode}")
    if actual_mode != "STABILIZE":
        print("WARNING: still not STABILIZE. Check QGC's Vehicle Messages "
              "panel for the specific rejection reason.\n")

    veh.arm()
    time.sleep(2)
    veh.update(blocking=False)
    if not veh.get_telemetry()["armed"]:
        print("WARNING: vehicle did not arm. Continuing anyway -- PID output")
        print("will still compute and display, but send_velocity() commands")
        print("won't be acted on by ArduSub while disarmed. Check QGC's")
        print("Vehicle Messages panel for the specific pre-arm failure.\n")

    controller = PoseController()
    engine = DecisionEngine()

    # ---- Live window ----
    window_name = "Pixhawk + Camera Integration Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    screen_res = get_screen_resolution()
    win_w, win_h = half_area_window_size(*(screen_res or (args.width, args.height)))
    cv2.resizeWindow(window_name, win_w, win_h)

    print("Running. Press 'q' to quit, 'r' to reset the decision engine.\n")

    last_time = time.time()
    try:
        while True:
            now = time.time()
            dt = max(now - last_time, 1e-3)
            last_time = now

            pose, frame = detector.capture_and_detect()
            veh.update(blocking=False)
            telem = veh.get_telemetry_deg()

            marker_detected = pose is not None
            if marker_detected:
                state = engine.update(
                    True, pose["x"], pose["y"], pose["z"], pose["yaw"],
                    telem["roll"], telem["pitch"],
                )
            else:
                state = engine.update(False)

            if engine.is_controlling() and marker_detected:
                vx, vy, yaw_rate = controller.compute(
                    pose["x"], pose["y"], pose["z"], pose["yaw"], dt
                )
                # PID output is already clamped to output_limit -- divide
                # by that same limit to normalize into MANUAL_CONTROL's
                # -1..1 stick range, so full PID output = full stick.
                x_norm = vx / controller.pid_surge.output_limit
                y_norm = vy / controller.pid_sway.output_limit
                r_norm = yaw_rate / controller.pid_yaw.output_limit
                veh.send_manual_control(x=x_norm, y=y_norm, z=0.5, r=r_norm)
            else:
                vx = vy = yaw_rate = 0.0
                veh.send_manual_control(x=0.0, y=0.0, z=0.5, r=0.0)
                controller.reset()

            # ---- Overlay everything ----
            y = 26
            draw_line(frame, f"state: {state.name}   controlling: {engine.is_controlling()}", y, (255, 255, 0)); y += 26
            if marker_detected:
                draw_line(frame, f"cam:  x={pose['x']:+.3f} y={pose['y']:+.3f} z={pose['z']:+.3f} "
                                  f"yaw={math.degrees(pose['yaw']):+.1f}deg", y, (0, 255, 0)); y += 24
                draw_line(frame, f"pid:  vx={vx:+.3f} vy={vy:+.3f} yaw_rate={yaw_rate:+.3f}", y, (0, 200, 255)); y += 24
            else:
                draw_line(frame, "no marker detected", y, (0, 0, 255)); y += 24

            draw_line(frame, f"pixhawk: roll={telem['roll_deg']:+.1f} pitch={telem['pitch_deg']:+.1f} "
                              f"depth={telem['depth']}  armed={telem['armed']}  mode={veh.get_mode_name()}", y, (255, 200, 0)); y += 24
            draw_line(frame, f"PWM: 1={telem['servo1']} 2={telem['servo2']} "
                              f"3={telem['servo3']} 4={telem['servo4']}", y, (255, 150, 150)); y += 24

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                print("Manual reset: decision engine and PID reset to SEARCHING.")
                engine = DecisionEngine()
                controller.reset()

    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down: disarming, stopping camera.")
        veh.send_manual_control(x=0.0, y=0.0, z=0.5, r=0.0)
        veh.disarm()
        detector.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
