import argparse
import time
import cv2
import numpy as np
from picamera2 import Picamera2

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def main():
    parser = argparse.ArgumentParser(description="ArUco detection on IMX708 CSI camera")
    parser.add_argument("--dict", default="DICT_4X4_50", choices=ARUCO_DICTS.keys(),
                         help="ArUco dictionary to use (default: DICT_4X4_50)")
    parser.add_argument("--width", type=int, default=1280, help="Capture width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="Capture height (default: 720)")
    parser.add_argument("--no-preview", action="store_true",
                         help="Run headless, no on-screen preview window")
    parser.add_argument("--notify-cooldown", type=float, default=1.0,
                         help="Seconds between repeated 'aruco detected' prints (default: 1.0)")
    args = parser.parse_args()

    # --- Set up the CSI camera ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # let auto-exposure/focus settle

    # --- Set up ArUco detector (old-style API, for OpenCV < 4.7) ---
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[args.dict])
    aruco_params = cv2.aruco.DetectorParameters_create()

    print(f"Camera started ({args.width}x{args.height}), dictionary: {args.dict}")
    print("Press Ctrl+C to quit." if args.no_preview else "Press 'q' in the preview window to quit.")

    last_print_time = 0.0

    try:
        while True:
            frame = picam2.capture_array()  # RGB888 numpy array

            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            if ids is not None:
                now = time.time()
                if now - last_print_time >= args.notify_cooldown:
                    last_print_time = now
                    print(f"aruco detected: {ids.flatten().tolist()}")
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if not args.no_preview:
                # Picamera2 gives RGB; OpenCV's imshow expects BGR for correct colors
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("ArUco Detection - IMX708", bgr)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        if not args.no_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
