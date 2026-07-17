"""
app.py

LAN web UI for the recovery rig: manual thruster control, Pixhawk
telemetry, and a live camera feed, for driving the Pi while it runs
headless. All hardware access goes through HardwareManager (hardware.py),
which wraps integration/mavlink_interface.py and integration/camFinal.py
unmodified -- this file and hardware.py are the only new code.

Run (from the repo root):
    uv run --with flask --with opencv-contrib-python python webui/app.py

Then open http://<pi-ip>:8000 from any browser on the same network.
Use --no-camera to run without a camera attached (e.g. bench-testing
the mavlink side only).

No authentication and no TLS. This is meant for a trusted LAN or a
direct point-to-point link during bench/tank testing, not the open
internet.
"""
import argparse
import time

from flask import Flask, Response, jsonify, render_template, request

from hardware import HardwareManager

app = Flask(__name__)
manager: HardwareManager = None  # assigned in main()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/state')
def api_state():
    return jsonify({
        'mavlink': manager.get_telemetry(),
        'camera': manager.get_camera_status(),
        'pose': manager.get_pose(),
    })


@app.route('/api/control', methods=['POST'])
def api_control():
    data = request.get_json(silent=True) or {}
    manager.set_control(data.get('x', 0.0), data.get('y', 0.0), data.get('r', 0.0))
    return jsonify({'ok': True})


@app.route('/api/arm', methods=['POST'])
def api_arm():
    manager.arm()
    return jsonify({'ok': True})


@app.route('/api/disarm', methods=['POST'])
def api_disarm():
    manager.disarm()
    return jsonify({'ok': True})


@app.route('/api/mode', methods=['POST'])
def api_mode():
    data = request.get_json(silent=True) or {}
    mode_name = data.get('mode')
    if not mode_name:
        return jsonify({'ok': False, 'error': 'missing "mode"'}), 400
    ack = manager.set_mode(mode_name)
    return jsonify({'ok': True, 'ack': ack})


def _mjpeg_stream():
    boundary = b'--frame'
    while True:
        jpeg = manager.get_jpeg_frame()
        if jpeg is not None:
            yield (boundary + b'\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
        time.sleep(0.05)


@app.route('/video_feed')
def video_feed():
    return Response(_mjpeg_stream(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    global manager
    parser = argparse.ArgumentParser(description="Recovery rig web control UI")
    parser.add_argument('--mavlink-conn', default='/dev/serial0',
                         help="MAVLink connection string, e.g. /dev/serial0 or "
                              "udp:127.0.0.1:14550 for SITL")
    parser.add_argument('--mavlink-baud', type=int, default=57600)
    parser.add_argument('--no-camera', action='store_true',
                         help="Skip camera startup (e.g. bench-testing off-Pi)")
    parser.add_argument('--host', default='0.0.0.0',
                         help="Bind address (0.0.0.0 so other devices on the "
                              "LAN can reach a headless Pi)")
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()

    manager = HardwareManager(
        mavlink_conn=args.mavlink_conn,
        mavlink_baud=args.mavlink_baud,
        enable_camera=not args.no_camera,
    )
    manager.start()
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        manager.stop()


if __name__ == '__main__':
    main()
