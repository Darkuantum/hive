"""
app.py

LAN web UI for the recovery rig: manual thruster control, autonomous
(camera-following) control, Pixhawk telemetry, and a live camera feed,
for driving the Pi while it runs headless (electronics in the water,
no monitor/keyboard attached). All hardware access goes through
HardwareManager (hardware.py), which wraps integration/mavlink_interface.py,
integration/camFinal.py, integration/pose_controller.py, and
integration/decision_engine.py unmodified -- this file and hardware.py
are the only new code.

Run (from the repo root):
    uv run --with flask --with opencv-contrib-python python webui/app.py

Then open http://<pi-ip>:8000 from any browser on the same network.
Use --no-camera to run without a camera attached (e.g. bench-testing
the mavlink side only, or auto mode obviously won't work without one).

No authentication and no TLS. This is meant for a trusted LAN or a
direct point-to-point link during bench/tank testing, not the open
internet.
"""
import argparse
import time

from flask import Flask, Response, jsonify, render_template, request

from hardware import HardwareManager, VALID_MODES

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
        'external': manager.get_external_telemetry(),
    })


@app.route('/api/params')
def api_params():
    return jsonify(manager.get_param_status() or [])


@app.route('/api/control', methods=['POST'])
def api_control():
    """Manual-mode stick input. Accepted (and stored) regardless of
    current control_mode, but only acted on while in 'manual' -- see
    hardware.py's _mavlink_thread."""
    data = request.get_json(silent=True) or {}
    manager.set_control(data.get('x', 0.0), data.get('y', 0.0), data.get('r', 0.0))
    return jsonify({'ok': True})


@app.route('/api/power', methods=['POST'])
def api_power():
    """Manual-mode thruster power scale, 0-100 (percent). Applied
    server-side in hardware.py before anything is sent to the Pixhawk --
    manual mode only, see HardwareManager.set_manual_power()."""
    data = request.get_json(silent=True) or {}
    if 'power' not in data:
        return jsonify({'ok': False, 'error': 'missing "power"'}), 400
    try:
        power_pct = float(data['power'])
    except (TypeError, ValueError):
        return jsonify({'ok': False, 'error': '"power" must be a number'}), 400
    manager.set_manual_power(power_pct / 100.0)
    return jsonify({'ok': True, 'power': manager.get_manual_power() * 100.0})


@app.route('/api/control_mode', methods=['POST'])
def api_control_mode():
    """Switch between 'manual' (web-page sticks) and 'auto' (camera +
    decision engine + PID drives the thrusters). NOT the same thing as
    /api/mode below, which sets ArduSub's own flight mode."""
    data = request.get_json(silent=True) or {}
    mode = data.get('mode')
    if mode not in VALID_MODES:
        return jsonify({'ok': False, 'error': f'mode must be one of {VALID_MODES}'}), 400
    manager.set_control_mode(mode)
    return jsonify({'ok': True, 'control_mode': mode})


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
    """ArduSub flight mode (e.g. 'STABILIZE'). NOT the manual/auto
    control_mode above -- see /api/control_mode for that."""
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
                         help="Skip camera startup (e.g. bench-testing off-Pi; "
                              "auto mode will have nothing to react to without it)")
    parser.add_argument('--no-external', action='store_true',
                         help="Skip the external ICM20948/leak sensor thread "
                              "(e.g. testing off-Pi, or before that hardware "
                              "is wired up)")
    parser.add_argument('--host', default='0.0.0.0',
                         help="Bind address (0.0.0.0 so other devices on the "
                              "LAN can reach a headless Pi)")
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()

    manager = HardwareManager(
        mavlink_conn=args.mavlink_conn,
        mavlink_baud=args.mavlink_baud,
        enable_camera=not args.no_camera,
        enable_external=not args.no_external,
    )
    manager.start()
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        manager.stop()


if __name__ == '__main__':
    main()
