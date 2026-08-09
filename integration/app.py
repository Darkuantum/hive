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


@app.route('/api/led', methods=['POST'])
def api_led():
    data = request.get_json(silent=True) or {}
    if 'brightness' not in data:
        return jsonify({'ok': False, 'error': 'missing "brightness"'}), 400
    try:
        brightness = float(data['brightness'])
    except (TypeError, ValueError):
        return jsonify({'ok': False, 'error': '"brightness" must be a number'}), 400
    manager.set_led_brightness(brightness / 100.0)
    return jsonify({'ok': True, 'brightness': manager.get_led_brightness() * 100.0})


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


# ------------------------------------------------------------------
# Calibration API endpoints
# ------------------------------------------------------------------

@app.route('/api/calibrate/run/start', methods=['POST'])
def api_calibrate_run_start():
    """Start a calibration logging run. Optional body: {"name": "run_name"}."""
    data = request.get_json(silent=True) or {}
    try:
        result = manager.start_logging_run(name=data.get('name'))
        return jsonify({'ok': True, **result})
    except Exception as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500


@app.route('/api/calibrate/run/stop', methods=['POST'])
def api_calibrate_run_stop():
    """Stop the active calibration run. Syncs tmpfs -> logs/."""
    try:
        result = manager.stop_logging_run()
        return jsonify({'ok': True, **result})
    except Exception as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500


@app.route('/api/calibrate/run/active')
def api_calibrate_run_active():
    """Get info about the active calibration run, or {"active": False}."""
    result = manager.get_active_run()
    if result is None:
        return jsonify({'active': False})
    return jsonify({'active': True, **result})


@app.route('/api/calibrate/gains')
def api_calibrate_gains():
    """Return current PoseController gains as a JSON dict matching gains.json schema."""
    try:
        pid_s = manager.controller.pid_surge
        pid_w = manager.controller.pid_sway
        pid_y = manager.controller.pid_yaw
        gains = {
            "version": 1,
            "surge": {"kp": pid_s.kp, "ki": pid_s.ki, "kd": pid_s.kd},
            "sway":  {"kp": pid_w.kp, "ki": pid_w.ki, "kd": pid_w.kd},
            "yaw":   {"kp": pid_y.kp, "ki": pid_y.ki, "kd": pid_y.kd},
        }
        return jsonify(gains)
    except Exception as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500


@app.route('/api/calibrate/gains/save', methods=['POST'])
def api_calibrate_gains_save():
    """Save current PoseController gains to file. Optional body: {"path": "..."}."""
    data = request.get_json(silent=True) or {}
    try:
        result = manager.save_gains(path=data.get('path'))
        return jsonify({'ok': True, 'gains': result})
    except Exception as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500


@app.route('/api/calibrate/gains/load', methods=['POST'])
def api_calibrate_gains_load():
    """Reload gains from configured file (or override path). Optional body: {"path": "..."}."""
    data = request.get_json(silent=True) or {}
    try:
        result = manager.reload_gains(path=data.get('path'))
        return jsonify({'ok': True, 'gains': result})
    except ValueError as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 400
    except Exception as exc:
        return jsonify({'ok': False, 'error': str(exc)}), 500


@app.route('/api/calibrate/step/run', methods=['POST'])
def api_calibrate_step_run():
    """Start an open-loop step response in the background. Non-blocking."""
    data = request.get_json() or {}
    try:
        result = manager.start_step_async(
            axis=data.get('axis', 'surge'),
            amplitude=float(data.get('amplitude', 0.3)),
            pre_duration=float(data.get('pre_duration', 2.0)),
            step_duration=float(data.get('step_duration', 5.0)),
            post_duration=float(data.get('post_duration', 3.0)),
            name=data.get('name'),
        )
        return jsonify(result)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/calibrate/step/status', methods=['GET'])
def api_calibrate_step_status():
    """Poll step execution status."""
    return jsonify(manager.get_step_status())


@app.route('/api/calibrate/identify', methods=['POST'])
def api_calibrate_identify():
    """Run offline identification on a CSV file. Synchronous."""
    data = request.get_json() or {}
    csv_path = data.get('csv_path')
    axis = data.get('axis', 'surge')
    tau_cl_raw = data.get('tau_cl')

    if not csv_path:
        return jsonify({"error": "csv_path required"}), 400

    try:
        from calibration.identify import identify_from_csv
        from calibration.tuning import compute_gains

        model = identify_from_csv(csv_path, axis=axis)
        tau_cl = float(tau_cl_raw) if tau_cl_raw is not None else None
        result = compute_gains(model, tau_cl=tau_cl)

        return jsonify({
            "model": {
                "axis": model.axis, "K": model.K, "tau": model.tau,
                "L": model.L, "v_ss": model.v_ss, "F_step": model.F_step,
                "R_squared": model.R_squared, "n_samples": model.n_samples,
            },
            "tuning": {
                "Kp": result.Kp, "Ki": result.Ki, "Kd": result.Kd,
                "tau_cl": result.tau_cl, "K_eff": result.K_eff,
                "method": result.method, "notes": result.notes,
            },
            "gains": result.gains.to_dict(),
        })
    except (ValueError, FileNotFoundError) as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/calibrate/runs', methods=['GET'])
def api_calibrate_runs():
    """List past calibration runs from logs/."""
    import os
    logs_dir = 'logs'
    try:
        entries = []
        for name in sorted(os.listdir(logs_dir), reverse=True):
            run_dir = os.path.join(logs_dir, name)
            if not os.path.isdir(run_dir):
                continue
            csv_path = os.path.join(run_dir, 'telemetry.csv')
            video_path = os.path.join(run_dir, 'video.mp4')
            entries.append({
                "run_id": name,
                "has_csv": os.path.exists(csv_path),
                "has_video": os.path.exists(video_path),
                "csv_size": os.path.getsize(csv_path) if os.path.exists(csv_path) else 0,
                "video_size": os.path.getsize(video_path) if os.path.exists(video_path) else 0,
            })
        return jsonify({"runs": entries})
    except FileNotFoundError:
        return jsonify({"runs": []})


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
                         help="Skip the external leak sensor thread "
                              "(e.g. testing off-Pi, or before that hardware "
                              "is wired up)")
    parser.add_argument('--num-leds', type=int, default=8,
                         help="Number of LEDs on the DotStar strip (default: 8)")
    parser.add_argument('--no-led', action='store_true',
                         help="Skip LED strip startup (e.g. bench-testing off-Pi)")
    parser.add_argument('--host', default='0.0.0.0',
                         help="Bind address (0.0.0.0 so other devices on the "
                              "LAN can reach a headless Pi)")
    parser.add_argument('--port', type=int, default=8000)
    # PID tuning (override pose_controller.py defaults at runtime)
    parser.add_argument('--kp', type=float, default=None, help='Surge/sway proportional gain')
    parser.add_argument('--ki', type=float, default=None, help='Surge/sway integral gain')
    parser.add_argument('--kd', type=float, default=None, help='Surge/sway derivative gain')
    parser.add_argument('--yaw-kp', type=float, default=None, help='Yaw proportional gain')
    parser.add_argument('--yaw-ki', type=float, default=None, help='Yaw integral gain')
    parser.add_argument('--yaw-kd', type=float, default=None, help='Yaw derivative gain')
    # Gains file (load PID gains from JSON; CLI args still override)
    parser.add_argument('--gains-file', default=None, type=str,
                         help='Path to gains.json for PID gain persistence')
    args = parser.parse_args()

    pose_kw = {}
    for key in ('kp', 'ki', 'kd', 'yaw_kp', 'yaw_ki', 'yaw_kd'):
        arg_val = getattr(args, key.replace('-', '_'))
        if arg_val is not None:
            pose_kw[key] = arg_val

    manager = HardwareManager(
        mavlink_conn=args.mavlink_conn,
        mavlink_baud=args.mavlink_baud,
        enable_camera=not args.no_camera,
        enable_external=not args.no_external,
        enable_led=not args.no_led,
        num_leds=args.num_leds,
        pose_controller_kw=pose_kw or None,
        gains_file=args.gains_file,
    )
    manager.start()
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        manager.stop()


if __name__ == '__main__':
    main()
