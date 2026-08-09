#!/usr/bin/env python3
"""Calibration CLI — step-response data collection, identification, and gain application.

Subcommands:
  step      Run an open-loop step on the vehicle (requires running server)
  identify  Fit a first-order model from a telemetry CSV (offline)
  apply     Identify + save computed gains to a JSON file (offline)
  metrics   Compute closed-loop tracking metrics from a telemetry CSV (offline)
  list      List past calibration runs (offline)

Usage:
  python scripts/calibrate_cli.py step --axis surge --amplitude 0.3
  python scripts/calibrate_cli.py identify --csv logs/20260809/telemetry.csv --axis surge
  python scripts/calibrate_cli.py apply --csv logs/20260809/telemetry.csv --axis surge
  python scripts/calibrate_cli.py metrics --csv logs/<run_id>/telemetry.csv --axis surge
  python scripts/calibrate_cli.py list
"""

import argparse
import csv
import json
import math
import os
import sys
import time
import urllib.request

# ---------------------------------------------------------------------------
# Path setup: ensure integration/ is importable for offline subcommands
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INTEGRATION_DIR = os.path.join(os.path.normpath(SCRIPT_DIR), '..', 'integration')
sys.path.insert(0, INTEGRATION_DIR)


# ---------------------------------------------------------------------------
# HTTP helpers (for online 'step' subcommand)
# ---------------------------------------------------------------------------

def _http_post(url, data):
    """POST JSON data to url, return parsed response dict."""
    req = urllib.request.Request(
        url,
        data=json.dumps(data).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
    )
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read())


def _http_get(url):
    """GET url, return parsed response dict."""
    with urllib.request.urlopen(url) as resp:
        return json.loads(resp.read())


# ---------------------------------------------------------------------------
# Subcommand: step (online — requires running server)
# ---------------------------------------------------------------------------

def cmd_step(args):
    server = args.server or os.environ.get('HIVE_SERVER', 'localhost:8000')
    base = f'http://{server}'

    # Build estimated duration for timeout
    estimated = args.pre_duration + args.step_duration + args.post_duration
    timeout = estimated + 30.0  # safety margin

    # Trigger the step
    step_params = {
        'axis': args.axis,
        'amplitude': args.amplitude,
        'pre_duration': args.pre_duration,
        'step_duration': args.step_duration,
        'post_duration': args.post_duration,
    }
    if args.name:
        step_params['name'] = args.name

    print(f'Starting step: axis={args.axis} amplitude={args.amplitude} '
          f'duration={args.step_duration}s')
    try:
        result = _http_post(f'{base}/api/calibrate/step/run', step_params)
    except Exception as exc:
        print(f'Error starting step: {exc}', file=sys.stderr)
        return 1

    if result.get('status') == 'already_running':
        print(f'Error: {result.get("message", "a step is already running")}',
              file=sys.stderr)
        return 1
    if 'error' in result:
        print(f'Error: {result["error"]}', file=sys.stderr)
        return 1

    print(f'Step started: {result.get("status")} '
          f'(estimated {result.get("estimated_duration", "?")}s)')

    # Poll status
    poll_interval = 0.5
    deadline = time.time() + timeout
    last_status = None
    while time.time() < deadline:
        time.sleep(poll_interval)
        try:
            status = _http_get(f'{base}/api/calibrate/step/status')
        except Exception as exc:
            print(f'  poll error: {exc}', file=sys.stderr)
            continue

        current = status.get('status')
        if current != last_status:
            print(f'  status: {current}')
            last_status = current

        if current == 'done':
            summary = status.get('summary', {})
            run_id = summary.get('run_id', '?')
            csv_path = summary.get('csv_path', '?')
            video_path = summary.get('video_path')
            duration = summary.get('duration_s', '?')
            ticks = summary.get('ticks_logged', '?')
            frames = summary.get('frames_written', '?')
            print(f'Done! run_id={run_id}  duration={duration}s  '
                  f'ticks={ticks}  frames={frames}')
            print(f'  CSV: {csv_path}')
            if video_path:
                print(f'  Video: {video_path}')
            return 0

        if current == 'error':
            print(f'Error: {status.get("error", "unknown")}', file=sys.stderr)
            return 1

    print(f'Timeout ({timeout:.0f}s) — step may still be running on the server.', file=sys.stderr)
    print(f'  To abort: python3 {sys.argv[0]} abort --server {server}', file=sys.stderr)
    return 1


# ---------------------------------------------------------------------------
# Subcommand: identify (offline)
# ---------------------------------------------------------------------------

def cmd_identify(args):
    try:
        from calibration.identify import identify_from_csv
        from calibration.tuning import compute_gains
    except ImportError as exc:
        print(f'Cannot import calibration modules: {exc}', file=sys.stderr)
        return 1

    csv_path = args.csv
    if not os.path.exists(csv_path):
        print(f'CSV not found: {csv_path}', file=sys.stderr)
        return 1

    try:
        model = identify_from_csv(csv_path, axis=args.axis)
    except Exception as exc:
        print(f'Identification failed: {exc}', file=sys.stderr)
        return 1

    # Print model summary
    print('=== Model ===')
    print(f'  {model.summary()}')

    # Fit quality
    r2 = model.R_squared
    if r2 >= 0.95:
        quality = 'good'
    elif r2 >= 0.8:
        quality = 'moderate'
    else:
        quality = 'poor'
    print(f'  Fit quality: {quality} (R²={r2:.4f})')

    # Print tuning
    tau_cl = float(args.tau_cl) if args.tau_cl is not None else None
    try:
        result = compute_gains(model, tau_cl=tau_cl)
    except Exception as exc:
        print(f'Gain computation failed: {exc}', file=sys.stderr)
        return 1

    print()
    print('=== Tuning ===')
    print(f'  {result.summary()}')
    print(f'  Method: {result.method}')
    if result.notes:
        print(f'  Notes: {result.notes}')

    print()
    print('=== Computed gains ===')
    g = result.gains
    print(f'  surge: kp={g.surge_kp:.4f}  ki={g.surge_ki:.4f}  kd={g.surge_kd:.4f}')
    print(f'  sway:  kp={g.sway_kp:.4f}  ki={g.sway_ki:.4f}  kd={g.sway_kd:.4f}')
    print(f'  yaw:   kp={g.yaw_kp:.4f}  ki={g.yaw_ki:.4f}  kd={g.yaw_kd:.4f}')

    return 0


# ---------------------------------------------------------------------------
# Subcommand: apply (offline — identify + save)
# ---------------------------------------------------------------------------

def cmd_apply(args):
    try:
        from calibration.identify import identify_from_csv
        from calibration.tuning import compute_gains
    except ImportError as exc:
        print(f'Cannot import calibration modules: {exc}', file=sys.stderr)
        return 1

    csv_path = args.csv
    if not os.path.exists(csv_path):
        print(f'CSV not found: {csv_path}', file=sys.stderr)
        return 1

    output = args.output or 'gains.json'

    if os.path.exists(output) and not args.force:
        print(f'Error: {output} already exists. Use --force to overwrite.',
              file=sys.stderr)
        return 1

    try:
        model = identify_from_csv(csv_path, axis=args.axis)
    except Exception as exc:
        print(f'Identification failed: {exc}', file=sys.stderr)
        return 1

    print(f'Model: {model.summary()}')

    tau_cl = float(args.tau_cl) if args.tau_cl is not None else None
    try:
        result = compute_gains(model, tau_cl=tau_cl)
    except Exception as exc:
        print(f'Gain computation failed: {exc}', file=sys.stderr)
        return 1

    try:
        result.gains.to_file(output)
    except Exception as exc:
        print(f'Failed to save gains: {exc}', file=sys.stderr)
        return 1

    print(f'Gains saved to {output}')
    g = result.gains
    print(f'  surge: kp={g.surge_kp:.4f}  ki={g.surge_ki:.4f}  kd={g.surge_kd:.4f}')
    print(f'  sway:  kp={g.sway_kp:.4f}  ki={g.sway_ki:.4f}  kd={g.sway_kd:.4f}')
    print(f'  yaw:   kp={g.yaw_kp:.4f}  ki={g.yaw_ki:.4f}  kd={g.yaw_kd:.4f}')

    return 0


# ---------------------------------------------------------------------------
# Subcommand: list (offline)
# ---------------------------------------------------------------------------

def cmd_list(args):
    logs_dir = args.logs_dir
    if not os.path.isdir(logs_dir):
        print('No runs found')
        return 0

    entries = []
    try:
        for name in sorted(os.listdir(logs_dir), reverse=True):
            run_dir = os.path.join(logs_dir, name)
            if not os.path.isdir(run_dir):
                continue
            csv_path = os.path.join(run_dir, 'telemetry.csv')
            video_path = os.path.join(run_dir, 'video.mp4')
            entries.append({
                'run_id': name,
                'has_csv': os.path.exists(csv_path),
                'has_video': os.path.exists(video_path),
                'csv_size': os.path.getsize(csv_path) if os.path.exists(csv_path) else 0,
                'video_size': os.path.getsize(video_path) if os.path.exists(video_path) else 0,
            })
    except PermissionError:
        print('No runs found')
        return 0

    if not entries:
        print('No runs found')
        return 0

    # Format as aligned table
    header = f"{'RUN ID':<26} {'CSV':>6} {'VIDEO':>6} {'CSV SIZE':>10} {'VIDEO SIZE':>11}"
    print(header)
    print('-' * len(header))
    for e in entries:
        csv_flag = 'yes' if e['has_csv'] else 'no'
        vid_flag = 'yes' if e['has_video'] else 'no'
        csv_sz = _fmt_size(e['csv_size'])
        vid_sz = _fmt_size(e['video_size'])
        print(f"{e['run_id']:<26} {csv_flag:>6} {vid_flag:>6} {csv_sz:>10} {vid_sz:>11}")

    return 0


def _fmt_size(n):
    """Format byte count as human-readable string."""
    if n == 0:
        return '-'
    if n < 1024:
        return f'{n}B'
    if n < 1024 * 1024:
        return f'{n / 1024:.1f}K'
    return f'{n / (1024 * 1024):.1f}M'


# ---------------------------------------------------------------------------
# Subcommand: metrics (offline)
# ---------------------------------------------------------------------------

def cmd_metrics(args):
    try:
        from calibration.metrics import compute_metrics
    except ImportError as exc:
        print(f'Cannot import calibration modules: {exc}', file=sys.stderr)
        return 1

    csv_path = args.csv
    if not os.path.exists(csv_path):
        print(f'CSV not found: {csv_path}', file=sys.stderr)
        return 1

    sp = float(args.setpoint) if args.setpoint is not None else None

    try:
        m = compute_metrics(csv_path, axis=args.axis, setpoint=sp)
    except Exception as exc:
        print(f'Metrics computation failed: {exc}', file=sys.stderr)
        return 1

    print(f'=== Closed-loop tracking metrics ===')
    print(f'  {m.summary()}')
    return 0


# ---------------------------------------------------------------------------
# Subcommand: abort (online — requires running server)
# ---------------------------------------------------------------------------

def cmd_abort(args):
    """Abort the currently running step on the server."""
    server = args.server or os.environ.get('HIVE_SERVER', 'localhost:8000')
    base = f'http://{server}'
    try:
        result = _http_post(f'{base}/api/calibrate/step/abort', {})
    except Exception as exc:
        print(f'Error: cannot reach server at {server}: {exc}', file=sys.stderr)
        return 1

    if result.get('abort_requested'):
        print('Abort requested. Step will stop within 0.2s, motors zeroed, run finalized.')
    else:
        print(f'No step running: {result.get("message", "")}')
    return 0


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Calibration CLI: step-response data collection and PID tuning',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    # step
    p_step = subparsers.add_parser('step', help='Run open-loop step (requires running server)')
    p_step.add_argument('--axis', required=True, choices=['surge', 'sway', 'yaw'],
                        help='Axis to step')
    p_step.add_argument('--amplitude', type=float, required=True,
                        help='Motor command amplitude (-0.5 to 0.5)')
    p_step.add_argument('--step-duration', type=float, default=5.0,
                        help='Step duration in seconds (default: 5)')
    p_step.add_argument('--pre-duration', type=float, default=2.0,
                        help='Pre-step baseline duration (default: 2)')
    p_step.add_argument('--post-duration', type=float, default=3.0,
                        help='Post-step settling duration (default: 3)')
    p_step.add_argument('--name', default=None,
                        help='Run name suffix for logging')
    p_step.add_argument('--server', default=None,
                         help='Server host:port (default: localhost:8000 or HIVE_SERVER env)')

    # identify
    p_id = subparsers.add_parser('identify', help='Fit model from telemetry CSV (offline)')
    p_id.add_argument('--csv', required=True, help='Path to telemetry.csv')
    p_id.add_argument('--axis', default='surge', choices=['surge', 'sway', 'yaw'],
                      help='Axis to identify (default: surge)')
    p_id.add_argument('--tau-cl', type=float, default=None,
                      help='Desired closed-loop time constant (default: auto)')

    # apply
    p_apply = subparsers.add_parser('apply', help='Identify + save gains to file (offline)')
    p_apply.add_argument('--csv', required=True, help='Path to telemetry.csv')
    p_apply.add_argument('--axis', default='surge', choices=['surge', 'sway', 'yaw'],
                         help='Axis to identify (default: surge)')
    p_apply.add_argument('--output', default=None,
                         help='Output gains file (default: gains.json)')
    p_apply.add_argument('--tau-cl', type=float, default=None,
                         help='Desired closed-loop time constant (default: auto)')
    p_apply.add_argument('--force', '-f', action='store_true',
                         help='overwrite existing gains file without prompting')

    # metrics
    p_metrics = subparsers.add_parser('metrics',
                                       help='Compute closed-loop tracking metrics (offline)')
    p_metrics.add_argument('--csv', required=True, help='Path to telemetry.csv')
    p_metrics.add_argument('--axis', default='surge', choices=['surge', 'sway', 'yaw'],
                           help='Axis to analyse (default: surge)')
    p_metrics.add_argument('--setpoint', type=float, default=None,
                           help='Target setpoint (default: auto-detect from data)')

    # list
    p_list = subparsers.add_parser('list', help='List past calibration runs (offline)')
    p_list.add_argument('--logs-dir', default='logs',
                        help='Logs directory (default: logs/)')

    # abort
    p_abort = subparsers.add_parser('abort', help='Abort running step (requires server)')
    p_abort.add_argument('--server', default=None,
                         help='Server host:port (default: localhost:8000 or HIVE_SERVER env)')

    args = parser.parse_args()

    if args.command == 'step':
        return cmd_step(args)
    elif args.command == 'identify':
        return cmd_identify(args)
    elif args.command == 'apply':
        return cmd_apply(args)
    elif args.command == 'metrics':
        return cmd_metrics(args)
    elif args.command == 'list':
        return cmd_list(args)
    elif args.command == 'abort':
        return cmd_abort(args)


if __name__ == '__main__':
    sys.exit(main())
