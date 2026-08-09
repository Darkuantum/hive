"""Run lifecycle + thread-safe CSV telemetry logger.

CSV column order is the single source of truth for all downstream consumers.
"""

import csv
import os
import shutil
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

# Exact column order — do not change without coordinating with downstream consumers.
TELEMETRY_COLUMNS = [
    "ts",                   # float, epoch seconds from time.time()
    "frame_idx",            # int, -1 if no video frame this tick
    "mode",                 # str: "manual" | "auto" | "calibration"
    "aruco_visible",        # int 0/1
    "yaw_pixhawk_rad",      # float, nan if unavailable
    "yaw_aruco_rad",        # float, nan if not visible
    "surge_setpoint", "surge_measured", "surge_p", "surge_i", "surge_d", "surge_out",
    "sway_setpoint",  "sway_measured",  "sway_p",  "sway_i",  "sway_d",  "sway_out",
    "yaw_setpoint",   "yaw_measured",   "yaw_p",   "yaw_i",   "yaw_d",   "yaw_out",
    "motor_x", "motor_y", "motor_z", "motor_r",   # int -1000..1000
    "battery_voltage",                                   # float, sanity check
]


@dataclass
class RunHandle:
    """Handle returned by start_run(). Thread-safe access to run metadata."""

    run_id: str           # ISO timestamp string YYYYmmdd_HHMMSS + optional name suffix
    tmpfs_dir: str        # active write path during run (e.g. /dev/shm/hive/<run_id>)
    final_dir: str        # final synced path (e.g. logs/<run_id>)
    csv_path: str         # path to telemetry.csv inside tmpfs_dir
    started_at: float     # epoch seconds
    frames_written: int = 0    # updated by VideoRecorder
    ticks_logged: int = 0      # updated by TelemetryLogger


class TelemetryLogger:
    """Thread-safe append-only CSV writer. One per active run.

    Called from the MAVLink thread (~10-20 Hz). Internally uses a
    threading.Lock so writes never interleave.
    """

    def __init__(self, csv_path: str):
        self._path = csv_path
        self._lock = threading.Lock()
        self._file = open(csv_path, "w", newline="", buffering=1)
        self._writer = csv.DictWriter(
            self._file, fieldnames=TELEMETRY_COLUMNS, extrasaction="ignore"
        )
        self._writer.writeheader()
        self._file.flush()
        self._ticks_logged = 0

    def log(self, row: dict) -> None:
        """Append one telemetry row. Thread-safe.

        Missing keys are filled with empty string (and a warning printed).
        Extra keys are silently dropped (DictWriter extrasaction="ignore").
        """
        # Validate keys — fill missing with empty string
        missing = [c for c in TELEMETRY_COLUMNS if c not in row]
        if missing:
            print(f"[telemetry] missing columns: {missing}, filling with empty",
                  file=sys.stderr)
            for c in missing:
                row[c] = ""

        try:
            with self._lock:
                self._writer.writerow(row)
                self._file.flush()
                self._ticks_logged += 1
        except Exception as exc:
            # Never crash the caller — the MAVLink thread must keep running.
            print(f"[telemetry] write error: {exc}", file=sys.stderr)

    @property
    def ticks_logged(self) -> int:
        return self._ticks_logged

    def close(self) -> None:
        """Close the CSV file. Idempotent."""
        with self._lock:
            if self._file and not self._file.closed:
                try:
                    self._file.close()
                except Exception:
                    pass


def _sanitize_name(name: str) -> str:
    """Strip non-alphanumeric chars from a run name suffix."""
    import re
    return re.sub(r"[^a-zA-Z0-9_-]", "", name)


def make_run_id(name: Optional[str] = None) -> str:
    """ISO timestamp + optional sanitized name suffix."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if name and name.strip():
        suffix = _sanitize_name(name.strip())
        if suffix:
            return f"{ts}_{suffix}"
    return ts


def start_run(name: Optional[str] = None,
              tmpfs_root: str = "/dev/shm/hive",
              final_root: str = "logs") -> RunHandle:
    """Create tmpfs run dir, init TelemetryLogger, return RunHandle.

    Idempotent on directory creation (os.makedirs exist_ok=True).
    """
    run_id = make_run_id(name)
    tmpfs_dir = os.path.join(tmpfs_root, run_id)
    final_dir = os.path.join(final_root, run_id)
    csv_path = os.path.join(tmpfs_dir, "telemetry.csv")

    os.makedirs(tmpfs_dir, exist_ok=True)

    logger = TelemetryLogger(csv_path)

    return RunHandle(
        run_id=run_id,
        tmpfs_dir=tmpfs_dir,
        final_dir=final_dir,
        csv_path=csv_path,
        started_at=time.time(),
    )


def finalize_run(handle: RunHandle) -> dict:
    """Close logger, sync tmpfs_dir -> final_dir (copy tree), remove tmpfs_dir.

    Returns summary dict with keys:
        run_id, duration_s, frames_written, ticks_logged, csv_path, video_path,
        final_dir.
    """
    duration_s = time.time() - handle.started_at

    # Determine what video file was produced (if any)
    video_filename = "video.mp4"
    video_tmp_path = os.path.join(handle.tmpfs_dir, video_filename)
    video_path = os.path.join(handle.final_dir, video_filename) if os.path.exists(video_tmp_path) else None

    csv_final_path = os.path.join(handle.final_dir, "telemetry.csv")

    summary = {
        "run_id": handle.run_id,
        "duration_s": round(duration_s, 2),
        "frames_written": handle.frames_written,
        "ticks_logged": handle.ticks_logged,
        "csv_path": csv_final_path,
        "video_path": video_path,
        "final_dir": handle.final_dir,
    }

    # Sync tmpfs -> final_dir
    try:
        os.makedirs(handle.final_dir, exist_ok=True)
        # Copy individual files from tmpfs to final dir
        for entry in os.listdir(handle.tmpfs_dir):
            src = os.path.join(handle.tmpfs_dir, entry)
            dst = os.path.join(handle.final_dir, entry)
            if os.path.isfile(src):
                shutil.copy2(src, dst)
            elif os.path.isdir(src):
                shutil.copytree(src, dst)
        summary["synced"] = True
    except Exception as exc:
        print(f"[run] failed to sync {handle.tmpfs_dir} -> {handle.final_dir}: {exc}",
              file=sys.stderr)
        summary["synced"] = False
        summary["sync_error"] = str(exc)

    # Remove tmpfs dir only on successful sync; preserve data on failure
    if summary.get("synced"):
        try:
            shutil.rmtree(handle.tmpfs_dir, ignore_errors=True)
        except Exception:
            pass
    else:
        print(f"[run] SYNC FAILED — tmpfs data preserved at {handle.tmpfs_dir} "
              f"for manual recovery", file=sys.stderr)

    return summary
