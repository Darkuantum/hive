# Calibration run logs

Each run produces a directory under `logs/<run_id>/` containing:

- `telemetry.csv` — synchronized telemetry at ~10-20 Hz with columns defined in
  `integration/calibration/logging.py:TELEMETRY_COLUMNS`
- `video.mp4` — frame-synchronized video recording from the camera

## Tmpfs sync model

During a run, files are written to a tmpfs (RAM disk) at `/dev/shm/hive/<run_id>/`
to avoid SD card wear. When the run is stopped, the entire directory is synced
to `logs/<run_id>/` and the tmpfs copy is removed.

## CSV column reference

See `TELEMETRY_COLUMNS` in `integration/calibration/logging.py` for the canonical
column order and descriptions.
