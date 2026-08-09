# Camera Scripts

## Production module

**`integration/camFinal.py`** — the canonical ArUco detection module used by the
running system. Exposes `ArucoDetector` with `capture_and_detect()` returning
`{id, x, y, z, yaw, frame}`. This is what `integration/hardware.py` imports.
Do not run standalone scripts as a substitute for this in production.

## Standalone benchmark / tuning scripts

These scripts are for offline testing, tuning, and demonstration. Each was
built iteratively to test a specific configuration. They are **not** imported
by the integration layer.

| Script | Purpose | LED | Key feature |
|--------|---------|-----|-------------|
| `camtestv5.py` | **Recommended for new tests** | Yes (10px) | Target-ID lock (id0 4x4_50 50mm marker), x/y/z-correction defaults |
| `camtestv4.py` | Dehaze/CLAHE/AE tuning | Yes | All enhancements fixed ON, X/Y/Z position correction tuning |
| `camtestv3_led.py` | LED illumination testing | Yes (10px) | Single DotStar strip, CSV trial logging |
| `camtestv3_led_dual.py` | Dual-LED testing | Yes (2 strips) | Drives two DotStar strips in sync |
| `camtestv3.py` | Base benchmark | No | CSV logging + wf-recorder screen capture |
| `camtest_record.py` | Simple recording | No | Lightweight recording variant |

### Which script should I use?

- **New benchmark / tuning session** → `camtestv5.py` (latest, most capable)
- **Testing LED illumination** → `camtestv5.py` or `camtestv3_led.py`
- **Dual-LED hardware test** → `camtestv3_led_dual.py`
- **Simple recording without LED** → `camtest_record.py`
- **Image enhancement tuning** → `camtestv4.py` (dehaze/CLAHE controls)

## Usage

All scripts accept `--help` for the full option list. Common patterns:

```bash
# Recommended: latest benchmark with LED + recording + CSV logging
uv run python camera/camtestv5.py

# Headless over SSH (no preview window)
uv run python camera/camtestv5.py --no-preview

# Disable recording (CSV-only run)
uv run python camera/camtestv5.py --no-record

# LED illumination test
uv run python camera/camtestv3_led.py --led

# Dual-LED hardware test
uv run python camera/camtestv3_led_dual.py --led

# Simple recording without LED or enhancements
uv run python camera/camtest_record.py

# Image enhancement tuning (dehaze/CLAHE/denoise controls)
uv run python camera/camtestv4.py --white-balance --denoise

# Full options for any script
uv run python camera/camtestv5.py --help
```

Common options across scripts: `--record`/`--no-record`, `--auto-log`,
`--dehaze`, `--white-balance`, `--clahe`/`--no-clahe`, `--denoise`,
`--temporal-filter`. LED variants add `--led`/`--no-led`.
`camtestv5.py` adds `--id-filter`/`--no-id-filter` for target-ID lock.

Results are written to `camera/results/` (CSV trial logs + screen recordings).

## Library

**`underwater_pipeline.py`** — image enhancement library (dehaze, white
balance, CLAHE, gamma, denoise) + Kalman-filtered PoseTracker. Not currently
imported by `camFinal.py` (which has its own simpler `enhance_low_light()`),
but available for future integration if underwater conditions require heavier
processing.

## Archive

`camera/archive/` contains earlier iterations (`aruco_detect.py`, `cam12cm.py`,
`camtest.py`, `camtestv2.py`) kept for reference. Use the current generation
above for new work.
