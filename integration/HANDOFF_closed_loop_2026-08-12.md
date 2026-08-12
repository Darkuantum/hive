# Closed-loop validation handoff — 2026-08-12

Branch: `calibration`. Task #7 ("Closed-loop validation: both directions per axis")
still **in progress** — surge positive direction got one fully-clean run, everything
else (surge negative, sway, yaw) not yet attempted this session.

## tl;dr for the next session

The vehicle just came back from a water-in-electronics-bay incident (dried out,
retested, motors confirmed fine — see earlier in this session's history). Once
back to calibration work, closed-loop testing kept producing "commanded surge,
got yaw" style results. Three real, separate bugs were found and fixed. A fourth
issue (yaw vision-tracking robustness) got a partial mitigation but is **not**
fully solved — see "Known open issues" below before trusting any closed-loop yaw
behavior.

**Do not assume any of this is fixed without re-verifying against current code**
— read the diffs, don't just trust this doc's descriptions.

## Bugs found and fixed this session

All three are real, root-caused, and confirmed via live telemetry — not guesses.

### 1. Closed-loop setpoint offset applied in the wrong frame
`hardware.py`'s `_compute_auto_control()` was subtracting the closed-loop
validation setpoint (`_cl_setpoint_x/y`, body-frame surge/sway) directly from
the **raw camera-frame** pose, before `PoseController.compute()`'s internal
`camera_to_body()` rotation. Under the camera's mount rotation, that meant a
"surge" test setpoint landed on the wrong body axis after rotation.

Fix: added `body_to_camera()` to `pose_controller.py` (inverse of
`camera_to_body()`, since it's a pure rotation — inverse = transpose) and used
it in `hardware.py` to rotate the offset into camera frame *before* subtracting,
so it lands on the intended body axis regardless of mount angle.

### 2. Yaw error not angle-wrapped
`pose_controller.py`'s `PoseController.compute()` had `error_yaw = -yaw_body`
with no wrapping. Once `yaw_body` (which is `yaw_cam + CAMERA_MOUNT_YAW_DEG`)
exceeded ±π, the yaw PID saw a huge fake error and saturated. Fixed to wrap to
`(-π, π]`.

### 3. `CAMERA_MOUNT_YAW_DEG` sign was wrong
Was `+90.0`, empirically confirmed should be `-90.0` — live `yaw_cam` sat near
+90° while things were actually aligned; +90° mount offset pushed `yaw_body` to
~180° (saturating), -90° brings it to ~0°. User confirmed the marker's physical
orientation is correct, so this was a code constant sign bug, not a physical
setup issue. The comment above it in code already flagged it as
unverified ("verify sign against the real net before trusting it") — now
verified and fixed.

**Important side effect**: `camera_to_body()`'s rotation matrix is shared
between yaw and surge/sway position. Flipping the mount-angle sign by 180°
total rotation negates **both** `x_body` and `y_body`, not just yaw. That means
the surge/sway gains in `gains.json`, which were identified (tasks #1-6) under
the old (+90°) sign convention, needed their sign flipped to match. This was
done: `surge_kp/ki/kd` and `sway_kp/ki/kd` in `gains.json` are now **positive**
(were negative). `yaw_kp/ki/kd` untouched — yaw only gets a constant additive
shift from the mount angle, not a sign flip, so its gain sign was never wrong.

This was **not** assumed — it was empirically confirmed: before the gain flip,
a test run showed `sway_measured` diverging away from center while
`motor_y` saturated at 1000 (positive-feedback runaway, the signature of a
wrong-signed gain). After flipping, the same test showed sway converging
toward center normally.

## Also touched: motor output bank — do NOT "fix" this without asking

Live `SERVOx_FUNCTION`/`MOT_PWM_TYPE` reads showed thrusters currently
configured on **MAIN1-4** (`SERVO1-4_FUNCTION=33-36`, `MOT_PWM_TYPE=0`), which
looks like a regression from an earlier (2026-08-05) fix that moved them to
AUX3-6 for DShot. **It is not a regression** — the user intentionally moved the
wiring/config back to MAIN1-4 at some point after 2026-08-05. This is the
correct, current, intended state. `pixhawk_set_output_bank.py --check` (repo
root, read-only) is safe to run anytime to see current live state; do **not**
run `pixhawk_set_output_bank.py aux` without first explicitly asking whether
the physical wiring is actually on AUX right now.

`AHRS_ORIENTATION=4` (ROTATION_YAW_180) is also correct/expected — the Pixhawk
board is physically mounted facing 180° opposite the vehicle's front, and this
param correctly compensates. Not a fault, don't touch it.

## Partial mitigation: ArUco single-marker yaw-flip filter

`camFinal.py`'s `ArucoDetector` now rejects implausible frame-to-frame yaw
jumps (>60°) via `_last_yaw`/`_pending_yaw`/`_pending_count` state, only
accepting a big jump as "real" if it repeats for 3 consecutive frames
(`yaw_confirm_frames`). This targets a real, confirmed phenomenon: OpenCV's
`estimatePoseSingleMarkers` has a well-known ~180° orientation ambiguity that
can flip between frames from small viewing-angle changes alone — confirmed
live by driving the vehicle in **manual mode** (no PID running at all) and
watching the raw camera-reported yaw snap between ~+90° and ~-90° purely from
translation, with zero possibility of real physical rotation causing it.

**This filter has a confirmed blind spot**: it only catches *brief* flickers.
In a later test run (`cl_surge_pos_v7`, see
`integration/logs/20260812_224744_cl_surge_pos_v7/telemetry.csv`, t≈100.2–101.4
relative to run start), a wrong ArUco reading held **stable for ~1.2 seconds
(13+ consecutive frames)** before snapping back — long enough to pass the
3-frame confirmation threshold, so the filter accepted it as "real" and the
yaw PID saturated (`motor_r=-1000`) for that whole stretch. The debounce
window is too short relative to how long a bad solution can stay locked in.

**Not yet solved.** Possible directions for next session:
- Cross-check the ArUco yaw against something independent (e.g. Pixhawk gyro
  integration over short windows) and reject readings inconsistent with recent
  angular rate, instead of relying purely on frame-to-frame continuity.
- Try to select the correct PnP solution directly rather than filtering after
  the fact — newer OpenCV ArUco APIs can return multiple candidate
  solutions with reprojection error; picking by reprojection error or by
  proximity to the previous frame's solution (a "solution voting" scheme
  across more than 3 frames) might be more robust than a hard jump threshold.
- Physically changing marker viewing distance/angle to move out of the
  near-degenerate geometry where the ambiguity is worst (untested whether
  practical given the rig's constraints).

## Unexplained: late-hold-phase sway/yaw drift

Same `cl_surge_pos_v7` run, t≈109–110 (near the end of a 5s surge hold,
*after* the yaw flip above had already resolved): `sway_measured` and
`yaw_measured` both grew **smoothly and together** (sway 0.032→0.064m, yaw
0.29→0.37rad) while `surge_measured` was actually converging fine. `motor_y`
hit -1000 (saturated) three times fighting it, without arresting the drift.

This does **not** look like a vision artifact — it's smooth, not a discrete
jump like the ArUco flip. Leading hypothesis: real physical coupling (surge
thrust inducing a sway/yaw disturbance — asymmetric thruster response, cable
drag, or a tether/mount reaction if the bench setup isn't fully free), which
current sway/yaw gains aren't strong/fast enough to fully counter. Not
root-caused. Needs either more data (does it happen on every run? does it
scale with surge amplitude/duration?) or physical observation during a run to
confirm whether it's a real disturbance vs. something else.

## What actually worked (proof the 3 fixes are real)

Run `cl_surge_pos_v7` (`integration/logs/20260812_224744_cl_surge_pos_v7/`):
full 10s duration, no marker-loss abort. Surge converged from 0 toward the
0.1m setpoint (steady-state error 0.05m, 0% overshoot, settling ~7.5s — a
tuning gap, not a bug). This is a genuine improvement over every earlier
attempt this session, which either diverged, saturated for the full run, or
aborted on marker loss within 1-5s.

## Files changed (uncommitted as of this doc)

```
 M integration/camFinal.py       # yaw jump-rejection filter
 M integration/hardware.py       # cl_setpoint offset rotated via body_to_camera()
 M integration/pose_controller.py # body_to_camera() added, yaw wrapped, mount angle sign fixed
?? integration/gains.json        # surge/sway gain signs flipped positive (untracked file, --gains-file default)
```

Nothing has been committed. Review diffs before committing —
`git diff integration/camFinal.py integration/hardware.py integration/pose_controller.py`.

## How to resume testing

```bash
cd /home/hive/hive/integration
python3 app.py --mavlink-conn /dev/ttyACM0 --mavlink-baud 115200 \
  --gains-file gains.json --af-mode once
```

Standard cautious loop used this session (repeat per axis/direction):
1. `curl -s localhost:8000/api/state` — confirm `camera.marker_detected: true`
   before doing anything else. If false, `curl -X POST
   localhost:8000/api/camera/refocus` and recheck.
2. `curl -X POST localhost:8000/api/arm`
3. `curl -X POST localhost:8000/api/calibrate/cl-step/run -d
   '{"axis":"surge","setpoint":0.05,"hold_duration":3.0,"pre_duration":2.0,"post_duration":2.0}'`
   — start small (0.05m, 3s) after any code/gain change, not the full 0.1m/5s.
4. Poll `curl localhost:8000/api/calibrate/step/status` until `"status":"done"`
   or `"status":"error"`.
5. **Immediately** `curl -X POST localhost:8000/api/disarm` regardless of
   outcome, then re-check `/api/state` — disarm doesn't always reflect
   instantly, poll again after ~1s before trusting it.
6. Inspect the run's `telemetry.csv` in `integration/logs/<run_id>/` before
   deciding whether to retry — don't just trust the `/api/calibrate/metrics`
   summary numbers, they can look deceptively OK (e.g. "0% overshoot" when the
   vehicle barely moved at all was the first symptom of bug #1 above).

## Remaining task list

- #7 Closed-loop validation (both directions per axis): still in progress.
  Only surge-positive got one clean run. Surge-negative, sway (both
  directions), yaw (both directions) not yet attempted post-fixes.
- Yaw vision-flip filter blind spot (long-duration stable-but-wrong readings):
  open, see above.
- Sway/yaw coupling during surge hold: open, not root-caused, see above.
