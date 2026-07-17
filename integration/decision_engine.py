"""
decision_engine.py

The state machine that decides what the platform should be doing, based
on fused camera + IMU + depth signals. This is a PURE state machine --
no MAVLink calls, no camera calls inside it. It takes already-read data
in, returns a state out. That's what makes it testable with fake data
(see the __main__ block) and keeps hardware concerns entirely in
mavlink_interface.py and camFinal.py instead.

States: SEARCHING -> DETECTED -> ALIGNING -> READY -> RECOVERING
"""

import time
from enum import Enum, auto


class RecoveryState(Enum):
    SEARCHING = auto()
    DETECTED = auto()
    ALIGNING = auto()
    READY = auto()
    RECOVERING = auto()


class DecisionEngine:
    def __init__(self,
                 center_tolerance=0.15,     # meters -- how close to 0 counts as centered
                 yaw_tolerance_rad=0.15,    # ~8.6 deg -- how close to aligned counts as aligned
                 stability_tolerance_rad=0.1,  # ~5.7 deg tilt -- platform "calm enough"
                 ready_dwell_s=2.5,         # must hold all conditions this long before READY
                 close_range_z=0.3,         # meters -- below this, marker loss = capture
                 detection_window=10,       # frames, for the confirmed/lost debounce
                 detection_confirm_count=8, # of the last N frames, how many must show a detection
                 aligning_timeout_s=60.0):  # give up and go back to SEARCHING if stuck this long
        self.center_tolerance = center_tolerance
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.stability_tolerance_rad = stability_tolerance_rad
        self.ready_dwell_s = ready_dwell_s
        self.close_range_z = close_range_z
        self.detection_window = detection_window
        self.detection_confirm_count = detection_confirm_count
        self.aligning_timeout_s = aligning_timeout_s

        self.state = RecoveryState.SEARCHING
        self._state_entered_at = time.time()
        self._detection_history = []   # rolling bool window
        self._ready_since = None       # timestamp conditions first became satisfied
        self._was_close_range = False  # tracks whether z has dipped under close_range_z

    # ------------------------------------------------------------------
    def update(self, marker_detected, x_body=None, y_body=None, z_body=None,
               yaw_body=None, platform_roll=None, platform_pitch=None):
        """Call this once per main-loop cycle with the latest fused data.
        Position/yaw args can be None when marker_detected is False."""
        now = time.time()

        self._detection_history.append(bool(marker_detected))
        self._detection_history = self._detection_history[-self.detection_window:]
        confirmed = sum(self._detection_history) >= self.detection_confirm_count
        lost = not confirmed

        centered = (
            x_body is not None and y_body is not None and
            abs(x_body) < self.center_tolerance and
            abs(y_body) < self.center_tolerance
        )
        aligned = (
            yaw_body is not None and abs(yaw_body) < self.yaw_tolerance_rad
        )
        platform_stable = (
            platform_roll is not None and platform_pitch is not None and
            abs(platform_roll) < self.stability_tolerance_rad and
            abs(platform_pitch) < self.stability_tolerance_rad
        )

        # Track whether we've ever gotten close, so a later dropout can
        # be correctly read as "captured" rather than "lost tracking".
        if z_body is not None and z_body < self.close_range_z:
            self._was_close_range = True
        elif confirmed and z_body is not None and z_body >= self.close_range_z:
            # Back at a safe distance with a good detection -- reset the
            # close-range flag so a dropout here is NOT mistaken for capture.
            self._was_close_range = False

        # ---------------- state transitions ----------------
        if self.state == RecoveryState.SEARCHING:
            if confirmed:
                self._transition(RecoveryState.DETECTED)

        elif self.state == RecoveryState.DETECTED:
            if lost:
                self._transition(RecoveryState.SEARCHING)
            elif centered and aligned:
                self._transition(RecoveryState.ALIGNING)
            # else: stay in DETECTED, correction can still run (see main
            # loop diagram -- DETECTED is grouped with ALIGNING for the
            # "actively controlling" branch in practice)

        elif self.state == RecoveryState.ALIGNING:
            if lost:
                if self._was_close_range:
                    # We were close, tracking is now gone -- capture,
                    # not failure.
                    self._transition(RecoveryState.RECOVERING)
                else:
                    self._transition(RecoveryState.SEARCHING)
            elif self._timed_out(self.aligning_timeout_s):
                self._transition(RecoveryState.SEARCHING)
            elif centered and aligned and platform_stable:
                if self._ready_since is None:
                    self._ready_since = now
                elif now - self._ready_since > self.ready_dwell_s:
                    self._transition(RecoveryState.READY)
            else:
                self._ready_since = None  # conditions broke, reset the dwell timer

        elif self.state == RecoveryState.READY:
            if lost:
                if self._was_close_range:
                    self._transition(RecoveryState.RECOVERING)
                else:
                    self._transition(RecoveryState.SEARCHING)
            elif not (centered and aligned and platform_stable):
                # Conditions broke while still tracking -- drop back to
                # ALIGNING, but keep controlling continuously the whole
                # time (this is a status downgrade, not a control pause).
                self._transition(RecoveryState.ALIGNING)
            # else: stay READY. Still correcting every cycle underneath --
            # READY is a status flag layered on top of continuous control,
            # not a signal to stop.

        elif self.state == RecoveryState.RECOVERING:
            pass  # terminal for this system -- see class docstring

        return self.state

    # ------------------------------------------------------------------
    def is_controlling(self):
        """Should the main loop be computing PID output and sending a
        velocity setpoint right now? True for every state except
        SEARCHING (nothing to correct toward) and RECOVERING (capture
        is done, nothing left to correct against)."""
        return self.state in (
            RecoveryState.DETECTED, RecoveryState.ALIGNING, RecoveryState.READY
        )

    def _transition(self, new_state):
        self.state = new_state
        self._state_entered_at = time.time()
        if new_state != RecoveryState.READY:
            self._ready_since = None

    def _timed_out(self, seconds):
        return time.time() - self._state_entered_at > seconds


# ---------------------------------------------------------------------
# Standalone test with synthetic data -- no camera or Pixhawk needed.
# Simulates an approach: far and off-center -> centering -> holding
# ready -> closing distance -> marker lost at close range (capture).
# ---------------------------------------------------------------------
if __name__ == '__main__':
    engine = DecisionEngine(ready_dwell_s=0.5)  # shortened for a fast demo

    scenario = [
        # (marker_detected, x, y, z, yaw, roll, pitch, dt)
        (True,  0.6,  0.4, 1.5, 0.4, 0.05, 0.05, 0.2),
        (True,  0.4,  0.3, 1.4, 0.3, 0.05, 0.05, 0.2),
        (True,  0.2,  0.1, 1.2, 0.15, 0.05, 0.05, 0.2),
        (True,  0.05, 0.05, 1.0, 0.05, 0.02, 0.02, 0.2),  # now centered+aligned+stable
        (True,  0.03, 0.02, 0.8, 0.03, 0.02, 0.02, 0.2),  # holding -> should hit READY
        (True,  0.02, 0.02, 0.6, 0.02, 0.02, 0.02, 0.2),
        (True,  0.02, 0.01, 0.35, 0.01, 0.02, 0.02, 0.2), # closing in
        (True,  0.01, 0.01, 0.28, 0.01, 0.02, 0.02, 0.2), # under close_range_z now
        (False, None, None, None, None, 0.02, 0.02, 0.2), # marker lost at close range -> RECOVERING
    ]

    for i, (detected, x, y, z, yaw, roll, pitch, dt) in enumerate(scenario):
        state = engine.update(detected, x, y, z, yaw, roll, pitch)
        print(f"step {i}: detected={detected}  z={z}  ->  state={state.name}  "
              f"controlling={engine.is_controlling()}")
        time.sleep(dt)
