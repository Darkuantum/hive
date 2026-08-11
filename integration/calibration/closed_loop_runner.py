"""Closed-loop step-response execution with safety monitoring.

Applies a position setpoint step through the PID controller by injecting
an offset into the measured pose before PoseController.compute().  This
produces a closed-loop tracking response whose CSV can be analysed by
metrics.compute_metrics().

Blocking execution — the caller is responsible for async wrapping.
"""

import time

from .trajectories import StepAborted, UPDATE_PERIOD, VALID_AXES


# ---------------------------------------------------------------------------
# Safety bounds
# ---------------------------------------------------------------------------
MAX_SETPOINT_M = 0.3       # max surge/sway setpoint (metres)
MAX_SETPOINT_RAD = 0.5     # max yaw setpoint (radians, ~29 deg)
MAX_HOLD = 30.0            # max hold_duration (seconds)
MIN_HOLD = 1.0             # min hold_duration (seconds)


class ClosedLoopStep:
    """Declarative description of a closed-loop position setpoint step.

    Phases: pre (zero setpoint, wait for marker lock) -> step (inject
    setpoint offset) -> post (clear setpoint, settle).
    """

    def __init__(self, axis: str, setpoint: float,
                 hold_duration: float = 5.0,
                 pre_duration: float = 2.0,
                 post_duration: float = 3.0):
        self.axis = axis
        self.setpoint = setpoint
        self.hold_duration = hold_duration
        self.pre_duration = pre_duration
        self.post_duration = post_duration

    # ---- validation -------------------------------------------------------

    def validate(self):
        """Clamp to safe bounds. Raises ValueError on invalid axis."""
        if self.axis not in VALID_AXES:
            raise ValueError(f"axis must be one of {VALID_AXES}, got '{self.axis}'")

        limit = MAX_SETPOINT_RAD if self.axis == 'yaw' else MAX_SETPOINT_M
        self.setpoint = max(-limit, min(limit, self.setpoint))
        self.hold_duration = max(MIN_HOLD, min(MAX_HOLD, self.hold_duration))
        self.pre_duration = max(1.0, min(60.0, self.pre_duration))
        self.post_duration = max(1.0, min(60.0, self.post_duration))

    # ---- helpers ----------------------------------------------------------

    def total_duration(self) -> float:
        return self.pre_duration + self.hold_duration + self.post_duration

    def get_offset(self, phase: str) -> tuple:
        """Return (dx, dy, dyaw) for the given phase.

        'pre' and 'post' return (0, 0, 0).  'step' returns the setpoint
        on the requested axis.
        """
        if phase == 'step':
            dx = self.setpoint if self.axis == 'surge' else 0.0
            dy = self.setpoint if self.axis == 'sway' else 0.0
            dyaw = self.setpoint if self.axis == 'yaw' else 0.0
            return (dx, dy, dyaw)
        return (0.0, 0.0, 0.0)


class ClosedLoopRunner:
    """Executes a ClosedLoopStep via AUTO mode + setpoint offset injection.

    Sequence:
    1. Validate step
    2. Check no run already active
    3. Start logging run
    4. Switch to AUTO mode
    5. Pre phase: zero setpoint, wait for marker lock
    6. Step phase: inject setpoint offset
    7. Post phase: clear setpoint
    8. Stop logging run, return summary

    Always clears the CL setpoint in the finally block.
    """

    MARKER_LOSS_ABORT_S = 1.0  # abort if ArUco marker lost for this long

    def __init__(self, hardware_manager):
        self.hm = hardware_manager

    def run(self, step: ClosedLoopStep, run_name: str = None) -> dict:
        """Execute a closed-loop step response. Returns the logging summary."""
        step.validate()

        # Reject if a run is already active
        if self.hm.get_active_run() is not None:
            raise StepAborted(
                "a logging run is already active; stop it first "
                f"(run_id={self.hm.get_active_run().get('run_id', 'unknown')})"
            )

        # NOTE: no "is a step already running" check here -- run() executes
        # INSIDE the async step's own thread (self.hm._step_thread), so a
        # check for that thread's aliveness here is always true (it's
        # checking itself, mid-execution) and would reject every single
        # closed-loop step unconditionally. start_cl_step_async() already
        # does the real version of this check before spawning the thread.

        self.hm.start_logging_run(run_name)

        original_mode = self.hm.get_control_mode()

        try:
            self.hm.set_control_mode('auto')

            # Pre phase — zero setpoint, wait for marker lock
            dx, dy, dyaw = step.get_offset('pre')
            self.hm.set_cl_setpoint(dx, dy, dyaw)
            self._wait(step.pre_duration, check_marker=True, expected_mode='auto')

            # Step phase — inject the setpoint offset
            dx, dy, dyaw = step.get_offset('step')
            self.hm.set_cl_setpoint(dx, dy, dyaw)
            self._wait(step.hold_duration, check_marker=True, expected_mode='auto')

            # Post phase — clear setpoint
            self.hm.clear_cl_setpoint()
            self._wait(step.post_duration, check_marker=False, expected_mode='auto')

        finally:
            self.hm.clear_cl_setpoint()
            # Zero motors explicitly (AUTO mode would do this when not
            # controlling, but belt-and-suspenders)
            self.hm.set_control(0.0, 0.0, 0.0)
            try:
                self.hm.set_control_mode(original_mode)
            except Exception:
                pass
            summary = self.hm.stop_logging_run()
            self.last_summary = summary

        return summary

    # ---- internal ---------------------------------------------------------

    def _wait(self, duration: float, check_marker: bool, expected_mode: str):
        """Sleep in UPDATE_PERIOD ticks while checking abort conditions."""
        ticks = int(duration / UPDATE_PERIOD)
        marker_lost_since = None

        for _ in range(ticks):
            # Shutdown?
            if self.hm.is_shutting_down:
                raise StepAborted("shutdown requested")

            # Operator abort?
            if self.hm._step_abort.is_set():
                raise StepAborted("abort requested by operator")

            # Mode changed?
            if self.hm.get_control_mode() != expected_mode:
                raise StepAborted(
                    f"control mode changed from '{expected_mode}' during step"
                )

            # Marker lost?
            if check_marker:
                pose = self.hm.get_pose()
                if pose is None:
                    if marker_lost_since is None:
                        marker_lost_since = time.time()
                    elif time.time() - marker_lost_since > self.MARKER_LOSS_ABORT_S:
                        raise StepAborted(
                            f"ArUco marker lost for >{self.MARKER_LOSS_ABORT_S}s"
                        )
                else:
                    marker_lost_since = None

            time.sleep(UPDATE_PERIOD)
