"""Step-response execution with safety monitoring.

Blocking execution: calls set_control() at UPDATE_HZ via the existing
manual-mode path, with abort on marker loss or mode change.
"""

import time

from .trajectories import StepInput, StepAborted, UPDATE_PERIOD


class StepRunner:
    """Executes a StepInput trajectory via set_control() with safety monitoring.

    Blocking call — runs in the calling thread (typically Flask thread for API,
    or main thread for CLI). The caller is responsible for async wrapping if needed.
    """

    MARKER_LOSS_ABORT_S = 1.0  # abort if ArUco marker lost for this long during step phase

    def __init__(self, hardware_manager):
        self.hm = hardware_manager

    def run(self, step: StepInput, run_name: str = None) -> dict:
        """Execute a complete step response. Returns the stop_logging_run() summary.

        Sequence:
        1. Validate step
        2. Check no run already active
        3. Start logging run
        4. Switch to manual mode
        5. Pre phase: zero motors for pre_duration
        6. Step phase: apply amplitude for step_duration
        7. Post phase: zero motors for post_duration
        8. Stop logging run, return summary

        Abort conditions (checked every UPDATE_PERIOD during all phases):
        - ArUco marker lost for > MARKER_LOSS_ABORT_S
        - Control mode changed away from 'manual' by user
        On abort: zero motors, stop logging run, raise StepAborted with partial summary.

        Always zeros motors in finally block — never leaves vehicle thrusting.
        """
        step.validate()

        # Reject if a run is already active (prevents CSV contamination)
        if self.hm.get_active_run() is not None:
            raise StepAborted(
                "a logging run is already active; stop it first "
                f"(run_id={self.hm.get_active_run().get('run_id', 'unknown')})"
            )

        # Start logging
        self.hm.start_logging_run(run_name)

        # Capture original mode BEFORE switching
        original_mode = self.hm.get_control_mode()

        try:
            # Switch to manual (inside try so finally can clean up)
            self.hm.set_control_mode('manual')

            # Pre phase (baseline)
            self._hold_zeros(step.pre_duration, check_marker=True)

            # Step phase (the actual step)
            self._hold_command(step.to_motor_command(), step.step_duration,
                                check_marker=True)

            # Post phase (settling)
            self._hold_zeros(step.post_duration, check_marker=False)

        finally:
            # ALWAYS: zero motors, restore mode, stop logging
            self.hm.set_control(0.0, 0.0, 0.0)
            try:
                self.hm.set_control_mode(original_mode)
            except Exception:
                pass
            summary = self.hm.stop_logging_run()
            self.last_summary = summary

        return summary

    def _hold_zeros(self, duration: float, check_marker: bool):
        """Send zero commands for duration seconds at UPDATE_HZ."""
        self._hold_command((0.0, 0.0, 0.0), duration, check_marker)

    def _hold_command(self, command: tuple, duration: float, check_marker: bool):
        """Send command at UPDATE_HZ for duration seconds, checking abort conditions."""
        ticks = int(duration / UPDATE_PERIOD)
        marker_lost_since = None

        for _ in range(ticks):
            # Check abort: shutdown requested?
            if self.hm.is_shutting_down():
                raise StepAborted("shutdown requested")

            # Check abort: operator requested abort?
            if self.hm._step_abort.is_set():
                raise StepAborted("abort requested by operator")

            # Check abort: mode changed?
            if self.hm.get_control_mode() != 'manual':
                raise StepAborted("control mode changed from 'manual' during step")

            # Check abort: marker lost?
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

            # Send command
            self.hm.set_control(*command)
            time.sleep(UPDATE_PERIOD)
