"""Declarative trajectory definitions + safety validation for open-loop calibration."""

from dataclasses import dataclass


MAX_AMPLITUDE = 0.5        # max |motor command| during step
MAX_STEP_DURATION = 30.0   # seconds at amplitude
MIN_STEP_DURATION = 1.0
MIN_PHASE_DURATION = 1.0
MAX_PHASE_DURATION = 60.0
UPDATE_HZ = 5.0            # set_control update rate (well within 0.5s watchdog)
UPDATE_PERIOD = 1.0 / UPDATE_HZ

VALID_AXES = ('surge', 'sway', 'yaw')


@dataclass
class StepInput:
    """Open-loop step input for system identification.

    Phases: pre (baseline at zero) → step (at amplitude) → post (settling at zero).
    The CSV logger captures position + motor commands throughout.
    """
    axis: str                    # 'surge' | 'sway' | 'yaw'
    amplitude: float             # motor command during step phase, -0.5..0.5
    pre_duration: float = 2.0    # seconds at zero before step
    step_duration: float = 5.0   # seconds at amplitude
    post_duration: float = 3.0   # seconds at zero after step

    def total_duration(self) -> float:
        return self.pre_duration + self.step_duration + self.post_duration

    def validate(self) -> None:
        """Validate and clamp to safe bounds. Raises ValueError on invalid axis."""
        if self.axis not in VALID_AXES:
            raise ValueError(f"axis must be one of {VALID_AXES}, got '{self.axis}'")
        self.amplitude = max(-MAX_AMPLITUDE, min(MAX_AMPLITUDE, self.amplitude))
        self.pre_duration = max(MIN_PHASE_DURATION, min(MAX_PHASE_DURATION, self.pre_duration))
        self.step_duration = max(MIN_STEP_DURATION, min(MAX_STEP_DURATION, self.step_duration))
        self.post_duration = max(MIN_PHASE_DURATION, min(MAX_PHASE_DURATION, self.post_duration))

    def to_motor_command(self) -> tuple:
        """Return (x, y, r) for set_control() during the step phase."""
        cmd = [0.0, 0.0, 0.0]
        idx = VALID_AXES.index(self.axis)
        cmd[idx] = self.amplitude
        return tuple(cmd)


class StepAborted(Exception):
    """Raised when a step run is aborted (marker loss, mode change, etc.)."""
    def __init__(self, reason: str, partial_summary: dict = None):
        self.reason = reason
        self.partial_summary = partial_summary or {}
        super().__init__(reason)
