"""IMU-based velocity damper for station-keeping resilience.

When the ArUco marker is lost, this provides velocity-proportional resistance
using body-frame accelerometer data. Makes the AUV 'heavy' against shoves
and waves. Cannot correct slow steady drift (needs a position reference).

Ported from frame_station_keep_mavlink.py (tuningv2).
"""

import math
import time

# Default parameters (overridable via gains.json velocity_damper section)
DEFAULT_KV = 0.5          # velocity-to-effort gain
DEFAULT_VEL_LEAK = 0.5    # exponential velocity decay (/s)
DEFAULT_ACCEL_LPF_HZ = 5.0  # low-pass cutoff on accelerometer
DEFAULT_ACCEL_DEADBAND = 0.05  # m/s^2 noise floor


class VelocityDamper:
    """Single-axis velocity damper using a leaky integrator on accel.

    Port from frame_station_keep_mavlink.py. Uses body-frame acceleration
    (gravity + bias removed by caller) to estimate pseudo-velocity, then
    produces an opposing effort proportional to that velocity.
    """

    def __init__(self, kv=DEFAULT_KV, vel_leak=DEFAULT_VEL_LEAK,
                 accel_lpf_hz=DEFAULT_ACCEL_LPF_HZ,
                 accel_deadband=DEFAULT_ACCEL_DEADBAND,
                 out_limit=0.4):
        self.kv = kv
        self.vel_leak = vel_leak
        self.accel_deadband = accel_deadband
        self.out_limit = out_limit
        self.vel = 0.0
        self.accel_f = 0.0
        self.prev_time = None
        # LPF alpha computed from nominal 20 Hz loop rate
        # (actual dt varies; approximation is fine for a 5 Hz LPF)
        dt_nominal = 1.0 / 20.0
        rc = 1.0 / (2 * math.pi * accel_lpf_hz)
        self.lpf_alpha = dt_nominal / (rc + dt_nominal)

    def reset(self):
        """Clear velocity estimate and timing state."""
        self.vel = 0.0
        self.accel_f = 0.0
        self.prev_time = None

    def update(self, accel: float, dt: float = None) -> float:
        """Process one accel sample, return damping effort in -out_limit..out_limit.

        Args:
            accel: body-frame acceleration (m/s²), bias-corrected
            dt: time step; if None, computed from wall clock
        Returns:
            Damping effort (velocity-proportional, opposing motion).
        """
        now = time.monotonic()
        if dt is None:
            dt = (1.0 / 20.0 if self.prev_time is None
                  else max(now - self.prev_time, 1e-4))
        self.prev_time = now

        # Low-pass filter the accel
        self.accel_f += self.lpf_alpha * (accel - self.accel_f)
        # Deadband
        a = self.accel_f if abs(self.accel_f) > self.accel_deadband else 0.0
        # Leaky integrator: velocity += a*dt, with exponential decay
        self.vel += a * dt
        self.vel -= self.vel_leak * self.vel * dt
        # Opposing effort, clamped
        effort = -self.kv * self.vel
        return max(-self.out_limit, min(self.out_limit, effort))
