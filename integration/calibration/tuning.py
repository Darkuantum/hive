"""Compute PID gains from an identified first-order velocity model.

The PoseController is a **position-hold** controller whose output (velocity
commands) is normalised by ``output_limit`` before being sent to the motors.
From the PID's perspective the plant is therefore:

    G(s) = K_eff / (s (τ s + 1))     where K_eff = K / output_limit

The tuning rule is a lambda / IMC approach for an integrating-plus-first-order
plant with a PID controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from .io import Gains

if TYPE_CHECKING:                       # pragma: no cover — runtime guard
    from .identify import StepResponseModel

# PoseController defaults — must match pose_controller.py constructor
DEFAULT_SURGE_OUTPUT_LIMIT = 0.4
DEFAULT_YAW_OUTPUT_LIMIT = 0.6


@dataclass
class TuningResult:
    """Result of gain computation from a step-response model."""

    gains: Gains
    method: str          # 'lambda_integrating_first_order'
    tau_cl: float        # closed-loop time constant used
    Kp: float
    Ki: float
    Kd: float
    K_eff: float         # effective plant gain  K / output_limit
    notes: str = ""

    def summary(self) -> str:
        return (
            f"Kp={self.Kp:.4f}  Ki={self.Ki:.4f}  Kd={self.Kd:.4f}  "
            f"(tau_cl={self.tau_cl:.3f}s, K_eff={self.K_eff:.4f})"
        )


def compute_gains(
    model: "StepResponseModel",
    tau_cl: float | None = None,
    surge_output_limit: float = DEFAULT_SURGE_OUTPUT_LIMIT,
    yaw_output_limit: float = DEFAULT_YAW_OUTPUT_LIMIT,
) -> TuningResult:
    """Compute PID gains for an integrating + first-order plant.

    Plant (PID's perspective): ``G(s) = K_eff / (s (τ s + 1))``
    where ``K_eff = K / output_limit``.

    Tuning (lambda / IMC style) for ``K_eff / (s (τ s + 1))`` with PID:

    * ``Kp = 1 / (K_eff · tau_cl)``
    * ``Ki = Kp / (4 · tau_cl)``
    * ``Kd = τ · Kp``                (cancels first-order lag)

    ``tau_cl`` defaults to ``max(τ, 0.5)`` — we don't try to be faster
    than the actuator.

    Only the axis present in *model* is updated in the returned :class:`Gains`;
    the other two axes keep their default values.
    """
    if tau_cl is None:
        tau_cl = max(model.tau, 0.5)

    # Choose the correct output_limit for this axis
    if model.axis == "yaw":
        output_limit = yaw_output_limit
    else:
        output_limit = surge_output_limit

    K_eff = model.K / output_limit if abs(output_limit) > 1e-9 else model.K

    if abs(K_eff) < 1e-6:
        raise ValueError(
            f"Effective plant gain K_eff={K_eff:.6f} too small for axis "
            f"'{model.axis}'"
        )

    Kp = 1.0 / (K_eff * tau_cl)
    Ki = Kp / (4.0 * tau_cl)
    Kd = model.tau * Kp

    # Build Gains dataclass — only update the identified axis
    gains = Gains()  # defaults
    if model.axis == "surge":
        gains.surge_kp = Kp
        gains.surge_ki = Ki
        gains.surge_kd = Kd
    elif model.axis == "sway":
        gains.sway_kp = Kp
        gains.sway_ki = Ki
        gains.sway_kd = Kd
    elif model.axis == "yaw":
        gains.yaw_kp = Kp
        gains.yaw_ki = Ki
        gains.yaw_kd = Kd

    # Warnings / recommendations
    notes = ""
    if model.R_squared < 0.9:
        notes += (
            f"Warning: model fit R²={model.R_squared:.3f} is low; "
            "gains may be unreliable. "
        )
    if model.tau < 0.1:
        notes += (
            f"Warning: time constant tau={model.tau:.3f}s is very fast; "
            "check data quality. "
        )

    return TuningResult(
        gains=gains,
        method="lambda_integrating_first_order",
        tau_cl=tau_cl,
        Kp=Kp,
        Ki=Ki,
        Kd=Kd,
        K_eff=K_eff,
        notes=notes,
    )
