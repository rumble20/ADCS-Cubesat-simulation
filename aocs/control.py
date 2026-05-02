from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .math_utils import quat_error


@dataclass
class ControlGains:
    """Gains for quaternion PD regulation."""

    kp: float = 0.08
    kd: float = 0.12
    max_torque: float = 2.0e-3


def quaternion_pd_torque(
    q_current: np.ndarray,
    omega_current: np.ndarray,
    q_reference: np.ndarray,
    omega_reference: np.ndarray,
    gains: ControlGains,
) -> np.ndarray:
    """Return a saturated quaternion PD torque command.

    Notes:
    - This controller is aimed at simple fixed-attitude regulation.
    - The reference in the supplied scenario is inertially fixed and has
      zero body-rate, so a compact PD law is sufficient.
    """
    q_err = quat_error(q_reference, q_current)

    # Use the shortest-path representation
    if q_err[0] < 0.0:
        q_err = -q_err

    omega_current = np.asarray(omega_current, dtype=float)
    omega_reference = np.asarray(omega_reference, dtype=float)

    torque = -gains.kp * q_err[1:] - gains.kd * (omega_current - omega_reference)

    return np.clip(torque, -gains.max_torque, gains.max_torque)
