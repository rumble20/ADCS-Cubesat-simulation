# Rigid-body attitude dynamics for the CubeSat AOCS demonstrator

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from .math_utils import quat_from_rotvec, quat_multiply, quat_normalize

@dataclass
class SpacecraftParams:
    """Physical parameters of the rigid spacecraft model."""

    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([0.010, 0.012, 0.018])
    )

    @property
    def inertia_inv(self) -> np.ndarray:
        """Return the inverse inertia matrix."""
        return np.linalg.inv(self.inertia)


def angular_acceleration(
    omega_body: np.ndarray,
    control_torque_body: np.ndarray,
    disturbance_torque_body: np.ndarray,
    params: SpacecraftParams,
) -> np.ndarray:
    """Compute angular acceleration from Euler's rigid-body equation.

    Equation:
        I * w_dot = tau_total - w x (I w)
    """
    omega_body = np.asarray(omega_body, dtype=float)
    control_torque_body = np.asarray(control_torque_body, dtype=float)
    disturbance_torque_body = np.asarray(disturbance_torque_body, dtype=float)

    total_torque = control_torque_body + disturbance_torque_body

    return params.inertia_inv @ (
        total_torque - np.cross(omega_body, params.inertia @ omega_body)
    )


def step_rigid_body(
    quaternion: np.ndarray,
    omega_body: np.ndarray,
    control_torque_body: np.ndarray,
    disturbance_torque_body: np.ndarray,
    dt: float,
    params: SpacecraftParams,
) -> tuple[np.ndarray, np.ndarray]:
    """Advance the attitude state by one time step.

    The angular rate is integrated with a forward Euler step.
    The quaternion is advanced using a finite rotation built from the
    midpoint angular rate. This is simple and more robust than applying
    a raw derivative step directly to the quaternion.
    """
    q = quat_normalize(np.asarray(quaternion, dtype=float))
    w = np.asarray(omega_body, dtype=float)

    w_dot = angular_acceleration(
        omega_body=w,
        control_torque_body=control_torque_body,
        disturbance_torque_body=disturbance_torque_body,
        params=params,
    )
    w_next = w + dt * w_dot
    w_mid = 0.5 * (w + w_next)

    dq = quat_from_rotvec(w_mid * dt)
    q_next = quat_normalize(quat_multiply(q, dq))

    return q_next, w_next


def disturbance_torque(time_s: float) -> np.ndarray:
    """Return a small deterministic disturbance torque.

    This is not a high-fidelity environmental model. It is only meant
    to keep the simulation from being unrealistically ideal.
    """
    return np.array(
        [
            1.5e-5 * np.sin(0.035 * time_s),
            -2.0e-5 * np.cos(0.028 * time_s),
            1.0e-5 * np.sin(0.045 * time_s + 0.25),
        ],
        dtype=float,
    )
