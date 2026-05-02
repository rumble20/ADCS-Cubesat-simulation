"""Error-state EKF for attitude and gyro-bias estimation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .math_utils import (
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    skew,
    small_angle_quat,
)


@dataclass
class EKFNoise:
    """Noise parameters used by the estimator."""

    gyro_noise_std: float = 7.5e-4
    gyro_bias_walk_std: float = 2.0e-6
    attitude_meas_std: float = np.deg2rad(0.35)


class AttitudeEKF:
    """Compact multiplicative EKF for attitude and gyro-bias estimation.

    State representation:
    - nominal quaternion q_hat (body to inertial)
    - gyro bias estimate b_hat
    - covariance on the 6D error state:
        x_err = [delta_theta_x, delta_theta_y, delta_theta_z, b_x, b_y, b_z]

    The quaternion itself is not stored inside the covariance state.
    The filter uses:
    - gyroscope propagation
    - a coarse attitude measurement from TRIAD
    """

    def __init__(
        self,
        q_init: np.ndarray,
        bias_init: np.ndarray | None = None,
        p0_attitude: float = 8e-2,
        p0_bias: float = 1e-3,
    ) -> None:
        self.q_hat = quat_normalize(np.asarray(q_init, dtype=float))
        self.bias_hat = (
            np.zeros(3, dtype=float)
            if bias_init is None
            else np.asarray(bias_init, dtype=float)
        )

        self.P = np.zeros((6, 6), dtype=float)
        self.P[0:3, 0:3] = np.eye(3) * p0_attitude
        self.P[3:6, 3:6] = np.eye(3) * p0_bias

    def predict(
        self,
        gyro_meas: np.ndarray,
        dt: float,
        noise: EKFNoise,
    ) -> None:
        """Prediction step using gyroscope data."""
        gyro_meas = np.asarray(gyro_meas, dtype=float)
        omega_hat = gyro_meas - self.bias_hat

        # Nominal quaternion propagation
        dq = small_angle_quat(omega_hat * dt)
        self.q_hat = quat_normalize(quat_multiply(self.q_hat, dq))

        # Linearized covariance propagation
        F = np.eye(6, dtype=float)
        F[0:3, 0:3] -= skew(omega_hat) * dt
        F[0:3, 3:6] = -np.eye(3) * dt

        q_var = (noise.gyro_noise_std**2) * dt
        b_var = (noise.gyro_bias_walk_std**2) * dt

        Q = np.zeros((6, 6), dtype=float)
        Q[0:3, 0:3] = np.eye(3) * q_var
        Q[3:6, 3:6] = np.eye(3) * b_var

        self.P = F @ self.P @ F.T + Q

    def update(
        self,
        q_meas: np.ndarray,
        noise: EKFNoise,
    ) -> None:
        """Correction step using a coarse quaternion attitude measurement."""
        q_meas = quat_normalize(np.asarray(q_meas, dtype=float))

        # For a right-multiplicative error:
        # q_true ≈ q_hat ⊗ delta_q
        # so the residual is built as:
        # delta_q_meas ≈ q_hat* ⊗ q_meas
        q_residual = quat_multiply(quat_conjugate(self.q_hat), q_meas)

        if q_residual[0] < 0.0:
            q_residual = -q_residual

        innovation = 2.0 * q_residual[1:]

        H = np.zeros((3, 6), dtype=float)
        H[:, 0:3] = np.eye(3)

        R = np.eye(3, dtype=float) * (noise.attitude_meas_std**2)

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        delta_x = K @ innovation
        delta_theta = delta_x[0:3]
        delta_bias = delta_x[3:6]

        self.q_hat = quat_normalize(
            quat_multiply(self.q_hat, small_angle_quat(delta_theta))
        )
        self.bias_hat = self.bias_hat + delta_bias

        identity = np.eye(6, dtype=float)
        joseph = identity - K @ H
        self.P = joseph @ self.P @ joseph.T + K @ R @ K.T
