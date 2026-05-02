from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .math_utils import dcm_to_quat, normalize, rotate_inertial_to_body


@dataclass
class SensorParams:
    """Noise and bias parameters for the simulated sensors."""

    gyro_noise_std: float = 7.5e-4
    gyro_bias_walk_std: float = 2.0e-6
    sun_noise_std: float = 2.5e-3
    mag_noise_std: float = 2.5e-3


def reference_vectors() -> tuple[np.ndarray, np.ndarray]:
    """Return two fixed inertial reference directions.

    These play the role of known environmental directions, similar to a
    sun vector and a magnetic-field vector, without introducing orbit
    propagation into the project.
    """
    sun_inertial = normalize(np.array([0.92, 0.25, -0.18], dtype=float))
    mag_inertial = normalize(np.array([-0.31, 0.12, 0.94], dtype=float))
    return sun_inertial, mag_inertial


def step_gyro_bias(
    current_bias: np.ndarray,
    dt: float,
    params: SensorParams,
    rng: np.random.Generator,
) -> np.ndarray:
    """Advance gyro bias with a simple random-walk model."""
    current_bias = np.asarray(current_bias, dtype=float)
    increment = rng.normal(
        loc=0.0,
        scale=params.gyro_bias_walk_std * np.sqrt(dt),
        size=3,
    )
    return current_bias + increment


def simulate_gyro_measurement(
    omega_true: np.ndarray,
    gyro_bias_true: np.ndarray,
    params: SensorParams,
    rng: np.random.Generator,
) -> np.ndarray:
    """Return a noisy gyroscope measurement."""
    noise = rng.normal(loc=0.0, scale=params.gyro_noise_std, size=3)
    return (
        np.asarray(omega_true, dtype=float)
        + np.asarray(gyro_bias_true, dtype=float)
        + noise
    )


def simulate_direction_measurements(
    quaternion_true: np.ndarray,
    sun_inertial: np.ndarray,
    mag_inertial: np.ndarray,
    params: SensorParams,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    """Return noisy body-frame direction measurements.

    The inputs are ideal inertial reference vectors.
    The outputs are body-frame measured directions.
    """
    sun_body_true = rotate_inertial_to_body(quaternion_true, sun_inertial)
    mag_body_true = rotate_inertial_to_body(quaternion_true, mag_inertial)

    sun_meas = normalize(
        sun_body_true + rng.normal(0.0, params.sun_noise_std, size=3)
    )
    mag_meas = normalize(
        mag_body_true + rng.normal(0.0, params.mag_noise_std, size=3)
    )

    return sun_meas, mag_meas


def _build_triad(v1: np.ndarray, v2: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """Build an orthonormal triad from two non-collinear vectors."""
    t1 = normalize(v1)
    cross = np.cross(t1, v2)
    cross_norm = np.linalg.norm(cross)

    if cross_norm < eps:
        raise ValueError("TRIAD cannot be formed from nearly collinear vectors.")

    t2 = cross / cross_norm
    t3 = np.cross(t1, t2)

    return np.column_stack((t1, t2, t3))


def triad_dcm(
    sun_body: np.ndarray,
    mag_body: np.ndarray,
    sun_inertial: np.ndarray,
    mag_inertial: np.ndarray,
) -> np.ndarray:
    """Return the body-to-inertial DCM from TRIAD.

    Using the standard TRIAD relation:

        C_IB = T_I * T_B^T

    where:
    - T_I is the inertial triad
    - T_B is the body triad
    """
    triad_body = _build_triad(sun_body, mag_body)
    triad_inertial = _build_triad(sun_inertial, mag_inertial)

    return triad_inertial @ triad_body.T


def triad_quaternion(
    sun_body: np.ndarray,
    mag_body: np.ndarray,
    sun_inertial: np.ndarray,
    mag_inertial: np.ndarray,
) -> np.ndarray:
    """Return a coarse body-to-inertial attitude quaternion from TRIAD."""
    return dcm_to_quat(
        triad_dcm(
            sun_body=sun_body,
            mag_body=mag_body,
            sun_inertial=sun_inertial,
            mag_inertial=mag_inertial,
        )
    )
