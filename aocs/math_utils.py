# Mathematical utilities for quaternion-based attitude simulations.

# Quaternion convention:
# - scalar-first: q = [q0, q1, q2, q3]
# - Hamilton product
# - q represents the rotation from body frame to inertial frame

# With this convention:
# - v_inertial = C(q) @ v_body
# - v_body = C(q).T @ v_inertial

from __future__ import annotations

import numpy as np


def normalize(vector: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Return a normalized copy of the input vector.

    If the norm is too small, the original vector is returned unchanged.
    """
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < eps:
        return vector.copy()
    return vector / norm


def skew(vector: np.ndarray) -> np.ndarray:
    """Return the skew-symmetric matrix of a 3D vector."""
    x, y, z = np.asarray(vector, dtype=float)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def quat_normalize(quaternion: np.ndarray) -> np.ndarray:
    """Return a unit quaternion."""
    return normalize(np.asarray(quaternion, dtype=float))


def quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    """Return the conjugate of a quaternion."""
    q0, q1, q2, q3 = np.asarray(quaternion, dtype=float)
    return np.array([q0, -q1, -q2, -q3], dtype=float)


def quat_multiply(q_a: np.ndarray, q_b: np.ndarray) -> np.ndarray:
    """Return the Hamilton product q_a ⊗ q_b."""
    a0, a1, a2, a3 = np.asarray(q_a, dtype=float)
    b0, b1, b2, b3 = np.asarray(q_b, dtype=float)

    return np.array(
        [
            a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3,
            a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
            a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1,
            a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0,
        ],
        dtype=float,
    )


def quat_from_rotvec(rotation_vector: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Build a quaternion from a rotation vector.

    The rotation vector direction is the rotation axis and its norm is
    the rotation angle in radians.
    """
    rotation_vector = np.asarray(rotation_vector, dtype=float)
    angle = np.linalg.norm(rotation_vector)

    if angle < eps:
        # First-order approximation for very small angles
        q = np.hstack((1.0, 0.5 * rotation_vector))
        return quat_normalize(q)

    axis = rotation_vector / angle
    half_angle = 0.5 * angle
    return np.hstack((np.cos(half_angle), axis * np.sin(half_angle)))


def small_angle_quat(delta_theta: np.ndarray) -> np.ndarray:
    """Return a quaternion from a small-angle error vector."""
    return quat_from_rotvec(np.asarray(delta_theta, dtype=float))


def quat_to_dcm(quaternion: np.ndarray) -> np.ndarray:
    """Convert a unit quaternion to a direction cosine matrix.

    The returned matrix maps body-frame vectors to inertial-frame vectors:
        v_inertial = C(q) @ v_body
    """
    q0, q1, q2, q3 = quat_normalize(quaternion)

    return np.array(
        [
            [1.0 - 2.0 * (q2**2 + q3**2), 2.0 * (q1 * q2 - q0 * q3), 2.0 * (q1 * q3 + q0 * q2)],
            [2.0 * (q1 * q2 + q0 * q3), 1.0 - 2.0 * (q1**2 + q3**2), 2.0 * (q2 * q3 - q0 * q1)],
            [2.0 * (q1 * q3 - q0 * q2), 2.0 * (q2 * q3 + q0 * q1), 1.0 - 2.0 * (q1**2 + q2**2)],
        ],
        dtype=float,
    )


def dcm_to_quat(dcm: np.ndarray) -> np.ndarray:
    """Convert a direction cosine matrix to a scalar-first unit quaternion."""
    c = np.asarray(dcm, dtype=float)
    trace = np.trace(c)

    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q0 = 0.25 * s
        q1 = (c[2, 1] - c[1, 2]) / s
        q2 = (c[0, 2] - c[2, 0]) / s
        q3 = (c[1, 0] - c[0, 1]) / s
    elif c[0, 0] > c[1, 1] and c[0, 0] > c[2, 2]:
        s = 2.0 * np.sqrt(1.0 + c[0, 0] - c[1, 1] - c[2, 2])
        q0 = (c[2, 1] - c[1, 2]) / s
        q1 = 0.25 * s
        q2 = (c[0, 1] + c[1, 0]) / s
        q3 = (c[0, 2] + c[2, 0]) / s
    elif c[1, 1] > c[2, 2]:
        s = 2.0 * np.sqrt(1.0 + c[1, 1] - c[0, 0] - c[2, 2])
        q0 = (c[0, 2] - c[2, 0]) / s
        q1 = (c[0, 1] + c[1, 0]) / s
        q2 = 0.25 * s
        q3 = (c[1, 2] + c[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + c[2, 2] - c[0, 0] - c[1, 1])
        q0 = (c[1, 0] - c[0, 1]) / s
        q1 = (c[0, 2] + c[2, 0]) / s
        q2 = (c[1, 2] + c[2, 1]) / s
        q3 = 0.25 * s

    q = quat_normalize(np.array([q0, q1, q2, q3], dtype=float))

    # Enforce a consistent sign choice for logging and comparisons
    if q[0] < 0.0:
        q = -q

    return q


def rotate_body_to_inertial(quaternion: np.ndarray, vector_body: np.ndarray) -> np.ndarray:
    """Rotate a vector from body frame to inertial frame."""
    return quat_to_dcm(quaternion) @ np.asarray(vector_body, dtype=float)


def rotate_inertial_to_body(quaternion: np.ndarray, vector_inertial: np.ndarray) -> np.ndarray:
    """Rotate a vector from inertial frame to body frame."""
    return quat_to_dcm(quaternion).T @ np.asarray(vector_inertial, dtype=float)


def quat_error(q_reference: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """Return the shortest-path attitude error quaternion.

    The error is defined as:
        q_err = q_reference* ⊗ q_current

    This is appropriate for fixed-reference tracking in this demonstrator.
    """
    q_err = quat_multiply(quat_conjugate(q_reference), q_current)
    q_err = quat_normalize(q_err)

    if q_err[0] < 0.0:
        q_err = -q_err

    return q_err


def quat_angle_error_deg(q_reference: np.ndarray, q_current: np.ndarray) -> float:
    """Return the principal angle between two attitudes in degrees."""
    q_err = quat_error(q_reference, q_current)
    scalar = np.clip(q_err[0], -1.0, 1.0)
    angle_rad = 2.0 * np.arccos(scalar)
    return float(np.degrees(angle_rad))
