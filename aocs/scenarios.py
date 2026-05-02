from __future__ import annotations
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from .control import ControlGains, quaternion_pd_torque
from .dynamics import SpacecraftParams, disturbance_torque, step_rigid_body
from .ekf import AttitudeEKF, EKFNoise
from .math_utils import quat_angle_error_deg, quat_normalize
from .sensors import (
    SensorParams,
    reference_vectors,
    simulate_direction_measurements,
    simulate_gyro_measurement,
    step_gyro_bias,
    triad_quaternion,
)


def _ensure_output_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def run_control_demo(save_path: str | Path, show: bool = False) -> dict:
    """Run the closed-loop attitude regulation demonstration."""
    params = SpacecraftParams()
    gains = ControlGains()

    dt = 0.1
    t_final = 240.0
    time = np.arange(0.0, t_final + dt, dt)

    q_ref = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    w_ref = np.zeros(3, dtype=float)

    q = quat_normalize(np.array([0.86, 0.18, -0.27, 0.38], dtype=float))
    w = np.array([0.035, -0.028, 0.022], dtype=float)

    q_log = np.zeros((time.size, 4), dtype=float)
    w_log = np.zeros((time.size, 3), dtype=float)
    torque_log = np.zeros((time.size, 3), dtype=float)
    error_log_deg = np.zeros(time.size, dtype=float)

    for k, t_now in enumerate(time):
        q_log[k] = q
        w_log[k] = w
        error_log_deg[k] = quat_angle_error_deg(q_ref, q)

        torque_cmd = quaternion_pd_torque(
            q_current=q,
            omega_current=w,
            q_reference=q_ref,
            omega_reference=w_ref,
            gains=gains,
        )
        torque_log[k] = torque_cmd

        q, w = step_rigid_body(
            quaternion=q,
            omega_body=w,
            control_torque_body=torque_cmd,
            disturbance_torque_body=disturbance_torque(t_now),
            dt=dt,
            params=params,
        )

    save_path = Path(save_path)
    _ensure_output_dir(save_path)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(time, error_log_deg, linewidth=2.0)
    axes[0].set_ylabel("Attitude error [deg]")
    axes[0].set_title("Quaternion PD attitude regulation")
    axes[0].grid(True, alpha=0.3)

    for idx, label in enumerate(["wx", "wy", "wz"]):
        axes[1].plot(time, w_log[:, idx], label=label)
    axes[1].set_ylabel("Angular rate [rad/s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    for idx, label in enumerate(["Tx", "Ty", "Tz"]):
        axes[2].plot(time, torque_log[:, idx], label=label)
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Control torque [N m]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=180)

    if show:
        plt.show()

    plt.close(fig)

    return {
        "time": time,
        "q_log": q_log,
        "w_log": w_log,
        "torque_log": torque_log,
        "attitude_error_deg": error_log_deg,
        "final_attitude_error_deg": float(error_log_deg[-1]),
        "final_rate_norm": float(np.linalg.norm(w_log[-1])),
        "plot_path": str(save_path),
    }


def run_estimation_demo(
    save_path: str | Path,
    seed: int = 7,
    show: bool = False,
) -> dict:
    """Run the attitude estimation demonstration.

    The true spacecraft evolves in free rotational motion with a small
    disturbance torque. Sensors provide:
    - gyroscope measurements with bias and noise
    - sun and magnetic-field body vectors with noise

    TRIAD gives a coarse attitude solution.
    The EKF refines attitude and gyro-bias estimates.
    """
    rng = np.random.default_rng(seed)

    params = SpacecraftParams()
    sensor_params = SensorParams()
    sun_i, mag_i = reference_vectors()

    ekf_noise = EKFNoise(
        gyro_noise_std=sensor_params.gyro_noise_std,
        gyro_bias_walk_std=sensor_params.gyro_bias_walk_std,
        attitude_meas_std=np.deg2rad(0.35),
    )

    dt = 0.1
    t_final = 240.0
    time = np.arange(0.0, t_final + dt, dt)

    q_true = quat_normalize(np.array([0.93, -0.15, 0.21, 0.25], dtype=float))
    w_true = np.array([0.018, -0.012, 0.016], dtype=float)
    bias_true = np.array([8.0e-4, -5.0e-4, 4.0e-4], dtype=float)

    estimator = AttitudeEKF(
        q_init=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        bias_init=np.zeros(3, dtype=float),
        p0_attitude=8e-2,
        p0_bias=1e-3,
    )

    ekf_error_deg = np.zeros(time.size, dtype=float)
    triad_error_deg = np.zeros(time.size, dtype=float)
    w_true_log = np.zeros((time.size, 3), dtype=float)
    gyro_log = np.zeros((time.size, 3), dtype=float)
    bias_true_log = np.zeros((time.size, 3), dtype=float)
    bias_est_log = np.zeros((time.size, 3), dtype=float)

    for k, t_now in enumerate(time):
        gyro_meas = simulate_gyro_measurement(
            omega_true=w_true,
            gyro_bias_true=bias_true,
            params=sensor_params,
            rng=rng,
        )

        sun_body_meas, mag_body_meas = simulate_direction_measurements(
            quaternion_true=q_true,
            sun_inertial=sun_i,
            mag_inertial=mag_i,
            params=sensor_params,
            rng=rng,
        )

        q_triad = triad_quaternion(
            sun_body=sun_body_meas,
            mag_body=mag_body_meas,
            sun_inertial=sun_i,
            mag_inertial=mag_i,
        )

        estimator.predict(
            gyro_meas=gyro_meas,
            dt=dt,
            noise=ekf_noise,
        )
        estimator.update(
            q_meas=q_triad,
            noise=ekf_noise,
        )

        triad_error_deg[k] = quat_angle_error_deg(q_true, q_triad)
        ekf_error_deg[k] = quat_angle_error_deg(q_true, estimator.q_hat)

        w_true_log[k] = w_true
        gyro_log[k] = gyro_meas
        bias_true_log[k] = bias_true
        bias_est_log[k] = estimator.bias_hat

        bias_true = step_gyro_bias(
            current_bias=bias_true,
            dt=dt,
            params=sensor_params,
            rng=rng,
        )

        q_true, w_true = step_rigid_body(
            quaternion=q_true,
            omega_body=w_true,
            control_torque_body=np.zeros(3, dtype=float),
            disturbance_torque_body=disturbance_torque(t_now),
            dt=dt,
            params=params,
        )

    save_path = Path(save_path)
    _ensure_output_dir(save_path)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(time, triad_error_deg, label="TRIAD error", alpha=0.7)
    axes[0].plot(time, ekf_error_deg, label="EKF error", linewidth=2.0)
    axes[0].set_ylabel("Attitude error [deg]")
    axes[0].set_title("Attitude estimation: TRIAD baseline and EKF refinement")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    for idx, label in enumerate(["wx true", "wy true", "wz true"]):
        axes[1].plot(time, w_true_log[:, idx], label=label)
    for idx, label in enumerate(["gyro x", "gyro y", "gyro z"]):
        axes[1].plot(
            time,
            gyro_log[:, idx],
            linestyle="--",
            alpha=0.55,
            label=label,
        )
    axes[1].set_ylabel("Rate [rad/s]")
    axes[1].legend(ncol=2, fontsize=8)
    axes[1].grid(True, alpha=0.3)

    for idx, axis_name in enumerate(["x", "y", "z"]):
        axes[2].plot(time, bias_true_log[:, idx], label=f"bias true {axis_name}")
        axes[2].plot(
            time,
            bias_est_log[:, idx],
            linestyle="--",
            label=f"bias est {axis_name}",
        )
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Gyro bias [rad/s]")
    axes[2].legend(ncol=2, fontsize=8)
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=180)

    if show:
        plt.show()

    plt.close(fig)

    return {
        "time": time,
        "triad_error_deg": triad_error_deg,
        "ekf_error_deg": ekf_error_deg,
        "bias_true_log": bias_true_log,
        "bias_est_log": bias_est_log,
        "final_attitude_error_deg": float(ekf_error_deg[-1]),
        "rms_attitude_error_deg": float(np.sqrt(np.mean(ekf_error_deg**2))),
        "plot_path": str(save_path),
    }
