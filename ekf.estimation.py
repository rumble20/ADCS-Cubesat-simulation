import os
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------
# Simple 1-axis attitude estimation with EKF
# State: [theta, omega, gyro_bias]
# Change these values if you want to experiment
# --------------------------------------------------

dt = 0.1
t_final = 200
time = np.arange(0, t_final + dt, dt)
n_steps = len(time)

Q = np.diag([1e-6, 5e-5, 1e-7])   # process noise
R = np.diag([5e-4, 2e-3])         # measurement noise

bias_rw_std = 2e-5                # true gyro bias random walk
np.random.seed(42)

# True state: [theta, omega, bias]
true_states = np.zeros((3, n_steps))

# Estimated state
x_est = np.zeros((3, n_steps))
P_est = np.zeros((3, 3, n_steps))

# Measurements: [gyro, sun sensor]
measurements = np.zeros((2, n_steps))

# Initial conditions
true_states[:, 0] = [0.25, 0.012, 0.002]
x_est[:, 0] = [0.0, 0.0, 0.0]
P_est[:, :, 0] = np.diag([0.1, 0.1, 0.01])


# --------------------------------------------------
# Models
# --------------------------------------------------

def true_dynamics(x, t):
    theta, omega, bias = x

    # Small angular acceleration to make the motion less trivial
    alpha = 4e-4 * np.sin(0.04 * t)

    theta_next = theta + dt * omega
    omega_next = omega + dt * alpha
    bias_next = bias + np.random.normal(0, bias_rw_std)

    return np.array([theta_next, omega_next, bias_next])


def ekf_dynamics(x):
    theta, omega, bias = x

    # Simpler model used by the filter:
    # angle integrates rate, rate and bias are assumed approximately constant
    return np.array([
        theta + dt * omega,
        omega,
        bias
    ])


def measurement_model(x):
    theta, omega, bias = x

    gyro = omega + bias
    sun_sensor = np.sin(theta)

    return np.array([gyro, sun_sensor])


def ekf_predict(x, P):
    F = np.array([
        [1, dt, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    x_pred = ekf_dynamics(x)
    P_pred = F @ P @ F.T + Q

    return x_pred, P_pred


def ekf_update(x_pred, P_pred, z):
    theta, omega, bias = x_pred

    h = np.array([
        omega + bias,
        np.sin(theta)
    ])

    H = np.array([
        [0, 1, 1],
        [np.cos(theta), 0, 0]
    ])

    y = z - h
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    x_upd = x_pred + K @ y
    P_upd = (np.eye(3) - K @ H) @ P_pred

    return x_upd, P_upd


# --------------------------------------------------
# Simulation loop
# --------------------------------------------------

for k in range(1, n_steps):
    true_states[:, k] = true_dynamics(true_states[:, k - 1], time[k - 1])

    z = measurement_model(true_states[:, k]) + np.random.multivariate_normal(
        np.zeros(2), R
    )
    measurements[:, k] = z

    x_pred, P_pred = ekf_predict(x_est[:, k - 1], P_est[:, :, k - 1])
    x_upd, P_upd = ekf_update(x_pred, P_pred, z)

    x_est[:, k] = x_upd
    P_est[:, :, k] = P_upd


# --------------------------------------------------
# Errors and metrics
# --------------------------------------------------

theta_error = true_states[0, :] - x_est[0, :]
omega_error = true_states[1, :] - x_est[1, :]
bias_error = true_states[2, :] - x_est[2, :]

theta_rmse = np.sqrt(np.mean(theta_error**2))
omega_rmse = np.sqrt(np.mean(omega_error**2))
bias_rmse = np.sqrt(np.mean(bias_error**2))

theta_3sigma = 3 * np.sqrt(P_est[0, 0, :])


# --------------------------------------------------
# Plot results
# --------------------------------------------------

os.makedirs("plots", exist_ok=True)

plt.figure(figsize=(10, 12))

plt.subplot(4, 1, 1)
plt.plot(time, true_states[0, :], label="True angle")
plt.plot(time, x_est[0, :], label="EKF estimate")
plt.ylabel("Angle [rad]")
plt.title("1-axis attitude estimation with EKF and gyro bias estimation")
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(time, true_states[1, :], label="True angular velocity")
plt.plot(time, x_est[1, :], label="EKF estimate")
plt.plot(time, measurements[0, :] - true_states[2, :], "--", alpha=0.7, label="Gyro minus true bias")
plt.ylabel("Angular velocity [rad/s]")
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(time, true_states[2, :], label="True gyro bias")
plt.plot(time, x_est[2, :], label="EKF estimate")
plt.ylabel("Bias [rad/s]")
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(time, theta_error, label="Angle estimation error")
plt.plot(time, theta_3sigma, "--", label="+3 sigma")
plt.plot(time, -theta_3sigma, "--", label="-3 sigma")
plt.xlabel("Time [s]")
plt.ylabel("Angle error [rad]")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("plots/ekf_estimation.png", dpi=180)
plt.show()

print("EKF estimation simulation completed.")
print("Plot saved to plots/ekf_estimation.png")
print(f"Angle RMSE: {theta_rmse:.6f} rad")
print(f"Angular velocity RMSE: {omega_rmse:.6f} rad/s")
print(f"Gyro bias RMSE: {bias_rmse:.6f} rad/s")
