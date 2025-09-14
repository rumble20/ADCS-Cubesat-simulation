import numpy as np
import matplotlib.pyplot as plt

# ===============================
# EKF Implementation for CubeSat
# ===============================

# Parameters
dt = 0.1   # time step (s)
T = 200    # total simulation time (s)
n_steps = int(T/dt)

# True system states: [theta, omega] (angle [rad], angular velocity [rad/s])
true_states = np.zeros((2, n_steps))

# Measurements: gyro (angular velocity), sun sensor (angle proxy)
measurements = np.zeros((2, n_steps))

# Estimated states via EKF
x_est = np.zeros((2, n_steps))
P_est = np.zeros((2,2,n_steps))

# Process and measurement noise covariances
Q = np.diag([1e-5, 1e-4])   # process noise (small)
R = np.diag([1e-3, 1e-2])   # measurement noise (larger)

# Initial state
true_states[:,0] = [0.2, 0.0]  # initial angle ~11°, angular vel 0
x_est[:,0] = [0.0, 0.0]
P_est[:,:,0] = np.eye(2)*0.1

# Dynamics: simple rotational system (like 1D CubeSat axis)
def dynamics(x):
    theta, omega = x
    # Here torque disturbances can be added, for now just free dynamics
    return np.array([theta + dt*omega,
                     omega])

# Measurement model: gyro + sun sensor proxy
def measure(x):
    theta, omega = x
    gyro = omega
    sun = np.sin(theta)  # a nonlinear proxy (like sun sensor response)
    return np.array([gyro, sun])

# EKF prediction step
def ekf_predict(x, P):
    # Jacobian of dynamics wrt state
    F = np.array([[1, dt],
                  [0, 1]])
    x_pred = dynamics(x)
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred

# EKF update step
def ekf_update(x_pred, P_pred, z):
    theta, omega = x_pred
    # Nonlinear measurement model
    h = np.array([omega, np.sin(theta)])
    # Jacobian wrt state
    H = np.array([[0, 1],
                  [np.cos(theta), 0]])
    y = z - h
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_upd = x_pred + K @ y
    P_upd = (np.eye(2) - K @ H) @ P_pred
    return x_upd, P_upd

# ===============================
# Simulation loop
# ===============================

np.random.seed(42)

for k in range(1, n_steps):
    # True dynamics
    true_states[:,k] = dynamics(true_states[:,k-1])

    # Measurements with noise
    z = measure(true_states[:,k]) + np.random.multivariate_normal(np.zeros(2), R)
    measurements[:,k] = z

    # EKF prediction
    x_pred, P_pred = ekf_predict(x_est[:,k-1], P_est[:,:,k-1])

    # EKF update
    x_upd, P_upd = ekf_update(x_pred, P_pred, z)

    # Store
    x_est[:,k] = x_upd
    P_est[:,:,k] = P_upd

# ===============================
# Plot results
# ===============================

time = np.arange(0,T,dt)

plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(time, true_states[0,:], label="True angle θ [rad]")
plt.plot(time, x_est[0,:], label="EKF estimate θ [rad]")
plt.xlabel("Time [s]")
plt.ylabel("Angle [rad]")
plt.legend()

plt.subplot(2,1,2)
plt.plot(time, true_states[1,:], label="True ω [rad/s]")
plt.plot(time, x_est[1,:], label="EKF estimate ω [rad/s]")
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")
plt.legend()

plt.tight_layout()
plt.show()
