import os
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------
# Simple CubeSat attitude control simulation
# Change these values if you want to experiment
# --------------------------------------------------

I = np.diag([0.010, 0.012, 0.018])   # inertia matrix [kg m^2]
I_inv = np.linalg.inv(I)

dt = 0.05
t_final = 200
time = np.arange(0, t_final + dt, dt)

Kp = 0.08
Kd = 0.12
max_torque = 2e-3

q_ref = np.array([1.0, 0.0, 0.0, 0.0])   # desired attitude
w_ref = np.array([0.0, 0.0, 0.0])        # desired angular velocity

q = np.array([0.86, 0.18, -0.27, 0.38])  # initial attitude
q = q / np.linalg.norm(q)

w = np.array([0.035, -0.028, 0.022])     # initial body rates [rad/s]


# --------------------------------------------------
# Helper functions
# --------------------------------------------------

def normalize(q):
    return q / np.linalg.norm(q)


def quat_mult(q1, q2):
    q10, q11, q12, q13 = q1
    q20, q21, q22, q23 = q2

    return np.array([
        q10*q20 - q11*q21 - q12*q22 - q13*q23,
        q10*q21 + q11*q20 + q12*q23 - q13*q22,
        q10*q22 - q11*q23 + q12*q20 + q13*q21,
        q10*q23 + q11*q22 - q12*q21 + q13*q20
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_error(q_ref, q):
    q_err = quat_mult(quat_conj(q_ref), q)
    if q_err[0] < 0:
        q_err = -q_err
    return normalize(q_err)


def small_rotation_quat(w_vec, step):
    rot_vec = w_vec * step
    angle = np.linalg.norm(rot_vec)

    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    axis = rot_vec / angle
    return np.array([
        np.cos(angle / 2),
        axis[0] * np.sin(angle / 2),
        axis[1] * np.sin(angle / 2),
        axis[2] * np.sin(angle / 2)
    ])


def disturbance_torque(t):
    return np.array([
        1.5e-5 * np.sin(0.035 * t),
        -2.0e-5 * np.cos(0.028 * t),
        1.0e-5 * np.sin(0.045 * t + 0.25)
    ])


# --------------------------------------------------
# Logs
# --------------------------------------------------

q_log = np.zeros((len(time), 4))
w_log = np.zeros((len(time), 3))
torque_log = np.zeros((len(time), 3))
dist_log = np.zeros((len(time), 3))
error_log_deg = np.zeros(len(time))


# --------------------------------------------------
# Simulation loop
# --------------------------------------------------

for k, t in enumerate(time):
    q_log[k] = q
    w_log[k] = w

    q_err = quat_error(q_ref, q)
    angle_error = 2 * np.arccos(np.clip(q_err[0], -1.0, 1.0))
    error_log_deg[k] = np.degrees(angle_error)

    torque = -Kp * q_err[1:] - Kd * (w - w_ref)
    torque = np.clip(torque, -max_torque, max_torque)
    torque_log[k] = torque

    tau_dist = disturbance_torque(t)
    dist_log[k] = tau_dist
    tau_total = torque + tau_dist

    w_dot = I_inv @ (tau_total - np.cross(w, I @ w))
    w_next = w + dt * w_dot
    w_mid = 0.5 * (w + w_next)

    dq = small_rotation_quat(w_mid, dt)
    q = quat_mult(q, dq)
    q = normalize(q)

    w = w_next


# --------------------------------------------------
# Simple performance metrics
# --------------------------------------------------

settling_threshold_deg = 2.0
settling_time = None

for k in range(len(time)):
    if np.all(error_log_deg[k:] < settling_threshold_deg):
        settling_time = time[k]
        break


# --------------------------------------------------
# Plot results
# --------------------------------------------------

os.makedirs("plots", exist_ok=True)

plt.figure(figsize=(10, 12))

plt.subplot(4, 1, 1)
plt.plot(time, error_log_deg)
plt.ylabel("Attitude error [deg]")
plt.title("CubeSat attitude regulation with quaternion PD control")
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(time, w_log[:, 0], label="wx")
plt.plot(time, w_log[:, 1], label="wy")
plt.plot(time, w_log[:, 2], label="wz")
plt.ylabel("Angular rate [rad/s]")
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(time, torque_log[:, 0], label="Tx control")
plt.plot(time, torque_log[:, 1], label="Ty control")
plt.plot(time, torque_log[:, 2], label="Tz control")
plt.plot(time, dist_log[:, 0], "--", alpha=0.7, label="Tx disturbance")
plt.plot(time, dist_log[:, 1], "--", alpha=0.7, label="Ty disturbance")
plt.plot(time, dist_log[:, 2], "--", alpha=0.7, label="Tz disturbance")
plt.ylabel("Torque [N m]")
plt.legend(ncol=2, fontsize=8)
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(time, q_log[:, 0], label="q0")
plt.plot(time, q_log[:, 1], label="q1")
plt.plot(time, q_log[:, 2], label="q2")
plt.plot(time, q_log[:, 3], label="q3")
plt.xlabel("Time [s]")
plt.ylabel("Quaternion")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("plots/adcs_simulation.png", dpi=180)
plt.show()

print("Attitude control simulation completed.")
print("Plot saved to plots/adcs_simulation.png")
print(f"Final attitude error: {error_log_deg[-1]:.3f} deg")
print(f"Final angular-rate norm: {np.linalg.norm(w_log[-1]):.6f} rad/s")
if settling_time is not None:
    print(f"Approximate settling time (< {settling_threshold_deg} deg): {settling_time:.2f} s")
else:
    print(f"Attitude error did not stay below {settling_threshold_deg} deg")
