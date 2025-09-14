# ADCS CubeSat Estimation and Control - Project Skeleton

# =============================
# 1️⃣ Imports and Dependencies
# =============================
import numpy as np
import matplotlib.pyplot as plt

# =============================
# 2️⃣ Constants and Parameters
# =============================
I = np.diag([0.01, 0.01, 0.02])  # Inertia matrix [kg*m^2]
dt = 0.01                        # Time step [s]
t_final = 300                    # Simulation time [s]
t = np.arange(0, t_final, dt)

# Reference orientation (for example: aligned with LVLH)
ref_quat = np.array([1, 0, 0, 0])

# =============================
# 3️⃣ Functions
# =============================
def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def quat_derivative(q, w):
    # Quaternion derivative: dq/dt = 0.5 * Omega(w) * q
    Omega = np.block([
        [0, -w[0], -w[1], -w[2]],
        [w[0], 0, w[2], -w[1]],
        [w[1], -w[2], 0, w[0]],
        [w[2], w[1], -w[0], 0]
    ])
    return 0.5 * Omega @ q

def normalize_quat(q):
    return q / np.linalg.norm(q)

# =============================
# 4️⃣ Initialization
# =============================
q = np.array([1, 0, 0, 0])  # initial quaternion
w = np.array([0.01, 0.01, -0.02])  # initial angular velocity [rad/s]

q_log = []
w_log = []

# =============================
# 5️⃣ Simulation Loop
# =============================
for ti in t:
    # Simple PD Controller to stabilize to ref_quat
    Kp = 0.05
    Kd = 0.02
    
    q_err = q * ref_quat  # quaternion multiplication for error (to refine)
    torque = -Kp * q_err[1:] - Kd * w  # PD control

    # Euler's rotation equation: I * dw/dt = torque - w x (I * w)
    dw = np.linalg.inv(I) @ (torque - np.cross(w, I @ w))
    w = w + dw * dt

    dq = quat_derivative(q, w)
    q = q + dq * dt
    q = normalize_quat(q)
    
    # Logging
    q_log.append(q)
    w_log.append(w)

# =============================
# 6️⃣ Plot Results
# =============================
q_log = np.array(q_log)
w_log = np.array(w_log)

plt.figure(figsize=(10, 6))
plt.plot(t, w_log[:, 0], label='wx [rad/s]')
plt.plot(t, w_log[:, 1], label='wy [rad/s]')
plt.plot(t, w_log[:, 2], label='wz [rad/s]')
plt.title('Angular Velocities over Time')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(t, q_log[:, 0], label='q0')
plt.plot(t, q_log[:, 1], label='q1')
plt.plot(t, q_log[:, 2], label='q2')
plt.plot(t, q_log[:, 3], label='q3')
plt.title('Quaternion over Time')
plt.xlabel('Time [s]')
plt.ylabel('Quaternion Components')
plt.legend()
plt.grid(True)
plt.show()

# =============================
# 7️⃣ Next Steps (TODO for you):
# =============================
# - Implement EKF to estimate attitude with noisy sensors
# - Add sensor simulation (gyro + magnetometer + sun sensor with noise)
# - Refine quaternion error computation
# - Add reaction wheel or magnetic torquer model for actuation
# - Add disturbances (gravity gradient, magnetic field, solar pressure)
# - Document functions with docstrings and add comments for clarity
# - Create a README with background theory and usage instructions
