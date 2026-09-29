# Python cross-check of the MATLAB model (steps 1-3: requirements, wheels, sensors).
# Same numbers as matlab/adcs_params.m. The noise is different (different random
# generator) so results only match approximately, not exactly.
import numpy as np

np.random.seed(1)

I = np.diag([0.010, 0.012, 0.018])
dt, t_final = 0.1, 300
Kp, Kd = 0.08, 0.12
tau_max, h_max = 1e-3, 5e-3
sig_sun, sig_mag, sig_gyro, bias_rw = 0.005, 0.010, 5e-4, 1e-5
r_sun = np.array([1.0, 0.0, 0.0])
r_mag = np.array([0.3, 0.5, 0.8]); r_mag /= np.linalg.norm(r_mag)


def quat_mult(a, b):
    return np.array([
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


def quat_to_dcm(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)],
                     [2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)],
                     [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]])


def small_rotation_quat(w, step):
    rot = w * step
    angle = np.linalg.norm(rot)
    if angle < 1e-12:
        return np.array([1.0, 0, 0, 0])
    return np.r_[np.cos(angle/2), rot/angle*np.sin(angle/2)]


def disturbance(t):
    return np.array([1.5e-5*np.sin(0.035*t), -2e-5*np.cos(0.028*t),
                     1e-5*np.sin(0.045*t + 0.25)])


def reaction_wheels(tau_cmd, h):
    tau_w = np.clip(-tau_cmd, -tau_max, tau_max)
    for i in range(3):
        if abs(h[i]) >= h_max and np.sign(tau_w[i]) == np.sign(h[i]):
            tau_w[i] = 0.0
    h_next = np.clip(h + dt*tau_w, -h_max, h_max)
    return -tau_w, h_next


def noisy_unit(v, sigma):
    b = v + sigma*np.random.randn(3)
    return b / np.linalg.norm(b)


t = np.arange(0, t_final + dt, dt)
q = np.array([0.86, 0.18, -0.27, 0.38]); q /= np.linalg.norm(q)
w = np.array([0.035, -0.028, 0.022])
h = np.zeros(3)
bias = np.array([0.002, -0.0015, 0.001])

err, wn, hmax_log, sun_err, mag_err = [], [], [], [], []
for tk in t:
    qe = quat_mult(np.array([1.0, 0, 0, 0]), q)          # q_ref = identity
    if qe[0] < 0:
        qe = -qe
    tau_cmd = -Kp*qe[1:] - Kd*w
    tau_body, h_next = reaction_wheels(tau_cmd, h)

    A = quat_to_dcm(q)
    for r, s, log in ((r_sun, sig_sun, sun_err), (r_mag, sig_mag, mag_err)):
        b = noisy_unit(A @ r, s)
        log.append(np.degrees(np.arccos(min(1.0, b @ (A @ r)))))
    bias = bias + bias_rw*np.sqrt(dt)*np.random.randn(3)   # gyro bias walk (not used yet)

    err.append(np.degrees(2*np.arccos(min(1.0, qe[0]))))
    wn.append(np.linalg.norm(w))
    hmax_log.append(np.abs(h).max())

    w_dot = np.linalg.solve(I, -np.cross(w, I @ w + h) + tau_body + disturbance(tk))
    w_next = w + dt*w_dot
    q = quat_mult(q, small_rotation_quat(0.5*(w + w_next), dt))
    q /= np.linalg.norm(q)
    w, h = w_next, h_next

err, wn = np.array(err), np.array(wn)
after = t >= 60
res = {
    "REQ-01 pointing error after 60 s [deg]": (err[after].max(), 2, "<"),
    "REQ-02 body rate after 60 s [rad/s]": (wn[after].max(), 0.01, "<"),
    "REQ-03 peak wheel momentum [% cap]": (max(hmax_log)/h_max*100, 80, "<"),
    "REQ-04 sun sensor RMS error [deg]": (np.sqrt(np.mean(np.square(sun_err))), 0.5, "<"),
    "REQ-05 magnetometer RMS error [deg]": (np.sqrt(np.mean(np.square(mag_err))), 1.0, "<"),
}
ok_all = True
for name, (val, lim, _) in res.items():
    ok = val < lim
    ok_all &= ok
    print(f"{name:42s} {val:10.4g}  (< {lim})  {'PASS' if ok else 'FAIL'}")
print("Overall:", "PASS" if ok_all else "FAIL")
