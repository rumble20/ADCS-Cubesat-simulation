function res = simulate_adcs(p)
% One closed-loop run of the ADCS simulation.
% Loop: sensors -> TRIAD (init) -> MEKF -> PD controller -> wheels -> dynamics
% The controller only sees the estimated state, not the true one.
% Returns a struct with all the logs (used by run_adcs and monte_carlo).

rng(p.seed);

t = 0:p.dt:p.t_final;
n = length(t);

% state
q = p.q0;
w = p.w0;
h = zeros(3,1);          % wheel momentum
bias = p.bias0;

% logs
err_deg  = zeros(1,n);
w_log    = zeros(3,n);
h_log    = zeros(3,n);
tau_log  = zeros(3,n);
gyro_err = zeros(3,n);
sun_err  = zeros(1,n);
mag_err  = zeros(1,n);
triad_err = zeros(1,n);
est_err  = zeros(3,n);    % attitude estimation error [deg], per axis
est_sig  = zeros(3,n);    % 1-sigma from the filter [deg]
bias_err = zeros(3,n);    % bias estimation error [rad/s]

for k = 1:n
    % --- sensors (using the true state at this step) ---
    A = quat_to_dcm(q);
    sun = sun_sensor(q, p);
    mag = magnetometer(q, p);
    [gyro, bias] = gyro_model(w, bias, p);

    % --- attitude determination (TRIAD, sun as primary vector) ---
    q_triad = triad(sun, mag, p.r_sun, p.r_mag);

    % --- attitude estimation (MEKF) ---
    if k == 1
        % start the filter from TRIAD, bias unknown
        q_est = q_triad;
        b_est = zeros(3,1);
        P = blkdiag(p.P0_att*eye(3), p.P0_bias*eye(3));
    else
        [q_est, P] = mekf_predict(q_est, b_est, P, gyro_prev, p);
        [q_est, b_est, P] = mekf_update(q_est, b_est, P, sun, p.r_sun, p.sig_sun);
        [q_est, b_est, P] = mekf_update(q_est, b_est, P, mag, p.r_mag, p.sig_mag);
    end
    gyro_prev = gyro;

    % --- control (uses the ESTIMATED attitude and rate now) ---
    qe_est = quat_error(p.q_ref, q_est);
    w_est  = gyro - b_est;
    tau_cmd = -p.Kp * qe_est(2:4) - p.Kd * w_est;

    % --- actuator ---
    [tau_body, h_next] = reaction_wheels(tau_cmd, h, p);

    % true pointing error, for logging and requirements
    qe = quat_error(p.q_ref, q);

    % --- logging ---
    err_deg(k)    = 2 * acosd(min(1, qe(1)));
    w_log(:,k)    = w;
    h_log(:,k)    = h;
    tau_log(:,k)  = tau_body;
    gyro_err(:,k) = gyro - w;
    sun_err(k)    = acosd(min(1, sun' * (A * p.r_sun)));
    mag_err(k)    = acosd(min(1, mag' * (A * p.r_mag)));
    qe_t = quat_error(q, q_triad);
    triad_err(k)  = 2 * acosd(min(1, qe_t(1)));
    dq_est = quat_error(q_est, q);          % true = est * dq
    est_err(:,k)  = 2 * dq_est(2:4) * 180/pi;
    est_sig(:,k)  = sqrt(diag(P(1:3,1:3))) * 180/pi;
    bias_err(:,k) = bias - b_est;

    % --- dynamics (Euler for the rate, midpoint rate for the quaternion) ---
    tau_dist = disturbance_torque(t(k), p);
    w_dot = p.I \ ( -cross(w, p.I*w + h) + tau_body + tau_dist );
    w_next = w + p.dt * w_dot;
    w_mid = 0.5 * (w + w_next);

    q = quat_mult(q, small_rotation_quat(w_mid, p.dt));
    q = q / norm(q);
    w = w_next;
    h = h_next;
end

res.t        = t;
res.err_deg  = err_deg;
res.w_log    = w_log;
res.h_log    = h_log;
res.tau_log  = tau_log;
res.gyro_err = gyro_err;
res.sun_err  = sun_err;
res.mag_err  = mag_err;
res.triad_err = triad_err;
res.est_err  = est_err;
res.est_sig  = est_sig;
res.bias_err = bias_err;
res.w_norm   = sqrt(sum(w_log.^2,1));
res.h_abs    = max(abs(h_log),[],1);
res.est_norm = sqrt(sum(est_err.^2,1));
res.bias_err_norm = sqrt(sum(bias_err.^2,1));
end
