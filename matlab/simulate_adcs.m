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
sun_err  = nan(1,n);     % NaN in eclipse
mag_err  = zeros(1,n);
triad_err = nan(1,n);
est_err  = nan(3,n);    % attitude estimation error [deg], per axis
est_sig  = nan(3,n);    % 1-sigma from the filter [deg]
bias_err = nan(3,n);    % bias estimation error [rad/s]
ecl_log  = false(1,n);
started  = false;         % becomes true when the MEKF has been started
t_start  = NaN;

for k = 1:n
    % --- orbit and environment ---
    r_pos = orbit_position(t(k), p);
    p.r_mag = mag_field_eci(r_pos, t(k), p);
    dark = in_eclipse(r_pos, p.r_sun, p);
    ecl_log(k) = dark;

    % --- sensors (using the true state at this step) ---
    A = quat_to_dcm(q);
    mag = magnetometer(q, p);
    [gyro, bias] = gyro_model(w, bias, p);
    if ~dark
        sun = sun_sensor(q, p);
    end

    % --- attitude estimation (MEKF) ---
    if ~started
        % start the filter from TRIAD (sun as primary vector), bias unknown.
        % TRIAD is bad when the two vectors are almost parallel (I saw 11.7 deg
        % error when they were 11 deg apart), so wait until they are 20 deg apart
        if ~dark && acosd(abs(p.r_sun' * p.r_mag)) > 20
            q_est = triad(sun, mag, p.r_sun, p.r_mag);
            b_est = zeros(3,1);
            P = blkdiag(p.P0_att*eye(3), p.P0_bias*eye(3));
            started = true;
            t_start = t(k);
        end
    else
        [q_est, P] = mekf_predict(q_est, b_est, P, gyro_prev, p);
        if ~dark
            [q_est, b_est, P] = mekf_update(q_est, b_est, P, sun, p.r_sun, p.sig_sun);
        end
        [q_est, b_est, P] = mekf_update(q_est, b_est, P, mag, p.r_mag, p.sig_mag);
    end
    gyro_prev = gyro;

    % --- control (uses the ESTIMATED attitude and rate) ---
    if started
        qe_est = quat_error(p.q_ref, q_est);
        w_est = gyro - b_est;
        % PD written as "track a commanded rate". The commanded rate is capped at
        % p.w_max, otherwise big slews spin up the wheels too much (see NOTES.md).
        % For small errors this is exactly the same as tau = -Kp*qv - Kd*w.
        w_cmd = -(p.Kp / p.Kd) * qe_est(2:4);
        if norm(w_cmd) > p.w_max
            w_cmd = w_cmd / norm(w_cmd) * p.w_max;
        end
        tau_cmd = -p.Kd * (w_est - w_cmd);
    else
        % no attitude yet (eclipse or bad TRIAD geometry): only damp the rates
        tau_cmd = -p.Kd * gyro;
    end

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
    mag_err(k)    = acosd(min(1, mag' * (A * p.r_mag)));
    if ~dark
        sun_err(k) = acosd(min(1, sun' * (A * p.r_sun)));
        qe_t = quat_error(q, triad(sun, mag, p.r_sun, p.r_mag));
        triad_err(k) = 2 * acosd(min(1, qe_t(1)));
    end
    if started
        dq_est = quat_error(q_est, q);          % true = est * dq
        est_err(:,k)  = 2 * dq_est(2:4) * 180/pi;
        est_sig(:,k)  = sqrt(diag(P(1:3,1:3))) * 180/pi;
        bias_err(:,k) = bias - b_est;
    end

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
res.eclipse  = ecl_log;
res.t_start  = t_start;
end
