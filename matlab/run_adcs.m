% run_adcs.m
% CubeSat attitude control with reaction wheels + simulated sensors.
% Run this file. Needs the other .m files in this folder.
%
% Loop: sensors -> TRIAD (init) -> MEKF -> PD controller -> wheels -> dynamics
% The controller only sees the estimated state, not the true one.

clear; clc; close all;
p = adcs_params();
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

fprintf('TRIAD attitude error: RMS %.3f deg, max %.3f deg\n', sqrt(mean(triad_err.^2)), max(triad_err));

est_norm = sqrt(sum(est_err.^2,1));
fprintf('MEKF attitude error: RMS %.3f deg, max after 60 s %.3f deg\n', sqrt(mean(est_norm.^2)), max(est_norm(t>=60)));
fprintf('Final bias error: %s rad/s\n', mat2str(bias_err(:,end)', 3));
inside = mean(abs(est_err(:)) < 3*est_sig(:)) * 100;
fprintf('Samples inside 3-sigma: %.1f %%\n', inside);

% --- requirements ---
res.t        = t;
res.err_deg  = err_deg;
res.w_norm   = sqrt(sum(w_log.^2,1));
res.h_abs    = max(abs(h_log),[],1);
res.sun_err  = sun_err;
res.mag_err  = mag_err;
res.est_norm = est_norm;
res.bias_err_norm = sqrt(sum(bias_err.^2,1));
check_requirements(res, p);

% --- plots ---
% save next to the python plots in the repo root, not inside matlab/
if ~exist('../plots','dir'); mkdir('../plots'); end

figure('Position',[100 100 800 900]);
subplot(4,1,1); plot(t, err_deg); grid on;
ylabel('Attitude error [deg]'); title('CubeSat attitude control with reaction wheels');
subplot(4,1,2); plot(t, w_log'); grid on;
ylabel('Rate [rad/s]'); legend('wx','wy','wz');
subplot(4,1,3); plot(t, tau_log'); grid on;
ylabel('Torque on body [N m]'); legend('x','y','z');
subplot(4,1,4); plot(t, h_log'); hold on;
% yline does not exist in Octave, so draw the limits by hand
plot([t(1) t(end)], [p.h_max p.h_max], 'k--');
plot([t(1) t(end)], [-p.h_max -p.h_max], 'k--'); grid on;
ylabel('Wheel momentum [N m s]'); xlabel('Time [s]'); legend('x','y','z');
saveas(gcf, '../plots/matlab_control.png');

figure('Position',[100 100 800 900]);
subplot(4,1,1); plot(t, sun_err); grid on; ylabel('Sun sensor error [deg]');
title('Simulated sensor errors');
subplot(4,1,2); plot(t, mag_err); grid on; ylabel('Magnetometer error [deg]');
subplot(4,1,3); plot(t, gyro_err'); grid on; ylabel('Gyro error [rad/s]'); legend('x','y','z');
subplot(4,1,4); plot(t, triad_err); grid on; ylabel('TRIAD error [deg]'); xlabel('Time [s]');
saveas(gcf, '../plots/matlab_sensors.png');

figure('Position',[100 100 800 900]);
lbl = {'x','y','z'};
for i = 1:3
    subplot(4,1,i); plot(t, est_err(i,:)); hold on;
    plot(t, 3*est_sig(i,:), 'r--'); plot(t, -3*est_sig(i,:), 'r--'); grid on;
    ylim([-1 1]);   % the first seconds have a huge sigma and hide everything else
    ylabel(['err ' lbl{i} ' [deg]']);
    if i == 1; title('MEKF attitude error with 3-sigma bounds'); end
end
subplot(4,1,4); plot(t, bias_err'); grid on;
ylabel('Bias error [rad/s]'); xlabel('Time [s]'); legend('x','y','z');
saveas(gcf, '../plots/matlab_mekf.png');
