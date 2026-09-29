% run_adcs.m
% CubeSat attitude control with reaction wheels + simulated sensors.
% Run this file. Needs the other .m files in this folder.
%
% NOTE: the controller still uses the TRUE attitude and rate.
% The sensors are only logged for now, the estimator comes later.

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

for k = 1:n
    % --- control ---
    qe = quat_error(p.q_ref, q);
    tau_cmd = -p.Kp * qe(2:4) - p.Kd * w;

    % --- actuator ---
    [tau_body, h_next] = reaction_wheels(tau_cmd, h, p);

    % --- sensors (using the true state at this step) ---
    A = quat_to_dcm(q);
    sun = sun_sensor(q, p);
    mag = magnetometer(q, p);
    [gyro, bias] = gyro_model(w, bias, p);

    % --- attitude determination (TRIAD, sun as primary vector) ---
    q_triad = triad(sun, mag, p.r_sun, p.r_mag);

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

% --- requirements ---
check_requirements(t, err_deg, sqrt(sum(w_log.^2,1)), max(abs(h_log),[],1), sun_err, mag_err, p);

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
