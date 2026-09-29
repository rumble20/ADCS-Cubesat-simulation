% run_simulink.m
% Runs the Simulink version of the control loop (adcs_model.slx) for 300 s
% and compares it with the MATLAB loop (simulate_adcs.m) from the same
% starting attitude.
% Difference: Simulink uses the TRUE attitude and rate (no sensors, no MEKF),
% the MATLAB loop uses the estimate. So the slew at the start should look
% the same and afterwards the MATLAB one should be a bit noisier.

clear; clc; close all;
addpath('../matlab');

p = adcs_params();
p.t_final = 300;

if ~exist('adcs_model.slx', 'file')
    build_adcs_model;
end
out = sim('adcs_model');

t_sl = out.tout;
q_sl = out.q_sl;    % one row per time step
w_sl = out.w_sl;
h_sl = out.h_sl;

n = length(t_sl);
err_sl = zeros(n,1);
for k = 1:n
    qe = quat_error(p.q_ref, q_sl(k,:)' / norm(q_sl(k,:)));
    err_sl(k) = 2 * acosd(min(1, qe(1)));
end
wnorm_sl = sqrt(sum(w_sl.^2, 2));

% same run with the MATLAB loop
res = simulate_adcs(p);

% only REQ-01..03 make sense here (no sensors in the Simulink model)
after = t_sl >= 60;
v   = [max(err_sl(after)), max(wnorm_sl(after)), max(abs(h_sl(:))) / p.h_max * 100];
lim = [2, 0.01, 80];
names = {'Pointing error after 60 s', 'Body rate after 60 s', 'Peak wheel momentum'};
units = {'deg', 'rad/s', '% cap'};
fprintf('\n---- Simulink model, requirements 1-3 ----\n');
for i = 1:3
    if v(i) < lim(i); s = 'PASS'; else; s = 'FAIL'; end
    fprintf('REQ-%02d  %-28s %10.4g %-6s (req < %g)  %s\n', i, names{i}, v(i), units{i}, lim(i), s);
end
fprintf('Pointing error at t = 30 s: Simulink %.3f deg, MATLAB loop %.3f deg\n', ...
    interp1(t_sl, err_sl, 30), interp1(res.t, res.err_deg, 30));
fprintf('Peak wheel momentum: Simulink %.2f %%, MATLAB loop %.2f %%\n', ...
    v(3), max(res.h_abs) / p.h_max * 100);

% ---- plot ----
if ~exist('../plots','dir'); mkdir('../plots'); end
figure('Position',[100 100 800 600]);
subplot(2,1,1);
semilogy(t_sl, max(err_sl, 1e-3), 'b', res.t, max(res.err_deg, 1e-3), 'r--');
grid on; ylabel('Pointing error [deg]');
legend('Simulink (true state)', 'MATLAB loop (MEKF)');
title('Simulink model vs MATLAB loop');
subplot(2,1,2);
plot(t_sl, h_sl, 'b', res.t, res.h_log', 'r--'); grid on;
ylabel('Wheel momentum [N m s]'); xlabel('Time [s]');
saveas(gcf, '../plots/simulink_vs_matlab.png');
