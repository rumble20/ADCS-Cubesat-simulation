% run_adcs.m
% Nominal run of the CubeSat ADCS simulation. Prints the requirements
% check and saves plots in ../plots. The actual loop is in simulate_adcs.m

clear; clc; close all;
p = adcs_params();
res = simulate_adcs(p);

% unpack so the plotting code below stays short
t = res.t; err_deg = res.err_deg; w_log = res.w_log; h_log = res.h_log;
tau_log = res.tau_log; gyro_err = res.gyro_err; sun_err = res.sun_err;
mag_err = res.mag_err; triad_err = res.triad_err; est_err = res.est_err;
est_sig = res.est_sig; bias_err = res.bias_err;

ok = ~isnan(triad_err);
fprintf('TRIAD attitude error (sunlit only): RMS %.3f deg, max %.3f deg\n', sqrt(mean(triad_err(ok).^2)), max(triad_err(ok)));
fprintf('Time in eclipse: %.1f min of %.1f min\n', sum(res.eclipse)*p.dt/60, t(end)/60);
% RMS only in sunlight, with the eclipse in it the number was meaningless
ok = ~isnan(res.est_norm) & ~res.eclipse;
fprintf('MEKF attitude error in sunlight: RMS %.3f deg\n', sqrt(mean(res.est_norm(ok).^2)));
fprintf('MEKF max error in sunlight %.3f deg, in eclipse %.3f deg (after 60 s)\n', max(res.est_norm(t>=60 & ~res.eclipse)), max(res.est_norm(t>=60 & res.eclipse)));
fprintf('Final bias error: %s rad/s\n', mat2str(bias_err(:,end)', 3));
ok = ~isnan(est_err(:));
inside = mean(abs(est_err(ok)) < 3*est_sig(ok)) * 100;
fprintf('Samples inside 3-sigma: %.1f %%\n', inside);

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
