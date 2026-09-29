% monte_carlo.m
% Runs simulate_adcs many times with random initial conditions and noise,
% then checks how many runs meet the requirements and whether the MEKF
% 3-sigma bounds are believable.
% Takes a while (about 10 s per run in Octave on my laptop, faster in MATLAB).

clear; clc; close all;

n_runs = 50;
base = adcs_params();

pass = false(n_runs, 7);
all_err = cell(n_runs,1);   % attitude estimation error, per run
all_sig = cell(n_runs,1);   % filter 1-sigma, per run
all_point = cell(n_runs,1); % true pointing error, per run

rng(100);   % for the random initial conditions (each run has its own noise seed)

for r = 1:n_runs
    p = base;
    p.seed = 1000 + r;

    % random initial attitude: random axis, angle between 0 and 180 deg
    ax = randn(3,1); ax = ax / norm(ax);
    ang = pi * rand;
    p.q0 = [cos(ang/2); ax*sin(ang/2)];

    % random initial rates and gyro bias
    p.w0    = 0.05  * (2*rand(3,1) - 1);     % up to 0.05 rad/s per axis
    p.bias0 = 0.003 * (2*rand(3,1) - 1);     % up to 0.003 rad/s per axis

    res = simulate_adcs(p);
    [ok, pass(r,:)] = check_requirements(res, p, false);

    all_err{r}   = res.est_err;
    all_sig{r}   = res.est_sig;
    all_point{r} = res.err_deg;

    fprintf('run %2d/%d  initial err %5.1f deg  -> %s\n', r, n_runs, 2*ang*90/pi, mat2str(double(pass(r,:))));
end
t = res.t;

% ---- summary ----
fprintf('\n---- Monte Carlo summary (%d runs) ----\n', n_runs);
for i = 1:7
    fprintf('REQ-%02d passed in %2d / %d runs\n', i, sum(pass(:,i)), n_runs);
end
fprintf('All requirements met in %d / %d runs\n', sum(all(pass,2)), n_runs);

% filter consistency: fraction of samples inside +-3 sigma, after the first 10 s
inside = 0; total = 0;
for r = 1:n_runs
    k = t >= 10;
    e = all_err{r}(:,k); s = all_sig{r}(:,k);
    inside = inside + sum(abs(e(:)) < 3*s(:));
    total  = total + numel(e);
end
fprintf('Estimation error inside 3-sigma: %.2f %% of samples (expect about 99.7 %%)\n', inside/total*100);

% worst pointing after 60 s over all runs
worst = zeros(n_runs,1);
for r = 1:n_runs
    worst(r) = max(all_point{r}(t >= 60));
end
fprintf('Pointing error after 60 s: median %.3f deg, worst %.3f deg\n', median(worst), max(worst));

% ---- plots ----
if ~exist('../plots','dir'); mkdir('../plots'); end
lbl = {'x','y','z'};

figure('Position',[100 100 800 800]);
for i = 1:3
    subplot(3,1,i); hold on;
    for r = 1:n_runs
        plot(t, all_err{r}(i,:), 'Color', [0.6 0.6 0.9]);
    end
    % mean 3-sigma over the runs
    sig_mean = zeros(size(t));
    for r = 1:n_runs
        sig_mean = sig_mean + all_sig{r}(i,:) / n_runs;
    end
    plot(t, 3*sig_mean, 'r', 'LineWidth', 1.5);
    plot(t, -3*sig_mean, 'r', 'LineWidth', 1.5);
    ylim([-1 1]); grid on;
    ylabel(['err ' lbl{i} ' [deg]']);
    if i == 1; title(sprintf('MEKF attitude error, %d Monte Carlo runs (red = mean 3-sigma)', n_runs)); end
end
xlabel('Time [s]');
saveas(gcf, '../plots/mc_estimation_error.png');

figure('Position',[100 100 800 400]);
hold on;
for r = 1:n_runs
    semilogy(t, max(all_point{r}, 1e-3), 'Color', [0.6 0.6 0.9]);
end
set(gca, 'YScale', 'log');
plot([60 60], [1e-3 200], 'k--');
plot([t(1) t(end)], [2 2], 'r--');
grid on; xlabel('Time [s]'); ylabel('Pointing error [deg]');
title('Pointing error, all runs (red = 2 deg requirement after 60 s)');
saveas(gcf, '../plots/mc_pointing_error.png');
