function [all_ok, results] = check_requirements(res, p, verbose)
% Compares one simulation run against docs/requirements.md
% res is a struct with the logs from run_adcs (see the end of run_adcs.m).
% Pointing, rate and knowledge are only checked for t >= 60 s.
% Set verbose = false to skip printing (used by the Monte Carlo).

if nargin < 3
    verbose = true;
end

after = res.t >= 60;
late  = res.t >= 120;

v(1) = max(res.err_deg(after));                    % deg
v(2) = max(res.w_norm(after));                     % rad/s
v(3) = max(res.h_abs) / p.h_max * 100;             % % of wheel capacity
sun_ok = ~isnan(res.sun_err);                      % no sun data in eclipse
v(4) = sqrt(mean(res.sun_err(sun_ok).^2));         % deg
v(5) = sqrt(mean(res.mag_err.^2));                 % deg
v(6) = max(res.est_norm(after));                   % deg
v(7) = max(res.bias_err_norm(late));               % rad/s

lim = [2, 0.01, 80, 0.5, 1.0, 0.5, 5e-4];
results = v < lim;
all_ok = all(results);

if ~verbose
    return
end

names = {'Pointing error after 60 s', 'Body rate after 60 s', 'Peak wheel momentum', ...
         'Sun sensor error (RMS)', 'Magnetometer error (RMS)', ...
         'Attitude knowledge after 60 s', 'Bias estimate error after 120 s'};
units = {'deg', 'rad/s', '% cap', 'deg', 'deg', 'deg', 'rad/s'};

fprintf('\n---- Requirements check ----\n');
for i = 1:length(v)
    if results(i)
        s = 'PASS';
    else
        s = 'FAIL';
    end
    fprintf('REQ-%02d  %-32s %10.4g %-6s (req < %g)  %s\n', i, names{i}, v(i), units{i}, lim(i), s);
end
if all_ok
    fprintf('Overall: PASS\n');
else
    fprintf('Overall: FAIL\n');
end
end
