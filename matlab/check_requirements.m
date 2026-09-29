function [all_ok, results] = check_requirements(res, p, verbose)
% Compares one simulation run against docs/requirements.md
% res is a struct with the logs from run_adcs (see the end of run_adcs.m).
% Pointing, rate and knowledge are only checked when the satellite is in
% the sun and has seen the sun for at least 60 s (at the start and after
% every eclipse). In eclipse only the magnetometer works and the attitude
% drifts, so those numbers are printed separately just to look at.
% The bias check (REQ-07) uses the same rule, after 120 s.
% Set verbose = false to skip printing (used by the Monte Carlo).

if nargin < 3
    verbose = true;
end

% time since the sun was last seen (starts counting at t = 0 too)
sun_time = zeros(size(res.t));
for k = 2:length(res.t)
    if res.eclipse(k)
        sun_time(k) = 0;
    else
        sun_time(k) = sun_time(k-1) + (res.t(k) - res.t(k-1));
    end
end
after = sun_time >= 60 & ~res.eclipse;
late  = res.t >= 120 & after;   % bias also only in sunlight (in eclipse it wanders)

v(1) = max(res.err_deg(after));                    % deg
v(2) = max(res.w_norm(after));                     % rad/s
v(3) = max(res.h_abs) / p.h_max * 100;             % % of wheel capacity
sun_ok = ~isnan(res.sun_err);                      % no sun data in eclipse
v(4) = sqrt(mean(res.sun_err(sun_ok).^2));         % deg
v(5) = sqrt(mean(res.mag_err.^2));                 % deg
v(6) = max(res.est_norm(after));                   % deg
v(7) = max(res.bias_err_norm(late));               % rad/s

lim = [2, 0.01, 80, 0.5, 1.0, 1.0, 5e-4];
results = v < lim;
all_ok = all(results);

if ~verbose
    return
end

names = {'Pointing error (sunlit)', 'Body rate (sunlit)', 'Peak wheel momentum', ...
         'Sun sensor error (RMS)', 'Magnetometer error (RMS)', ...
         'Attitude knowledge (sunlit)', 'Bias estimate error (sunlit)'};
units = {'deg', 'rad/s', '% cap', 'deg', 'deg', 'deg', 'rad/s'};

fprintf('\n---- Requirements check ----\n');
fprintf('(REQ-01, 02, 06 only in sunlight, 60 s after the sun is visible)\n');
for i = 1:length(v)
    if results(i)
        s = 'PASS';
    else
        s = 'FAIL';
    end
    fprintf('REQ-%02d  %-32s %10.4g %-6s (req < %g)  %s\n', i, names{i}, v(i), units{i}, lim(i), s);
end
if any(res.eclipse)
    fprintf('Just for info, in eclipse: max pointing error %.2f deg, max knowledge error %.2f deg\n', ...
        max(res.err_deg(res.eclipse)), max(res.est_norm(res.eclipse)));
end
if all_ok
    fprintf('Overall: PASS\n');
else
    fprintf('Overall: FAIL\n');
end
end
