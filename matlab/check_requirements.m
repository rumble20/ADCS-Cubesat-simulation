function all_ok = check_requirements(t, err_deg, w_norm, h_abs, sun_err, mag_err, p)
% Compares the simulation results against docs/requirements.md
% Only looks at t >= 60 s for the pointing and rate requirements.

after = t >= 60;

r1 = max(err_deg(after));                 % deg
r2 = max(w_norm(after));                  % rad/s
r3 = max(h_abs) / p.h_max * 100;          % % of wheel capacity
r4 = sqrt(mean(sun_err.^2));              % deg
r5 = sqrt(mean(mag_err.^2));              % deg

results = [ r1 < 2,  r2 < 0.01,  r3 < 80,  r4 < 0.5,  r5 < 1.0 ];

fprintf('\n---- Requirements check ----\n');
print_line('REQ-01', 'Pointing error after 60 s',    r1, 'deg',   '< 2',     results(1));
print_line('REQ-02', 'Body rate after 60 s',          r2, 'rad/s', '< 0.01',  results(2));
print_line('REQ-03', 'Peak wheel momentum',           r3, '% cap', '< 80',    results(3));
print_line('REQ-04', 'Sun sensor error (RMS)',        r4, 'deg',   '< 0.5',   results(4));
print_line('REQ-05', 'Magnetometer error (RMS)',      r5, 'deg',   '< 1.0',   results(5));

all_ok = all(results);
if all_ok
    fprintf('Overall: PASS\n');
else
    fprintf('Overall: FAIL\n');
end
end

function print_line(id, name, value, unit, limit, ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
fprintf('%s  %-28s %10.4g %-6s (req %s)  %s\n', id, name, value, unit, limit, s);
end
