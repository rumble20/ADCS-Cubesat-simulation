function s = sun_vector_eci(t, p)
% Unit vector from Earth to Sun in ECI.
% Low precision formula (good to about 0.01 deg), from the Astronomical Almanac.
% p.days_j2000 is the epoch in days since 1 Jan 2000 12:00.

n = p.days_j2000 + t / 86400;
L = mod(280.460 + 0.9856474*n, 360);          % mean longitude [deg]
g = mod(357.528 + 0.9856003*n, 360);          % mean anomaly [deg]
lam = L + 1.915*sind(g) + 0.020*sind(2*g);    % ecliptic longitude [deg]
eps = 23.439 - 0.0000004*n;                   % obliquity [deg]

s = [cosd(lam); cosd(eps)*sind(lam); sind(eps)*sind(lam)];
end
