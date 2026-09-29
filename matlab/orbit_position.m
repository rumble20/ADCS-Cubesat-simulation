function r = orbit_position(t, p)
% Position of the satellite in the inertial frame (ECI) [m].
% Circular orbit only: radius, inclination, RAAN and the starting
% argument of latitude u0. No J2, no drag.

a = p.R_earth + p.alt;
n_orb = sqrt(p.mu / a^3);          % mean motion [rad/s]
u = p.u0 + n_orb * t;              % argument of latitude

i = p.inc;
W = p.raan;

r = a * [ cos(u)*cos(W) - sin(u)*cos(i)*sin(W);
          cos(u)*sin(W) + sin(u)*cos(i)*cos(W);
          sin(u)*sin(i) ];
end
