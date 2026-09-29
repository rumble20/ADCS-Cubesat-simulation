function b = mag_field_eci(r, t, p)
% Direction of the Earth magnetic field at position r (ECI), unit vector.
% Tilted dipole model (much simpler than IGRF, but the direction changes
% along the orbit in a realistic way). Only the direction is used because
% the magnetometer model works with unit vectors.

% dipole axis in the Earth-fixed frame. The dipole moment points roughly
% to the geographic SOUTH (the geomagnetic north pole is at about 80.7 N, 72.7 W)
lat = 80.7; lon = -72.7;
m_ecef = -[cosd(lat)*cosd(lon); cosd(lat)*sind(lon); sind(lat)];

% Earth rotation. The start angle is just set to 0.
th = p.w_earth * t;
Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
m = Rz * m_ecef;

rh = r / norm(r);
b = 3*(m'*rh)*rh - m;       % dipole field shape (without the strength factor)
b = b / norm(b);
end
