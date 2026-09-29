function tau = gravity_gradient(q, r, p)
% Gravity gradient torque [N m]. Tiny for a cubesat but free to add now
% that there is an orbit.
nb = quat_to_dcm(q) * (r / norm(r));   % direction to the satellite, in body frame
tau = 3 * p.mu / norm(r)^3 * cross(nb, p.I * nb);
end
