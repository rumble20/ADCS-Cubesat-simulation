function b = magnetometer(q, p)
% magnetic field direction in the body frame (unit vector) + noise
b = quat_to_dcm(q) * p.r_mag + p.sig_mag * randn(3,1);
b = b / norm(b);
end
