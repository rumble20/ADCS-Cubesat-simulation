function b = sun_sensor(q, p)
% sun direction in the body frame = true direction + noise, then renormalised
b = quat_to_dcm(q) * p.r_sun + p.sig_sun * randn(3,1);
b = b / norm(b);
end
