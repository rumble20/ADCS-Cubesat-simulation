function [q_est, b_est, P] = mekf_update(q_est, b_est, P, b_meas, r_ref, sigma)
% MEKF update with one unit-vector measurement (sun or magnetometer).
%   b_meas : measured unit vector in body frame
%   r_ref  : same vector in inertial frame
%   sigma  : noise std of each component of the measurement

A = quat_to_dcm(q_est);
b_pred = A * r_ref;

% if the true attitude is q_est rotated by a small dtheta:
%   b_true ~ b_pred + skew(b_pred)*dtheta
H = [ skew(b_pred), zeros(3) ];
R = sigma^2 * eye(3);

y = b_meas - b_pred;
S = H*P*H' + R;
K = P*H' / S;
dx = K * y;

% put the correction back into the quaternion and bias
dq = [1; 0.5*dx(1:3)];
q_est = quat_mult(q_est, dq);
q_est = q_est / norm(q_est);
b_est = b_est + dx(4:6);

% Joseph form, keeps P symmetric and positive definite
I6 = eye(6);
P = (I6 - K*H)*P*(I6 - K*H)' + K*R*K';
end
