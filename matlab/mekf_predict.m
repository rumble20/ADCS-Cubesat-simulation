function [q_est, P] = mekf_predict(q_est, b_est, P, gyro, p)
% MEKF prediction step.
% The filter keeps the full attitude quaternion q_est outside the filter,
% and only estimates a small 3-component attitude error + gyro bias error:
%   x = [dtheta; dbias]   (6 states)
% This is the "multiplicative" EKF from Markley & Crassidis ch. 6, simplified.

w_est = gyro - b_est;

% propagate the attitude with the bias-corrected gyro
q_est = quat_mult(q_est, small_rotation_quat(w_est, p.dt));
q_est = q_est / norm(q_est);

% error state transition (first order)
F = [ eye(3) - skew(w_est)*p.dt,  -eye(3)*p.dt;
      zeros(3),                    eye(3)      ];

% process noise: gyro white noise drives dtheta, bias random walk drives dbias
Q = [ (p.sig_gyro^2 * p.dt) * eye(3),   zeros(3);
      zeros(3),                        (p.bias_rw^2 * p.dt) * eye(3) ];

P = F*P*F' + Q;
end
