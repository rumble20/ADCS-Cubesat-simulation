function p = adcs_params()
% All the numbers live here so I only change them in one place.
% Quaternions are scalar-first [q0; q1; q2; q3], vectors are columns.

% spacecraft (roughly a 3U cubesat, numbers are guesses)
p.I = diag([0.010 0.012 0.018]);      % inertia [kg m^2]

% simulation
p.dt      = 0.1;                      % [s]
p.t_final = 300;                      % [s]
p.seed    = 1;                        % so the noise is repeatable

% controller (same gains as the first python version)
p.Kp = 0.08;
p.Kd = 0.12;

% reaction wheels (one per axis, ideal, no friction)
p.tau_max = 1e-3;                     % max wheel torque [N m]
p.h_max   = 5e-3;                     % max wheel momentum [N m s]

% initial state
q0 = [0.86; 0.18; -0.27; 0.38];
p.q0 = q0 / norm(q0);
p.w0 = [0.035; -0.028; 0.022];        % [rad/s]
p.q_ref = [1; 0; 0; 0];

% fixed reference vectors in the inertial frame (no orbit yet)
p.r_sun = [1; 0; 0];
r_mag   = [0.3; 0.5; 0.8];
p.r_mag = r_mag / norm(r_mag);

% sensor noise (1 sigma)
p.sig_sun  = 0.005;                   % per component, unit vector [-]
p.sig_mag  = 0.010;                   % per component, unit vector [-]
p.sig_gyro = 5e-4;                    % white noise [rad/s]
p.bias_rw  = 1e-5;                    % bias random walk [rad/s/sqrt(s)]
p.bias0    = [0.002; -0.0015; 0.001]; % initial gyro bias [rad/s]

% estimator (MEKF)
p.P0_att  = (5*pi/180)^2;              % initial attitude error variance [rad^2]
p.P0_bias = (0.005)^2;                 % initial bias error variance [(rad/s)^2]
p.Q_scale = 1;                        % multiply process noise (tuning knob)

% disturbance torque amplitudes [N m]
p.dist_amp = [1.5e-5; 2.0e-5; 1.0e-5];
end
