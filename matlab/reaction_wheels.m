function [tau_body, h_next] = reaction_wheels(tau_cmd, h, p)
% Simple reaction wheel model, 3 wheels aligned with the body axes.
%   tau_cmd  : torque the controller WANTS on the spacecraft [N m]
%   h        : wheel momentum now [N m s]
% Returns the torque the spacecraft really gets and the new wheel momentum.
% To push the body one way the wheel has to be pushed the other way.

tau_w = -tau_cmd;                          % torque on the wheels

% torque limit
tau_w = max(min(tau_w, p.tau_max), -p.tau_max);

% momentum limit: if a wheel is full, it can't be spun up more in that direction
for i = 1:3
    if abs(h(i)) >= p.h_max && sign(tau_w(i)) == sign(h(i))
        tau_w(i) = 0;
    end
end

h_next = h + p.dt * tau_w;
h_next = max(min(h_next, p.h_max), -p.h_max);   % safety clip

tau_body = -tau_w;                         % what the spacecraft feels
end
