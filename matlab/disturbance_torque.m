function tau = disturbance_torque(t, p)
% made-up sinusoidal disturbances, just to have something to reject
tau = [  p.dist_amp(1)*sin(0.035*t);
        -p.dist_amp(2)*cos(0.028*t);
         p.dist_amp(3)*sin(0.045*t + 0.25) ];
end
