% build_adcs_model.m
% Builds adcs_model.slx from code. I did it this way because the .slx file
% is binary and git can't show what changed in it, while this script can
% be read. Run it again after changing it, it overwrites the model.
%
% What is in the model (only the control loop, no sensors or MEKF yet):
%   Controller -> flip sign -> torque limit -> momentum limit -> wheels
%   wheels -> torque on body -> rigid body dynamics -> integrators (q, w)
% The blocks call the same .m functions as the MATLAB loop (quat_error,
% quat_mult, disturbance_torque), so ../matlab has to be on the path.
% The numbers come from the struct p in the base workspace (adcs_params).

addpath('../matlab');
mdl = 'adcs_model';

if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

% fixed step solver, same step as the MATLAB loop
set_param(mdl, 'Solver', 'ode4', 'FixedStep', 'p.dt', 'StopTime', 'p.t_final');

% ---- blocks ----
% MATLAB Function blocks: the code is set below with sfroot
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Controller'], ...
    'Position', [250 100 370 170]);
add_block('simulink/Math Operations/Gain', [mdl '/Flip sign'], ...
    'Gain', '-1', 'Position', [420 120 460 150]);
add_block('simulink/Discontinuities/Saturation', [mdl '/Wheel torque limit'], ...
    'UpperLimit', 'p.tau_max', 'LowerLimit', '-p.tau_max', 'Position', [500 120 540 150]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Wheel momentum limit'], ...
    'Position', [590 110 710 180]);
add_block('simulink/Continuous/Integrator', [mdl '/Wheel momentum h'], ...
    'InitialCondition', 'zeros(3,1)', 'Position', [770 200 810 240]);
add_block('simulink/Math Operations/Gain', [mdl '/Torque on body'], ...
    'Gain', '-1', 'Position', [770 120 810 150]);
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [620 300 650 330]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Disturbance'], ...
    'Position', [690 290 810 340]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Rigid body'], ...
    'Position', [880 100 1010 260]);
add_block('simulink/Continuous/Integrator', [mdl '/q'], ...
    'InitialCondition', 'p.q0', 'Position', [1070 120 1110 160]);
add_block('simulink/Continuous/Integrator', [mdl '/w'], ...
    'InitialCondition', 'p.w0', 'Position', [1070 200 1110 240]);

% logging (goes into the "out" object returned by sim)
add_block('simulink/Sinks/To Workspace', [mdl '/log q'], 'VariableName', 'q_sl', ...
    'SaveFormat', 'Array', 'Position', [1200 60 1260 90]);
add_block('simulink/Sinks/To Workspace', [mdl '/log w'], 'VariableName', 'w_sl', ...
    'SaveFormat', 'Array', 'Position', [1200 280 1260 310]);
add_block('simulink/Sinks/To Workspace', [mdl '/log h'], 'VariableName', 'h_sl', ...
    'SaveFormat', 'Array', 'Position', [880 360 940 390]);
add_block('simulink/Sinks/Scope', [mdl '/Rates'], 'Position', [1200 330 1230 360]);
add_block('simulink/Sinks/Scope', [mdl '/Wheel momentum'], 'Position', [880 420 910 450]);

% ---- code inside the MATLAB Function blocks ----
code_ctrl = [ ...
    'function tau_cmd = ctrl(q, w, p)' newline ...
    '% same PD with capped slew rate as simulate_adcs.m, but with the TRUE q and w' newline ...
    'qe = quat_error(p.q_ref, q / norm(q));' newline ...
    'w_cmd = -(p.Kp / p.Kd) * qe(2:4);' newline ...
    'if norm(w_cmd) > p.w_max' newline ...
    '    w_cmd = w_cmd / norm(w_cmd) * p.w_max;' newline ...
    'end' newline ...
    'tau_cmd = -p.Kd * (w - w_cmd);' newline];

code_hlim = [ ...
    'function tau_w = hlim(tau_w_in, h, p)' newline ...
    '% if a wheel is full it cannot be pushed further the same way' newline ...
    '% (same rule as reaction_wheels.m)' newline ...
    'tau_w = tau_w_in;' newline ...
    'for i = 1:3' newline ...
    '    if abs(h(i)) >= p.h_max && sign(tau_w(i)) == sign(h(i))' newline ...
    '        tau_w(i) = 0;' newline ...
    '    end' newline ...
    'end' newline];

code_dist = [ ...
    'function tau_dist = dist(t, p)' newline ...
    'tau_dist = disturbance_torque(t, p);' newline];

code_body = [ ...
    'function [q_dot, w_dot] = body(q, w, h, tau_body, tau_dist, p)' newline ...
    '% Euler equation with the wheel momentum, and quaternion kinematics' newline ...
    'q = q / norm(q);' newline ...
    'w_dot = p.I \ (-cross(w, p.I*w + h) + tau_body + tau_dist);' newline ...
    'q_dot = 0.5 * quat_mult(q, [0; w]);' newline];

blocks = {'Controller', code_ctrl; 'Wheel momentum limit', code_hlim; ...
          'Disturbance', code_dist; 'Rigid body', code_body};
rt = sfroot;
for i = 1:size(blocks, 1)
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' blocks{i,1}]);
    ch.Script = blocks{i,2};
    % p is not an input port, it is read from the workspace
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'p');
    d.Scope = 'Parameter';
    d.Tunable = false;
end

% ---- wires ----
add_line(mdl, 'Controller/1', 'Flip sign/1');
add_line(mdl, 'Flip sign/1', 'Wheel torque limit/1');
add_line(mdl, 'Wheel torque limit/1', 'Wheel momentum limit/1');
add_line(mdl, 'Wheel momentum limit/1', 'Wheel momentum h/1', 'autorouting', 'on');
add_line(mdl, 'Wheel momentum limit/1', 'Torque on body/1', 'autorouting', 'on');
add_line(mdl, 'Wheel momentum h/1', 'Wheel momentum limit/2', 'autorouting', 'on');
add_line(mdl, 'Wheel momentum h/1', 'Rigid body/3', 'autorouting', 'on');
add_line(mdl, 'Wheel momentum h/1', 'log h/1', 'autorouting', 'on');
add_line(mdl, 'Wheel momentum h/1', 'Wheel momentum/1', 'autorouting', 'on');
add_line(mdl, 'Torque on body/1', 'Rigid body/4', 'autorouting', 'on');
add_line(mdl, 'Clock/1', 'Disturbance/1');
add_line(mdl, 'Disturbance/1', 'Rigid body/5', 'autorouting', 'on');
add_line(mdl, 'Rigid body/1', 'q/1');
add_line(mdl, 'Rigid body/2', 'w/1');
add_line(mdl, 'q/1', 'log q/1', 'autorouting', 'on');
add_line(mdl, 'w/1', 'log w/1', 'autorouting', 'on');
add_line(mdl, 'w/1', 'Rates/1', 'autorouting', 'on');
add_line(mdl, 'q/1', 'Controller/1', 'autorouting', 'on');
add_line(mdl, 'w/1', 'Controller/2', 'autorouting', 'on');
add_line(mdl, 'w/1', 'Rigid body/2', 'autorouting', 'on');
add_line(mdl, 'q/1', 'Rigid body/1', 'autorouting', 'on');

save_system(mdl);
fprintf('built %s.slx\n', mdl);
