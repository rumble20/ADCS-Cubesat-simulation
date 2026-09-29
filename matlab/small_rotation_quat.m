function dq = small_rotation_quat(w, step)
% quaternion for rotating with rate w for time "step"
rot = w * step;
angle = norm(rot);
if angle < 1e-12
    dq = [1; 0; 0; 0];
    return
end
axis = rot / angle;
dq = [cos(angle/2); axis*sin(angle/2)];
end
