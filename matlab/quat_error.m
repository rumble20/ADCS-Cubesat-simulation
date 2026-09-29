function qe = quat_error(q_ref, q)
% error quaternion between reference and current attitude
qe = quat_mult(quat_conj(q_ref), q);
if qe(1) < 0          % take the short way round
    qe = -qe;
end
qe = qe / norm(qe);
end
