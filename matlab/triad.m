function q = triad(b1, b2, r1, r2)
% TRIAD attitude from two vector observations.
%   b1, b2 : measured unit vectors in the body frame
%   r1, r2 : same vectors in the inertial frame
% b1/r1 is trusted more (use the sun sensor there, it is less noisy).
% Returns the quaternion of A (inertial -> body).

t1b = b1;
t2b = cross(b1, b2); t2b = t2b / norm(t2b);
t3b = cross(t1b, t2b);

t1r = r1;
t2r = cross(r1, r2); t2r = t2r / norm(t2r);
t3r = cross(t1r, t2r);

A = [t1b t2b t3b] * [t1r t2r t3r]';
q = dcm_to_quat(A);
end
