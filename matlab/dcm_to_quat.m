function q = dcm_to_quat(A)
% Inverse of quat_to_dcm.m (A rotates inertial -> body), scalar first.
% Picks the largest of the four "diagonal" terms to avoid dividing by
% something close to zero (found this trick in Markley & Crassidis).
tr = trace(A);
d = [tr, A(1,1), A(2,2), A(3,3)];
[~, i] = max(d);

switch i
    case 1
        q0 = 0.5*sqrt(1 + tr);
        q = [q0; (A(2,3)-A(3,2))/(4*q0); (A(3,1)-A(1,3))/(4*q0); (A(1,2)-A(2,1))/(4*q0)];
    case 2
        q1 = 0.5*sqrt(1 + 2*A(1,1) - tr);
        q = [(A(2,3)-A(3,2))/(4*q1); q1; (A(1,2)+A(2,1))/(4*q1); (A(1,3)+A(3,1))/(4*q1)];
    case 3
        q2 = 0.5*sqrt(1 + 2*A(2,2) - tr);
        q = [(A(3,1)-A(1,3))/(4*q2); (A(1,2)+A(2,1))/(4*q2); q2; (A(2,3)+A(3,2))/(4*q2)];
    case 4
        q3 = 0.5*sqrt(1 + 2*A(3,3) - tr);
        q = [(A(1,2)-A(2,1))/(4*q3); (A(1,3)+A(3,1))/(4*q3); (A(2,3)+A(3,2))/(4*q3); q3];
end

if q(1) < 0
    q = -q;
end
q = q / norm(q);
end
