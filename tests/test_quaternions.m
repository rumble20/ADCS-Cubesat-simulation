% Quick sanity checks for the quaternion helpers and TRIAD.
% Run from the tests/ folder:  test_quaternions
% Not a real test framework, just asserts.

addpath('../matlab');   % note: run() cd's into this folder, so pwd is tests/
rng(3);
tol = 1e-9;

for k = 1:200
    q = randn(4,1); q = q/norm(q);
    if q(1) < 0; q = -q; end

    % dcm -> quat -> dcm round trip
    q2 = dcm_to_quat(quat_to_dcm(q));
    assert(norm(q - q2) < tol, 'dcm_to_quat round trip failed');

    % A should be a proper rotation
    A = quat_to_dcm(q);
    assert(norm(A*A' - eye(3)) < tol && abs(det(A) - 1) < tol, 'A not orthonormal');

    % composition: A(q1*q2) = A(q2)*A(q1) for this convention
    qb = randn(4,1); qb = qb/norm(qb);
    assert(norm(quat_to_dcm(quat_mult(q, qb)) - quat_to_dcm(qb)*quat_to_dcm(q)) < tol, ...
        'composition rule wrong');

    % TRIAD with perfect measurements gives the exact attitude
    r1 = randn(3,1); r1 = r1/norm(r1);
    r2 = randn(3,1); r2 = r2/norm(r2);
    qt = triad(A*r1, A*r2, r1, r2);
    assert(norm(qt - q) < 1e-8, 'TRIAD wrong with perfect measurements');
end
disp('test_quaternions: all checks passed');
