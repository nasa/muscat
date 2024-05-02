function R = quaternionToRotationMatrix(q)
% Extract vector and scalar parts of the quaternion
qv = q(1:3);
qw = q(4);

% Compute the skew-symmetric matrix for qv
qv_skew = [0, -q(3), q(2);
    q(3), 0, -q(1);
    -q(2), q(1), 0];

% Compute the rotation matrix using the correct matrix expression
R = eye(3) + 2*qw*qv_skew + 2*qv_skew^2;
end