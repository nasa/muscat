function R = quaternionToRotationMatrix(q)
% q = [qx, qy, qz, qw] in Breckenridge (JPL) convention.
% This returns the passive Direction Cosine Matrix (DCM).

% Extract vector and scalar parts of the quaternion
qv = q(1:3);
qw = q(4);

% Compute the skew-symmetric matrix for qv
qv_skew = skew(qv);

% Compute the rotation matrix using the correct matrix expression
R = eye(3) - 2*qw*qv_skew + 2*qv_skew^2;
end