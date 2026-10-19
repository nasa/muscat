function q = rotationMatrixToQuaternion(R)
% rotationMatrixToQuaternion converts a 3x3 DCM to a
% scalar-last JPL/Breckenridge quaternion [qx; qy; qz; qw].
% This follows the passive rotation convention (JPL memo).

if size(R,1) ~= 3 || size(R,2) ~= 3
    error('Input matrix must be 3x3');
end

traceR = trace(R);

if traceR > 0
    S = sqrt(traceR + 1.0) * 2; 
    qw = 0.25 * S;
    % Signs flipped for Breckenridge (Passive)
    qx = (R(2,3) - R(3,2)) / S;
    qy = (R(3,1) - R(1,3)) / S;
    qz = (R(1,2) - R(2,1)) / S;
elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
    S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; 
    qw = (R(2,3) - R(3,2)) / S;
    qx = 0.25 * S;
    qy = (R(1,2) + R(2,1)) / S;
    qz = (R(1,3) + R(3,1)) / S;
elseif R(2,2) > R(3,3)
    S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; 
    qw = (R(3,1) - R(1,3)) / S;
    qx = (R(1,2) + R(2,1)) / S;
    qy = 0.25 * S;
    qz = (R(2,3) + R(3,2)) / S;
else
    S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; 
    qw = (R(1,2) - R(2,1)) / S;
    qx = (R(1,3) + R(3,1)) / S;
    qy = (R(2,3) + R(3,2)) / S;
    qz = 0.25 * S;
end

q = qnorm([qx qy qz qw]');

end
