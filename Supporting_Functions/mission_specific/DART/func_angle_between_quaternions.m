%% Helper function: Calculate angle between quaternions

function angle = func_angle_between_quaternions(obj, q1, q2)
% Calculates the angle between two quaternions in radians
% This is the minimum rotation angle to get from one orientation to the other

% Ensure quaternions are normalized
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

% Calculate the quaternion product q1^-1 * q2
% For a quaternion, the inverse is the conjugate if it's normalized
q1_conj = q1;
q1_conj(1:3) = -q1_conj(1:3); % Conjugate: negate vector part

% Quaternion multiplication
q_diff = zeros(4,1);
if length(q1) == 4 && length(q2) == 4
    q_diff(1) = q1_conj(4)*q2(1) + q1_conj(1)*q2(4) + q1_conj(2)*q2(3) - q1_conj(3)*q2(2);
    q_diff(2) = q1_conj(4)*q2(2) + q1_conj(2)*q2(4) + q1_conj(3)*q2(1) - q1_conj(1)*q2(3);
    q_diff(3) = q1_conj(4)*q2(3) + q1_conj(3)*q2(4) + q1_conj(1)*q2(2) - q1_conj(2)*q2(1);
    q_diff(4) = q1_conj(4)*q2(4) - q1_conj(1)*q2(1) - q1_conj(2)*q2(2) - q1_conj(3)*q2(3);

    % The rotation angle is 2*acos(q_diff(4))
    % Clamp scalar part to [-1,1] to avoid numerical errors
    angle = 2 * acos(min(1, max(-1, q_diff(4))));
else
    % Handle invalid quaternions by returning a large angle
    angle = pi;
    warning('Invalid quaternion dimensions in func_angle_between_quaternions');
end
end
