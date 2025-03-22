%% [ ] Methods: Get Attitude Error

function error = func_get_attitude_error(obj, mission, i_SC)
% This method calculates attitude error more efficiently
% by working directly in quaternion space

% Get quaternions and ensure proper format/normalization
q_actual = mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude;
q_desired = obj.desired_attitude;

% Ensure column vectors and normalize once
if size(q_actual,1) == 1
    q_actual = q_actual';
end
if size(q_desired,1) == 1
    q_desired = q_desired';
end

q_actual = q_actual / norm(q_actual);
q_desired = q_desired / norm(q_desired);

% Compute quaternion error: q_error = q_desired^(-1) * q_actual
% For quaternion conjugate, negate the vector part (first 3 elements)
q_desired_conj = q_desired;
q_desired_conj(1:3) = -q_desired_conj(1:3);

% Quaternion multiplication (using standard quaternion multiplication formula)
% This is q_error = q_desired_conj ⊗ q_actual
q_error = zeros(4,1);
q_error(1) = q_desired_conj(4)*q_actual(1) + q_desired_conj(1)*q_actual(4) + q_desired_conj(2)*q_actual(3) - q_desired_conj(3)*q_actual(2);
q_error(2) = q_desired_conj(4)*q_actual(2) + q_desired_conj(2)*q_actual(4) + q_desired_conj(3)*q_actual(1) - q_desired_conj(1)*q_actual(3);
q_error(3) = q_desired_conj(4)*q_actual(3) + q_desired_conj(3)*q_actual(4) + q_desired_conj(1)*q_actual(2) - q_desired_conj(2)*q_actual(1);
q_error(4) = q_desired_conj(4)*q_actual(4) - q_desired_conj(1)*q_actual(1) - q_desired_conj(2)*q_actual(2) - q_desired_conj(3)*q_actual(3);

% Extract error angle from quaternion (in radians)
% The scalar part of the quaternion (q_error(4)) is cos(θ/2)
% So θ = 2*acos(q_error(4)) is the rotation angle
error_angle = 2 * acos(min(1, max(-1, q_error(4))));

% Return error as a 3x1 vector (for compatibility with existing code)
% Scale by rotation axis to get component errors
if error_angle > 1e-10  % Avoid division by zero
    axis = q_error(1:3) / sin(error_angle/2);
    error = error_angle * axis;
else
    % For very small errors, just return the vector part (scaled)
    error = 2 * q_error(1:3);
end
end