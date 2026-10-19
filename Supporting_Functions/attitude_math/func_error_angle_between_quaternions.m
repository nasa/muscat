function error_angle = func_error_angle_between_quaternions(q1, q2)

% Ensure quaternions are normalized
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

% Compute the dot product between the quaternions (considering scalar part as 4th term)
% Take absolute value to ensure shortest path between the quaternions
dot_product = abs(dot(q1,q2));

% Ensure the dot product is within the valid range for acos
dot_product = max(min(dot_product, 1), -1);

% Compute the angle in radians
error_angle = 2 * acos(dot_product); % [rad]

end