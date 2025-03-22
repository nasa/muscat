function error_angle = func_error_angle_between_quaternions(q1, q2)

% Ensure quaternions are normalized
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

% Compute the dot product between the quaternions (considering scalar part as 4th term)
dot_product = q1(4)*q2(4) + q1(1:3)*q2(1:3)';

% Ensure the dot product is within the valid range for acos
dot_product = max(min(dot_product, 1), -1);

% Compute the angle in radians
error_angle = 2 * acos(dot_product);

end