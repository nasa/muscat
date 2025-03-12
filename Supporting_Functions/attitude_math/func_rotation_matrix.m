function Rotation_matrix = func_rotation_matrix(theta)

% theta is in radians

Rotation_matrix = [cos(theta) -sin(theta) 0;
                   sin(theta)  cos(theta) 0;
                   0           0          1];