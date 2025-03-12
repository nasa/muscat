function [roll, pitch, yaw] = func_quat2euler(q)
    % Input q is Nx4 matrix, each row a quaternion (scalar last)
    
    % Normalize each quaternion
    q = q ./ vecnorm(q, 2, 2); % Normalize each row
    
    % Extract quaternion components
    qw = q(:, 4);
    qx = q(:, 1);
    qy = q(:, 2);
    qz = q(:, 3);
    
    % Roll (X-axis) - vectorized
    sinr_cosp = 2 * (qw .* qx + qy .* qz);
    cosr_cosp = 1 - 2 * (qx.^2 + qy.^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (Y-axis) - vectorized
    sinp = 2 * (qw .* qy - qz .* qx);
    
    % Handle the case where sinp >= 1 (set to pi/2)
    pitch = asin(sinp);
    pitch(sinp >= 1) = pi/2;
    pitch(sinp <= -1) = -pi/2;
    
    % Yaw (Z-axis) - vectorized
    siny_cosp = 2 * (qw .* qz + qx .* qy);
    cosy_cosp = 1 - 2 * (qy.^2 + qz.^2);
    yaw = atan2(siny_cosp, cosy_cosp);
end