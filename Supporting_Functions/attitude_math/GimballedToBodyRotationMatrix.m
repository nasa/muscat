function R = GimballedToBodyRotationMatrix(gimbal)
    % Extract Inner and Outer Gimbal Angles
    gimbal_in = gimbal(1); % Inner gimbal angle
    gimbal_out = gimbal(2); % Outer gimbal angle
    
    % Rotation Matrix Mapping from Gimballed to Body Frame
    R = [cosd(gimbal_out) sind(gimbal_out) 0;
         -sind(gimbal_out) cosd(gimbal_out) 0;
         0 0 1]' * ...
        [0 sind(gimbal_in) -cosd(gimbal_in); 
         0 cosd(gimbal_in) sind(gimbal_in);
         1 0 0]';
end