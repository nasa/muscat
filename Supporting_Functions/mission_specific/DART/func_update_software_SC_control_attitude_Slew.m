%% [ ] Methods: Control Attitude Slew
% Slew directly moves the SC's attitude and rotation matrix, without the control actuators
function obj = func_update_software_SC_control_attitude_Slew(obj, mission, i_SC)

obj.desired_slew_angle_change = func_error_angle_between_quaternions(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude, obj.desired_attitude); % [rad]

slew_angle_threshold = obj.max_slew_rate * mission.true_time.time_step_attitude; % [rad]

if obj.desired_slew_angle_change <= slew_angle_threshold
    % No slew needed
    obj.flag_slew = 0;

    % Oracle Move!
    mission.true_SC{i_SC}.true_SC_adc.attitude = obj.desired_attitude; % [quaternion]
    mission.true_SC{i_SC}.true_SC_adc.angular_velocity = obj.desired_angular_velocity; % [rad/sec]

else
    % Slew is needed
    obj.flag_slew = 1;

    q_start = mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude; % [quaternion]
    q_end = obj.desired_attitude; % [quaternion]

    w_max = obj.max_slew_rate; % [rad/sec]
    dt = mission.true_time.time_step_attitude; % [sec] 

    [q_path, time_taken] = func_generate_slerp_path(q_start, q_end, w_max, dt);
    
    q1 = func_quaternion_properize(q_path(:,1)'); % [quaternion] starting 
    q2 = func_quaternion_properize(q_path(:,2)'); % [quaternion] next attitude in this time step
    mission.true_SC{i_SC}.true_SC_adc.attitude = q2; 

    % Compute Angular Velocity
    [w_body, w_inertial] = func_quat_to_angular_velocity(q1, q2, dt);
    mission.true_SC{i_SC}.true_SC_adc.angular_velocity = 0*w_body; % [rad/sec] Dont update angular velocity because it causes the attitude dynamics to push to SC further out. 


end

% Compute Rotation Matrix
func_update_true_SC_ADC_rotation_matrix(mission.true_SC{i_SC}.true_SC_adc);
end



%% --- Helper Functions ---

function [q_path, time_taken] = func_generate_slerp_path(q_start, q_end, w_max, dt)
    % Normalize inputs
    q_start = q_start / norm(q_start);
    q_end   = q_end / norm(q_end);
    
    % Ensure shortest path around the 4D unit hypersphere
    if dot(q_start, q_end) < 0
        q_end = -q_end;
    end
    
    % Compute total rotation angle (theta) from start to target
    q_err = func_quaternion_multiply(q_end, func_quaternion_conjugate(q_start));
    total_angle = 2 * acos(min(max(q_err(4), -1.0), 1.0)); % [rad] Clamp to avoid numerical acos errors
    
    % Calculate time required to stay within w_max constraint
    time_taken = total_angle / w_max;
    num_steps = max(2, ceil(time_taken / dt));
    
    % Interpolate path using SLERP
    q_path = zeros(4, num_steps);
    for k = 1:num_steps
        tau = (k - 1) / (num_steps - 1); % Normalized step parameter [0, 1]
        q_path(:, k) = func_slerp(q_start, q_end, tau);
    end
end


% Spherical Linear Interpolation (SLERP)
function q_out = func_slerp(q1, q2, t)
    cos_theta = dot(q1, q2);
    
    % Linear interpolation fallback for near-identical quaternions to avoid division by zero
    if cos_theta > 0.9995
        q_out = (1 - t) * q1 + t * q2;
        q_out = q_out / norm(q_out);
        return;
    end
    
    theta = acos(cos_theta);
    q_out = (sin((1 - t) * theta) * q1 + sin(t * theta) * q2) / sin(theta);
    q_out = q_out / norm(q_out);
end

% Quaternion change to Angular Velocity
function [w_body, w_inertial] = func_quat_to_angular_velocity(q1, q2, dt)
    % QUAT_TO_ANGULAR_VELOCITY Computes body and inertial angular velocities 
    % for a transition from quaternion q1 to q2 over time step dt.
    % 
    % Inputs:
    %   q1 - Initial quaternion [q1; q2; q3; q4] (scalar 4th)
    %   q2 - Target quaternion  [q1; q2; q3; q4] (scalar 4th)
    %   dt - Time step in seconds
    %
    % Outputs:
    %   w_body     - Angular velocity vector in the body frame [rad/s]
    %   w_inertial - Angular velocity vector in the inertial frame [rad/s]

    % Normalize quaternions
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);

    % Ensure shortest path around the 4D sphere
    if dot(q1, q2) < 0
        q2 = -q2;
    end

    %% Method 1: Error Quaternion / Axis-Angle (Most Robust for Slew Steps)
    % Compute relative rotation quaternion q_err = q1_conj * q2
    q_err = func_quaternion_multiply(func_quaternion_conjugate(q1), q2);
    
    % Extract rotation angle (delta_theta) and vector component
    v_err = q_err(1:3);
    s_err = min(max(q_err(4), -1.0), 1.0); % Clamp to prevent acos numerical errors
    
    delta_theta = 2 * acos(s_err);
    
    % Compute angular velocity vector
    if delta_theta < 1e-8
        % Small angle approximation to avoid division by zero
        w_body = (2 / dt) * v_err;
    else
        axis = v_err / norm(v_err);
        w_body = (delta_theta / dt) * axis;
    end

    %% Convert Body Angular Velocity to Inertial Frame
    % Transform w_body into inertial frame: w_inertial = q1 * [w_body; 0] * q1_conj
    q_w_body = [w_body, 0];
    q_w_inertial = func_quaternion_multiply(q1, func_quaternion_multiply(q_w_body, func_quaternion_conjugate(q1)));
    w_inertial = q_w_inertial(1:3);
end

