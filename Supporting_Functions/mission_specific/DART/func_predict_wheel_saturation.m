%% [ ] Methods: Predict wheel saturation

function [will_saturate_quickly, time_steps_to_saturation] = func_predict_wheel_saturation(obj, mission, i_SC)
% Simplified prediction of reaction wheel saturation

% Initialize output
will_saturate_quickly = false;
time_steps_to_saturation = Inf;

% Minimum acceptable time steps before saturation
min_acceptable_time_steps = 5;

% Skip calculation if desired torque is negligible
if norm(obj.desired_control_torque) < 1e-6
    return;
end

% Calculate required wheel accelerations using pseudo-inverse
wheel_accelerations = obj.pinv_reaction_wheel_contribution_matrix * obj.desired_control_torque';

% Check each wheel
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    wheel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i};

    % Skip unhealthy wheels
    if ~wheel.health
        continue;
    end

    % Skip if the commanded acceleration is negligible
    if abs(wheel_accelerations(i)) < 1e-6
        continue;
    end

    % Limit command to maximum acceleration
    cmd_accel = wheel_accelerations(i);
    if abs(cmd_accel) > wheel.maximum_acceleration
        cmd_accel = sign(cmd_accel) * wheel.maximum_acceleration;
    end

    % Current angular velocity
    current_velocity = wheel.angular_velocity;

    % Saturation threshold (80% of max, matching the wheel's internal check)
    saturation_threshold = wheel.max_angular_velocity * 0.8;

    % Two main checks:

    % 1. Direction reversal check - these are problematic
    if abs(current_velocity) > 0.3 * wheel.max_angular_velocity && ...
            sign(current_velocity) ~= sign(current_velocity + cmd_accel * mission.true_time.time_step_attitude)
        % Large direction reversal detected
        will_saturate_quickly = true;
        time_steps_to_saturation = 1;
        return;
    end

    % 2. Acceleration toward saturation
    if sign(cmd_accel) == sign(current_velocity)
        % Wheel accelerating in same direction (toward saturation)
        remaining_velocity = saturation_threshold - abs(current_velocity);

        if remaining_velocity <= 0
            % Already at or beyond saturation threshold
            will_saturate_quickly = true;
            time_steps_to_saturation = 0;
            return;
        end

        % Time steps until saturation at current acceleration
        steps = remaining_velocity / (abs(cmd_accel) * mission.true_time.time_step_attitude);

        if steps < min_acceptable_time_steps
            will_saturate_quickly = true;
            time_steps_to_saturation = steps;
            return;
        end

        % Keep track of minimum time to saturation across all wheels
        if steps < time_steps_to_saturation
            time_steps_to_saturation = steps;
        end
    end
end

% Check for significant disturbance torque (simplified version)
if isfield(mission.true_SC{i_SC}.true_SC_adc, 'disturbance_torque')
    disturbance_torque = mission.true_SC{i_SC}.true_SC_adc.disturbance_torque;
    desired_torque_mag = norm(obj.desired_control_torque);

    % If significant disturbance present relative to command
    if desired_torque_mag > 1e-6 && norm(disturbance_torque) > 0.5 * desired_torque_mag
        will_saturate_quickly = true;
        time_steps_to_saturation = min_acceptable_time_steps; % Conservative estimate
    end
end
end


