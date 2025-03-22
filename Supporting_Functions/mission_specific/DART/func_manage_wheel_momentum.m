%% [ ] Methods: Manage Wheel Momentum to Avoid Direction Reversals

function wheel_accelerations = func_manage_wheel_momentum(obj, mission, i_SC, wheel_accelerations)
% Apply momentum management to avoid abrupt direction changes
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel

    % Skip unhealthy wheels
    if ~mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.health
        wheel_accelerations(i) = 0;
        continue;
    end

    % Get current wheel velocity
    current_velocity = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.angular_velocity;

    % Calculate absolute velocity as percentage of max
    velocity_percentage = abs(current_velocity) / mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity;

    % ADDED: Special handling for mode transitions
    if isfield(obj.data, 'mode_transition_detected') && obj.data.mode_transition_detected
        % During mode transitions, be extremely conservative with commands

        % Calculate expected velocity after one time step
        predicted_vel = current_velocity + wheel_accelerations(i) * mission.true_time.time_step_attitude;

        % Check for problematic direction reversal
        if sign(predicted_vel) ~= sign(current_velocity) && abs(current_velocity) > 0.3 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity
            % Direction would reverse and current speed is significant

            % Instead of allowing the direction change, apply gradual deceleration
            % Set deceleration to a fraction of what would be needed to stop in 5 time steps
            safe_decel = -sign(current_velocity) * min(abs(current_velocity) / (5 * mission.true_time.time_step_attitude), 0.05 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration);

            % Limit the change and log
            wheel_accelerations(i) = safe_decel;

            % Log this limiting with detailed diagnostics
            disp(['MODE TRANSITION SAFETY: Limiting wheel ', num2str(i), ' acceleration during mode change.']);
            disp(['Current velocity: ', num2str(current_velocity), ' rad/s (', num2str(velocity_percentage*100), '% of max)']);
            disp(['Original command: ', num2str(wheel_accelerations(i)), ' rad/s^2']);
            disp(['Safe deceleration: ', num2str(safe_decel), ' rad/s^2']);

            continue; % Skip the rest of the checks for this wheel
        end

        % Even when not reversing direction, limit acceleration during transitions
        max_safe_accel = 0.1 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration;
        if abs(wheel_accelerations(i)) > max_safe_accel
            wheel_accelerations(i) = sign(wheel_accelerations(i)) * max_safe_accel;
            disp(['Limiting wheel ', num2str(i), ' acceleration to ', num2str(max_safe_accel), ' rad/s^2 during mode transition']);
        end

        continue; % Skip further checks during mode transitions
    end

    % Check for direction reversal (wheel going at high speed in one direction
    % and acceleration trying to go in the opposite)
    if abs(current_velocity) > 0.5 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity && sign(current_velocity) ~= sign(wheel_accelerations(i))
        % Calculate expected velocity after one time step
        predicted_vel = current_velocity + wheel_accelerations(i) * mission.true_time.time_step_attitude;

        % If direction would reverse, limit the acceleration to begin deceleration
        % rather than attempting a full reversal
        if sign(predicted_vel) ~= sign(current_velocity)
            % Set acceleration to a safe deceleration value (~ 5% of max velocity per second)
            safe_decel = -sign(current_velocity) * min(abs(wheel_accelerations(i)), 0.05 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity / mission.true_time.time_step_attitude);

            % Limit the change
            wheel_accelerations(i) = safe_decel;

            % Log this limiting
            if abs(wheel_accelerations(i)) > 0.01
                disp(['Limiting wheel ', num2str(i), ' acceleration to prevent direction reversal. Current vel: ', ...
                    num2str(current_velocity), ', cmd accel: ', num2str(wheel_accelerations(i))]);
            end
        end
    end

    % ADDED: Additional safety for high-speed wheels
    if velocity_percentage > 0.7
        % For wheels already at high speeds, be more conservative with acceleration
        max_safe_accel = (1 - velocity_percentage) * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration;

        if abs(wheel_accelerations(i)) > max_safe_accel && sign(wheel_accelerations(i)) == sign(current_velocity)
            % Only limit acceleration in the same direction as current velocity
            wheel_accelerations(i) = sign(wheel_accelerations(i)) * max_safe_accel;
        end
    end
end
end
