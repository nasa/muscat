%% [ ] Methods: Calculate maximum reaction wheel torque capability

function max_torque = func_calculate_max_reaction_wheel_torque(obj, mission, i_SC)
% Calculate the maximum torque that can be provided by the reaction wheel array

% Initialize maximum torque
max_torque = 0;

% Count active (healthy) wheels
active_wheels = 0;

% Check each reaction wheel
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    wheel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i};

    if wheel.health
        % For each healthy wheel, add its maximum torque capability
        wheel_max_torque = wheel.maximum_torque;

        % Calculate projection of wheel torque onto each axis
        wheel_torque_projection = wheel.orientation * wheel_max_torque;

        % Add to overall torque capability
        max_torque = max_torque + norm(wheel_torque_projection);

        active_wheels = active_wheels + 1;
    end
end

% Apply a configuration factor based on wheel geometry
% This is a conservative estimate of what the wheel array can actually provide
if active_wheels > 0
    % The factor accounts for the fact that not all wheels can contribute equally
    % to torque in all directions
    config_factor = 1/sqrt(active_wheels);
    max_torque = max_torque * config_factor;
end

return;
end
