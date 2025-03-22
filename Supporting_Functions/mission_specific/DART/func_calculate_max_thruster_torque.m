%% [ ] Methods: Calculate maximum micro thruster torque capability

function max_torque = func_calculate_max_thruster_torque(obj, mission, i_SC)
% Calculate the maximum torque that can be provided by micro thrusters

% Use the thruster contribution matrix and maximum thrust values
% to calculate maximum possible torque

% Initialize maximum torque
max_torque = 0;

% Calculate maximum torque for each principal axis
for axis = 1:3
    % Calculate maximum positive torque
    pos_torque = 0;
    neg_torque = 0;

    for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
        thruster = mission.true_SC{i_SC}.true_SC_micro_thruster{i};

        % Skip unhealthy thrusters
        if ~thruster.health
            continue;
        end

        % Get torque contribution of this thruster for this axis
        torque_contrib = obj.thruster_contribution_matix(axis, i);

        % Add to positive or negative torque as appropriate
        if torque_contrib > 0
            pos_torque = pos_torque + torque_contrib * thruster.maximum_thrust;
        elseif torque_contrib < 0
            neg_torque = neg_torque + abs(torque_contrib) * thruster.maximum_thrust;
        end
    end

    % The maximum torque for this axis is the minimum of positive and negative capability
    % (since we need to be able to control in both directions)
    axis_max_torque = min(pos_torque, neg_torque);

    % The overall maximum torque is the maximum across all axes
    max_torque = max(max_torque, axis_max_torque);
end

end
