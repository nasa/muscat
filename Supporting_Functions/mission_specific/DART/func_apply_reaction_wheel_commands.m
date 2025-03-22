%% [ ] Methods: Safely Apply Reaction Wheels Command

function obj = func_apply_reaction_wheel_commands(obj, mission, i_SC, wheel_accelerations)
% Safely apply calculated commands to the reaction wheels

for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    if mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.health && abs(wheel_accelerations(i)) > 0
        % Additional safety check before applying command
        if abs(wheel_accelerations(i)) > mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration
            % Limit command to maximum acceleration while preserving direction
            wheel_accelerations(i) = sign(wheel_accelerations(i)) * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration;
        end

        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.flag_executive = 1;
        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = wheel_accelerations(i);
    else
        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.flag_executive = 0;
        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = 0;
    end
end
end
