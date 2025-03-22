%% [ ] Methods: Reset Reaction Wheels

function obj = func_reset_wheels(obj, mission, i_SC)
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.flag_executive = 0;
    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = 0;
end
end
