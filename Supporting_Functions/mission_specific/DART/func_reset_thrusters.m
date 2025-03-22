%% [ ] Methods: Reset Micro Thrusters

function obj = func_reset_thrusters(obj, mission, i_SC)
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 0;
    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = 0;
end
end
