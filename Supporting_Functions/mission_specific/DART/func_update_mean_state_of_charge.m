%% [ ] Methods: Update Mean SoC
% Updates mean_state_of_charge in Software_SC_Power

function obj = func_update_mean_state_of_charge(obj, mission, i_SC)

total_instantaneous_capacity = 0; % [Watts * hr]
total_maximum_capacity = 0; % [Watts * hr]

for i_batt = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

    total_instantaneous_capacity = total_instantaneous_capacity + mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity; % [Watts * hr]
    total_maximum_capacity = total_maximum_capacity + mission.true_SC{i_SC}.true_SC_battery{i_batt}.maximum_capacity; % [Watts * hr]

end

obj.mean_state_of_charge = 100 * total_instantaneous_capacity / total_maximum_capacity; % [percentage]

end
