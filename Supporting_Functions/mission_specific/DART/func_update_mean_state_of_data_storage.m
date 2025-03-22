%% [ ] Methods: Update Mean SoDS
% Updates mean_state_of_data_storage

function obj = func_update_mean_state_of_data_storage(obj, mission, i_SC)

total_instantaneous_capacity = 0; % [kb]
total_maximum_capacity = 0; % [kb]

for i_memory = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory

    total_instantaneous_capacity = total_instantaneous_capacity + mission.true_SC{i_SC}.true_SC_onboard_memory{i_memory}.instantaneous_capacity; % [kb]
    total_maximum_capacity = total_maximum_capacity + mission.true_SC{i_SC}.true_SC_onboard_memory{i_memory}.maximum_capacity; % [kb]

end

obj.mean_state_of_data_storage = 100 * total_instantaneous_capacity / total_maximum_capacity; % [percentage]

obj.total_data_storage = total_instantaneous_capacity; % [kb]

end
