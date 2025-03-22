%% [ ] Methods: Estimate Orbit Truth
% Main Function

function obj = func_update_software_SC_estimate_orbit_Truth(obj, mission, i_SC)


% Update compute time
obj.compute_time = mission.true_SC{i_SC}.software_SC_executive.time;

obj.instantaneous_data_generated_per_sample = (1e-3)*8*36; % [kb] i.e. 36 Bytes per sample

obj.position = mission.true_SC{i_SC}.true_SC_navigation.position; % [km]
obj.position_uncertainty = zeros(1,3);

obj.velocity = mission.true_SC{i_SC}.true_SC_navigation.velocity; % [km/sec]
obj.velocity_uncertainty = zeros(1,3);

obj.position_relative_target = mission.true_SC{i_SC}.true_SC_navigation.position_relative_target; % [km]
obj.position_relative_target_uncertainty = zeros(1,3);

obj.velocity_relative_target = mission.true_SC{i_SC}.true_SC_navigation.velocity_relative_target; % [km/sec]
obj.velocity_relative_target_uncertainty = zeros(1,3);

obj.position_target = mission.true_target{obj.index_relative_target}.position; % [km]
obj.position_target_uncertainty = zeros(1,3);

obj.velocity_target = mission.true_target{obj.index_relative_target}.velocity; % [km/sec]
obj.velocity_target_uncertainty = zeros(1,3);

end
