%% [ ] Methods: Estimate Orbit Truth With Error Growth
% Realistic model with error growth when camera doesn't have target in view

function obj = func_update_software_SC_estimate_orbit_TruthWithErrorGrowth(obj, mission, i_SC)
% Update compute time
obj.compute_time = mission.true_SC{i_SC}.software_SC_executive.time;

obj.instantaneous_data_generated_per_sample = (1e-3)*8*36; % [kb] i.e. 36 Bytes per sample

% Get the basic true values first
obj.position = mission.true_SC{i_SC}.true_SC_navigation.position; % [km]
obj.velocity = mission.true_SC{i_SC}.true_SC_navigation.velocity; % [km/sec]
obj.position_relative_target = mission.true_SC{i_SC}.true_SC_navigation.position_relative_target; % [km]
obj.velocity_relative_target = mission.true_SC{i_SC}.true_SC_navigation.velocity_relative_target; % [km/sec]
obj.position_target = mission.true_target{obj.index_relative_target}.position; % [km]
obj.velocity_target = mission.true_target{obj.index_relative_target}.velocity; % [km/sec]

% Check if any camera has the target in view
target_is_visible = false;
for i_HW = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
    if mission.true_SC{i_SC}.true_SC_camera{i_HW}.flag_target_visible == 1
        target_is_visible = true;
        break;
    end
end

% Update the uncertainty based on target visibility
if target_is_visible
    % Reset uncertainties when target becomes visible
    obj.data.last_target_visible_time = obj.compute_time;

    % Set low base uncertainty when target is visible
    obj.position_uncertainty = [0.01, 0.01, 0.01]; % [km]
    obj.velocity_uncertainty = [0.001, 0.001, 0.001]; % [km/sec]
    obj.position_relative_target_uncertainty = [0.01, 0.01, 0.01]; % [km]
    obj.velocity_relative_target_uncertainty = [0.001, 0.001, 0.001]; % [km/sec]
    obj.position_target_uncertainty = [0.01, 0.01, 0.01]; % [km]
    obj.velocity_target_uncertainty = [0.001, 0.001, 0.001]; % [km/sec]
else
    % Calculate time since target was last visible
    time_since_target_visible = obj.compute_time - obj.data.last_target_visible_time; % [sec]

    % Ensure time is positive (handles initial case)
    if time_since_target_visible < 0
        time_since_target_visible = 3600; % Default to 1 hour if no prior visibility
    end

    % Calculate position uncertainty growth based on time since last visible
    base_position_uncertainty = 0.01; % [km] Base uncertainty when target is visible
    position_uncertainty_growth = obj.data.position_error_growth_rate * time_since_target_visible; % [km]

    % Calculate velocity uncertainty growth
    base_velocity_uncertainty = 0.001; % [km/sec] Base uncertainty when target is visible
    velocity_uncertainty_growth = obj.data.velocity_error_growth_rate * time_since_target_visible; % [km/sec]

    % Apply uncertainty to all position and velocity components
    obj.position_uncertainty = [1, 1, 1] * (base_position_uncertainty + position_uncertainty_growth);
    obj.velocity_uncertainty = [1, 1, 1] * (base_velocity_uncertainty + velocity_uncertainty_growth);
    obj.position_relative_target_uncertainty = [1, 1, 1] * (base_position_uncertainty + position_uncertainty_growth);
    obj.velocity_relative_target_uncertainty = [1, 1, 1] * (base_velocity_uncertainty + velocity_uncertainty_growth);
    obj.position_target_uncertainty = [1, 1, 1] * (base_position_uncertainty + position_uncertainty_growth * 0.5); % Target position known better
    obj.velocity_target_uncertainty = [1, 1, 1] * (base_velocity_uncertainty + velocity_uncertainty_growth * 0.5); % Target velocity known better
end
end