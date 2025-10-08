%% [ ] Methods: NISAR Set Pointing Vectors

function obj = func_set_pointing_vectors_NISAR(obj, mission, i_SC)

% Navigation and Guidance
switch mission.true_SC{i_SC}.software_SC_executive.this_sc_mode

    case 'Science Mode'
        % Point L-band to target
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_remote_sensing{1}.orientation); % In body frame
        obj.data.desired_primary_vector = func_normalize_vec(mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

        % (Optional) Optimize for solar pannels orientation to Sun
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

    case 'Telecom'
        % Point Radio Antenna to target
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation); % In body frame
        obj.data.desired_primary_vector = func_normalize_vec(mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

        % (Optional) Optimize for solar pannels orientation to Sun
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

    case 'Maximize SP Power'
        % Primary vector is the solar pannels to sun
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_primary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

        % (Optional) Point L-band to target
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_remote_sensing{1}.orientation); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

    otherwise
        error('Attitude Control mode for this SC mode not defined!')
end


end
