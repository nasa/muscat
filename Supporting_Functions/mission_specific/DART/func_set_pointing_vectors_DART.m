%% [ ] Methods: DART Set Pointing Vectors

function obj = func_set_pointing_vectors_DART(obj, mission, i_SC)

% Navigation and Guidance
switch mission.true_SC{i_SC}.software_SC_executive.this_sc_mode

    case 'Point camera to Target'
        % Point camera to target
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_camera{1}.orientation); % In body frame
        obj.data.desired_primary_vector = func_normalize_vec(mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame
        % Optimize for solar pannels orientation to sun
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

    case 'DTE Comm'
        % Point antenna to earth
        % DART has one antenna
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation); % In body frame
        obj.data.desired_primary_vector = func_normalize_vec( ...
            mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - ...
            mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

        % Optimize for solar pannels orientation to sun
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame


    case 'Point Thruster along DeltaV direction'
        % Point thruster in direction of desired deltaV
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_chemical_thruster.orientation); % In body frame

        % Use the deltaV vector direction instead of target direction
        deltaV = mission.true_SC{i_SC}.software_SC_control_orbit.desired_control_DeltaV;
        obj.data.desired_primary_vector = func_normalize_vec(deltaV)'; % In Inertial frame

        % Secondary vector remains optimized for solar panels
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame


    case 'Maximize SP Power'
        % Primary vector is the solar pannels to sun
        obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
        obj.data.desired_primary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

        % [Optional] Optimize for DTE to earth
        obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation); % In body frame
        obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

    otherwise
        error('Attitude Control mode for this SC mode not defined!')
end


end
