%% [ ] Methods: Communication for NISAR
% Telecom Mode

function obj = func_update_software_SC_communication_NISAR(obj, mission, i_SC)
    
    if strcmp(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode, 'Telecom')

        Telecom_GS_index = mission.true_SC{i_SC}.software_SC_executive.data.Telecom_GS_index;
    
        % Switch SC communication from Rx to Tx, and the contrary for GS
        mission.true_SC{i_SC}.true_SC_radio_antenna{1}.mode_true_SC_radio_antenna_selector = "TX"; % Switching SC Antenna to TX
        mission.true_GS_radio_antenna{Telecom_GS_index}.mode_true_GS_radio_antenna_selector = "RX"; % Switching GS Antenna to RX
    
        %         % Check Attitude
        %         this_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation')'; % [unit vector]
        %         vec_Earth_from_SC_normalized = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % [unit vector]
        %         obj.this_attitude_error = func_angle_between_vectors(this_orientation', vec_Earth_from_SC_normalized'); % [rad]
        %
        %         if obj.this_attitude_error <= obj.attitude_error_threshold
        %
        %             % Switch on Communication Link
        %             mission.true_SC{i_SC}.true_SC_communication_link{1}.flag_executive = 1;
        %
        %         end

        % Switch on Communication Link
        mission.true_SC{i_SC}.true_SC_communication_link{2*Telecom_GS_index-1}.flag_executive = 1;

    else

        error('Should not reach here!')
    
    end

end