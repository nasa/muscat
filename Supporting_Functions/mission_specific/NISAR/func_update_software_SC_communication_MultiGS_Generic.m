%% [ ] Methods: Communication for Multi-GS Generic
% Telecom Mode

function obj = func_update_software_SC_communication_MultiGS_Generic(obj, mission, i_SC)
    
    if strcmp(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode, 'Telecom') 

        Telecom_GS_index = mission.true_SC{i_SC}.software_SC_executive.data.Telecom_GS_index;
    
        % Switch SC communication from Rx to Tx, and the contrary for GS
        mission.true_SC{i_SC}.true_SC_radio_antenna{1}.mode_true_SC_radio_antenna_selector = "TX"; % Switching SC Antenna to TX
        mission.true_GS_radio_antenna{Telecom_GS_index}.mode_true_GS_radio_antenna_selector = "RX"; % Switching GS Antenna to RX
            
        % Switch on Communication Link
        mission.true_SC{i_SC}.true_SC_communication_link{2*Telecom_GS_index-1}.flag_executive = 1;

    else

        error('SC Executive mode incorrect... Should not reach here!')
    
    end

end