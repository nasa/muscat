%% [ ] Methods: Communication Link
% Each GS can talk to multiple SC at a time

function obj = func_communication_link_MultiSC_per_GS(obj, mission, i_SC)

% Check Link Visibility
if obj.TX_spacecraft == 0
    obj.flag_TX_RX_visible = mission.true_SC{obj.RX_spacecraft}.true_SC_navigation.flag_visible_Earth;

elseif obj.RX_spacecraft == 0
    obj.flag_TX_RX_visible = mission.true_SC{obj.TX_spacecraft}.true_SC_navigation.flag_visible_Earth;

else
    error('Need to write this code where both are SC!')

end

% Compute Data Rate (if visible)
if obj.flag_TX_RX_visible == 1
    % LOS to Earth avaiable
    if obj.flag_compute_data_rate == 1
        obj.this_data_rate = 0; % [kbps]
        error('Write code to compute Link Margin!')
    else
        obj.this_data_rate = obj.given_data_rate; % [kbps]
    end
else
    % In eclipse
    obj.this_data_rate = 0; % [kbps]
end

% Perform Data Transfer
if (obj.flag_executive == 1) % && (obj.flag_TX_RX_visible == 1)

    if (obj.TX_spacecraft > 0) && (obj.flag_TX_RX_visible == 1) && ((obj.this_data_rate * mission.true_time.time_step) >= mission.true_SC{obj.TX_spacecraft}.software_SC_data_handling.total_data_storage)
        obj.this_data_rate = mission.true_SC{obj.TX_spacecraft}.software_SC_data_handling.total_data_storage/mission.true_time.time_step; % [kbps]
    end

    % Start TX and transmit data
    if obj.TX_spacecraft == 0
        % This is GS
        if mission.true_GS_radio_antenna{obj.TX_spacecraft_Radio_HW}.flag_executive ~= 0
            warning('[GS] TX link is already on!')
        else
            mission.true_GS_radio_antenna{obj.TX_spacecraft_Radio_HW}.flag_executive = 1;
            mission.true_GS_radio_antenna{obj.TX_spacecraft_Radio_HW}.mode_true_GS_radio_antenna_selector = 'TX';
            mission.true_GS_radio_antenna{obj.TX_spacecraft_Radio_HW}.instantaneous_data_rate_transmitted = obj.this_data_rate; % [kbps]
        end

    else
        % This is SC
        tx_sc = mission.true_SC{obj.TX_spacecraft};
        if iscell(tx_sc.true_SC_radio_antenna)
            tx_antenna = tx_sc.true_SC_radio_antenna{obj.TX_spacecraft_Radio_HW};
        else
            if obj.TX_spacecraft_Radio_HW ~= 1
                error('TX_spacecraft_Radio_HW index is %d, but true_SC_radio_antenna is not a cell array.', obj.TX_spacecraft_Radio_HW);
            end
            tx_antenna = tx_sc.true_SC_radio_antenna;
        end

        if tx_antenna.flag_executive ~= 0
            error('[SC] TX link is already on!')
        else
            tx_antenna.flag_executive = 1;
            tx_antenna.mode_true_SC_radio_antenna_selector = 'TX';
            tx_antenna.instantaneous_data_rate_removed = obj.this_data_rate; % [kbps]
        end

    end

    % Start RX and receive data
    if obj.RX_spacecraft == 0
        % This is GS
        if mission.true_GS_radio_antenna{obj.RX_spacecraft_Radio_HW}.flag_executive ~= 0
            % warning('[GS] RX link is already on!') % Avoid too many warnings! 
        else
            mission.true_GS_radio_antenna{obj.RX_spacecraft_Radio_HW}.flag_executive = 1;
            mission.true_GS_radio_antenna{obj.RX_spacecraft_Radio_HW}.mode_true_GS_radio_antenna_selector = 'RX';
            mission.true_GS_radio_antenna{obj.RX_spacecraft_Radio_HW}.instantaneous_data_rate_received = obj.this_data_rate; % [kbps]
        end

    else
        % This is SC
        rx_sc = mission.true_SC{obj.RX_spacecraft};
        if iscell(rx_sc.true_SC_radio_antenna)
            rx_antenna = rx_sc.true_SC_radio_antenna{obj.RX_spacecraft_Radio_HW};
        else
            if obj.RX_spacecraft_Radio_HW ~= 1
                error('RX_spacecraft_Radio_HW index is %d, but true_SC_radio_antenna is not a cell array.', obj.RX_spacecraft_Radio_HW);
            end
            rx_antenna = rx_sc.true_SC_radio_antenna;
        end

        if rx_antenna.flag_executive ~= 0
            error('[SC] RX link is already on!')
        else
            rx_antenna.flag_executive = 1;
            rx_antenna.mode_true_SC_radio_antenna_selector = 'RX';
            rx_antenna.instantaneous_data_rate_generated = obj.this_data_rate; % [kbps]
        end
    end

    % Update power consumption for this link's control overhead
    if i_SC > 0
        func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
    end
end

end