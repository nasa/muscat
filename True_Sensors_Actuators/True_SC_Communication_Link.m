%% Class: True_SC_Communication_Link
% Tracks the Links between Radio Antennas

classdef True_SC_Communication_Link < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        TX_spacecraft % [integer] Use O for Ground Station
        TX_spacecraft_Radio_HW % [integer]

        RX_spacecraft % [integer] Use O for Ground Station
        RX_spacecraft_Radio_HW % [integer]

        % TODO - Decide if this is should stay here or go to software comm
        wait_time_comm_dte % [sec] : Wait time without transmiting to earth. This is a constant.
        last_communication_time % [sec] : Last time that data has been sent. This is updated in SC_Executive

        flag_compute_data_rate % [Boolean]

        given_data_rate % [kbps]

        instantaneous_power_consumed % [Watts] : Communication Link control overhead power consumption

        mode_true_SC_communication_link_selector % [string]

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Comm Link i'

        TX_name % [string]

        RX_name % [string]

        this_data_rate % [kbps] Actually used in simulation

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        flag_TX_RX_visible % [Boolean]

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Communication_Link(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Comm Link ',num2str(i_HW)];
            end

            obj.flag_executive = 0;
            obj.flag_TX_RX_visible = 0;

            obj.TX_spacecraft = init_data.TX_spacecraft;
            obj.TX_spacecraft_Radio_HW = init_data.TX_spacecraft_Radio_HW;

            obj.RX_spacecraft = init_data.RX_spacecraft;
            obj.RX_spacecraft_Radio_HW = init_data.RX_spacecraft_Radio_HW;

            obj.flag_compute_data_rate = init_data.flag_compute_data_rate;
            obj.given_data_rate = init_data.given_data_rate; % [kbps]

            obj.mode_true_SC_communication_link_selector = init_data.mode_true_SC_communication_link_selector;

            % Initialize the power consumption property
            if isfield(init_data, 'instantaneous_power_consumed')
                obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed;
            else
                obj.instantaneous_power_consumed = 0.5; % [W] Default low power for link control overhead
            end

            if obj.flag_compute_data_rate == 1
                obj.this_data_rate = 0; % [kbps]
            else
                obj.this_data_rate = obj.given_data_rate; % [kbps]
            end

            % Handle TX name
            if obj.TX_spacecraft == 0
                obj.TX_name = mission.true_GS_radio_antenna{obj.TX_spacecraft_Radio_HW}.name;
            else
                tx_sc = mission.true_SC{obj.TX_spacecraft};
                if iscell(tx_sc.true_SC_radio_antenna)
                    tx_antenna = tx_sc.true_SC_radio_antenna{obj.TX_spacecraft_Radio_HW};
                else
                    if obj.TX_spacecraft_Radio_HW ~= 1
                        error('TX_spacecraft_Radio_HW index is %d, but true_SC_radio_antenna is not a cell array.', obj.TX_spacecraft_Radio_HW);
                    end
                    tx_antenna = tx_sc.true_SC_radio_antenna;
                end
                obj.TX_name = ['SC ', num2str(obj.TX_spacecraft), ' ', tx_antenna.name];
            end

            % Handle RX name
            if obj.RX_spacecraft == 0
                obj.RX_name = mission.true_GS_radio_antenna{obj.RX_spacecraft_Radio_HW}.name;
            else
                rx_sc = mission.true_SC{obj.RX_spacecraft};
                if iscell(rx_sc.true_SC_radio_antenna)
                    rx_antenna = rx_sc.true_SC_radio_antenna{obj.RX_spacecraft_Radio_HW};
                else
                    if obj.RX_spacecraft_Radio_HW ~= 1
                        error('RX_spacecraft_Radio_HW index is %d, but true_SC_radio_antenna is not a cell array.', obj.RX_spacecraft_Radio_HW);
                    end
                    rx_antenna = rx_sc.true_SC_radio_antenna;
                end
                obj.RX_name = ['SC ', num2str(obj.RX_spacecraft), ' ', rx_antenna.name];
            end

            % Initialize Variables to store
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive)); % [integer]
            obj.store.flag_TX_RX_visible = zeros(mission.storage.num_storage_steps, length(obj.flag_TX_RX_visible)); % [integer]
            obj.store.this_data_rate = zeros(mission.storage.num_storage_steps, length(obj.this_data_rate)); % [kbps]

            % Update Storage
            obj = func_update_true_SC_communication_link_store(obj, mission);

            % Register with power system if this is a spacecraft component (not a ground station)
            if i_SC > 0
                func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            end
        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_communication_link_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1

                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [integer]
                obj.store.flag_TX_RX_visible(mission.storage.k_storage,:) = obj.flag_TX_RX_visible; % [integer]
                obj.store.this_data_rate(mission.storage.k_storage,:) = obj.this_data_rate; % [kbps]

            end

        end

        %% [ ] Methods: Main
        % Update all variables

        function obj = func_main_true_SC_communication_link(obj, mission, i_SC)

            switch obj.mode_true_SC_communication_link_selector

                case 'Generic'
                    % Each GS can talk to only 1 SC at a time
                    obj = func_communication_link_generic(obj, mission, i_SC);

                case 'Multi-SC per GS'
                    % Each GS can talk to multiple SC at a time
                    obj = func_communication_link_MultiSC_per_GS(obj, mission, i_SC);

                otherwise
                    error('Comm Link not defined!')
            end


            % Update Storage
            obj = func_update_true_SC_communication_link_store(obj, mission);

            % Reset All Variables
            obj.flag_executive = 0;

        end

        %% [ ] Methods: Generic Communication Link

        function obj = func_communication_link_generic(obj, mission, i_SC)

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
                        error('[GS] TX link is already on!')
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
                        error('[GS] RX link is already on!')
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

    end
end