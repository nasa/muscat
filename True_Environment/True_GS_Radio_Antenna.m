%% Class: True_GS_Radio_Antenna
% Tracks the GS's Radio Antennas

classdef True_GS_Radio_Antenna < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        antenna_type % [string]
        % - 'Dipole'
        % - 'High Gain'

        mode_true_GS_radio_antenna_selector % [string]
        % - TX
        % - RX
        
        % Optional (only for Link Margin Calculations)

        antenna_gain % [dB] gain of Earth receiver

        noise_temperature % [K] temperature noise

        beamwidth % [MHz] receiver beamwwidth

        energy_bit_required % [dB] Minimum energy bit required

        line_loss % [dB] Loss due to pointing or others

        coding_gain % [dB] Coding gain


        %% [ ] Properties: Variables Computed Internally

        name % [string] 'GS Radio Antenna i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        instantaneous_data_rate_transmitted % [kbps] : Data rate, in kilo bits per sec (kbps) due to RX

        instantaneous_data_rate_received % [kbps] : Data rate, in kilo bits per sec (kbps) due to TX

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_GS_Radio_Antenna(init_data, mission, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['GS Radio Antenna ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.flag_executive = 0;

            obj.antenna_type = init_data.antenna_type;

            obj.mode_true_GS_radio_antenna_selector = init_data.mode_true_GS_radio_antenna_selector;
            
            if isfield(init_data, 'antenna_gain')

                obj.antenna_gain = init_data.antenna_gain; % [dB]
                obj.noise_temperature = init_data.noise_temperature; % [K]
                obj.beamwidth = init_data.beamwidth; % [MHz]
                obj.energy_bit_required = init_data.energy_bit_required; % [dB]
                obj.line_loss = init_data.line_loss; % [dB]
                obj.coding_gain = init_data.coding_gain; % [dB]

            end

            obj.instantaneous_data_rate_transmitted = 0; % [kbps]
            obj.instantaneous_data_rate_received = 0; % [kbps]

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize Variables to store: flag_executive mode_TX_RX
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive)); % [integer]
            obj.store.mode_TX_RX = zeros(mission.storage.num_storage_steps, 1); % [integer]
            obj.store.instantaneous_data_rate_transmitted = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_transmitted)); % [kbps]
            obj.store.instantaneous_data_rate_received = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_received)); % [kbps]

            % Update Storage
            obj = func_update_true_GS_radio_antenna_store(obj, mission);

            % Update Ground Station Class (Generated and Removed)
            func_initialize_list_HW_data_transmitted(mission.true_ground_station, obj, mission);

            func_initialize_list_HW_data_received(mission.true_ground_station, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_GS_radio_antenna_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [integer]
                obj.store.instantaneous_data_rate_transmitted(mission.storage.k_storage,:) = obj.instantaneous_data_rate_transmitted; % [kbps]
                obj.store.instantaneous_data_rate_received(mission.storage.k_storage,:) = obj.instantaneous_data_rate_received; % [kbps]

                switch obj.mode_true_GS_radio_antenna_selector
                    case 'TX'
                        obj.store.mode_TX_RX(mission.storage.k_storage,1) = 1;
                    case 'RX'
                        obj.store.mode_TX_RX(mission.storage.k_storage,1) = 2;
                    otherwise
                        error('Should not reach here!')
                end

            end

        end

        %% [ ] Methods: Main
        % Update Camera

        function obj = func_main_true_GS_radio_antenna(obj, mission)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % TX or RX Data

                % Update SC Data Handling Class (Generated and Removed)
                func_update_instantaneous_data_transmitted(mission.true_ground_station, obj, mission);

                func_update_instantaneous_data_received(mission.true_ground_station, obj, mission);

            else
                % Do nothing

            end

            % Update Storage
            obj = func_update_true_GS_radio_antenna_store(obj, mission);

            % Reset All Variables
            obj.flag_executive = 0;
            obj.instantaneous_data_rate_transmitted = 0; % [kbps]
            obj.instantaneous_data_rate_received = 0; % [kbps]

        end

    end
end

