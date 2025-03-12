%% Class: True_SC_Radio_Antenna
% Tracks the Radio Antennas

classdef True_SC_Radio_Antenna < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        antenna_type % [string]
        % - 'Dipole'
        % - 'High Gain'

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        mode_true_SC_radio_antenna_selector % [string]
        % - TX
        % - RX

        TX_power_consumed % [Watts] : Power consumed during TX

        RX_power_consumed % [Watts] : Power consumed during RX

        base_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps) due to Health Keeping

        % Optional (only for Link Margin Calculations)

        antenna_gain % [dB] gain of Earth receiver

        noise_temperature % [K] temperature noise

        beamwidth % [MHz] receiver beamwwidth

        energy_bit_required % [dB] Minimum energy bit required

        line_loss % [dB] Loss due to pointing or others

        coding_gain % [dB] Coding gain


        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Radio Antenna i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps) due to RX

        instantaneous_data_rate_removed % [kbps] : Data rate, in kilo bits per sec (kbps) due to TX

        data % Other useful data

        maximum_data_rate

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Radio_Antenna(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Radio Antenna ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]
            obj.flag_executive = 0;
            % 
            obj.antenna_type = init_data.antenna_type;
            % 
            % obj.mode_true_SC_radio_antenna_selector = init_data.mode_true_SC_radio_antenna_selector;
            % 
            obj.TX_power_consumed = init_data.TX_power_consumed; % [Watts]
            obj.RX_power_consumed = init_data.RX_power_consumed; % [Watts]
            obj.instantaneous_power_consumed = obj.TX_power_consumed; % [Watts] Will be modified dynamically but a value is needed to register

            obj.location = init_data.location; % [m]
            obj.orientation = init_data.orientation; % [unit vector]

            % switch obj.mode_true_SC_radio_antenna_selector
            %     case 'TX'
            %         obj.instantaneous_power_consumed = obj.TX_power_consumed; % [Watts]
            %     case 'RX'
            %         obj.instantaneous_power_consumed = obj.RX_power_consumed; % [Watts]
            %     otherwise
            %         error('Should not reach here!')
            % end

            if isfield(init_data, 'antenna_gain')

                obj.antenna_gain = init_data.antenna_gain; % [dB]
                obj.noise_temperature = init_data.noise_temperature; % [K]
                obj.beamwidth = init_data.beamwidth; % [MHz]
                obj.energy_bit_required = init_data.energy_bit_required; % [dB]
                obj.coding_gain = init_data.coding_gain; % [dB]

            end
            % 
            obj.base_data_rate_generated = init_data.base_data_rate_generated; % [kbps]
            obj.instantaneous_data_rate_generated = obj.base_data_rate_generated; % [kbps]
            obj.instantaneous_data_rate_removed = 0; % [kbps]
            obj.maximum_data_rate = obj.maximum_data_rate; % [kbps]

            % if isfield(init_data, 'data')
            %     obj.data = init_data.data;
            % else
            %     obj.data = [];
            % end
            % 
            % Initialize Variables to store
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive)); % [integer]
            obj.store.mode_TX_RX = zeros(mission.storage.num_storage_steps, 1); % [integer]
            obj.store.instantaneous_data_rate_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_generated)); % [kbps]
            obj.store.instantaneous_data_rate_removed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_removed)); % [kbps]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]

            % Update Storage
            obj = func_update_true_SC_radio_antenna_store(obj, mission);
            % 
            % % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            % 
            % % Update SC Data Handling Class (Generated and Removed)
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
            func_initialize_list_HW_data_removed(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_radio_antenna_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [integer]

                if obj.flag_executive == 1
                    obj.store.instantaneous_data_rate_generated(mission.storage.k_storage,:) = obj.instantaneous_data_rate_generated; % [kbps]
                    obj.store.instantaneous_data_rate_removed(mission.storage.k_storage,:) = obj.instantaneous_data_rate_removed; % [kbps]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]

                    switch obj.mode_true_SC_radio_antenna_selector
                        case 'TX'
                            obj.store.mode_TX_RX(mission.storage.k_storage,1) = 1;
                        case 'RX'
                            obj.store.mode_TX_RX(mission.storage.k_storage,1) = 2;
                        otherwise
                            error('[True_SC_Radio_Antenna] Should not reach here!')
                    end

                end

            end

        end

        %% [ ] Methods: Main
        % Update all variables

        function obj = func_main_true_SC_radio_antenna(obj, mission, i_SC)
                   
            if (obj.flag_executive == 1) && (obj.health == 1)

                switch obj.mode_true_SC_radio_antenna_selector
                    case 'TX'
                        obj.instantaneous_power_consumed = obj.TX_power_consumed; % [Watts]
                    case 'RX'
                        obj.instantaneous_power_consumed = obj.RX_power_consumed; % [Watts]
                    otherwise
                        error('[True_SC_Radio_Antenna] Should not reach here!')
                end
               
                % Update SC Power Class
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power,obj, mission);
                
                func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

                % Update SC Data Handling Class (Generated and Removed)
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

                func_update_instantaneous_data_removed(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end          

            % Update Storage
            obj = func_update_true_SC_radio_antenna_store(obj, mission);

            % Reset All Variables
            obj.flag_executive = 0;
            obj.instantaneous_data_rate_generated = obj.base_data_rate_generated; % [kbps]
            obj.instantaneous_data_rate_removed = 0; % [kbps]

        end

    end
end

