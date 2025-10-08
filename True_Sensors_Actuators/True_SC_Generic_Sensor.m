%% Class: True_SC_Generic_Sensor
% Generic Sensor class

classdef True_SC_Generic_Sensor < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps)
                
        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Generic Sensor i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Generic_Sensor(init_data, mission, i_SC, i_HW)
            
            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Generic Sensor ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]

            obj.flag_executive = 0;
           
            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize Variables to store: measurement_vector
            obj.store = [];
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.instantaneous_data_rate_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_generated)); % [kbps]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]

            % Update Storage
            obj = func_update_true_SC_generic_sensor_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end
        
        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_generic_sensor_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [Boolean]

                if obj.flag_executive == 1
                    obj.store.instantaneous_data_rate_generated(mission.storage.k_storage,:) = obj.instantaneous_data_rate_generated; % [kbps]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]
                end
            end

        end

        %% [ ] Methods: Main
        % Update Generic Sensor

        function obj = func_main_true_SC_generic_sensor(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if isfield(obj.data, 'instantaneous_power_consumed_per_SC_mode')
                    obj.instantaneous_power_consumed = obj.data.instantaneous_power_consumed_per_SC_mode(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode_value); % [W]

                    % Update power value for Slew only
                    if (mission.true_SC{i_SC}.software_SC_executive.flag_slew_sc_mode_exists == 1) && (mission.true_SC{i_SC}.software_SC_control_attitude.flag_slew == 1)
                        obj.instantaneous_power_consumed = obj.data.instantaneous_power_consumed_per_SC_mode( func_find_this_sc_mode_value(mission.true_SC{i_SC}.software_SC_executive, 'Slew') ); % [W]
                    end
                end
         
                % Update Power Consumed
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            else
                % Do nothing                
            end

            % Update Storage
            obj = func_update_true_SC_generic_sensor_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

    end
end

