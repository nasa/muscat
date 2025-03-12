%% Class: True_SC_Science_Processor
% Tracks the Science Processor

classdef True_SC_Science_Processor < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps)

        instantaneous_data_removed_per_sample % [kb] : Data in kilo bits (kb)

        flag_show_science_processor_plot % [Boolean] : 1 = Shows the Science Processor plot

        mode_true_SC_science_processor_selector % [string] Select which Mode to run

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

        function obj = True_SC_Science_Processor(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Science Processor ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]
            obj.instantaneous_data_removed_per_sample = init_data.instantaneous_data_removed_per_sample; % [kb]
            obj.flag_show_science_processor_plot = init_data.flag_show_science_processor_plot; % [Boolean]
            obj.mode_true_SC_science_processor_selector = init_data.mode_true_SC_science_processor_selector; % [string]

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
            obj.store.instantaneous_data_removed_per_sample = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_removed_per_sample)); % [kb]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]

            % Additional Science Processor Variables
            switch obj.mode_true_SC_science_processor_selector

                case 'Nightingale'
                    obj = func_true_SC_science_processor_Nightingale_constructor(obj, mission, i_SC);

                otherwise
                    % Do nothing!
            end

            % Update Storage
            obj = func_update_true_SC_science_processor_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
            func_initialize_list_HW_data_removed(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_science_processor_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage_attitude,:) = obj.flag_executive; % [Boolean]                

                if obj.flag_executive == 1
                    obj.store.instantaneous_data_rate_generated(mission.storage.k_storage,:) = obj.instantaneous_data_rate_generated; % [kbps]
                    obj.store.instantaneous_data_removed_per_sample(mission.storage.k_storage,:) = obj.instantaneous_data_removed_per_sample; % [kb]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]
                end
            end

        end

        %% [ ] Methods: Main
        % Update Science Processor

        function obj = func_main_true_SC_science_processor(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if isfield(obj.data, 'instantaneous_power_consumed_per_SC_mode')
                    obj.instantaneous_power_consumed = obj.data.instantaneous_power_consumed_per_SC_mode(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode_value); % [W]
                end

                switch obj.mode_true_SC_science_processor_selector

                    case 'Nightingale'
                        obj = func_true_SC_science_processor_Nightingale(obj, mission, i_SC);

                    otherwise
                        error('Should not reach here!')
                end                

                % Update Power Consumed
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
                
                % Update Data Handling
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
                func_update_instantaneous_data_removed(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
                
            else
                % If not active, set power to a low standby value
                obj.instantaneous_power_consumed = obj.instantaneous_power_consumed * 0.1; % 10% of normal power when in standby
                
                % Still update power system even when in standby
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            end

            % Update Storage
            obj = func_update_true_SC_science_processor_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

            
    end
end


