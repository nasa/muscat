%% Class: True_SC_Science_Processor
% Tracks the Science Processor

classdef True_SC_Science_Processor < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps)

        instantaneous_data_type = 0 % 0 = Data rate, 1 = Data per sample
        % used to avoid calling isprop() every time
        % step to determine if using instantaneous_data_rate_generated or
        % instantaneous_data_generated_per_sample, which is slow

        instantaneous_data_removed_per_sample % [kb] : Data in kilo bits (kb)

        mode_true_SC_science_processor_selector % [string] Select which Mode to run

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Generic Sensor i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        data % Other useful mission specific data

        mode_selector_name % [string] Alternative name of mode

        flag_mission_specific_storage_variables_exists % [Boolean] Do additional mission-specific storage variables exisit? 

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
            obj.mode_true_SC_science_processor_selector = init_data.mode_true_SC_science_processor_selector; % [string]

            obj.flag_executive = 0;

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            if isfield(init_data, 'mode_selector_name')
                obj.mode_selector_name = init_data.mode_selector_name;
            else
                obj.mode_selector_name = mission.name;
            end

            % Initialize Variables to store: measurement_vector
            obj.store = [];
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.instantaneous_data_rate_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_generated)); % [kbps]
            obj.store.instantaneous_data_removed_per_sample = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_removed_per_sample)); % [kb]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]


            % Additional Science Processor Variables
            switch obj.mode_true_SC_science_processor_selector

                case 'mission specific'
                    func_name = ['func_true_SC_sci_pro_constructor_', obj.mode_selector_name];

                    if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                        disp('Additional Mission-Specific Science Processor constructor function exists!');
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, mission, i_SC, i_HW);

                    else
                        disp('Additional Mission-Specific Science Processor constructor function does NOT exist.');
                        % Do nothing!
                    end

                otherwise
                    disp('Additional Science Processor constructor mode not defined!')

            end

            % Additional Mission-Specific Storage Variables
            func_name = ['func_update_true_SC_sci_pro_store_', obj.mode_selector_name];

            if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                disp('Additional Mission-Specific Science Processor storage function exists!');
                obj.flag_mission_specific_storage_variables_exists = true;                
            else
                disp('Additional Mission-Specific Science Processor storage function does NOT exist.');
                obj.flag_mission_specific_storage_variables_exists = false;
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
                    
                    % Additional Mission-Specific Storage Variables
                    if obj.flag_mission_specific_storage_variables_exists
                        func_name = ['func_update_true_SC_sci_pro_store_', obj.mode_selector_name];
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, mission);
                    end

                end
            end

        end

        %% [ ] Methods: Main
        % Update Science Processor

        function obj = func_main_true_SC_science_processor(obj, mission, i_SC, i_HW)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if isfield(obj.data, 'instantaneous_power_consumed_per_SC_mode')
                    obj.instantaneous_power_consumed = obj.data.instantaneous_power_consumed_per_SC_mode(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode_value); % [W]
                end

                switch obj.mode_true_SC_science_processor_selector

                    case 'mission specific'
                        func_name = ['func_update_true_SC_sci_pro_', obj.mode_selector_name];
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, mission, i_SC, i_HW);
                    
                    otherwise
                        error('Science Processor mode not defined!')

                end

            end

            % Updates of Power and Data should happen within the function

            % Update Storage
            obj = func_update_true_SC_science_processor_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

    end
end


