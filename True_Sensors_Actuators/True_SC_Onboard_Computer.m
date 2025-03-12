%% Class: True_SC_Onboard_Computer
% Onboard Computer class for spacecraft

classdef True_SC_Onboard_Computer < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps)
        
        processor_utilization % [%] : CPU usage (0-100%)
                
        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Onboard Computer i'

        health % [integer] Health of computer
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of computer

        flag_executive % [Boolean] Executive has told this computer to do its job

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Onboard_Computer(init_data, mission, i_SC, i_HW)
            
            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Onboard Computer ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 15; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]
            
            if isfield(init_data, 'processor_utilization')
                obj.processor_utilization = init_data.processor_utilization; % [%]
            else
                obj.processor_utilization = 10; % [%] default utilization
            end

            obj.flag_executive = 0;
           
            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize Variables to store
            obj.store = [];
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.instantaneous_data_rate_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_generated)); % [kbps]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]
            obj.store.processor_utilization = zeros(mission.storage.num_storage_steps, 1); % [%]

            % Update Storage
            obj = func_update_true_SC_onboard_computer_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end
        
        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_onboard_computer_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage_attitude,:) = obj.flag_executive; % [Boolean]

                if obj.flag_executive == 1
                    obj.store.instantaneous_data_rate_generated(mission.storage.k_storage,:) = obj.instantaneous_data_rate_generated; % [kbps]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]
                    obj.store.processor_utilization(mission.storage.k_storage,:) = obj.processor_utilization; % [%]
                end
            end

        end

        %% [ ] Methods: Main
        % Update Onboard Computer

        function obj = func_main_true_SC_onboard_computer(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Operate computer - constant power and data rate regardless of processor utilization
                
                % Update power consumed for spacecraft power budget
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

                % Update data generated for spacecraft data handling
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            else
                % Computer is off or malfunctioning               
            end

            % Update Storage
            obj = func_update_true_SC_onboard_computer_store(obj, mission);

            % Do not reset flag_executive because computer is always on

        end

    end
end 