%% Class: True_SC_Onboard_Memory
% Tracks the onboard Memory state

classdef True_SC_Onboard_Memory < handle
    
    %% Properties
    properties
        
        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed 

        instantaneous_data_rate_generated % [kbps] : Data rate generated during current time step, in kilo bits (kb) per sec

        maximum_capacity % [kb] : Maximum data storage capacity of the Memeory

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Memory i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        instantaneous_capacity % [kb] : Instantaneous capacity of Memeory

        state_of_data_storage % [percentage] : SoDS is defined by = 100× instantaneous_capacity / maximum_capacity

        %% [ ] Properties: Storage Variables

        store


    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Onboard_Memory(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Memory ',num2str(i_HW)]; 
            end
            
            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]

            obj.maximum_capacity = init_data.maximum_capacity; % [kb]
            obj.instantaneous_capacity = 0; % [kb]
            obj.state_of_data_storage = 100*obj.instantaneous_capacity/obj.maximum_capacity; % [percentage]

            % Initialize Variables to store: instantaneous_capacity state_of_charge
            obj.store = [];

            obj.store.instantaneous_capacity = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_capacity));
            obj.store.state_of_data_storage = zeros(mission.storage.num_storage_steps, length(obj.state_of_data_storage));
            obj.store.maximum_capacity = obj.maximum_capacity; % [kb]

            obj = func_update_true_SC_onboard_memory_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_onboard_memory_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_capacity(mission.storage.k_storage,:) = obj.instantaneous_capacity; % [kb]
                obj.store.state_of_data_storage(mission.storage.k_storage,:) = obj.state_of_data_storage; % [percentage]
            end

        end

        %% [ ] Methods: Main
        % Update Memory SoDS

        function obj = func_main_true_SC_onboard_memory(obj, mission, i_SC)

            %             if obj.instantaneous_capacity <= 0
            %                 obj.instantaneous_capacity = 1e-3; % [kb]
            %             end

            obj.state_of_data_storage = 100*obj.instantaneous_capacity/obj.maximum_capacity; % [percentage]

            obj = func_update_true_SC_onboard_memory_store(obj, mission);

            % Update Power Consumed
            func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        
    end
end

