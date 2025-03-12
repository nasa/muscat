%% Class: Software_SC_Data_Handling
% Track the data_handling status onboard the spacecraft

classdef Software_SC_Data_Handling < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_software_SC_data_handling_selector % [string] Different Data Handling modes
        % - 'Generic'

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Data Handling

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        total_data_storage % [kb] : Total data in all memories

        mean_state_of_data_storage % [percentage] : SoDS is defined by = 100× Sum (instantaneous_capacity) / Sum (maximum_capacity)

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Data_Handling(init_data, mission, i_SC)

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Data Handling']; % [string]
            obj.flag_executive = 0;

            obj.instantaneous_data_generated_per_sample = init_data.instantaneous_data_generated_per_sample; % [kb]

            obj.mode_software_SC_data_handling_selector = init_data.mode_software_SC_data_handling_selector;

            obj.mode_software_SC_data_handling_selector = 'Generic';

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Update Mean SoC
            obj = func_update_mean_state_of_data_storage(obj, mission, i_SC);

            obj.mean_state_of_data_storage = 0;

            % Initialize Variables to store
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.mean_state_of_data_storage = zeros(mission.storage.num_storage_steps, length(obj.mean_state_of_data_storage));
            obj.store.total_data_storage = zeros(mission.storage.num_storage_steps, length(obj.total_data_storage));

            % Update Storage
            obj = func_update_software_SC_data_handling_store(obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variables

        function obj = func_update_software_SC_data_handling_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [Boolean]                
                obj.store.mean_state_of_data_storage(mission.storage.k_storage,:) = obj.mean_state_of_data_storage; % [percentage]            
                obj.store.total_data_storage(mission.storage.k_storage,:) = obj.total_data_storage; % [kb]            
            end

        end

        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_data_handling(obj, mission, i_SC)

    
            switch obj.mode_software_SC_data_handling_selector
    
                case 'Generic'
                    % Update Mean SoC
                    obj = func_update_mean_state_of_data_storage(obj, mission, i_SC);
    
                case 'Nightingale'
                    % Update Mean SoC for (num_onboard_memory-1) only
                    obj = func_update_mean_state_of_data_storage_Nightingale(obj, mission, i_SC);                        
    
                otherwise
                    error('Data Handling mode not defined!')
            end
    
            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);


            % Update Storage
            obj = func_update_software_SC_data_handling_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end


        %% [ ] Methods: Update Mean SoDS
        % Updates mean_state_of_data_storage

        function obj = func_update_mean_state_of_data_storage(obj, mission, i_SC)

            total_instantaneous_capacity = 0; % [kb]
            total_maximum_capacity = 0; % [kb]

            for i_memory = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory

                total_instantaneous_capacity = total_instantaneous_capacity + mission.true_SC{i_SC}.true_SC_onboard_memory{i_memory}.instantaneous_capacity; % [kb]
                total_maximum_capacity = total_maximum_capacity + mission.true_SC{i_SC}.true_SC_onboard_memory{i_memory}.maximum_capacity; % [kb]

            end

            obj.mean_state_of_data_storage = 100 * total_instantaneous_capacity / total_maximum_capacity; % [percentage]

            obj.total_data_storage = total_instantaneous_capacity; % [kb]            

        end        

        
    end
end

