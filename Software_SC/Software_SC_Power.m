%% Class: Software_SC_Power
% Track the power status onboard the spacecraft

classdef Software_SC_Power < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_software_SC_power_selector % [string] Different power modes
        % - 'Generic' 

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Power

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        mean_state_of_charge % [percentage] : Mean SoC is defined by = 100× Sum (instantaneous_capacity) / Sum(maximum_capacity)

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Power(init_data, mission, i_SC)

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Power']; % [string]
            obj.flag_executive = 0;

            obj.instantaneous_data_generated_per_sample = init_data.instantaneous_data_generated_per_sample; % [kb]

            obj.mode_software_SC_power_selector = init_data.mode_software_SC_power_selector;
                        
            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Update Mean SoC
            obj = func_update_mean_state_of_charge(obj, mission, i_SC);

            % Initialize Variables to store
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.mean_state_of_charge = zeros(mission.storage.num_storage_steps, length(obj.mean_state_of_charge));

            % Update Storage
            obj = func_update_software_SC_power_store(obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variables

        function obj = func_update_software_SC_power_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [Boolean]                
                obj.store.mean_state_of_charge(mission.storage.k_storage,:) = obj.mean_state_of_charge; % [percentage]                
            end

        end

        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_power(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_power_selector

                    case 'Generic'

                        % Update Mean SoC
                        obj = func_update_mean_state_of_charge(obj, mission, i_SC);
                    
                    case 'Nightingale'
                        % obj = func_update_software_SC_power_Nightingale(obj,mission,i_SC);
                        
                        % Update Mean SoC
                        obj = func_update_mean_state_of_charge(obj, mission, i_SC);
                    
                    otherwise
                        error('Power mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end

            % Update Storage
            obj = func_update_software_SC_power_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end


        %% [ ] Methods: Update Mean SoC
        % Updates mean_state_of_charge

        function obj = func_update_mean_state_of_charge(obj, mission, i_SC)

            total_instantaneous_capacity = 0; % [Watts * hr]
            total_maximum_capacity = 0; % [Watts * hr]

            for i_batt = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

                total_instantaneous_capacity = total_instantaneous_capacity + mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity; % [Watts * hr]
                total_maximum_capacity = total_maximum_capacity + mission.true_SC{i_SC}.true_SC_battery{i_batt}.maximum_capacity; % [Watts * hr]

            end

            obj.mean_state_of_charge = 100 * total_instantaneous_capacity / total_maximum_capacity; % [percentage]

        end        
        
        
    end
end

