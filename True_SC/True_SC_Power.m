%% Class: True_SC_Power
% Track the power status onboard the spacecraft

classdef True_SC_Power < handle

    %% Properties
    properties
        %% [ ] Properties: Initialized Variables

        power_loss_rate % [float] Fraction of the total power loss: Power loss [W] = (1 + power_loss_rate) * total_instantaneous_power_consumed 
        
        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Power Subsystem'

        instantaneous_total_power_consumed % [W] Total power comsumed by all sensors and actuators over mission.true_time.time_step sec

        instantaneous_total_power_generated % [W] Total power generated by solar panels or RTG over mission.true_time.time_step sec

        instantaneous_energy % [W hr] Converted into Energy for storage in battery

        instantaneous_energy_unused % [W hr] Excess Energy, that is usually going to heat the SC

        list_HW_energy_consumed % List of HW that consumes power

        list_HW_energy_generated % List of HW that generates power

        array_HW_energy_consumed % [W hr] Total energy consumed by this HW

        array_HW_energy_generated % [W hr] Total energy generated by this HW

        warning_counter % [integer] Counter stops the warning after 10 displays

        power_emergency % [boolean] Flag to indicate a critical power deficit when batteries are empty
        
        power_deficit % [W hr] Track power deficit when batteries are empty

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Power(init_data, mission)

            obj.instantaneous_total_power_consumed = 0; % [W]
            obj.instantaneous_total_power_generated = 0; % [W]
            obj.instantaneous_energy = 0; % [W hr]
            obj.instantaneous_energy_unused = 0; % [W hr]

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = 'Power Subsystem';
            end

            obj.power_loss_rate = init_data.power_loss_rate; % [float]
            
            obj.list_HW_energy_consumed = [];
            obj.list_HW_energy_generated = [];
            obj.array_HW_energy_consumed = [];
            obj.array_HW_energy_generated = [];
            obj.warning_counter = 0;
            obj.power_emergency = false;
            obj.power_deficit = 0;

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end
            obj.data.store_instantaneous_energy = obj.instantaneous_energy; % [W hr]

            % Initialize Variables to store: power and energy
            obj.store = [];

            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_total_power_consumed));
            obj.store.instantaneous_power_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_total_power_generated));
            obj.store.instantaneous_energy = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_energy));
            obj.store.instantaneous_energy_unused = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_energy_unused));

        end

        %% [ ] Methods: Initialize list_HW_energy_consumed
        % Initialize list_HW_energy_consumed for all HW

        function obj = func_initialize_list_HW_energy_consumed(obj, equipment, mission)

            this_name = equipment.name;
            flag_name_exists = 0;

            for i = 1:1:length(obj.list_HW_energy_consumed)
                if strcmp( obj.list_HW_energy_consumed{i}, this_name )
                    flag_name_exists = 1;
                end
            end

            if flag_name_exists == 0
                i = length(obj.list_HW_energy_consumed);
                obj.list_HW_energy_consumed{i+1} = this_name;
                obj.array_HW_energy_consumed(1,i+1) = equipment.instantaneous_power_consumed * (mission.true_time.time_step/3600); % [W hr]
            end

        end

        %% [ ] Methods: Initialize list_HW_energy_generated
        % Initialize list_HW_energy_generated for Solar Panels or RTG

        function obj = func_initialize_list_HW_energy_generated(obj, equipment, mission)

            this_name = equipment.name;
            flag_name_exists = 0;

            for i = 1:1:length(obj.list_HW_energy_generated)
                if strcmp( obj.list_HW_energy_generated{i}, this_name )
                    flag_name_exists = 1;
                end
            end

            if flag_name_exists == 0
                i = length(obj.list_HW_energy_generated);
                obj.list_HW_energy_generated{i+1} = this_name;
                obj.array_HW_energy_generated(1,i+1) = equipment.instantaneous_power_generated * (mission.true_time.time_step/3600); % [W hr]
            end

        end

        %% [ ] Methods: Initialize Store
        % Initialize store of array_HW_energy_consumed and array_HW_energy_generated

        function obj = func_initialize_store_HW_power_consumed_generated(obj, mission)

            obj.store.list_HW_energy_consumed = obj.list_HW_energy_consumed;
            obj.store.list_HW_energy_generated = obj.list_HW_energy_generated;

            obj.store.array_HW_energy_consumed = zeros(mission.storage.num_storage_steps, length(obj.array_HW_energy_consumed));
            obj.store.array_HW_energy_generated = zeros(mission.storage.num_storage_steps, length(obj.array_HW_energy_generated));

            obj = func_update_true_SC_power_store(obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_power_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_total_power_consumed; % [W]
                obj.store.instantaneous_power_generated(mission.storage.k_storage,:) = obj.instantaneous_total_power_generated; % [W]
                obj.store.instantaneous_energy(mission.storage.k_storage,:) = obj.data.store_instantaneous_energy; % [W hr]
                obj.store.instantaneous_energy_unused(mission.storage.k_storage,:) = obj.instantaneous_energy_unused; % [W hr]

                obj.store.array_HW_energy_consumed(mission.storage.k_storage,:) = obj.array_HW_energy_consumed; % [W hr]
                obj.store.array_HW_energy_generated(mission.storage.k_storage,:) = obj.array_HW_energy_generated; % [W hr]
            end

        end

        %% [ ] Methods: Main
        % Main power code

        function obj = func_main_true_SC_power(obj, mission, i_SC)

            obj.instantaneous_energy = (obj.instantaneous_total_power_generated - obj.instantaneous_total_power_consumed) * (mission.true_time.time_step/3600); % [W hr]
            obj.data.store_instantaneous_energy = obj.instantaneous_energy; % [W hr]


            if obj.instantaneous_energy > 0
                % Excess Energy use to Recharge Battery
                obj.power_emergency = false; % Clear emergency flag when we have excess energy

                for i_batt = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery:-1:1

                    if (mission.true_SC{i_SC}.true_SC_battery{i_batt}.health == 1) && (obj.instantaneous_energy > 0)

                        if mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity >= mission.true_SC{i_SC}.true_SC_battery{i_batt}.maximum_capacity
                            % Do nothing!

                        elseif (mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity + (mission.true_SC{i_SC}.true_SC_battery{i_batt}.charging_efficiency * obj.instantaneous_energy)) <= mission.true_SC{i_SC}.true_SC_battery{i_batt}.maximum_capacity
                            % Charge this Battery
                            mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity = mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity + (mission.true_SC{i_SC}.true_SC_battery{i_batt}.charging_efficiency * obj.instantaneous_energy); % [W hr]
                            obj.instantaneous_energy = 0; % [W hr]

                        else
                            % Fill this Battery
                            obj.instantaneous_energy = obj.instantaneous_energy - (mission.true_SC{i_SC}.true_SC_battery{i_batt}.maximum_capacity - mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity)/mission.true_SC{i_SC}.true_SC_battery{i_batt}.charging_efficiency; % [W hr]
                            mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity = mission.true_SC{i_SC}.true_SC_battery{i_batt}.maximum_capacity; % [W hr]

                        end

                    end

                end

                if obj.instantaneous_energy > 0
                    obj.instantaneous_energy_unused = obj.instantaneous_energy; % [W hr]
                end

            else
                % Discharge Battery (obj.instantaneous_energy < 0)

                for i_batt = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

                    if (mission.true_SC{i_SC}.true_SC_battery{i_batt}.health == 1) && (obj.instantaneous_energy < 0)

                        if mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity <= 0
                            % Do nothing!

                        elseif (mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity * mission.true_SC{i_SC}.true_SC_battery{i_batt}.discharging_efficiency) >= abs(obj.instantaneous_energy)
                            % Lots of extra Energy
                            mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity = mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity - (abs(obj.instantaneous_energy) / mission.true_SC{i_SC}.true_SC_battery{i_batt}.discharging_efficiency); % [W hr]
                            obj.instantaneous_energy = 0;

                        else
                            % Empty this Battery!
                            obj.instantaneous_energy = obj.instantaneous_energy + (mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity * mission.true_SC{i_SC}.true_SC_battery{i_batt}.discharging_efficiency); % [W hr]
                            mission.true_SC{i_SC}.true_SC_battery{i_batt}.instantaneous_capacity = 0; % [W hr]
                        end

                    end

                end

                if obj.instantaneous_energy < 0
                    % All batteries are empty but we still need power
                    % Set power emergency flag and calculate the deficit
                    obj.power_emergency = true;
                    obj.power_deficit = abs(obj.instantaneous_energy); % Track the deficit
                    
                    % Limit the number of warnings displayed
                    if obj.warning_counter < 10
                        warning('All Batteries are Empty! Power deficit: %0.2f W-hr', obj.power_deficit);
                        obj.warning_counter = obj.warning_counter + 1;
                    end
                    
                    % Reset instantaneous_energy to 0 to prevent negative values
                    obj.instantaneous_energy = 0;
                else
                    obj.power_emergency = false;
                    obj.warning_counter = 0;
                    obj.power_deficit = 0;
                end

            end

            % Store
            obj = func_update_true_SC_power_store(obj, mission);

            % Reset All Variables
            obj.instantaneous_total_power_consumed = 0; % [W]
            obj.instantaneous_total_power_generated = 0; % [W]
            obj.instantaneous_energy = 0; % [W hr]
            obj.instantaneous_energy_unused = 0; % [W hr]            

        end



        %% [ ] Methods: Update Instantaneous Power Consumed
        % Updates instantaneous_power_consumed by all HW

        function obj = func_update_instantaneous_power_consumed(obj, equipment, mission)
            
            obj.instantaneous_total_power_consumed = obj.instantaneous_total_power_consumed + equipment.instantaneous_power_consumed * (1 + obj.power_loss_rate); % [W]

            this_name = equipment.name;
            flag_name_exists = 0;
            this_idx = 0;

            for i = 1:1:length(obj.list_HW_energy_consumed)
                if strcmp( obj.list_HW_energy_consumed{i}, this_name )
                    flag_name_exists = 1;
                    this_idx = i;
                end
            end


            if flag_name_exists == 0
                error('HW not found!')
            else
                obj.array_HW_energy_consumed(1,this_idx) = obj.array_HW_energy_consumed(1,this_idx) + (equipment.instantaneous_power_consumed*(mission.true_time.time_step/3600)); % [W hr]
            end            
        end

        %% [ ] Methods: Update Instantaneous Power Consumed Attitude
        % Updates instantaneous_power_consumed by all HW, within ADL

        function obj = func_update_instantaneous_power_consumed_attitude(obj, equipment, mission)
            obj.instantaneous_total_power_consumed = obj.instantaneous_total_power_consumed + equipment.instantaneous_power_consumed*(mission.true_time.time_step_attitude/mission.true_time.time_step); % [W]

            this_name = equipment.name;
            flag_name_exists = 0;
            this_idx = 0;

            for i = 1:1:length(obj.list_HW_energy_consumed)
                if strcmp( obj.list_HW_energy_consumed{i}, this_name )
                    flag_name_exists = 1;
                    this_idx = i;
                end
            end

            if flag_name_exists == 0
                error('HW not found!')
            else
                obj.array_HW_energy_consumed(1,this_idx) = obj.array_HW_energy_consumed(1,this_idx) + (equipment.instantaneous_power_consumed*(mission.true_time.time_step_attitude/3600)); % [W hr]
            end

        end



        %% [ ] Methods: Update Instantaneous Power Generated
        % Updates instantaneous_power_generated by Solar Panels or RTG

        function obj = func_update_instantaneous_power_generated(obj, equipment, mission)
            obj.instantaneous_total_power_generated = obj.instantaneous_total_power_generated + equipment.instantaneous_power_generated; % [W]

            this_name = equipment.name;
            flag_name_exists = 0;
            this_idx = 0;

            for i = 1:1:length(obj.list_HW_energy_generated)
                if strcmp( obj.list_HW_energy_generated{i}, this_name )
                    flag_name_exists = 1;
                    this_idx = i;
                end
            end

            if flag_name_exists == 0
                error('HW not found!')
            else
                obj.array_HW_energy_generated(1,this_idx) = obj.array_HW_energy_generated(1,this_idx) + (equipment.instantaneous_power_generated*(mission.true_time.time_step/3600)); % [W hr]
            end

        end


    end

end


