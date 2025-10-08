%% Class: True_SC_Battery
% Tracks the Battery state of charge

classdef True_SC_Battery < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed 

        instantaneous_data_rate_generated % [kbps] : Data rate generated during current time step, in kilo bits (kb) per sec

        maximum_capacity % [Watts * hr] : Maximum energy storage capacity of the battery

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Batt i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        instantaneous_capacity % [Watts * hr] : Instantaneous capacity of battery

        state_of_charge % [percentage] : SoC is defined by = 100× instantaneous_capacity / maximum_capacity

        charging_efficiency % float ∈ [0, 1]
        
        discharging_efficiency % float ∈ [0, 1]

        %% [ ] Properties: Storage Variables

        store


    end

    %% Methods
    methods
        
        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Battery(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Battery ',num2str(i_HW)]; 
            end
            
            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]

            obj.maximum_capacity = init_data.maximum_capacity; % [W hr]
            obj.instantaneous_capacity = obj.maximum_capacity; % [W hr]
            obj.state_of_charge = 100*obj.instantaneous_capacity/obj.maximum_capacity; % [percentage]

            obj.charging_efficiency = init_data.charging_efficiency;
            obj.discharging_efficiency = init_data.discharging_efficiency; % [float <= 1]

            % Initialize Variables to store: instantaneous_capacity state_of_charge
            obj.store = [];

            obj.store.instantaneous_capacity = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_capacity));
            obj.store.state_of_charge = zeros(mission.storage.num_storage_steps, length(obj.state_of_charge));

            obj = func_update_true_SC_battery_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_battery_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_capacity(mission.storage.k_storage,:) = obj.instantaneous_capacity; % [W hr]
                obj.store.state_of_charge(mission.storage.k_storage,:) = obj.state_of_charge; % [percentage]
            end

        end

        %% [ ] Methods: Main
        % Update Battery SoC

        function obj = func_main_true_SC_battery(obj, mission, i_SC)

            obj.state_of_charge = 100*obj.instantaneous_capacity/obj.maximum_capacity; % [percentage]

            obj = func_update_true_SC_battery_store(obj, mission);

            % Update Power Consumed
            func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            % Check Battery is Non-negative
            if obj.instantaneous_capacity < 0
                error('Battery SoC is negative!')
            end

        end

    end
end

