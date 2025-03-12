%% Class: True_SC_Fuel_Tank
% Tracks the fuel state of the spacecraft propellant tank

classdef True_SC_Fuel_Tank < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables
        name                    % [string] 'Fuel Tank i'
        health                  % [integer] Health of fuel tank (0: Off, 1: On)
        temperature             % [deg C] Temperature of fuel tank
        
        instantaneous_power_consumed      % [Watts] Power consumed by the fuel tank (e.g., heaters)
        instantaneous_data_rate_generated % [kbps] Data rate generated during current time step

        maximum_capacity        % [kg] Maximum fuel mass capacity
        instantaneous_fuel_mass % [kg] Current fuel mass in the tank
        fuel_density            % [kg/m^3] Density of the propellant
        
        location                % [m] Location of the tank in the body frame
        shape_model             % Structure containing shape model information
        
        flag_update_SC_body     % [Boolean] Flag to signal when SC body mass needs to be updated

        %% [ ] Properties: Storage Variables
        store                   % Structure to store historical data
    end

    %% Methods
    methods
        %% Constructor
        function obj = True_SC_Fuel_Tank(init_data, mission, i_SC, i_HW)
            % Constructor for the fuel tank class
            
            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Fuel Tank ', num2str(i_HW)];
            end
            
            obj.health = 1; % Default to healthy
            obj.temperature = 20; % Default temperature in Celsius
            
            % Power and data parameters
            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed;
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated;
            
            % Fuel parameters
            obj.maximum_capacity = init_data.maximum_capacity;
            obj.instantaneous_fuel_mass = init_data.initial_fuel_mass;
            obj.fuel_density = init_data.fuel_density;
            
            % Physical parameters
            obj.location = init_data.location;
            
            % Shape model if provided
            if isfield(init_data, 'shape_model')
                obj.shape_model = init_data.shape_model;
            else
                obj.shape_model = [];
            end
            
            % Flag for body update
            obj.flag_update_SC_body = 0;
            
            % Initialize storage variables
            obj.store = [];
            obj.store.instantaneous_fuel_mass = zeros(mission.storage.num_storage_steps, 1);
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, 1);
            
            % Register with power system
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
        end

        %% Update Storage
        function obj = func_update_true_SC_fuel_tank_store(obj, mission)
            % Update storage variables for the fuel tank
            
            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_fuel_mass(mission.storage.k_storage, :) = obj.instantaneous_fuel_mass;
                obj.store.instantaneous_power_consumed(mission.storage.k_storage, :) = obj.instantaneous_power_consumed;
            end
        end

        %% Consume Fuel
        function obj = func_consume_fuel(obj, fuel_mass_consumed)
            % Consume fuel from the tank
            
            % Check if there's enough fuel
            if fuel_mass_consumed > obj.instantaneous_fuel_mass
                warning('Fuel tank %s: Attempted to consume more fuel than available!', obj.name);
                fuel_mass_consumed = obj.instantaneous_fuel_mass;
            end
            
            % Update fuel mass
            obj.instantaneous_fuel_mass = obj.instantaneous_fuel_mass - fuel_mass_consumed;
            
            % Set flag to update SC body
            obj.flag_update_SC_body = 1;
        end

        %% Main Function
        function obj = func_main_true_SC_fuel_tank(obj, mission, i_SC)
            % Main function for the fuel tank
            
            % Update propellant mass in the spacecraft body
            if obj.flag_update_SC_body == 1
                % Instead of directly updating the mass, just set the flag for the body to update
                % The body's func_update_SC_body_mass will grab the latest fuel mass
                mission.true_SC{i_SC}.true_SC_body.flag_update_SC_body_total_mass_COM_MI = 1;
                
                % Reset update flag
                obj.flag_update_SC_body = 0;
            end
            
            % Update power system with this tank's power consumption (heaters, valves, etc.)
            func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            
            % Update storage
            obj = func_update_true_SC_fuel_tank_store(obj, mission);
        end
    end
end 