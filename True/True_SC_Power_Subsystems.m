classdef True_SC_Power_Subsystems < handle
    %SC_POWER Summary of this class goes here
    %   Detailed explanation goes here

    properties

        % dynamic parameters
        time
        time_step

        instantaneous_power_consumed
        
        instantaneous_power_generated

        instantaneous_energy

        instantaneous_energy_unused
       
    end

    methods
        function obj = True_SC_PoTrue_SC_Power_Subsystemswer(True_time)
            %SC_POWER Construct an instance of this class
            %   Detailed explanation goes here
            obj.instantaneous_power_consumed=0;
            obj.instantaneous_power_generated=0;
            obj.instantaneous_energy = 0;
            obj.instantaneous_energy_unused = 0;

            obj.time = True_time.time; % [sec]
            obj.time_step = True_time.time_step; % [sec]
        end

        function obj = fun_update_instantaneous_power_consumed(obj,equipement)
            obj.instantaneous_power_consumed = obj.instantaneous_power_consumed + equipement.instantaneous_power_consumed; % [Watts]
        end
        
        function obj = fun_update_instantaneous_energy(obj,true_SC_battery)
            obj.instantaneous_energy = (obj.instantaneous_power_generated - obj.instantaneous_power_consumed) * (obj.time_step/3600);
            obj.instantaneous_energy_unused = true_SC_battery.energy_unused;
        end

        function obj = fun_update_power_generated(obj, true_SC_solar_panel)
            obj.instantaneous_power_generated = obj.instantaneous_power_generated + true_SC_solar_panel.instantaneous_power_generated;
        end

    end

end


