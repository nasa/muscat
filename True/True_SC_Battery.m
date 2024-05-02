classdef True_SC_Battery < handle
    %SC_POWER Summary of this class goes here
    %   Detailed explanation goes here

    properties

        % dynamic parameters
        time

        num_battery

        battery_data
%         – # name = ‘Battery i’
%         – health : Health of ith battery
%             0. Switched off
%             1. Switched on, works nominally
%         – temperature [deg C] : Temperature of ith battery
%         – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith bat-
%             tery (irrespective of whether it is charging / discharging/ idle)
%         – maximum_capacity [Watts * hr] : Maximum energy storage capacity of the battery
%         – instantaneous_capacity [Watts * hr] : Instantaneous capacity of battery
%         – # state_of_charge [percentage] : SoC is defined by = 100× instantaneous_capacity / maximum_capacity
%         – charging_efficiency : ∈ [0, 1]
%         – discharging_efficiency : ∈ [0, 1]

        energy_unused

    end

    methods
        
        function obj = True_SC_Battery(sc_body_init_data,true_time)

            obj.time = true_time.time; % [sec]

            % Battery Variables
            obj.num_battery = sc_body_init_data.num_battery;
            for i=1:1:obj.num_battery
                obj.battery_data(i).name = ['Battery ',num2str(i)];
                obj.battery_data(i).health = 1;
                obj.battery_data(i).temperature = 20;
                obj.battery_data(i).instantaneous_power_consumption = sc_body_init_data.battery_instantaneous_power_consumption(i);
                obj.battery_data(i).instantaneous_power_consumed = 0;
                obj.battery_data(i).maximum_capacity = sc_body_init_data.battery_maximum_capacity(i);
                obj.battery_data(i).instantaneous_capacity = obj.battery_data(i).maximum_capacity;
                obj.battery_data(i).state_of_charge = 100*obj.battery_data(i).instantaneous_capacity/obj.battery_data(i).maximum_capacity;
                obj.battery_data(i).charging_efficiency = sc_body_init_data.charging_efficiency(i);
                obj.battery_data(i).discharging_efficiency = sc_body_init_data.discharging_efficiency(i);
            end

            obj.energy_unused = 0;
        end

        function obj = func_update_battery(obj,true_SC_power)

            SC_energy_net = true_SC_power.instantaneous_energy;

            if SC_energy_net >= 0

                % recharge all batteries
                for i=1:1:obj.num_battery

                    obj.battery_data(i).instantaneous_power_consumed = 0;

                    if obj.battery_data(i).health == 1

                        if obj.battery_data(i).instantaneous_capacity >= obj.battery_data(i).maximum_capacity
                            % Do nothing!
                            obj.battery_data(i).instantaneous_capacity = obj.battery_data(i).maximum_capacity;
                            obj.battery_data(i).instantaneous_power_consumed = 0; % [Watts]
                            % SC_energy_net not changed

                        elseif (obj.battery_data(i).instantaneous_capacity + obj.battery_data(i).charging_efficiency*SC_energy_net) >= obj.battery_data(i).maximum_capacity
                            % Charge battery to full, use remaining charge for next battery
                            SC_energy_net = SC_energy_net - (obj.battery_data(i).maximum_capacity - obj.battery_data(i).instantaneous_capacity)/obj.battery_data(i).charging_efficiency; % [Watts * hr]
                            obj.battery_data(i).instantaneous_power_consumed = (obj.battery_data(i).maximum_capacity - obj.battery_data(i).instantaneous_capacity)*(3600/true_SC_power.time_step); % [Watts]
                            obj.battery_data(i).instantaneous_capacity = obj.battery_data(i).maximum_capacity; % [Watts * hr]

                        else
                            % Charge this battery
                            obj.battery_data(i).instantaneous_capacity = obj.battery_data(i).instantaneous_capacity + obj.battery_data(i).charging_efficiency*SC_energy_net; % [Watts * hr]
                            obj.battery_data(i).instantaneous_power_consumed = SC_energy_net*(3600/true_SC_power.time_step); % [Watts]
                            SC_energy_net = 0; % [Watts * hr]

                        end
                        % always add the leakage consumption
                        obj.battery_data(i).instantaneous_power_consumed = obj.battery_data(i).instantaneous_power_consumed + obj.battery_data(i).instantaneous_power_consumption;
                    end
                end

            else % SC_energy_net < 0

                % discharge energy from battery
                for i=1:1:obj.num_battery

                    obj.battery_data(i).instantaneous_power_consumed = 0;

                    if obj.battery_data(i).health == 1

                        if obj.battery_data(i).instantaneous_capacity <= 0
                            % Do nothing!
                            obj.battery_data(i).instantaneous_capacity = 0;
                            obj.battery_data(i).instantaneous_power_consumed = 0; % [Watts]
                            % SC_energy_net not changed

                        elseif obj.battery_data(i).instantaneous_capacity * obj.battery_data(i).discharging_efficiency <= abs(SC_energy_net)
                            SC_energy_net = SC_energy_net + (obj.battery_data(i).discharging_efficiency * obj.battery_data(i).instantaneous_capacity); % [Watts * hr]
                            obj.battery_data(i).instantaneous_power_consumed = -obj.battery_data(i).instantaneous_capacity*(3600/true_SC_power.time_step); % [Watts]
                            obj.battery_data(i).instantaneous_capacity = 0; % [Watts * hr]

                        else
                            obj.battery_data(i).instantaneous_capacity = obj.battery_data(i).instantaneous_capacity - ((1/obj.battery_data(i).discharging_efficiency) * abs(SC_energy_net)); % [Watts * hr]
                            obj.battery_data(i).instantaneous_power_consumed = SC_energy_net*(3600/true_SC_power.time_step); % [Watts]
                            SC_energy_net = 0; % [Watts * hr]
                        end
                        % always add the leakage consumption
                        obj.battery_data(i).instantaneous_power_consumed = obj.battery_data(i).instantaneous_power_consumed + obj.battery_data(i).instantaneous_power_consumption;
                    end
                end



            end

            obj.energy_unused = SC_energy_net; % [Watts * hr]

            % Update State of charge
            for i=1:1:obj.num_battery
                obj.battery_data(i).state_of_charge = 100*obj.battery_data(i).instantaneous_capacity/obj.battery_data(i).maximum_capacity;
            end

        end


        

    end
end

