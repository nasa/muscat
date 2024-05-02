classdef True_SC_Onboard_Clock_Sensor < handle
    %ONBOARD_CLOCK_SENSOR Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
        time

        num_onboard_clock

        onboard_clock_data
%             – # name = ‘Onboard Clock i’
%             – health : Health of ith Onboard Clock
%             0. Switched off
%             1. Switched on, works nominally
%             – temperature [deg C] : Temperature of ith Onboard Clock
%             – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith On-
%             board Clock (if it is switched on)
%             – # instantaneous_data_generated [kb] : Data generated during current time step, in
%             kilo bits (kb)
%             – measurement_frequency [Hz] : How many measurements per seconds are possible?
%             – # measurement_wait_time [sec] = (1 / measurement_frequency)
%             – # accumulated_wait_time = 0 [sec]
%             – # take_measurement [Boolean] : 1 = Take measurement this time step
%             – orientation [unit vector] : Normal vector of measuring element in body frame B
%             – location [m] : Location of sensor, in body frame B
%             – measurement_noise [sec] : Added to every dimension of time
%             – # measurement_vector [sec] : Noisy time
%             – # measurement_available [Boolean] : 1 = Measurement taken this time step
%             – # measurement_time [sec] : Time when latest Measurement was taken
    end

    methods
        function obj = True_SC_Onboard_Clock_Sensor(sc_body_init_data)
            %ONBOARD_CLOCK_SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.time = 0;

            obj.num_onboard_clock = sc_body_init_data.num_onboard_clock;

            for i=1:obj.num_onboard_clock

                obj.onboard_clock_data(i).name = ['Onboard Clock ',num2str(i)];
                obj.onboard_clock_data(i).health = 1;
                obj.onboard_clock_data(i).temperature = 20;
                obj.onboard_clock_data(i).instantaneous_power_consumed = 0;
                obj.onboard_clock_data(i).instantaneous_data_generated = 0;
                obj.onboard_clock_data(i).measurement_frequency = 50;
                obj.onboard_clock_data(i).measurement_wait_time = 1/obj.onboard_clock_data(i).measurement_frequency;
                obj.onboard_clock_data(i).accumulated_wait_time = 0;
                obj.onboard_clock_data(i).take_measurement = 1;
                obj.onboard_clock_data(i).orientation = sc_body_init_data.onboard_clock_orientation;
                obj.onboard_clock_data(i).location = sc_body_init_data.onboard_clock_location;
                obj.onboard_clock_data(i).measurement_noise = 0;
                obj.onboard_clock_data(i).measurement_vector = 0;
                obj.onboard_clock_data(i).measurement_available = 0;
                obj.onboard_clock_data(i).measurement_time = 0;

            end

        end

        function obj = func_update_accumulated_wait_time(obj, mission_true_time)
            % Update current time
            % Update accumulated time since last measurement

            for i = 1:obj.num_onboard_clock

                if obj.onboard_clock_data(i).health == 1
                    obj.onboard_clock_data(i).accumulated_wait_time = mission_true_time.time - obj.onboard_clock_data(i).measurement_time;
                else
                    obj.onboard_clock_data(i).accumulated_wait_time = 0;
                end

            end
            
        end

        function obj = func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time

            for i=1:obj.num_onboard_clock

                if obj.onboard_clock_data(i).accumulated_wait_time >= obj.onboard_clock_data(i).measurement_wait_time
                    obj.onboard_clock_data(i).take_measurement = 1;
                else
                    obj.onboard_clock_data(i).take_measurement = 0;
                end
            
            end

        end

        function obj = func_get_time(obj,mission_true_time)

            obj = func_update_accumulated_wait_time(obj, mission_true_time);    % update accumulated time
            obj = func_check_wait_time(obj);                                    % check for measurement trigerring

            for i=1:obj.num_onboard_clock

                if obj.onboard_clock_data(i).take_measurement == 1 && obj.onboard_clock_data(i).health == 1

                    % measurement
                    obj.onboard_clock_data(i).measurement_vector = mission_true_time.time + obj.onboard_clock_data(i).measurement_noise;
                    obj.time = obj.onboard_clock_data(i).measurement_vector;
                    obj.onboard_clock_data(i).measurement_available = 1;
                    obj.onboard_clock_data(i).measurement_time = mission_true_time.time;
                    % data and power
                    obj.onboard_clock_data(i).instantaneous_power_consumed = 120e-3; % [Watts] https://www.microsemi.com/product-directory/clocks-frequency-references/3824-chip-scale-atomic-clock-csac
                    obj.onboard_clock_data(i).instantaneous_data_generated = 1e-3*(1*16); % [kilo bits]

                else
                    obj.onboard_clock_data(i).measurement_available = 0;
                    obj.onboard_clock_data(i).instantaneous_power_consumed = 0;
                    obj.onboard_clock_data(i).instantaneous_data_generated = 0;
                end
            end
        end
    end
end

