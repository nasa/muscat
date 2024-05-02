classdef True_SC_Sun_Sensor < handle
    %SUN_SENSOR Summary of this class goes here
    %   Detailed explanation goes here

    properties

        time            % [s] current true time

        num_sun_sensor  % [-] number of sun sensors

        sun_sensor_data % data structure for the i^th sun sensor
%                 – # name = ‘Sun Sensor i’
%                 – health : Health of ith sun sensor
%                       0. Switched off
%                       1. Switched on, works nominally
%                 – temperature [deg C] : Temperature of ith sun sensor
%                 – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith sun
%                   sensor (if it is switched on)
%                 – # instantaneous_data_generated [kb] : Data generated during current time step, in
%                   kilo bits (kb)
%                 – measurement_frequency [Hz] : How many measurements per seconds are possible?
%                 – # measurement_wait_time [sec] = (1 / measurement_frequency)
%                 – # accumulated_wait_time = 0 [sec]
%                 – # take_measurement [Boolean] : 1 = Take measurement this time step
%                 – orientation [unit vector] : Normal vector of measuring element in body frame B
%                 – location [m] : Location of sensor, in body frame B
%                 – measurement_noise [deg] : Added to every dimension of attitude quaternion
%                 – # measurement_vector [quaternion] : Noisy attitude quaternion
%                 – # measurement_available [Boolean] : 1 = Measurement taken this time step
%                 – # measurement_time [sec] : Time when latest Measurement was taken
    end

    methods

        function obj = True_SC_Sun_Sensor(mission_true_time,sc_body_init_data)
            
            obj.time = mission_true_time.time;

            obj.num_sun_sensor = sc_body_init_data.num_sun_sensor;

            obj.sun_sensor_data = [];

            for i=1:obj.num_sun_sensor
                obj.sun_sensor_data(i).name = ['Sun Sensor ',num2str(i)];
                obj.sun_sensor_data(i).health = 1;
                obj.sun_sensor_data(i).temperature = 10;
                obj.sun_sensor_data(i).instantaneous_power_consumption = sc_body_init_data.sun_sunsor_instantaneous_power_consumption;
                obj.sun_sensor_data(i).instantaneous_power_consumed = 0; % [W]
                obj.sun_sensor_data(i).measurement_noise = sc_body_init_data.sun_sunsor_measurement_noise;
                obj.sun_sensor_data(i).instantaneous_data_volume = sc_body_init_data.sun_sunsor_instantaneous_data_volume;
                obj.sun_sensor_data(i).instantaneous_data_generated = 0; % [kilo bits]
                obj.sun_sensor_data(i).measurement_frequency = sc_body_init_data.sun_sunsor_measurement_frequency; % [Hz] between to measurement
                obj.sun_sensor_data(i).measurement_wait_time = 1/obj.sun_sensor_data(i).measurement_frequency; % [sec] between to measurement
                obj.sun_sensor_data(i).accumulated_wait_time = 0; % [sec]
                obj.sun_sensor_data(i).take_measurement = 1; % [Boolean] : 1 = Take measurement this time step
                obj.sun_sensor_data(i).orientation = sc_body_init_data.sun_sensor_orientation(i,:); % [unit vector] : Normal vector of measuring element in body frame B
                obj.sun_sensor_data(i).location = sc_body_init_data.sun_sensor_location(i,:); % [m] : Location of sensor, in body frame B
                obj.sun_sensor_data(i).measurement_vector = [0 0 0 0]'; % [quaternion] : Noisy attitude quaternion
                obj.sun_sensor_data(i).measurement_available = 0; % [Boolean] : 1 = Measurement taken this time step
                obj.sun_sensor_data(i).measurement_time = 0; % [sec] : Time when latest Measurement was taken
            end

        end

        function obj = func_update_accumulated_wait_time(obj, True_time)
            % Update current time
            % Update accumulated time since last measurement

            obj.time = True_time.time;

            for i = 1:obj.num_sun_sensor

                if obj.sun_sensor_data(i).health == 1
                    obj.sun_sensor_data(i).accumulated_wait_time = obj.time - obj.sun_sensor_data(i).measurement_time;
                else
                    obj.sun_sensor_data(i).accumulated_wait_time = 0;
                end

            end
            
        end

        function obj = func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time

            for i=1:obj.num_sun_sensor

                if obj.sun_sensor_data(i).accumulated_wait_time >= obj.sun_sensor_data(i).measurement_wait_time
                    obj.sun_sensor_data(i).take_measurement = 1;
                else
                    obj.sun_sensor_data(i).take_measurement = 0;
                end
            
            end

        end

        function obj = func_get_sun_sensor_measurement(obj, True_SC_adc, True_time)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            
            obj = func_update_accumulated_wait_time(obj, True_time);    % update time
            obj = func_check_wait_time(obj);                            % check for measurement trigerring

            for i = 1:obj.num_sun_sensor
                
                if obj.sun_sensor_data(i).take_measurement == 1

                    % update measurement
                    obj.sun_sensor_data(i).measurement_vector = True_SC_adc.attitude + obj.sun_sensor_data(i).measurement_noise*randn(4,1);
                    obj.sun_sensor_data(i).measurement_vector = obj.sun_sensor_data(i).measurement_vector/norm(obj.sun_sensor_data(i).measurement_vector);
            
                    obj.sun_sensor_data(i).measurement_time = obj.time;
                    obj.sun_sensor_data(i).measurement_available = 1;
                    obj.sun_sensor_data(i).take_measurement = 0;
                    obj.sun_sensor_data(i).instantaneous_data_generated = obj.sun_sensor_data(i).instantaneous_data_volume;
                    obj.sun_sensor_data(i).instantaneous_power_consumed = obj.sun_sensor_data(i).instantaneous_power_consumption;
                else
                    obj.sun_sensor_data(i).measurement_available = 0;
                    obj.sun_sensor_data(i).instantaneous_data_generated = 0;
                end

            end

        end
        
    end

end

