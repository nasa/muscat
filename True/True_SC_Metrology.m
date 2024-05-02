classdef True_SC_Metrology < handle
    %SUN_SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        time            % [s] current true time
        
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
        name
        health
        temperature
        instantaneous_power_consumption
        instantaneous_power_consumed
        instantaneous_data_volume
        instantaneous_data_generated
        measurement_frequency
        measurement_wait_time
        accumulated_wait_time
        take_measurement
        orientation
        location
        measurement_noise
        measurement_vector
        measurement_available
        measurement_time
        
    end
    
    methods
        
        function obj = True_SC_Metrology(mission_true_time,sc_body_init_data)
            
            obj.time = mission_true_time.time;
            
            obj.name = 'GPS Receiver';
            obj.health = 1;
            obj.temperature = 10;
            obj.instantaneous_power_consumption = 50; % [W]
            obj.instantaneous_power_consumed = 0; % [W]
            obj.measurement_noise = 1e-9; % [km] 1 um
            obj.instantaneous_data_volume = (1+1) * 16e-3; % [kb] : range and range rate, each of 16-bit depth
            obj.instantaneous_data_generated = 0; % [kilo bits]
            obj.measurement_frequency = 50e3; % [Hz] between to measurement
            obj.measurement_wait_time = 1/obj.measurement_frequency; % [sec] between to measurement
            obj.accumulated_wait_time = 0; % [sec]
            obj.take_measurement = 1; % [Boolean] : 1 = Take measurement this time step
            obj.orientation = sc_body_init_data.laser_metrology_orientation; % [unit vector] : Normal vector of measuring element in body frame B
            obj.location = sc_body_init_data.laser_metrology_location; % [m] : Location of sensor, in body frame B
            obj.measurement_vector = [0 0 0 0]'; % [quaternion] : Noisy attitude quaternion
            obj.measurement_available = 0; % [Boolean] : 1 = Measurement taken this time step
            obj.measurement_time = 0; % [sec] : Time when latest Measurement was taken
            
        end
        
        function obj = func_update_accumulated_wait_time(obj, True_time)
            % Update current time
            % Update accumulated time since last measurement
            obj.time = True_time.time;
            if obj.health == 1
                obj.accumulated_wait_time = obj.time - obj.measurement_time;
            else
                obj.accumulated_wait_time = 0;
            end
        end
        
        function obj = func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time
            if obj.accumulated_wait_time >= obj.measurement_wait_time
                obj.take_measurement = 1;
            else
                obj.take_measurement = 0;
            end
        end
        
        function obj = func_get_laser_metrology_measurement(obj, i_SC, true_SC_all, true_time)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            obj = func_update_accumulated_wait_time(obj, true_time);    % update time
            obj = func_check_wait_time(obj);                            % check for measurement trigerring
            
            n_meas = numel(true_SC_all)-1;
            if obj.take_measurement == 1
                % update measurement
                obj.measurement_vector = zeros(n_meas, 1);
                for j_SC = 1:numel(true_SC_all)
                    % TODO: Use this function for measurements (c.f. Development/Test_SwarmOrbitDetermination.m)
                    [y, H, R] = compute_range_rangerate_bearing_angles(links, config, rv, flag_true)
                end
                obj.measurement_vector = obj.measurement_vector + mvnrnd(zeros(n_meas, 1), obj.measurement_noise);
                
                obj.measurement_time = obj.time;
                obj.measurement_available = 1;
                obj.take_measurement = 0;
                obj.instantaneous_data_generated = obj.instantaneous_data_volume;
                obj.instantaneous_power_consumed = obj.instantaneous_power_consumption;
            else
                obj.measurement_available = 0;
                obj.instantaneous_data_generated = 0;
            end
        end
    end
end

