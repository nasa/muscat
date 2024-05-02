classdef True_SC_Star_Tracker_Sensor < handle
    %STAR TRACKER Summary of this class goes here
    %   Detailed explanation goes here

    properties

        time            % [s] current true time

        num_star_tracker  % [-] number of star tracker

        star_tracker_data % data structure for the i^th star tracker
%                 – # name = Star Tracker i
%                 – health : Health of ith star_tracker
%                       0. Switched off
%                       1. Switched on, works nominally
%                 – temperature [deg C] : Temperature of ith star_tracker
%                 – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith star_tracker (if it is switched on)
%                 – # instantaneous_data_generated [kb] : Data generated during current time step, in
%                   kilo bits (kb)
%                 – measurement_frequency [Hz] : How many measurements per seconds are possible?
%                 – # measurement_wait_time [sec] = (1 / measurement_frequency)
%                 – # accumulated_wait_time = 0 [sec]
%                 – # take_measurement [Boolean] : 1 = Take measurement this time step
%                 – orientation [unit vector] : Normal vector of measuring element in body frame B
%                 – location [m] : Location of sensor, in body frame B
%                 – field_of_view [deg] : Not used in code!
%                 – Sun_avoidance_angle [deg] : Sun-to-boresight angle to be avoided
%                 – measurement_noise [deg] : Added to every dimension of attitude quaternion
%                 – # measurement_vector [quaternion] : Noisy attitude quaternion
%                 – # measurement_available [Boolean] : 1 = Measurement taken this time step
%                 – # measurement_time [sec] : Time when latest Measurement was taken
    end

    methods

        function obj = True_SC_Star_Tracker_Sensor(mission_true_time,sc_body_init_data)
            
            obj.time = mission_true_time.time;

            obj.num_star_tracker = sc_body_init_data.num_star_tracker;

            obj.star_tracker_data = [];

            for i=1:obj.num_star_tracker
                obj.star_tracker_data(i).name = ['Star Tracker ',num2str(i)];
                obj.star_tracker_data(i).health = 1;
                obj.star_tracker_data(i).temperature = 10;
                obj.star_tracker_data(i).instantaneous_power_consumed = 0;
                obj.star_tracker_data(i).instantaneous_power_consumption = sc_body_init_data.star_tracker_instantaneous_power_consumption;
                obj.star_tracker_data(i).measurement_noise = sc_body_init_data.star_tracker_measurement_noise;
                obj.star_tracker_data(i).instantaneous_data_generated = 0; % [kilo bits]
                obj.star_tracker_data(i).instantaneous_data_volume = sc_body_init_data.star_tracker_instantaneous_data_volume;
                obj.star_tracker_data(i).measurement_frequency = sc_body_init_data.star_tracker_measurement_frequency;
                obj.star_tracker_data(i).measurement_wait_time = 1/obj.star_tracker_data(i).measurement_frequency; % [sec] between to measurement
                obj.star_tracker_data(i).accumulated_wait_time = 0; % [sec]
                obj.star_tracker_data(i).take_measurement = 1; % [Boolean] : 1 = Take measurement this time step
                obj.star_tracker_data(i).orientation = sc_body_init_data.star_tracker_orientation(i,:); % [unit vector] : Normal vector of measuring element in body frame B
                obj.star_tracker_data(i).location = sc_body_init_data.star_tracker_location(i,:); % [m] : Location of sensor, in body frame B
                obj.star_tracker_data(i).measurement_vector = [0 0 0 0]'; % [quaternion] : Noisy attitude quaternion
                obj.star_tracker_data(i).measurement_available = 0; % [Boolean] : 1 = Measurement taken this time step
                obj.star_tracker_data(i).measurement_time = 0; % [sec] : Time when latest Measurement was taken
%                 obj.star_tracker_data(i).field_of_view = 0; (10 x 12 deg)
%                 obj.star_tracker_data(i).Sun_avoidance_angle = 45; % [deg]
            end

        end

        function obj = func_update_accumulated_wait_time(obj, True_time)
            % Update current time
            % Update accumulated time since last measurement

            obj.time = True_time.time;

            for i = 1:obj.num_star_tracker

                if obj.star_tracker_data(i).health == 1
                    obj.star_tracker_data(i).accumulated_wait_time = obj.time - obj.star_tracker_data(i).measurement_time;
                else
                    obj.star_tracker_data(i).accumulated_wait_time = 0;
                end

            end
            
        end

        function obj = func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time

            for i=1:obj.num_star_tracker

                if obj.star_tracker_data(i).accumulated_wait_time >= obj.star_tracker_data(i).measurement_wait_time
                    obj.star_tracker_data(i).take_measurement = 1;
                else
                    obj.star_tracker_data(i).take_measurement = 0;
                end
            
            end

        end

        function obj = func_get_star_tracker_measurement(obj, True_SC_adc, True_time)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            
            obj = func_update_accumulated_wait_time(obj, True_time);    % update time
            obj = func_check_wait_time(obj);                            % check for measurement trigerring

            for i = 1:obj.num_star_tracker
                
                if obj.star_tracker_data(i).take_measurement == 1

                    % update measurement
                    obj.star_tracker_data(i).measurement_vector = True_SC_adc.attitude + obj.star_tracker_data(i).measurement_noise*randn(4,1);
                    obj.star_tracker_data(i).measurement_vector = obj.star_tracker_data(i).measurement_vector/norm(obj.star_tracker_data(i).measurement_vector);
            
                    obj.star_tracker_data(i).measurement_time = obj.time;
                    obj.star_tracker_data(i).measurement_available = 1;
                    obj.star_tracker_data(i).take_measurement = 0;
                    obj.star_tracker_data(i).instantaneous_data_generated = obj.star_tracker_data(i).instantaneous_data_volume;
                    obj.star_tracker_data(i).instantaneous_power_consumed = obj.star_tracker_data(i).instantaneous_power_consumption;
                else
                    obj.star_tracker_data(i).measurement_available = 0;
                    obj.star_tracker_data(i).instantaneous_data_generated = 0;
                    obj.star_tracker_data(i).instantaneous_power_consumed = 0;
                end

            end

        end
        
    end

end

