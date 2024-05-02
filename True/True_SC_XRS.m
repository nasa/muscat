classdef True_SC_XRS
    %True_SC_XRS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        time            % [s] current true time
        
        name                             % name = ‘XRS’
        health                           % health : Health of ith XRS
        temperature                      % temperature [deg C] : Temperature of XRS
        instantaneous_power_consumption  % 
        instantaneous_power_consumed     % instantaneous_power_consumed [Watts] : Instantaneous power consumed by XRS
        instantaneous_data_volume        % instantaneous_data_volume [kb] : Data volume of XRS
        instantaneous_data_generated     % # instantaneous_data_generated [kb] : Data generated during current time step
        measurement_frequency            % measurement_frequency [Hz] : How many measurements per seconds are possible?
        measurement_wait_time            % # measurement_wait_time [sec] = (1 / measurement_frequency)
        accumulated_wait_time            % # accumulated_wait_time = 0 [sec]
        take_measurement                 % take_measurement [Boolean] : 1 = Take measurement this time step
        location                         % Location of sensor, in body frame B
        orientation                      % orientation [unit vector] : Normal vector of measuring element in body frame B
        measurement_available            % # measurement_available [Boolean] : 1 = Measurement taken this time step
        measurement_time                 % # measurement_time [sec] : Time when latest Measurement was taken

        desired_attitude_for_SB_achieved
    end
    
    methods
        
        function obj = True_SC_XRS(mission_true_time,sc_body_init_data)
            
            obj.time = mission_true_time.time;
            
            obj.name = ['XRS'];
            obj.health = 1;
            obj.temperature = 10;
            obj.instantaneous_power_consumption = sc_body_init_data.xrs_power_consumption;
            obj.instantaneous_power_consumed = 0;
            obj.instantaneous_data_volume = sc_body_init_data.xrs_data_volume;
            obj.instantaneous_data_generated = 0; % [kilo bits]
            obj.measurement_frequency = sc_body_init_data.xrs_measurement_frequency;
            obj.measurement_wait_time = 1/obj.measurement_frequency; % [sec] between to measurement
            obj.accumulated_wait_time = 0; % [sec]
            obj.take_measurement = 1; % [Boolean] : 1 = Take measurement this time step
            obj.location = sc_body_init_data.xrs_location; % [m] : Location of sensor, in body frame B
            obj.orientation = sc_body_init_data.xrs_orientation; % [m] : Location of sensor, in body frame B
            obj.measurement_available = 0; % [Boolean] : 1 = Measurement taken this time step
            obj.measurement_time = 0; % [sec] : Time when latest Measurement was taken

            obj.desired_attitude_for_SB_achieved = 0; 
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

        function obj = func_check_attitude_for_comm(obj,SC_control_attitude,SC_estimate_attitude)

            obj.desired_attitude_for_SB_achieved = 0;

            % Check desired attitude for SB pointing is achieved
            if (SC_control_attitude.desired_SC_attitude_mode == 1) && (norm(SC_control_attitude.desired_attitude - SC_estimate_attitude.attitude) <= 0.1) 
                obj.desired_attitude_for_SB_achieved = 1;
            else
                obj.desired_attitude_for_SB_achieved = 0;
            end

        end
        
        function obj = func_get_xrs_measurement(obj, True_time)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            
            obj = func_update_accumulated_wait_time(obj, True_time);    % update time
            obj = func_check_wait_time(obj);                            % check for measurement trigerring
            
            if (obj.take_measurement == 1) && (obj.desired_attitude_for_SB_achieved == 1)
                obj.measurement_time = obj.time;
                obj.measurement_available = 1;
                obj.take_measurement = 0;
                obj.instantaneous_data_generated = obj.instantaneous_data_volume;
                obj.instantaneous_power_consumed = obj.instantaneous_power_consumption;
            else
                obj.measurement_available = 0;
                obj.instantaneous_data_generated = 0;
                obj.instantaneous_power_consumed = 0;
            end
            
        end
        
    end
    
end

