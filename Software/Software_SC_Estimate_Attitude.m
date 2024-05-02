classdef Software_SC_Estimate_Attitude < handle
    %SC_ESTIMATE_ATTITUDE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % static parameters
        moment_of_inertia
        moment_of_inertia_units
        
        
        % dynamic parameters
        time
        time_units
        attitude
        attitude_uncertainty
        attitude_units
        angular_velocity
        angular_velocity_uncertainty
        angular_velocity_units
        rotation_matrix
        rotation_matrix_units
        dot_angular_velocity
        
        % boolean parameters
        get_imu_measurement
        get_star_tracker_measurement
        get_sun_sensor_measurement
        
        measurement_available_imu
        measurement_available_star_tracker
        measurement_available_sun_sensor
        
        % wait times
        wait_time_imu
        wait_time_star_tracker
        wait_time_sun_sensor
        wait_time_units
        
        % previous values
        prev_time
        x_km1_km1
        P_km1_km1
        x_Km1
        P_Km1
        x_K
        P_K
    end
    
    methods
        
        function obj = Software_SC_Estimate_Attitude(mission_true_SC,mission_true_time)
            %SC_ESTIMATE_ATTITUDE Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = -inf;
            obj.time_units = 'sec';
            obj.moment_of_inertia_units = 'kg m^2';
            obj.attitude_units = 'quaternion (4-scalar)';
            obj.angular_velocity_units = 'rad/sec';
            obj.wait_time_units = 'sec';
            obj.rotation_matrix_units = 'Rotation matrix';
            
            obj.wait_time_imu = 0;
            obj.wait_time_star_tracker = 1/5; % [sec]
            obj.wait_time_sun_sensor = 1/10; % [sec]
            obj.moment_of_inertia = mission_true_SC.true_SC_adc.moment_of_intertia.total_MI;
            
            % Initialize KF
            obj.x_km1_km1 = [
                mission_true_SC.true_SC_adc.angular_velocity;
                mission_true_SC.true_SC_adc.attitude;
                ] + [ ...
                mission_true_SC.true_SC_imu_sensor.imu_sensor_data(1).measurement_noise*randn(1,3), ...
                mission_true_SC.true_SC_sun_sensor.sun_sensor_data(1).measurement_noise*randn(1,4), ...
                ]';
            
            % Quaternion
            obj.x_km1_km1(4:7) = obj.x_km1_km1(4:7)/norm(obj.x_km1_km1(4:7));
            obj.x_K = obj.x_km1_km1;
            
            obj.P_km1_km1 = zeros(7,7);
            if mission_true_SC.true_SC_body.flag_hardware_exists.adc_imu==1
                obj.P_km1_km1(1:3,1:3) = mission_true_SC.true_SC_imu_sensor.imu_sensor_data(1).measurement_noise*eye(3,3);
            end
            if mission_true_SC.true_SC_body.flag_hardware_exists.adc_sun_sensor==1
                obj.P_km1_km1(4:7,4:7) = mission_true_SC.true_SC_sun_sensor.sun_sensor_data(1).measurement_noise*eye(4,4);
            end
            obj.P_K = obj.P_km1_km1;
            
            obj.prev_time = mission_true_time.time;
            
            % Update attitude and angular velocity
            obj.attitude = obj.x_K(4:7);
            obj.angular_velocity = obj.x_K(1:3);
            
            uncertainty_vector = sqrt(diag(obj.P_K));
            obj.attitude_uncertainty = uncertainty_vector(4:7,1);
            obj.angular_velocity_uncertainty = uncertainty_vector(1:3,1);
            
            obj.dot_angular_velocity = [0 0 0]';
            
            meas_rw_momentum = mission_true_SC.true_SC_rwa_actuator.func_calc_meas_rw_momentum();
            obj = func_compute_rotation_matrix(obj, mission_true_SC.true_SC_adc, meas_rw_momentum);
        end
        
        function obj = func_update_attitude_with_truth_data(obj,True_SC_adc)
            
            % Update attitude and angular velocity
            obj.attitude = True_SC_adc.attitude;
            obj.angular_velocity = True_SC_adc.angular_velocity;
            obj.dot_angular_velocity = True_SC_adc.dot_angular_velocity;
            
            obj.attitude_uncertainty = 1e-4*ones(4,1);
            obj.angular_velocity_uncertainty = 1e-5*ones(3,1);
            
        end
        
        function obj = func_start_getting_sensor_measurements(obj,True_SC_IMU_Sensor,SC_star_tracker_sensor,True_SC_Sun_Sensor)
            
            obj.measurement_available_imu = 0;
            obj.measurement_available_star_tracker = 0;
            obj.measurement_available_sun_sensor = 0;
            
            if (obj.time - True_SC_IMU_Sensor.measurement_time) >= obj.wait_time_imu
                obj.get_imu_measurement = 1;
            else
                obj.get_imu_measurement = 0;
            end
            
            if (obj.time - SC_star_tracker_sensor.measurement_time) >= obj.wait_time_star_tracker
                obj.get_star_tracker_measurement = 1;
            else
                obj.get_star_tracker_measurement = 0;
            end
            
            if (obj.time - True_SC_Sun_Sensor.measurement_time) >= obj.wait_time_sun_sensor
                obj.get_sun_sensor_measurement = 1;
            else
                obj.get_sun_sensor_measurement = 0;
            end
            
        end
        
        function obj = func_KF_propagation_step(obj,SC_control_attitude,SC_micro_thruster,SC_executive)
            
            % Generate Discrete Dynamics Equations
            old_omega = obj.x_km1_km1(1:3);
            J_omega_bar = obj.moment_of_inertia*old_omega;
            S_J_omega_bar = skew(J_omega_bar);
            
            Y_omega_bar = 0.5*[
                0,             +old_omega(3), -old_omega(2), old_omega(1);
                -old_omega(3),             0, +old_omega(1), old_omega(2);
                +old_omega(2), -old_omega(1),             0, old_omega(3);
                -old_omega(1), -old_omega(2), -old_omega(3),            0];
            
            A = zeros(7,7);
            A(1:3,1:3) = inv(obj.moment_of_inertia)*S_J_omega_bar;
            A(4:7,4:7) = Y_omega_bar;
            
            B = zeros(7,3);
            B(1:3,1:3) = inv(obj.moment_of_inertia)*eye(3,3);
            
            time_step = obj.time - obj.prev_time;
            
            F_k = expm(A*time_step);
            Q_k = zeros(7,7);
            
            % Direct integration for Singular A
            sum_G_k = zeros(7,3);
            d_tau = time_step/100;
            for tau = d_tau:d_tau:time_step
                sum_G_k = sum_G_k + expm(A*(time_step-tau))*B*d_tau;
            end
            G_k = sum_G_k;
            
            % Prediction Step
            obj.x_Km1 = F_k*obj.x_km1_km1 + G_k*SC_control_attitude.control_torque;
            obj.P_Km1 = F_k*obj.P_km1_km1*F_k' + Q_k;
            
        end
        
        function obj = func_KF_update_step(obj,True_SC_IMU_Sensor,True_SC_Sun_Sensor,True_SC_Star_Tracker_Sensor)
            
            if (obj.measurement_available_imu + obj.measurement_available_star_tracker + obj.measurement_available_sun_sensor) > 0
                
                % Update Step
                H = [];
                z = [];
                R = [];
                
                H_IMU = [eye(3,3), zeros(3,4)];
                R_IMU = True_SC_IMU_Sensor.imu_sensor_data(1).measurement_noise*eye(3,3);
                
                H_ST = [zeros(4,3), eye(4,4)];
                R_ST = True_SC_Star_Tracker_Sensor.star_tracker_data(1).measurement_noise*eye(4,4);
                
                H_SS = [zeros(4,3), eye(4,4)];
                R_SS = True_SC_Sun_Sensor.sun_sensor_data(1).measurement_noise*eye(4,4);
                
                if obj.measurement_available_imu == 1
                    H = [H; H_IMU];
                    z = [z; True_SC_IMU_Sensor.imu_sensor_data(1).measurement_vector];
                    R = blkdiag(R, R_IMU);
                end
                
                if obj.measurement_available_star_tracker == 1
                    H = [H; H_ST];
                    z = [z; True_SC_Star_Tracker_Sensor.star_tracker_data(1).measurement_vector];
                    R = blkdiag(R, R_ST);
                    
                end
                
                if obj.measurement_available_sun_sensor == 1
                    H = [H; H_SS];
                    z = [z; True_SC_Sun_Sensor.sun_sensor_data(1).measurement_vector];
                    R = blkdiag(R, R_SS);
                end
                
                y = z - H*obj.x_Km1;
                
                S = H * obj.P_Km1 * H' + R;
                K = obj.P_Km1 * H' / S;
                J = eye(7,7) - K * H;
                
                obj.x_K = obj.x_Km1 + K * y;
                % obj.P_K = (eye(7,7) - K*H)*obj.P_Km1;
                obj.P_K = J * obj.P_K * J' + K * R * K';
                
                % Normalizing quaternion estimates
                obj.x_K(4:7) = obj.x_K(4:7)/norm(obj.x_K(4:7));
                
                
            else
                % No measurements
                obj.x_K = obj.x_Km1;
                obj.P_K = obj.P_Km1;
                
            end
            
            % Update attitude and angular velocity
            obj.attitude = obj.x_K(4:7);
            obj.angular_velocity = obj.x_K(1:3);
            
            uncertainty_vector = sqrt(diag(obj.P_K));
            obj.attitude_uncertainty = uncertainty_vector(4:7,1);
            obj.angular_velocity_uncertainty = uncertainty_vector(1:3,1);
            
            % Reseting old values for next KF iteration
            obj.x_km1_km1 = obj.x_K;
            obj.P_km1_km1 = obj.P_K;
            obj.prev_time = obj.time;
            
        end
        
        
        function obj = func_compute_rotation_matrix(obj,SC_control_attitude, meas_rw_momentum)
            
            % Compute Rotation Matrix
            SC_True_e_current = obj.attitude(1:3)/norm(obj.attitude(1:3));
            SC_True_Phi_current = 2*acos(obj.attitude(4)); % [rad]
            obj.rotation_matrix = func_create_Rot_matrix(SC_True_e_current, SC_True_Phi_current);
            
            new_X_Quaternion_Omega_current = [obj.attitude; obj.angular_velocity];
            X_dot = func_attitude_Quaternion_spacecraft_v2(obj.time,new_X_Quaternion_Omega_current,obj.moment_of_inertia,SC_control_attitude.control_torque,[0 0 0]',meas_rw_momentum);
            obj.dot_angular_velocity = X_dot(1:3);
            
        end
        
    end
end

