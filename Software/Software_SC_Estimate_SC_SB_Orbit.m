classdef Software_SC_Estimate_SC_SB_Orbit < handle
    %SC_ESTIMATE_SB_SC_ORBITS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % dynamic parameters
        time
        time_units
        
        SC_position
        SC_velocity
        SB_position
        SB_velocity
        
        SC_position_relative_SB % [km] : Current position of SC relative to SB-center J2000 inertial frame
        SC_velocity_relative_SB % [km/sec] : Current velocity of SC relative to SB-center J2000 inertial frame
        
        SC_position_uncertainty
        SC_velocity_uncertainty
        SB_position_uncertainty
        SB_velocity_uncertainty
        
        SC_position_relative_SB_uncertainty
        SC_velocity_relative_SB_uncertainty
        
        
        Sun_position
        Sun_velocity
        
        Earth_position
        Earth_velocity
        
        position_units
        velocity_units
        
        factor
        SC_position_measurement
        SC_velocity_measurement
        SC_measurement_available
        SB_position_measurement
        SB_velocity_measurement
        SB_measurement_available
        
        measurement_noise_position
        measurement_noise_velocity
        
        % previous values
        prev_time
        x_km1_km1
        P_km1_km1
        x_k_km1
        P_k_km1
        x_k_k
        P_k_k
        
        mu_SB
        mu_Sun
        mass
        SB_radius
    end
    
    methods
        function obj = Software_SC_Estimate_SC_SB_Orbit(True_SC_Body, True_SC_Navigation,True_time,True_Small_Body, True_Solar_System)
            %SC_ESTIMATE_SB_SC_ORBITS Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = -inf;
            obj.time_units = 'sec';
            obj.position_units = 'km';
            obj.velocity_units = 'km/sec';
            
            % Orbit KF values
            v_SC = True_SC_Navigation.velocity + 0.01*randn(3,1);
            p_SC = True_SC_Navigation.position + 1*randn(3,1);
            
            v_SB = True_Small_Body.velocity_SB + 0.01*randn(3,1);
            p_SB = True_Small_Body.position_SB + 1*randn(3,1);
            
            obj.x_km1_km1 = [v_SC; p_SC; v_SB; p_SB];
            obj.x_k_k = obj.x_km1_km1;
            
            obj.P_km1_km1 = zeros(12,12);
            obj.P_km1_km1(1:3,1:3) = 0.01*eye(3,3);
            obj.P_km1_km1(4:6,4:6) = 1*eye(3,3);
            obj.P_km1_km1(7:9,7:9) = 0.01*eye(3,3);
            obj.P_km1_km1(10:12,10:12) = 1*eye(3,3);
            obj.P_k_k = obj.P_km1_km1;
            
            obj.prev_time = True_time.time;
            
            obj.mu_SB = True_Small_Body.mu_SB;
            obj.mu_Sun = True_Solar_System.mu_Sun;
            obj.mass = True_SC_Body.mass;
            obj.SB_radius = True_Small_Body.radius;
            
            obj.factor = 0.001;
            
            % Update SC and SB positions
            obj.SC_velocity = obj.x_k_k(1:3,1);
            obj.SC_position = obj.x_k_k(4:6,1);
            obj.SB_velocity = obj.x_k_k(7:9,1);
            obj.SB_position = obj.x_k_k(10:12,1);
            
            obj.SC_position_relative_SB = obj.SC_position - obj.SB_position;
            obj.SC_velocity_relative_SB = obj.SC_velocity - obj.SB_velocity;
            
            uncertainty_vector = sqrt(diag(obj.P_k_k));
            obj.SC_velocity_uncertainty = uncertainty_vector(1:3,1);
            obj.SC_position_uncertainty = uncertainty_vector(4:6,1);
            obj.SB_velocity_uncertainty = uncertainty_vector(7:9,1);
            obj.SB_position_uncertainty = uncertainty_vector(10:12,1);
            
            obj.SC_position_relative_SB_uncertainty = obj.SC_position_uncertainty + obj.SB_position_uncertainty;
            obj.SC_velocity_relative_SB_uncertainty = obj.SC_velocity_uncertainty + obj.SB_velocity_uncertainty;
            
            obj.Sun_position = True_Solar_System.position_Sun;
            obj.Sun_velocity = True_Solar_System.velocity_Sun;
            
            obj.Earth_position = True_Solar_System.position_Earth;
            obj.Earth_velocity = True_Solar_System.position_Earth;
            
        end
        
        function obj = func_update_orbit_with_truth_data(obj,True_SC_Navigation, True_Small_Body, True_Solar_System)
            
            obj.SC_position = True_SC_Navigation.position;
            obj.SC_velocity = True_SC_Navigation.velocity;
            
            obj.SB_position = True_Small_Body.position_SB;
            obj.SB_velocity = True_Small_Body.velocity_SB;
            
            obj.Sun_position = True_Solar_System.position_Sun;
            obj.Sun_velocity = True_Solar_System.velocity_Sun;
            
            obj.Earth_position = True_Solar_System.position_Earth;
            obj.Earth_velocity = True_Solar_System.position_Earth;
            
            obj.SC_position_relative_SB = True_SC_Navigation.position_relative_SB;
            obj.SC_velocity_relative_SB = True_SC_Navigation.velocity_relative_SB;
            
            % Update SC and SB positions
            
            obj.SC_velocity_uncertainty = 1e-5*ones(3,1); % [km/sec]
            obj.SC_position_uncertainty = 1e-3*ones(3,1); % [km]
            obj.SB_velocity_uncertainty = 1e-5*ones(3,1); % [km/sec]
            obj.SB_position_uncertainty = 1e-3*ones(3,1); % [km]
            
            obj.SC_position_relative_SB_uncertainty = obj.SC_position_uncertainty + obj.SB_position_uncertainty;
            obj.SC_velocity_relative_SB_uncertainty = obj.SC_velocity_uncertainty + obj.SB_velocity_uncertainty;
            
        end
        
        function obj = func_get_orbit_measurement(obj,True_SC_body,SC_NAC_camera,SC_WAC_camera,SC_executive)
            
            obj.SC_measurement_available = 0;
            obj.SB_measurement_available = 0;
            
            obj.Sun_position = True_SC_body.Sun_pos;
            obj.Sun_velocity = True_SC_body.Sun_vel;
            
            distance_SB_SC = norm(True_SC_body.position - True_SC_body.SB_pos);
            relative_velocity_SB_SC = norm(True_SC_body.velocity - True_SC_body.SB_vel);
            
            obj.measurement_noise_position = obj.factor*distance_SB_SC;
            obj.measurement_noise_velocity = obj.factor*relative_velocity_SB_SC;
            
            if (SC_executive.get_camera_image == 1) && ((SC_NAC_camera.health == 1) || (SC_WAC_camera.health == 1))
                
                if (SC_NAC_camera.SB_visible == 1) || (SC_WAC_camera.SB_visible == 1)
                    % SB is visible in Camera image
                    
                    obj.SC_position_measurement = True_SC_body.position + obj.measurement_noise_position*randn(3,1);
                    obj.SC_velocity_measurement = True_SC_body.velocity + obj.measurement_noise_velocity*randn(3,1);
                    
                    obj.SB_position_measurement = True_SC_body.SB_pos + obj.measurement_noise_position*randn(3,1);
                    obj.SB_velocity_measurement = True_SC_body.SB_vel + obj.measurement_noise_velocity*randn(3,1);
                    
                    obj.SC_measurement_available = 1;
                    obj.SB_measurement_available = 1;
                    
                else
                    % SB is not visible in Camera image
                    
                    obj.SC_position_measurement = True_SC_body.position + obj.measurement_noise_position*randn(3,1);
                    obj.SC_velocity_measurement = True_SC_body.velocity + obj.measurement_noise_velocity*randn(3,1);
                    
                    obj.SC_measurement_available = 1;
                    obj.SB_measurement_available = 0;
                    
                end
                
            end
        end
        
        function obj = func_orbit_KF_propagation_step(obj,SC_chem_thruster,SC_estimate_attitude,SC_micro_thruster)
            
            d_SUN_SC = norm(obj.SC_position - obj.Sun_position);
            d_SB_SC  = norm(obj.SC_position - obj.SB_position);
            d_SUN_SB = norm(obj.SB_position - obj.Sun_position);
            
            A = zeros(12,12);
            A(1:3,4:6)   = -((obj.mu_Sun/(d_SUN_SC^3)) + (obj.mu_SB/(d_SB_SC^3)))*eye(3,3);
            A(1:3,10:12) = +(obj.mu_SB/(d_SB_SC^3))*eye(3,3);
            A(4:6,1:3)   = eye(3,3);
            A(7:9,10:12) = -(obj.mu_Sun/(d_SUN_SB^3))*eye(3,3);
            A(10:12,7:9) = eye(3,3);
            
            B = zeros(12,3);
            B(1:3,1:3) = 1e-3*(1/obj.mass)*eye(3,3);
            
            time_step = obj.time - obj.prev_time;
            
            F_k = expm(A*time_step);
            
            Q_k = zeros(12,12);
            %             Q_k(1:3,1:3) = 0.1*obj.measurement_noise_velocity*eye(3,3);
            %             Q_k(4:6,4:6) = 0.1*obj.measurement_noise_position*eye(3,3);
            %             Q_k(7:9,7:9) = 0.1*obj.measurement_noise_velocity*eye(3,3);
            %             Q_k(10:12,10:12) = 0.1*obj.measurement_noise_position*eye(3,3);
            Q_k(1:3,1:3) = Q_k(1:3,1:3) + SC_micro_thruster.thruster_noise*eye(3,3);
            Q_k(7:9,7:9) = Q_k(7:9,7:9) + SC_micro_thruster.thruster_noise*eye(3,3);
            
            if SC_chem_thruster.command_thrust > 0
                
                % Direct integration for Singular A
                sum_G_k = zeros(12,3);
                d_tau = time_step/100;
                for tau = d_tau:d_tau:time_step
                    sum_G_k = sum_G_k + expm(A*(time_step-tau))*B*d_tau;
                end
                G_k = sum_G_k;
                
                %             % Compute Rotation Matrix
                %             SC_Estimated_e_current = SC_estimate_attitude.attitude(1:3)/norm(SC_estimate_attitude.attitude(1:3));
                %             SC_Estimated_Phi_current = 2*acos(SC_estimate_attitude.attitude(4)); % [rad]
                %             Rot_mat_Estimated_current = func_create_Rot_matrix(SC_Estimated_e_current, SC_Estimated_Phi_current);
                %             Rot_mat_Estimated_current = SC_estimate_attitude.rotation_matrix;
                
                control_thrust_vector = SC_chem_thruster.command_thrust*(SC_estimate_attitude.rotation_matrix*SC_chem_thruster.direction');
                
                Q_k(1:3,1:3) = Q_k(1:3,1:3) + SC_chem_thruster.noise*eye(3,3);
                
                % Prediction Step
                obj.x_k_km1 = F_k*obj.x_km1_km1 + G_k*control_thrust_vector;
                obj.P_k_km1 = F_k*obj.P_km1_km1*F_k' + Q_k;
                
            else
                
                % Prediction Step
                obj.x_k_km1 = F_k*obj.x_km1_km1;
                obj.P_k_km1 = F_k*obj.P_km1_km1*F_k' + Q_k;
                
            end
            
            
        end
        
        function obj = func_orbit_KF_update_step(obj)
            
            if (obj.SC_measurement_available + obj.SB_measurement_available) > 0
                
                % Update Step
                H_k = [];
                z_k = [];
                R_k = [];
                
                if obj.SB_measurement_available == 1
                    H_k = eye(12,12);
                    
                    z_k = [obj.SC_velocity_measurement; obj.SC_position_measurement; obj.SB_velocity_measurement; obj.SB_position_measurement];
                    
                    R_k = zeros(12,12);
                    R_k(1:3,1:3) = obj.measurement_noise_velocity*eye(3,3);
                    R_k(4:6,4:6) = obj.measurement_noise_position*eye(3,3);
                    R_k(7:9,7:9) = obj.measurement_noise_velocity*eye(3,3);
                    R_k(10:12,10:12) = obj.measurement_noise_position*eye(3,3);
                    
                else
                    H_k = zeros(6,12);
                    H_k(1:3,1:3) = eye(3,3);
                    H_k(4:6,4:6) = eye(3,3);
                    
                    z_k = [obj.SC_velocity_measurement; obj.SC_position_measurement];
                    
                    R_k = zeros(6,6);
                    R_k(1:3,1:3) = obj.measurement_noise_velocity*eye(3,3);
                    R_k(4:6,4:6) = obj.measurement_noise_position*eye(3,3);
                    
                end
                
                y_k = z_k - H_k*obj.x_k_km1;
                
                S_k = H_k*obj.P_k_km1*H_k' + R_k;
                K_k = obj.P_k_km1*H_k'*inv(S_k);
                
                obj.x_k_k = obj.x_k_km1 + K_k*y_k;
                obj.P_k_k = (eye(12,12) - K_k*H_k)*obj.P_k_km1;
                
            else
                % No measurements
                obj.x_k_k = obj.x_k_km1;
                obj.P_k_k = obj.P_k_km1;
            end
            
            % Update SC and SB positions
            obj.SC_velocity = obj.x_k_k(1:3,1);
            obj.SC_position = obj.x_k_k(4:6,1);
            obj.SB_velocity = obj.x_k_k(7:9,1);
            obj.SB_position = obj.x_k_k(10:12,1);
            
            obj.SC_position_relative_SB = obj.SC_position - obj.SB_position;
            obj.SC_velocity_relative_SB = obj.SC_velocity - obj.SB_velocity;
            
            uncertainty_vector = sqrt(diag(obj.P_k_k));
            obj.SC_velocity_uncertainty = uncertainty_vector(1:3,1);
            obj.SC_position_uncertainty = uncertainty_vector(4:6,1);
            obj.SB_velocity_uncertainty = uncertainty_vector(7:9,1);
            obj.SB_position_uncertainty = uncertainty_vector(10:12,1);
            
            obj.SC_position_relative_SB_uncertainty = obj.SC_position_uncertainty + obj.SB_position_uncertainty;
            obj.SC_velocity_relative_SB_uncertainty = obj.SC_velocity_uncertainty + obj.SB_velocity_uncertainty;
            
            % Reseting old values for next KF iteration
            obj.x_km1_km1 = obj.x_k_k;
            obj.P_km1_km1 = obj.P_k_k;
            obj.prev_time = obj.time;
            
        end
    end
end

