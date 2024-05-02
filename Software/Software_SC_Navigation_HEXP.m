classdef Software_SC_Navigation_HEXP < handle
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
        
        Moon_position
        Moon_velocity
        
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
        time_prev
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
        function obj = Software_SC_Navigation_HEXP(True_SC_Body, True_SC_Navigation,True_time,True_Small_Body, True_Solar_System)
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
            
            obj.time_prev = True_time.time;
            
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
            
            obj.Moon_position = True_Solar_System.position_Moon;
            obj.Moon_velocity = True_Solar_System.velocity_Moon;
        end
        
        function obj = set_time(obj, time)
            obj.time = time;
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
            
            obj.Moon_position = True_Solar_System.position_Moon;
            obj.Moon_velocity = True_Solar_System.velocity_Moon;
            
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
        
    end
end

