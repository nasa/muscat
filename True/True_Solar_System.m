classdef True_Solar_System < handle
    %TRUE_SOLAR_SYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mu_Sun % [km^3 sec^-2] : Sun’s standard gravitational parameter μ = GM
        position_Sun % [km] : Current position of Sun in inertial frame I. (Note that this should be [0, 0, 0]T .)
        velocity_Sun % [km/sec] : Current velocity of Sun in inertial frame I. (Note that this should be [0, 0, 0]T .)
        
        mu_Earth % [km^3 sec^-2] : Earth’s standard gravitational parameter μ = GM
        position_Earth % [km] : Current position of Earth in inertial frame I
        velocity_Earth % [km/sec] : Current velocity of Earth in inertial frame I
        
        mu_Moon % [km^3 sec^-2] : Moon’s standard gravitational parameter μ = GM
        position_Moon % [km] : Current position of Moon in inertial frame I
        velocity_Moon % [km/sec] : Current velocity of Moon in inertial frame I
        
        solar_constant_AU % [W/m^2]
        AU_distance % [km]
        light_speed % [m/sec]
    end
    
    methods
        function obj = True_Solar_System(mission_true_time)
            %TRUE_SOLAR_SYSTEM Construct an instance of this class
            
            obj.mu_Sun = 1.32712440018e11; % [km^3 sec^-2]
            obj.mu_Earth = 3.986004418e5; % [km^3 sec^-2]
            obj.mu_Moon = 4.902799e3; % [km^3 sec^-2]
            
            obj.solar_constant_AU = 1361; % [W/m^2]
            obj.AU_distance = 1.49598e8; % [km]
            obj.light_speed = 299792458; % [m/sec]
            
            cspice_furnsh('../MuSCAT_Supporting_Files/SPICE/de440s.bsp')
            
            obj = obj.func_update_Sun_Earth_position_velocity(mission_true_time);
        end
        
        function obj = func_update_Sun_Earth_position_velocity(obj,mission_true_time)
            % Function to update position_Sun, position_Earth, ... given current time
            
            Sun_pos_vel_this_time = cspice_spkezr('SUN',mission_true_time.date,'J2000','NONE','SUN');
            obj.position_Sun = Sun_pos_vel_this_time(1:3);
            obj.velocity_Sun = Sun_pos_vel_this_time(4:6);
            
            Earth_pos_vel_this_time = cspice_spkezr('EARTH',mission_true_time.date,'J2000','NONE','SUN');
            obj.position_Earth = Earth_pos_vel_this_time(1:3);
            obj.velocity_Earth = Earth_pos_vel_this_time(4:6);
            
            Moon_pos_vel_this_time = cspice_spkezr('MOON',mission_true_time.date,'J2000','NONE','SUN');
            obj.position_Moon = Moon_pos_vel_this_time(1:3);
            obj.velocity_Moon = Moon_pos_vel_this_time(4:6);
        end
        
    end
end
