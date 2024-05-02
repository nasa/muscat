classdef True_Spacecraft < handle
    properties
        % Spacecraft ID
        id
        
        % Sensors
        true_SC_sun_sensor
        true_SC_star_tracker_sensor
        true_SC_imu_sensor
        true_SC_onboard_clock_sensor
        true_SC_camera_sensor
        true_SC_gps_receiver
        true_SC_body
        true_SC_navigation
        true_SC_adc
        true_SC_gnss_receiver
        true_SC_metrology
        
        % Data
        true_SC_data_handling
        true_SC_onboard_memory
        
        % Force
        true_SC_gravity_gradient
        true_SC_srp
        
        % Power
        true_SC_solar_panel
        true_SC_power
        true_SC_battery
        
        % Payload
        true_SC_science_camera
        true_SC_altimeter
        true_SC_radar
        true_SC_radio
        true_SC_telescope
        
        % Communication
        true_SC_communication
        true_earth_dte_communication
        
        % Actuators
        true_SC_micro_thruster_actuator
        true_SC_rwa_actuator
        true_SC_chemical_thruster_actuator
        
        % Software
        software_SC_communication
        software_SC_executive
        software_SC_estimate_attitude
        software_SC_estimate_SC_SB_orbit
        software_SC_control_attitude
        software_SC_control_orbit
    end
    
    methods
        function obj = True_Spacecraft(id)
            obj.id = id;
        end
    end
end
