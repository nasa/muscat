classdef Software_SC_Estimate_Orbit_Swarm < handle
    %Software_SC_Estimate_Orbit_Swarm Summary of this class goes here
    %   This class estimates the absolute position of the chief spacecraft
    %   and the relative position of the deputy spacecraft with respect to
    %   the chief spacecraft.
    
    properties
        time % [s]
        time_prev % [s]
        time_step % [s]
        
        orbit_det_apps
        measurement_available
        
        % Relative state estimation
        relative_position_velocity
        relative_position_velocity_uncertainty
        
        % Interface
        SC_position
        SC_position_uncertainty
        SC_velocity
        SC_velocity_uncertainty
        SB_position
        SB_position_uncertainty
        SB_velocity
        SB_velocity_uncertainty
        SC_position_relative_SB
        SC_position_relative_SB_uncertainty
        SC_velocity_relative_SB
        SC_velocity_relative_SB_uncertainty
        
        Sun_position
        Sun_velocity
        Earth_position
        Earth_velocity
        Moon_position
        Moon_velocity
    end
    
    methods
        function obj = Software_SC_Estimate_Orbit_Swarm(orbit_det_data, sc_id, true_SC_all, true_small_body, true_solar_system, true_time)
            obj.time = -inf;
            obj.time_prev = true_time.time;
            obj.time_step = true_time.time_step;
            
            % Initial position and velocity
            N_sc = numel(true_SC_all);
            rv_true = zeros(N_sc, 6);
            for i_sc = 1:N_sc
                rv_true(i_sc,:) = [
                    true_SC_all{i_sc}.true_SC_navigation.position_relative_SB;
                    true_SC_all{i_sc}.true_SC_navigation.velocity_relative_SB];
            end
            rtn_true = zeros(N_sc - 1, 6);
            for i_sc = 2:N_sc
                rtn_true(i_sc-1,:) = Rv2Rtn(rv_true(1,:)', rv_true(i_sc,:)');
            end
            
            % Orbit determination
            if isfield(orbit_det_data, 'absolute')
                orbit_det_data.absolute.rv_true = rv_true;
                orbit_det_data.absolute.time_step = true_time.time_step;
                orbit_det_data.absolute.sc_id = sc_id;
                obj.orbit_det_apps.absolute = Software_SC_Orbit_Determination(orbit_det_data.absolute, true_small_body, true_time);
            end
            if isfield(orbit_det_data, 'relative')
                orbit_det_data.relative.rtn_true = rtn_true;
                orbit_det_data.relative.time_step = true_time.time_step;
                orbit_det_data.relative.sc_id = sc_id;
                orbit_det_data.relative.rv_chief = obj.orbit_det_apps.absolute.state_estimate(1:6);
                obj.orbit_det_apps.relative = Software_SC_Orbit_Determination(orbit_det_data.relative, true_small_body, true_time);
            end
            
            % Interface
            obj.func_update_with_estimated_data(true_small_body, true_solar_system);
        end
        
        function func_update_orbit_with_truth_data(obj,true_sc_nav, true_small_body, true_solar_system)
            
            obj.update_true_bodies(true_small_body, true_solar_system);
            
            obj.SC_position_relative_SB = true_sc_nav.position_relative_SB;
            obj.SC_velocity_relative_SB = true_sc_nav.velocity_relative_SB;
            obj.SC_position = obj.SB_position + obj.SC_position_relative_SB;
            obj.SC_velocity = obj.SB_velocity + obj.SC_velocity_relative_SB;
            
            % Update SC and SB positions
            obj.SC_velocity_uncertainty = 1e-5*ones(3,1); % [km/sec]
            obj.SC_position_uncertainty = 1e-3*ones(3,1); % [km]
            obj.SB_velocity_uncertainty = 1e-5*ones(3,1); % [km/sec]
            obj.SB_position_uncertainty = 1e-3*ones(3,1); % [km]
            
            obj.SC_position_relative_SB_uncertainty = obj.SC_position_uncertainty + obj.SB_position_uncertainty;
            obj.SC_velocity_relative_SB_uncertainty = obj.SC_velocity_uncertainty + obj.SB_velocity_uncertainty;
            
        end
        
        function func_update_with_estimated_data(obj, true_small_body, true_solar_system)
            
            state_estimate = obj.orbit_det_apps.absolute.state_estimate;
            covariance_estimate = obj.orbit_det_apps.absolute.covariance_estimate;
            
            obj.SC_position_relative_SB = state_estimate(1:3);
            obj.SC_velocity_relative_SB = state_estimate(4:6);
            
            % Update SC and SB positions
            obj.SC_velocity_uncertainty = diag(covariance_estimate(4:6,4:6));
            obj.SC_position_uncertainty = diag(covariance_estimate(1:3,1:3));
            
            if isfield(obj.orbit_det_apps, 'relative')
                obj.relative_position_velocity = obj.orbit_det_apps.relative.state_estimate;
                obj.relative_position_velocity_uncertainty = diag(obj.orbit_det_apps.relative.covariance_estimate);
            end
            
            obj.update_true_bodies(true_small_body, true_solar_system);
            obj.SC_position = obj.SB_position + obj.SC_position_relative_SB;
            obj.SC_velocity = obj.SB_velocity + obj.SC_velocity_relative_SB;
            obj.SB_velocity_uncertainty = zeros(3,1); % [km/sec]
            obj.SB_position_uncertainty = zeros(3,1); % [km]
            obj.SC_position_relative_SB_uncertainty = obj.SC_position_uncertainty + obj.SB_position_uncertainty;
            obj.SC_velocity_relative_SB_uncertainty = obj.SC_velocity_uncertainty + obj.SB_velocity_uncertainty;
            
        end
        
        function update_true_bodies(obj, true_small_body, true_solar_system)
            
            obj.SB_position = true_small_body.position_SB;
            obj.SB_velocity = true_small_body.velocity_SB;
            
            obj.Sun_position = true_solar_system.position_Sun;
            obj.Sun_velocity = true_solar_system.velocity_Sun;
            
            obj.Earth_position = true_solar_system.position_Earth;
            obj.Earth_velocity = true_solar_system.position_Earth;
            
            obj.Moon_position = true_solar_system.position_Moon;
            obj.Moon_velocity = true_solar_system.position_Moon;
        end
        
        function step(obj, true_small_body, true_solar_system)
            if isfield(obj.orbit_det_apps, 'absolute')
                obj.orbit_det_apps.absolute.step(obj.measurement_available);
            end
            if isfield(obj.orbit_det_apps, 'relative')
                obj.orbit_det_apps.relative.config.rv_chief = obj.orbit_det_apps.absolute.state_estimate(1:6);
                obj.orbit_det_apps.relative.step(obj.measurement_available);
            end
            obj.func_update_with_estimated_data(true_small_body, true_solar_system);
        end
        
        function func_get_orbit_measurement(obj, true_sc)
            obj.measurement_available = [];
            if length(true_sc.true_SC_gnss_receiver) >= 1 &&  true_sc.true_SC_gnss_receiver.measurement_available
                obj.measurement_available.gnss_absolute = true_sc.true_SC_gnss_receiver.measurement_absolute;
                obj.measurement_available.gnss_relative = true_sc.true_SC_gnss_receiver.measurement_relative;
            else
                obj.measurement_available.gnss_absolute = [];
                obj.measurement_available.gnss_relative = [];
            end
            if length(true_sc.true_SC_metrology) >= 1 && true_sc.true_SC_metrology.measurement_available
                obj.measurement_available.range = true_sc.true_SC_metrology.measurement_range;
                obj.measurement_available.rangerate = true_sc.true_SC_metrology.measurement_rangerate;
                obj.measurement_available.bearing_angles = true_sc.true_SC_metrology.measurement_bearing_angles;
            else
                obj.measurement_available.range = [];
                obj.measurement_available.rangerate = [];
                obj.measurement_available.bearing_angles = [];
            end
        end % end of func_get_orbit_measurement
        
    end % end of methods
    
end % end of classdef
