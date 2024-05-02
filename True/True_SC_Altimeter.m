classdef True_SC_Altimeter < handle
    %TRUE_TIME Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        time
        
        health
        temperature
        
        % power & data
        instantaneous_power_consumed
        instantaneous_power_consumption
        instantaneous_data_volume
        instantaneous_data_generated
        
        measurement_frequency
        measurement_wait_time
        measurement_time
        accumulated_wait_time
        take_measurement
        
        orientation
        
        measurement_vector
        measurement_available
        
        % Points mesh
        mesh_selection
        num_points
        points
        observed_points
        num_point_observed
        
        estimated_SB_radius
        
        desired_attitude_for_SB_achieved
    end
    
    methods
        
        function obj = True_SC_Altimeter(sc_body_init_data,mission_true_time)
            
            obj.time = mission_true_time.time;
            
            obj.health = 1; % 1 = ok
            obj.temperature = 20;  % [deg]
            
            % power & data
            obj.instantaneous_power_consumed = 0;
            obj.instantaneous_power_consumption = sc_body_init_data.altimeter_power_consumption;
            obj.instantaneous_data_volume = sc_body_init_data.altimeter_data_volume;
            obj.instantaneous_data_generated = 0;
            
            
            obj.measurement_frequency = sc_body_init_data.altimeter_measurement_frequency;
            obj.measurement_wait_time = 1/obj.measurement_frequency;
            obj.accumulated_wait_time = 0;
            obj.take_measurement = 0;
            obj.measurement_time = 0;
            
            obj.orientation = sc_body_init_data.altimeter_orientation;
            
            obj.measurement_vector = [0 0];
            obj.measurement_available = 0;
            
            % load the science points
            obj.mesh_selection = sc_body_init_data.altimeter_mesh_selection;
            [obj.num_points, obj.points] = func_load_science_points(obj.mesh_selection);
            obj.observed_points = zeros(obj.num_points,2);
            obj.num_point_observed = 0;
            
            obj.estimated_SB_radius = zeros(obj.num_points,1);
            
            obj.desired_attitude_for_SB_achieved = 0;
            
        end
        
        function func_update_accumulated_wait_time(obj, True_time)
            % Update current time
            % Update accumulated time since last measurement
            
            obj.time = True_time.time;
            
            if obj.health == 1
                obj.accumulated_wait_time = obj.time - obj.measurement_time;
            else
                obj.accumulated_wait_time = 0;
            end
            
        end
        
        function func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time
            
            if obj.accumulated_wait_time >= obj.measurement_wait_time
                obj.take_measurement = 1;
            else
                obj.take_measurement = 0;
            end
            
        end
        
        function func_check_attitude_for_comm(obj,SC_control_attitude,SC_estimate_attitude)
            
            obj.desired_attitude_for_SB_achieved = 0;
            
            % Check desired attitude for SB pointing is achieved
            if (SC_control_attitude.desired_SC_attitude_mode == 1) && (norm(SC_control_attitude.desired_attitude - SC_estimate_attitude.attitude) <= 0.1)
                obj.desired_attitude_for_SB_achieved = 1;
            else
                obj.desired_attitude_for_SB_achieved = 0;
            end
            
            
        end
        
        function func_get_altimeter_measurement(obj, true_time, true_small_body, true_SC_navigation)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            
            obj.func_update_accumulated_wait_time(true_time);    % update time
            obj.func_check_wait_time();                            % check for measurement trigerring
            
            if (obj.take_measurement == 1) && (obj.desired_attitude_for_SB_achieved == 1)
                
                Rotation_matrix = func_rotation_matrix(true_small_body.rotation_rate * true_time.time) * true_small_body.rotation_matrix_pole_RA_Dec;
                
                %%%%%%%%%%%%%% find closest observed point in science mesh points
                % this_pos_points = obj.points * Rotation_matrix';                                 % [km] science mesh point (SB centered)
                this_mesh_norm = normr(obj.points * Rotation_matrix');                                            % [km] points of the science mesh normalized (SB centered)
                SC_pos_sb = true_SC_navigation.position_relative_SB;                                % [km] SC position in SB frame (SB centered)
                SC_pos_sb_norm = SC_pos_sb/norm(SC_pos_sb);                                         % [km] SC position in SB frame normalized (SB centered)
                dot_prod_angle_array = real(acosd(this_mesh_norm * SC_pos_sb_norm));                % [deg] angle between the vectors SC-SB and mesh_point-SB
                [M_angle,index_point_science_mesh] = min(dot_prod_angle_array);                     % find the closest
                closest_mesh_point_norm = this_mesh_norm(index_point_science_mesh,:);               % OBSERVED POINT : closest point in the loaded science mesh from the SC-SB vector
                
                %%%%%%%%%%%%%% find closest true SB vertices from the observed science mesh point
                SB_vertices = (Rotation_matrix * true_small_body.shape_model.Vertices')';                               % [km] true SB vertices om J2000 frame
                vertex_norm = normr(SB_vertices);                                                                       % [km] true SB vertices normalized
                dot_prod_angle_SB_shape = real(acosd(vertex_norm*closest_mesh_point_norm'));                            % [deg] angle between the vectors SC-SB and mesh_point-SB
                [M_angle,index_point_SB_shape] = min(dot_prod_angle_SB_shape);                                          % index of closest point for TRUE SB vertices
                closest_true_vertex = SB_vertices(index_point_SB_shape,:)*(1e-3);                                       % CLOSEST TRUE VERTEX
                
                distance_SC_from_surface = norm(SC_pos_sb-closest_true_vertex');                                        % Distance measurement
                
                obj.measurement_vector = [distance_SC_from_surface, index_point_science_mesh];
                
                
                if obj.observed_points(index_point_science_mesh,1) == 0                                                 % store measurement if not already observed
                    % if this points has not already been observed
                    obj.num_point_observed = obj.num_point_observed + 1;
                    obj.observed_points(index_point_science_mesh,:) = obj.measurement_vector;
                    obj.estimated_SB_radius(index_point_science_mesh) = norm(SC_pos_sb)-distance_SC_from_surface;       % estimate SB radius from that POV (used for plot)
                end
                
                
                % Update routine flag, time, power, data
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

