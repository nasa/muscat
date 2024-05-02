classdef True_SC_Radar < handle
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
        
        measurement_point
        measurement_available
        
        limb_angle
        
        % Points mesh
        mesh_selection
        num_points
        points
        
        % observation monostatic
        monostatic_observed_point
        monostatic_num_point_observed
        % observation antipodal
        antipodal_observed_point
        antipodal_num_point_observed
        antipodal_angle_array
        antipodal_inter_SC_angle
        % observation bistatic
        bistatic_observed_point
        bistatic_num_point_observed
        
        desired_attitude_for_SB_achieved
    end
    
    methods
        
        function obj = True_SC_Radar(sc_body_init_data,mission_true_time,true_SC_navigation)
            
            obj.time = mission_true_time.time;
            
            obj.health = 1; % 1 = ok
            obj.temperature = 20;  % [deg]
            
            % power & data
            obj.instantaneous_power_consumed = 0;
            obj.instantaneous_power_consumption = sc_body_init_data.radar_power_consumption;
            obj.instantaneous_data_volume = sc_body_init_data.radar_data_volume;
            obj.instantaneous_data_generated = 0;
            
            
            obj.measurement_frequency = sc_body_init_data.radar_measurement_frequency;
            obj.measurement_wait_time = 1/obj.measurement_frequency;
            obj.accumulated_wait_time = 0;
            obj.take_measurement = 0;
            obj.measurement_time = 0;
            
            obj.orientation = sc_body_init_data.radar_orientation;
            
            obj.measurement_point = [0 0 0];
            obj.measurement_available = 0;
            
            % load the science points
            obj.mesh_selection = sc_body_init_data.radar_mesh_selection;
            [obj.num_points, obj.points] = func_load_science_points(obj.mesh_selection);
            
            % limb angle
            obj.limb_angle = 180-2*(90-asind(1/true_SC_navigation.ratio_orbit_radius_SB_radius));
            
            % monostatic data
            obj.monostatic_observed_point = zeros(obj.num_points,1);
            obj.monostatic_num_point_observed = 0;
            
            % antipodal data
            obj.antipodal_angle_array = [1 5 10 15 20 obj.limb_angle];
            obj.antipodal_observed_point = zeros(obj.num_points,length(obj.antipodal_angle_array));
            obj.antipodal_inter_SC_angle = zeros(obj.num_points,length(obj.antipodal_angle_array));
            obj.antipodal_num_point_observed = zeros(size(obj.antipodal_angle_array));
            
            % bistatic data
            obj.bistatic_num_point_observed = 0;
            
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
        
        function func_get_radar_measurement(obj, true_time, true_small_body, true_SC_navigation, number_of_SC, this_SC, mission_true_SC)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            
            obj.func_update_accumulated_wait_time(true_time);    % update time
            obj.func_check_wait_time();                            % check for measurement trigerring
            
            if (obj.take_measurement == 1) && (obj.desired_attitude_for_SB_achieved == 1)
                
                Rotation_matrix = func_rotation_matrix(true_small_body.rotation_rate * true_time.time) * true_small_body.rotation_matrix_pole_RA_Dec;
                
                %%%%%%%%%%%%%% find closest observed point in science mesh points
                this_pos_points =  obj.points * Rotation_matrix';                                 % [km] science mesh point (SB centered)
                this_mesh_norm = this_pos_points; %normr(this_pos_points);                                            % [km] points of the science mesh normalized (SB centered)
                SC_pos_sb = true_SC_navigation.position_relative_SB;                                % [km] SC position in SB frame (SB centered)
                SC_pos_sb_norm = SC_pos_sb/norm(SC_pos_sb);                                         % [km] SC position in SB frame normalized (SB centered)
                dot_prod_angle_array = real(acosd(this_mesh_norm * SC_pos_sb_norm));                % [deg] angle between the vectors SC-SB and mesh_point-SB
                [M_angle,index_point_science_mesh] = min(dot_prod_angle_array);                     % find the closest
                
                
                % MONOSTATIC
                if obj.monostatic_observed_point(index_point_science_mesh,1) == 0                             % store measurement if not already observed
                    % if this points has not already been observed
                    obj.measurement_point = this_mesh_norm(index_point_science_mesh,:);
                    obj.monostatic_num_point_observed = obj.monostatic_num_point_observed + 1;
                    obj.monostatic_observed_point(index_point_science_mesh,:) = index_point_science_mesh;
                end
                
                % ANTIPODAL
                for i_SC=1:number_of_SC
                    
                    if (i_SC ~= this_SC) && (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_radar == 1)
                        % among other SC, do they have a radar ?
                        
                        % compute orbit angular separation
                        other_SC_current_observed_point = mission_true_SC{i_SC}.true_SC_radar.measurement_point;  % point currently observed by other SC
                        interSC_angle = real(acosd(dot(other_SC_current_observed_point,obj.measurement_point'))); % [deg]
                        
                        for i_angle = 1:length(obj.antipodal_angle_array)
                            % parse through the different antipodale angular tolerance
                            if (interSC_angle >= (180-obj.antipodal_angle_array(i_angle))) && (obj.antipodal_observed_point(index_point_science_mesh,i_angle) == 0)
                                % if this interSC angle is in range fot this tolerance, and has not been measured yet, save the measurement
                                obj.antipodal_observed_point(index_point_science_mesh,i_angle) = index_point_science_mesh;
                                obj.antipodal_num_point_observed(i_angle) = obj.antipodal_num_point_observed(i_angle) + 1;
                            end
                        end
                    end
                end
                
                % BISTATIC
                for i_SC=1:number_of_SC
                    
                    if (i_SC ~= this_SC) && (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_radar == 1)
                        % among other SC, do they have a radar ?
                        
                        % compute orbit angular separation
                        other_SC_current_observed_point = mission_true_SC{i_SC}.true_SC_radar.measurement_point;  % point currently observed by other SC
                        interSC_angle = real(acosd(dot(other_SC_current_observed_point,obj.measurement_point'))); % [deg]
                        
                        observed_pair = [index_point_science_mesh, this_SC, i_SC];
                        if obj.bistatic_num_point_observed == 0
                            % first iteration, add the observed pair
                            obj.bistatic_num_point_observed = obj.bistatic_num_point_observed + 1;
                            obj.bistatic_observed_point = observed_pair;
                        else
                            if any(ismember(obj.bistatic_observed_point, observed_pair, 'rows')==1)
                                % this observed pair has already been observed
                            else
                                obj.bistatic_num_point_observed = obj.bistatic_num_point_observed + 1;
                                obj.bistatic_observed_point(obj.bistatic_num_point_observed,:) = observed_pair;
                            end
                        end
                        
                    end
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

