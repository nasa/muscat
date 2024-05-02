classdef True_SC_Science_Camera < handle
    %TRUE_SC_SCIENCE_CAMERA Summary of this class goes here
    %   Detailed explanation goes here

    properties
        time

        % Points mesh
        mesh_selection
        num_points
        points

        % Science Camera Thresholds
        camera_normal_Sun_angle_array     = [0 15 30 45 60 75 90]; % [deg]
        camera_normal_Sun_angle_threshold = 7.5; % [deg]
        camera_normal_SC_angle            = 90; % [deg] Max SC-SB Center-SB Surface angle that camera can observe

        % Storage Array
        first_observed_camera_array
        observed_camera_data
        observed_camera_data_units
    end

    methods
        function obj = True_SC_Science_Camera(sc_body_init_data,mission_true_time)
            %TRUE_SC_SCIENCE_CAMERA Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = mission_true_time.time;

            % load the science points
            obj.mesh_selection = sc_body_init_data.science_camera_mesh_selection;
            [obj.num_points, obj.points] = func_load_science_points(obj.mesh_selection);

            % Storage Array
            obj.first_observed_camera_array = zeros(obj.num_points,3*length(obj.camera_normal_Sun_angle_array)); % [time SC resolution]
    
            for pt_i = 1:1:obj.num_points
                for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)
                    obj.observed_camera_data{pt_i,i_ang}  = [];
                end
            end
            obj.observed_camera_data_units = '[time index_SC image_resolution incidence_Sun_angle emergence_SC_angle phase_Sun_SC_angle index_camera]';

        end

        function obj = func_get_science_camera_measurements(obj, true_time, true_small_body, true_navigation, true_camera_sensor,index_camera, true_solar_system, index_SC)

            obj.time = true_time.time;

            Rotation_matrix = func_rotation_matrix(true_small_body.rotation_rate * true_time.time) * true_small_body.rotation_matrix_pole_RA_Dec;
            this_pos_points = (Rotation_matrix * obj.points')';

            % Check SC - Point angle
            this_SC_pos = true_navigation.position_relative_SB; % [km] SC position in SB frame (SB centered)
            this_SC_pos_normalized = this_SC_pos/norm(this_SC_pos); % normalized

            dot_prod_angle_array_SC_points = real(acosd(this_pos_points * this_SC_pos_normalized)); % [deg]
            logical_points_visible = logical(dot_prod_angle_array_SC_points <= obj.camera_normal_SC_angle);

            % Check FOV
            this_SC_pos_to_point = this_pos_points*true_small_body.radius - this_SC_pos';
            this_SC_pos_to_point_normalized = this_SC_pos_to_point./vecnorm(this_SC_pos_to_point,2,2);

            dot_prod_angle_array_FOV = real(acosd(this_SC_pos_to_point_normalized * (-this_SC_pos_normalized))); % [deg]
            logical_points_within_FOV = logical(dot_prod_angle_array_FOV <= true_camera_sensor.camera_data(index_camera).field_of_view/2);

            % Make list of points
            idx = find(logical_points_visible & logical_points_within_FOV);
            observed_points = [];
            if ~isempty(idx)
                observed_points = idx;
            end

            % Actual Camera Observations
            for pt_i = 1:1:length(observed_points)
                observed_points_camera_i = observed_points(pt_i);
                
                % Check Sun Angle
                this_Sun_pos = true_solar_system.position_Sun - true_small_body.position_SB; % [km]
                this_Sun_pos = this_Sun_pos/norm(this_Sun_pos);
                this_point_pos = this_pos_points(observed_points_camera_i,:);
                this_point_incidence_Sun_angle = real(acosd(dot(this_point_pos, this_Sun_pos))); % [deg]
                
                for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)
                    if (this_point_incidence_Sun_angle >= obj.camera_normal_Sun_angle_array(i_ang)-obj.camera_normal_Sun_angle_threshold) && (this_point_incidence_Sun_angle <= obj.camera_normal_Sun_angle_array(i_ang)+obj.camera_normal_Sun_angle_threshold)
                        % Take this data point!
                        
                        this_SC_pos_to_point = 1e3*norm(this_point_pos*true_small_body.radius - this_SC_pos); % [m]

                        CX_FOV = 2*this_SC_pos_to_point*tand(true_camera_sensor.camera_data(index_camera).field_of_view/2);
                        image_resolution = CX_FOV/true_camera_sensor.camera_data(index_camera).resolution(1); % [m/px]

                        this_point_emergence_SC_angle = real(acosd(dot(this_point_pos, this_SC_pos_normalized))); % [deg]
                        this_point_phase_Sun_SC_angle = real(acosd(dot(this_SC_pos_normalized, this_Sun_pos))); % [deg]
                    
                        this_data_point = [obj.time index_SC image_resolution this_point_incidence_Sun_angle this_point_emergence_SC_angle this_point_phase_Sun_SC_angle index_camera];
                        exisiting_data_points = obj.observed_camera_data{observed_points_camera_i,i_ang};
                        obj.observed_camera_data{observed_points_camera_i,i_ang}  = [exisiting_data_points; this_data_point];
                        
                        if obj.first_observed_camera_array(observed_points_camera_i,i_ang) == 0
                            obj.first_observed_camera_array(observed_points_camera_i,i_ang) = obj.time;
                            obj.first_observed_camera_array(observed_points_camera_i,i_ang+length(obj.camera_normal_Sun_angle_array)) = index_SC;
                            obj.first_observed_camera_array(observed_points_camera_i,i_ang+2*length(obj.camera_normal_Sun_angle_array)) = image_resolution;
                        end
                    end
                end
                
            end

        end
    end
end

