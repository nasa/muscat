classdef True_SC_Camera_Sensor < handle
    %CAMERA_SENSOR Summary of this class goes here
    %   Detailed explanation goes here

    properties
        time

        num_camera

        camera_data
        %             – # name = ‘Camera i’
        %             – health : Health of ith Camera
        %                 0. Switched off
        %                 1. Switched on, works nominally
        %             – temperature [deg C] : Temperature of ith Camera
        %             – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith Cam-
        %                 era (if it is switched on)
        %             – # instantaneous_data_generated [kb] : Data generated during current time step, in kilo bits (kb)
        %             – measurement_frequency [Hz] : How many measurements per seconds are possible?
        %             – # measurement_wait_time [sec] = (1 / measurement_frequency)
        %             – # accumulated_wait_time = 0 [sec]
        %             – # take_measurement [Boolean] : 1 = Take measurement this time step
        %             – orientation [unit vector] : Normal vector of measuring element in body frame B
        %             – location [m] : Location of sensor, in body frame B
        %             – measurement_noise : Added to every pixel in the image
        %             – # measurement_vector [image] : Noisy image
        %             – # measurement_available [Boolean] : 1 = Measurement taken this time step
        %             – # measurement_time [sec] : Time when latest Measurement was taken
        %             – # plot_handle : Matlab plot handle for the image
        %             – resolution [x y] : Resolution of the camera in pixels (e.g. [1024 1024])
        %             – field_of_view [deg] : Field of view (FOV) of the camera in deg
        %             – flag_show_stars [Boolean] : 1 = Show stars upto True_Stars.maximum_magnitude.
        %                 This takes a lot of time for wide FOV cameras.
        %             – # flag_SB_visible [Boolean] : 1 = SB is visible to this camera

    end

    methods
        function obj = True_SC_Camera_Sensor(sc_body_init_data, true_time)
            %CAMERA_SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = true_time.time;

            obj.num_camera = sc_body_init_data.num_camera;

            for i=1:obj.num_camera
                obj.camera_data(i).name = ['Camera ',num2str(i)];
                obj.camera_data(i).health = 1;
                obj.camera_data(i).temperature = 10;
                obj.camera_data(i).instantaneous_power_consumption = sc_body_init_data.camera_instantaneous_power_consumption(i);
                obj.camera_data(i).instantaneous_power_consumed = 0; % [W]
                obj.camera_data(i).resolution = sc_body_init_data.camera_resolution(i,:);
                obj.camera_data(i).field_of_view = sc_body_init_data.camera_FOV(i);
                obj.camera_data(i).flag_show_stars = sc_body_init_data.camera_flag_show_stars(i);
                obj.camera_data(i).instantaneous_data_volume = 0;%(1e-3)*obj.camera_data(i).resolution(1)*obj.camera_data(i).resolution(2)*16;
                obj.camera_data(i).instantaneous_data_generated = 0; % [kilo bits]
                % obj.camera_data(i).measurement_frequency = sc_body_init_data.camera_measurement_frequency;  % [Hz] between to measurement
                % obj.camera_data(i).measurement_wait_time = 1/obj.camera_data(i).measurement_frequency;      % [sec] between to measurement
                % obj.camera_data(i).accumulated_wait_time = 0; % [sec]
                % obj.camera_data(i).take_measurement = 1; % [Boolean] : 1 = Take measurement this time step
                obj.camera_data(i).rotation_matrix = eul2rotm(sc_body_init_data.camera_angle_array(i,:)); % [unit vector] : Normal vector of measuring element in body frame B
                obj.camera_data(i).location = sc_body_init_data.camera_location(i,:); % [m] : Location of sensor, in body frame B
                obj.camera_data(i).measurement_noise = sc_body_init_data.camera_measurement_noise;
                obj.camera_data(i).measurement_vector = [0 0 0 0]'; % [quaternion] : Noisy attitude quaternion
                obj.camera_data(i).measurement_available = 0; % [Boolean] : 1 = Measurement taken this time step
                obj.camera_data(i).measurement_time = 0; % [sec] : Time when latest Measurement was taken
                obj.camera_data(i).flag_SB_visible = 0;
                obj.camera_data(i).flag_show_camera_plot = sc_body_init_data.flag_show_camera_plot(i);

            end
        end

        function obj = func_update_accumulated_wait_time(obj, true_time)
            % Update current time
            % Update accumulated time since last measurement

            obj.time = true_time.time;

            for i = 1:obj.num_camera

                if obj.camera_data(i).health == 1
                    obj.camera_data(i).accumulated_wait_time = obj.time - obj.camera_data(i).measurement_time;
                else
                    obj.camera_data(i).accumulated_wait_time = 0;
                end

            end

        end

        function obj = func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time

            for i=1:obj.num_camera

                if obj.camera_data(i).accumulated_wait_time >= obj.camera_data(i).measurement_wait_time
                    obj.camera_data(i).take_measurement = 1;
                else
                    obj.camera_data(i).take_measurement = 0;
                end

            end

        end

        function obj = func_get_camera_image(obj,i,true_time,true_small_body,mission_true_SC,this_SC_index,true_solar_system,true_stars,mission_init_data)

            if obj.camera_data(i).flag_show_camera_plot == 1

                scale_up_factor_factor = 250;
                for j=1:mission_init_data.num_SC
                    if j ~= this_SC_index

                        % plot radar line to SB
                        pos_sb_this_sc_frame = true_small_body.position_SB - mission_true_SC{this_SC_index}.true_SC_navigation.position;
                        pos_sc_this_sc_frame = (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB + scale_up_factor_factor*(1e-3)*mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*mission_true_SC{j}.true_SC_camera_sensor.camera_data(i).location'  -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
                        plot3([pos_sb_this_sc_frame(1)', pos_sc_this_sc_frame(1)'] , [pos_sb_this_sc_frame(2)', pos_sc_this_sc_frame(2)'] , [pos_sb_this_sc_frame(3)', pos_sc_this_sc_frame(3)'], 'Color','green','Linewidth',2);
                        % plot SC shape model
                        this_SC_model = []; this_SC_model.Vertices = mission_true_SC{j}.true_SC_body.shape_model.Vertices; this_SC_model.Faces = mission_true_SC{j}.true_SC_body.shape_model.Faces;
                        this_SC_model.Vertices = (mission_true_SC{j}.true_SC_adc.rotation_matrix_SC * this_SC_model.Vertices')';
                        this_SC_model.Vertices = scale_up_factor_factor * this_SC_model.Vertices * (1e-3);
                        this_SC_model.Vertices = this_SC_model.Vertices + (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
                        patch(this_SC_model,'FaceColor',[1 1 1],'Facelighting','flat');
                        %plot Solar panels shape model
                        hold on;
                        for sp=1:mission_true_SC{j}.true_SC_solar_panel.num_solar_panels
                            this_SP_model = []; this_SP_model.Vertices = mission_true_SC{j}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Vertices; this_SP_model.Faces = mission_true_SC{j}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Faces;
                            this_SP_model.Vertices = (mission_true_SC{j}.true_SC_adc.rotation_matrix_SC *  this_SP_model.Vertices')';
                            this_SP_model.Vertices = scale_up_factor_factor * this_SP_model.Vertices * (1e-3);
                            this_SP_model.Vertices = this_SP_model.Vertices + (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
                            patch(this_SP_model,'FaceColor','blue','Facelighting','flat'); % 'Facelighting','gouraud'
                            hold on;
                        end
                        % plot SC local frame
                        pos_sc_this_sc_frame = (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
                        arrow_size=scale_up_factor_factor*0.8*1e-3;
                        mArrow3(pos_sc_this_sc_frame',pos_sc_this_sc_frame'+mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*[arrow_size 0 0]', 'facealpha', 1, 'color', 'r', 'stemWidth', 0.005);
                        hold on;
                        mArrow3(pos_sc_this_sc_frame',pos_sc_this_sc_frame'+mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*[0 arrow_size 0]', 'facealpha', 1, 'color', 'g', 'stemWidth', 0.005);
                        hold on;
                        mArrow3(pos_sc_this_sc_frame',pos_sc_this_sc_frame'+mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*[0 0 arrow_size]', 'facealpha', 1, 'color', 'b', 'stemWidth', 0.005);
                    end
                end

                % Plot SB model
                scale_up_factor_factor = 1;
                this_SB_Shape_Model = true_small_body.shape_model;
                Rotation_matrix = func_rotation_matrix(true_small_body.rotation_rate * true_time.time) * true_small_body.rotation_matrix_pole_RA_Dec;
                this_SB_Shape_Model.Vertices = (Rotation_matrix * this_SB_Shape_Model.Vertices')'*(1e-3) * scale_up_factor_factor;
                this_SB_Shape_Model.Vertices = this_SB_Shape_Model.Vertices + (true_small_body.position_SB - mission_true_SC{this_SC_index}.true_SC_navigation.position)';
                patch(this_SB_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'none'); % 'Facelighting','gouraud'

                axis vis3d off
                axis equal

                material([0 1 0])

                % Manage view from current attitude
                x_true_hat = mission_true_SC{this_SC_index}.true_SC_adc.rotation_matrix_SC*obj.camera_data(i).rotation_matrix*[1 0 0]';
                target_distance = norm(true_small_body.position_SB - mission_true_SC{this_SC_index}.true_SC_navigation.position);
                x_true = target_distance * x_true_hat;

                y_true_hat = mission_true_SC{this_SC_index}.true_SC_adc.rotation_matrix_SC*obj.camera_data(i).rotation_matrix*[0 1 0]';

                camtarget([x_true(1), x_true(2), x_true(3)])
                campos([0, 0, 0])
                camup([y_true_hat(1), y_true_hat(2), y_true_hat(3)])


                % plot stars
                if (obj.camera_data(i).flag_show_stars == 1)
                    dot_product_angle_array = acosd(true_stars.all_stars_unit_vector * x_true_hat);
                    flag_in_FOV = logical(dot_product_angle_array <= (obj.camera_data(i).field_of_view/sqrt(2)));
                    flag_magnitude_limit = logical(true_stars.magnitude_visible <= true_stars.maximum_magnitude);
                    flag_magnitude_limit_FOV = (flag_in_FOV & flag_magnitude_limit);
                    idx_array = find(flag_magnitude_limit_FOV);
                    hold on
                    for s = 1:1:length(idx_array)
                        this_star = target_distance * true_stars.all_stars_unit_vector(idx_array(s),:)';
                        plot3(this_star(1),this_star(2),this_star(3),'*w','MarkerSize',(1 + true_stars.maximum_magnitude - true_stars.magnitude_visible(idx_array(s))),'MarkerFaceColor','w','MarkerEdgeColor','w')
                        %
                    end
                end

                % Light comming from sun
                %         camlight('headlight')
                h = light;
                h.Position = (true_solar_system.position_Sun - mission_true_SC{this_SC_index}.true_SC_navigation.position)';
                h.Style = 'local';

                % field of view
                camva(obj.camera_data(i).field_of_view);

                camproj('perspective');
                axis image
                axis off;
                set(gca, 'Units', 'pixels', 'Position', [1 1 obj.camera_data(i).resolution]);

                drawnow limitrate

            end

            % equipement routine : update measurement time, data, power, ...
            obj.camera_data(i).measurement_time = obj.time;
            obj.camera_data(i).measurement_available = 1;
            obj.camera_data(i).take_measurement = 0;
            obj.camera_data(i).instantaneous_data_generated = obj.camera_data(i).instantaneous_data_volume;
            obj.camera_data(i).instantaneous_power_consumed = obj.camera_data(i).instantaneous_power_consumption;


        end


        function obj = func_process_image(obj,true_SC_navigation, true_small_body,true_SC_adc)

            SB_relative_pos = (true_small_body.position_SB - true_SC_navigation.position)';
            SB_relative_pos_hat = SB_relative_pos/norm(SB_relative_pos);

            for i=1:obj.num_camera

                x_true_hat = true_SC_adc.rotation_matrix_SC*obj.camera_data(i).rotation_matrix*[1 0 0]';

                SB_Camera_angle = acosd(dot(SB_relative_pos_hat, x_true_hat)); % [deg]

                if SB_Camera_angle <= (obj.camera_data(i).field_of_view/sqrt(2))
                    obj.camera_data(i).flag_SB_visible = 1;
                else
                    obj.camera_data(i).flag_SB_visible = 0;
                end

            end

        end

    end
end

