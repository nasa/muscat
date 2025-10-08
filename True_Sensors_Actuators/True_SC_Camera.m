%% Class: True_SC_Camera
% SC's Navigation Camera

classdef True_SC_Camera < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_generated_per_pixel % [kb] : Data generated per pixel in camera image

        instantaneous_data_compression % [float <= 1] : Camera data compression (Optional)

        mode_true_SC_camera_selector % [string]
        % - Simple

        measurement_wait_time % [sec] How often can measurements be taken?

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        orientation_up % [unit vector] : Up Normal vector from location in camera frame

        resolution % [x y] : Resolution of the camera in pixels (e.g. [1024 1024])

        field_of_view % [deg] : Field of view (FOV) of the camera in deg

        flag_show_camera_plot % [Boolean] : 1 = Shows the camera plot

        flag_show_stars % [Boolean] : 1 = Show stars upto mission.true_stars.maximum_magnitude. This takes a lot of time for wide FOV cameras. (Optional)

        flag_show_coverage_plot % [Boolean] : 1 = Shows the camera coverage plot

        wait_time_visualize_SC_camera_coverage_during_sim % [sec] (Optional)

        num_points % [integer] Number of points in mesh


        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Camera i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [Image]

        measurement_time % [sec] SC time when this measurement was taken

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        plot_handle_camera_image % Matlab plot handle for the image

        flag_target_visible % [Boolean] : 1 = Target is visible to this camera

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        pos_points % [unit vector] Location of points in mesh

        spherical_points % [unit vector] Location of points in Sphere

        monostatic_observed_point % [array] Monostatic: which point is observed how many times

        monostatic_num_point_observed % [integer] Monostatic: total number of points observed

        prev_time_visualize_SC_camera_coverage_during_sim % [sec]

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Camera(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Camera ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_generated_per_pixel = init_data.instantaneous_data_generated_per_pixel; % [kb]
            obj.instantaneous_data_generated_per_sample = 0; % [kb] Data is only generated when an image is captured

            obj.mode_true_SC_camera_selector = init_data.mode_true_SC_camera_selector; % [string]
            obj.measurement_wait_time = init_data.measurement_wait_time; % [sec]

            obj.flag_executive = 0;

            obj.measurement_time = 0; % [sec]

            obj.location = init_data.location; % [m]
            obj.orientation = init_data.orientation; % [unit vector]
            obj.orientation_up = init_data.orientation_up; % [unit vector]

            obj.resolution = init_data.resolution;  % [x y] pixel
            obj.field_of_view = init_data.field_of_view; % [deg]

            obj.flag_show_camera_plot = init_data.flag_show_camera_plot;

            if isfield(init_data, 'flag_show_stars')
                obj.flag_show_stars = init_data.flag_show_stars;
            else
                obj.flag_show_stars = 0;
            end

            obj.flag_target_visible = 0;

            if isfield(init_data, 'instantaneous_data_compression')
                obj.instantaneous_data_compression = init_data.instantaneous_data_compression;
            else
                obj.instantaneous_data_compression = 1;
            end

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end
            obj.data.flag_take_picture = 0;

            % Coverage

            obj.flag_show_coverage_plot = init_data.flag_show_coverage_plot; % [Boolean]

            if isfield(init_data, 'wait_time_visualize_SC_camera_coverage_during_sim')
                obj.wait_time_visualize_SC_camera_coverage_during_sim = init_data.wait_time_visualize_SC_camera_coverage_during_sim;
            else
                obj.wait_time_visualize_SC_camera_coverage_during_sim = 0; % [sec]
            end
            obj.prev_time_visualize_SC_camera_coverage_during_sim = -inf;

            % load the science points
            obj.num_points = init_data.num_points; % [integer]
            obj.pos_points = func_load_science_points_v2(obj.num_points);

            obj.spherical_points = zeros(obj.num_points, 2);
            for i = 1:1:obj.num_points

                [radius, lon, lat] = cspice_reclat(obj.pos_points(i,:)'); % [radius, longitude [rad], latitude [rad] ]

                obj.spherical_points(i,:) = [rad2deg(lon), rad2deg(lat)];
            end

            % monostatic data
            obj.monostatic_observed_point = zeros(1,obj.num_points);
            obj.monostatic_num_point_observed = 0;


            % Initialize Variables to store: flag_target_visible
            obj.store = [];
            obj.store.flag_target_visible = zeros(mission.storage.num_storage_steps, length(obj.flag_target_visible));
            obj.store.flag_take_picture = zeros(mission.storage.num_storage_steps, length(obj.data.flag_take_picture));
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.instantaneous_data_generated_per_sample = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_generated_per_sample)); % [kbps]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]
            obj.store.monostatic_num_point_observed = zeros(mission.storage.num_storage_steps, length(obj.monostatic_num_point_observed));


            % Update Storage
            obj = func_update_true_SC_camera_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);


            % Store video
            if (mission.storage.plot_parameters.flag_save_video == 1) && (obj.flag_show_camera_plot == 1)
                obj.data.video_filename = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Camera',num2str(i_HW),'.mp4'];
                obj.data.myVideo = VideoWriter(obj.data.video_filename, 'MPEG-4');
                obj.data.myVideo.FrameRate = 30;  % Default 30
                obj.data.myVideo.Quality = 100;    % Default 75
                open(obj.data.myVideo);
            end

            if (mission.storage.plot_parameters.flag_save_video == 1) && (obj.flag_show_coverage_plot == 1)
                obj.data.video_filename_coverage = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Camera',num2str(i_HW),'_Coverage.mp4'];
                obj.data.myVideo_coverage = VideoWriter(obj.data.video_filename_coverage, 'MPEG-4');
                obj.data.myVideo_coverage.FrameRate = 30;  % Default 30
                obj.data.myVideo_coverage.Quality = 100;    % Default 75
                open(obj.data.myVideo_coverage);
            end


        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_camera_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [Boolean]
                obj.store.monostatic_num_point_observed(mission.storage.k_storage,:) = obj.monostatic_num_point_observed; % [integer]

                if obj.flag_executive == 1
                    obj.store.flag_target_visible(mission.storage.k_storage,:) = obj.flag_target_visible; % [Boolean]
                    obj.store.flag_take_picture(mission.storage.k_storage,:) = obj.data.flag_take_picture; % [Boolean]

                    obj.store.instantaneous_data_generated_per_sample(mission.storage.k_storage,:) = obj.instantaneous_data_generated_per_sample; % [kbps]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]
                end
            end

        end

        %% [ ] Methods: Main
        % Update Camera

        function obj = func_main_true_SC_camera(obj, mission, i_SC, i_HW)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time - obj.measurement_time) >= obj.measurement_wait_time

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time; % [sec]
                    obj.data.flag_take_picture = 1;

                    switch obj.mode_true_SC_camera_selector

                        case 'Simple'
                            obj = func_true_SC_camera_Simple(obj, mission, i_SC, i_HW);

                            % Coverage
                            obj = func_coverage_Generic(obj, mission, i_SC);
                            obj.prev_time_visualize_SC_camera_coverage_during_sim = -inf;

                        otherwise
                            error('Camera mode not defined!')
                    end


                    % Update Data Generated
                    func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

                else
                    % Data not generated in this time step
                    obj.instantaneous_data_generated_per_sample = 0; % [kb] Data is only generated when an image is captured
                    obj.data.flag_take_picture = 0;
                end

                % Update Power Consumed
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            else
                % Do nothing
                obj.data.flag_take_picture = 0;
            end

            % Update Storage
            obj = func_update_true_SC_camera_store(obj, mission);

            % Plot Camera Coverage
            if (obj.flag_show_coverage_plot == 1) && (mission.true_SC{i_SC}.software_SC_executive.time - obj.prev_time_visualize_SC_camera_coverage_during_sim >= obj.wait_time_visualize_SC_camera_coverage_during_sim )
                obj = func_visualize_SC_camera_coverage_during_sim(obj, mission, i_SC, i_HW);
            end

            % Reset Variables
            obj.flag_executive = 0;

        end

        %% [ ] Methods: Simple Camera
        % Simple Camera mode


        function obj = func_true_SC_camera_Simple(obj, mission, i_SC, i_HW)

            obj.instantaneous_data_generated_per_sample = obj.instantaneous_data_compression * obj.instantaneous_data_generated_per_pixel * obj.resolution(1) * obj.resolution(2); % [kb]

            if obj.flag_show_camera_plot == 1
                % Show Camera Plot

                obj.plot_handle_camera_image = figure( (5*i_SC) + i_HW);
                clf
                set(obj.plot_handle_camera_image,'Color',rgb('Black'));
                set(obj.plot_handle_camera_image, 'Units', 'pixels', 'Position', [1 1 obj.resolution])
                %                 set(obj.plot_handle,'PaperPositionMode','auto');
                set(obj.plot_handle_camera_image, 'Resize','off');
                hold on

                % Plot Target Body
                for i_target = 1:1:mission.num_target

                    func_plot_target_shape(i_target, mission);

                end

                axis vis3d off
                axis equal
                material([0 1 0])

                % field of view
                camva(obj.field_of_view);

                camproj('perspective');
                axis image
                axis off;


                % Light comming from sun
                %         camlight('headlight')
                h = light;
                h.Position = (mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_target{mission.true_SC{i_SC}.true_SC_navigation.index_relative_target}.position);
                h.Style = 'local';

                % Move SC away from Target
                campos(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target);

                target_distance = norm(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target);


                % Manage view from current attitude
                x_true_hat = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation')';

                y_true_hat = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation_up')';

                camtarget(target_distance * x_true_hat)
                camup([y_true_hat(1), y_true_hat(2), y_true_hat(3)])

                % Plot Stars
                if obj.flag_show_stars == 1



                    dot_product_angle_array = acosd(mission.true_stars.all_stars_unit_vector * x_true_hat');
                    flag_in_FOV = logical(dot_product_angle_array <= (obj.field_of_view*2));

                    flag_magnitude_limit = logical(mission.true_stars.magnitude_visible <= mission.true_stars.maximum_magnitude);

                    flag_magnitude_limit_FOV = (flag_in_FOV & flag_magnitude_limit);

                    idx_array = find(flag_magnitude_limit_FOV);
                    hold on

                    for s = 1:1:length(idx_array)
                        this_star = target_distance * mission.true_stars.all_stars_unit_vector(idx_array(s),:);
                        plot3(this_star(1),this_star(2),this_star(3),'*w','MarkerSize',(1 + mission.true_stars.maximum_magnitude - mission.true_stars.magnitude_visible(idx_array(s))),'MarkerFaceColor','w','MarkerEdgeColor','w')
                    end
                end

                drawnow limitrate

                if (mission.storage.plot_parameters.flag_save_video == 1)
                    open(obj.data.myVideo);
                    writeVideo(obj.data.myVideo, getframe(obj.plot_handle_camera_image));
                end

            end

            obj = func_true_SC_camera_target_visible(obj, mission, i_SC);

            % Update Storage
            obj = func_update_true_SC_camera_store(obj, mission);

        end


        %%  [ ] Methods: Check Target Visible
        % Check if Target is visible in Camera image

        function obj = func_true_SC_camera_target_visible(obj, mission, i_SC)

            Target_relative_pos_hat = func_normalize_vec(mission.true_target{mission.true_SC{i_SC}.true_SC_navigation.index_relative_target}.position - mission.true_SC{i_SC}.true_SC_navigation.position);

            x_true_hat = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation')';

            Camera_angle = acosd(dot(Target_relative_pos_hat, x_true_hat)); % [deg]

            if Camera_angle <= (obj.field_of_view)
                obj.flag_target_visible = 1;
            else
                obj.flag_target_visible = 0;
            end

        end


        %% [ ] Methods: Visualize Camera Coverage
        % Visualize all SC attitude orbit during simulation

        function obj = func_visualize_SC_camera_coverage_during_sim(obj, mission, i_SC, i_HW)

            obj.prev_time_visualize_SC_camera_coverage_during_sim = mission.true_SC{i_SC}.software_SC_executive.time; % [sec]

            plot_handle = figure((40*i_SC) + i_HW);
            clf
            set(plot_handle,'Name',['SC ',num2str(i_SC),' Camera ',num2str(i_HW),' Coverage'])
            set(plot_handle,'Color',[1 1 1]);
            set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
            set(plot_handle,'PaperPositionMode','auto');

            time_sim_elapsed = seconds(mission.true_time.time - mission.true_time.t_initial);
            time_sim_elapsed.Format = 'dd:hh:mm:ss';

            sgtitle(['SC ',num2str(i_SC),', ',obj.name,', Simulation Time = ',char(time_sim_elapsed)],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

            subplot(1,3,1)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % 3D Attitude Vizualization % %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            func_plot_single_SC_attitude_v2(mission, i_SC);

            title(['SC ',num2str(i_SC), ', ConOps Mode = ',mission.true_SC{i_SC}.software_SC_executive.this_sc_mode], 'FontSize', mission.storage.plot_parameters.standard_font_size)




            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % 3D Camera Vizualization % %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            subplot(1,3,2)
            hold on

            plot3(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(1), mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(2), mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(3), 's','MarkerSize',15, 'MarkerFaceColor',rgb('Gray'), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)

            if obj.store.flag_executive(mission.storage.k_storage,1) == 1
                % Plot Camera Orientation
                this_location = mission.true_SC{i_SC}.true_SC_navigation.position_relative_target;
                this_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation')';

                quiver3(this_location(1), this_location(2), this_location(3), this_orientation(1), this_orientation(2), this_orientation(3), ...
                    'LineWidth',3,'DisplayName',obj.name,'Color',rgb('Orange'), 'AutoScaleFactor',200*mission.storage.plot_parameters.quiver_auto_scale_factor);
            end

            i_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target;
            func_plot_target_shape(i_target, mission);

            this_pos_points = mission.true_target{i_target}.radius * (mission.true_target{i_target}.rotation_matrix * obj.pos_points')'; % [km]

            % Define a colormap
            cmap = jet(max(obj.monostatic_observed_point)+1); % Use a colormap with max(obj.monostatic_observed_point)+1 colors
            cmap(1,:) = NaN*[1 1 1]; % Set 1 to transparent

            % Map the values to colors
            colors = cmap(1+obj.monostatic_observed_point', :);

            scatter3(this_pos_points(:,1), this_pos_points(:,2), this_pos_points(:,3), 10, colors, 'filled','DisplayName','Camera Points'); % 50 is the size of the markers

            grid on

            view(3)
            axis equal
            legend('Location','southwest')
            xlabel('X axis [km]')
            ylabel('Y axis [km]')
            zlabel('Z axis [km]')
            set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
            title(['3D Camera Coverage in ', mission.true_target{i_target}.name,'-centered Frame'],'FontSize',mission.storage.plot_parameters.standard_font_size)

            hold off

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % 2D Camera Vizualization % %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            subplot(2,3,3)
            hold on

            scatter(obj.spherical_points(:,1), obj.spherical_points(:,2), 10, colors, 'filled');

            [radius, lon, lat] = cspice_reclat(mission.true_target{i_target}.rotation_matrix' * mission.true_SC{i_SC}.true_SC_navigation.position_relative_target'); % [radius, longitude [rad], latitude [rad] ]
            %                 plot(rad2deg(lat), rad2deg(lon), 's','MarkerSize',15, 'MarkerFaceColor',rgb('Gray'), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)
            plot(rad2deg(lon), rad2deg(lat), 's','MarkerSize',15, 'MarkerFaceColor',rgb('Gray'), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)

            % Add colorbar to show mapping
            colorbar;
            caxis([0 max(obj.monostatic_observed_point)+1]);

            grid on

            axis equal
            xlabel('Longitude [deg]')
            ylabel('Latitude [deg]')
            set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
            title(['2D Camera Coverage in ', mission.true_target{i_target}.name,'-centered Static Frame'],'FontSize',mission.storage.plot_parameters.standard_font_size)


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % Monostatic Camera Coverage % %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            subplot(2,3,6)
            hold on

            kd = mission.storage.k_storage;

            plot(mission.true_time.store.time(1:kd), (100/obj.num_points)*obj.store.monostatic_num_point_observed(1:kd), '-k', 'LineWidth',2, 'DisplayName','Actual')

            grid on
            xlabel('Time [sec]')
            ylabel('Coverage [%]')
            set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
            title(['Camera Coverage = ',num2str((100/obj.num_points)*obj.monostatic_num_point_observed),'%'],'FontSize',mission.storage.plot_parameters.standard_font_size)



            drawnow limitrate

            if (mission.storage.plot_parameters.flag_save_video == 1) && (mission.flag_stop_sim == 0)
                open(obj.data.myVideo_coverage);
                writeVideo(obj.data.myVideo_coverage, getframe(plot_handle));
            end

            if (mission.storage.plot_parameters.flag_save_plots == 1) && (mission.flag_stop_sim == 1)
                saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Camera',num2str(i_HW),'_Coverage.png'])
            end

        end


    end
end

