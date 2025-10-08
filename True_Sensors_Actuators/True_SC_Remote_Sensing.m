%% Class: True_SC_Remote_Sensing
% Tracks the onboard remote sensing measurements and coverage

classdef True_SC_Remote_Sensing < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_rate_generated % [kbps] : Data rate, in kilo bits per sec (kbps)

        mode_true_SC_remote_sensing_selector % [string]

        measurement_wait_time % [sec]

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        field_of_view % [deg] : Field of view (FOV) of the remote sensing instrument in deg
        % Set to 0 to select the cloest point

        flag_show_coverage_plot % [Boolean] : 1 = Shows the remote sensing coverage plot

        wait_time_visualize_SC_remote_sensing_coverage_during_sim % [sec] (Optional)

        num_points % [integer] Number of points in mesh

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Remote Sensing i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [Image]

        measurement_time % [sec] SC time when this measurement was taken

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        pos_points % [unit vector] Location of points in mesh

        spherical_points % [unit vector] Location of points in Sphere

        monostatic_observed_point % [array] Monostatic: which point is observed how many times

        monostatic_num_point_observed % [integer] Monostatic: total number of points observed

        prev_time_visualize_SC_remote_sensing_coverage_during_sim % [sec]

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Remote_Sensing(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Remote Sensing ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.mode_true_SC_remote_sensing_selector = init_data.mode_true_SC_remote_sensing_selector; % [string]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]

            obj.measurement_wait_time = init_data.measurement_wait_time; % [sec]
            obj.measurement_time = -inf; % [sec]

            obj.flag_executive = 0;

            obj.location = init_data.location; % [m]
            obj.orientation = init_data.orientation; % [unit vector]

            obj.field_of_view = init_data.field_of_view; % [deg]

            obj.flag_show_coverage_plot = init_data.flag_show_coverage_plot; % [Boolean]

            if isfield(init_data, 'wait_time_visualize_SC_remote_sensing_coverage_during_sim')
                obj.wait_time_visualize_SC_remote_sensing_coverage_during_sim = init_data.wait_time_visualize_SC_remote_sensing_coverage_during_sim;
            else
                obj.wait_time_visualize_SC_remote_sensing_coverage_during_sim = 0; % [sec]
            end
            obj.prev_time_visualize_SC_remote_sensing_coverage_during_sim = -inf;

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

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


            % Initialize Variables to store: monostatic_num_point_observed
            obj.store = [];

            obj.store.monostatic_num_point_observed = zeros(mission.storage.num_storage_steps, length(obj.monostatic_num_point_observed));
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.instantaneous_data_rate_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_rate_generated)); % [kbps]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]

            % Update Storage
            obj = func_update_true_SC_remote_sensing_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            % Store video of func_visualize_SC_orbit_during_sim
            if (mission.storage.plot_parameters.flag_save_video == 1) && (obj.flag_show_coverage_plot == 1)
                obj.data.video_filename = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Remote_Sensing',num2str(i_HW),'.mp4'];
                obj.data.myVideo = VideoWriter(obj.data.video_filename, 'MPEG-4');
                obj.data.myVideo.FrameRate = 30;  % Default 30
                obj.data.myVideo.Quality = 100;    % Default 75
                open(obj.data.myVideo);
            end
        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_remote_sensing_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.monostatic_num_point_observed(mission.storage.k_storage,:) = obj.monostatic_num_point_observed; % [integer]
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [integer]

                if obj.flag_executive == 1
                    obj.store.instantaneous_data_rate_generated(mission.storage.k_storage,:) = obj.instantaneous_data_rate_generated; % [kbps]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]
                end

            end

        end

        %% [ ] Methods: Main
        % Update Remote Sensing

        function obj = func_main_true_SC_remote_sensing(obj, mission, i_SC, i_HW)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time - obj.measurement_time) >= obj.measurement_wait_time

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time; % [sec]

                    switch obj.mode_true_SC_remote_sensing_selector

                        case 'Generic'
                            obj = func_coverage_Generic(obj, mission, i_SC);

                        case 'GoldenDome'
                            obj = func_coverage_GoldenDome(obj, mission, i_SC);

                        otherwise
                            error('Remote Sensing mode not defined!')
                    end

                    obj.prev_time_visualize_SC_remote_sensing_coverage_during_sim = -inf;

                else
                    % Data not generated in this time step
                    % Data is only generated when a remote sensing measurement is performed

                end

                if isfield(obj.data, 'instantaneous_power_consumed_per_SC_mode')
                    obj.instantaneous_power_consumed = obj.data.instantaneous_power_consumed_per_SC_mode(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode_value); % [W]

                    % Dont do the Slew update
                end

                % Update Power Consumed
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            else
                % Do nothing

            end


            % Update Storage
            obj = func_update_true_SC_remote_sensing_store(obj, mission);

            % Plot Remote Sensing Coverage
            if (obj.flag_show_coverage_plot == 1) && (mission.true_SC{i_SC}.software_SC_executive.time - obj.prev_time_visualize_SC_remote_sensing_coverage_during_sim >= obj.wait_time_visualize_SC_remote_sensing_coverage_during_sim )
                obj = func_visualize_SC_remote_sensing_coverage_during_sim(obj, mission, i_SC, i_HW);
            end

            % Reset Variables
            obj.flag_executive = 0;

        end




        %% [ ] Methods: Visualize Remote Sensing Coverage
        % Visualize all SC attitude orbit during simulation

        function obj = func_visualize_SC_remote_sensing_coverage_during_sim(obj, mission, i_SC, i_HW)

            obj.prev_time_visualize_SC_remote_sensing_coverage_during_sim = mission.true_SC{i_SC}.software_SC_executive.time; % [sec]

            plot_handle = figure((30*i_SC) + i_HW);
            clf
            set(plot_handle,'Name',['SC ',num2str(i_SC),' Remote Sensing ',num2str(i_HW),' Coverage'])
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
            % % 3D Remote Sensing Vizualization % %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            subplot(1,3,2)
            hold on

            plot3(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(1), mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(2), mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(3), 's','MarkerSize',15, 'MarkerFaceColor',rgb('Gray'), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)

            if obj.store.flag_executive(mission.storage.k_storage,1) == 1
                % Plot Remote Sensing Orientation
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

            scatter3(this_pos_points(:,1), this_pos_points(:,2), this_pos_points(:,3), 10, colors, 'filled','DisplayName','Remote Sensing Points'); % 50 is the size of the markers

            grid on

            view(3)
            axis equal
            legend('Location','southwest')
            xlabel('X axis [km]')
            ylabel('Y axis [km]')
            zlabel('Z axis [km]')
            set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
            title(['3D Remote Sensing Coverage in ', mission.true_target{i_target}.name,'-centered Frame'],'FontSize',mission.storage.plot_parameters.standard_font_size)

            hold off

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % 2D Remote Sensing Vizualization % %
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
            title(['2D Remote Sensing Coverage in ', mission.true_target{i_target}.name,'-centered Static Frame'],'FontSize',mission.storage.plot_parameters.standard_font_size)


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % Monostatic Remote Sensing Coverage % %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            subplot(2,3,6)
            hold on

            kd = mission.storage.k_storage;

            plot(mission.true_time.store.time(1:kd), (100/obj.num_points)*obj.store.monostatic_num_point_observed(1:kd), '-k', 'LineWidth',2, 'DisplayName','Actual')

            grid on
            xlabel('Time [sec]')
            ylabel('Coverage [%]')
            set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
            title(['Remote Sensing Coverage = ',num2str((100/obj.num_points)*obj.monostatic_num_point_observed),'%'],'FontSize',mission.storage.plot_parameters.standard_font_size)



            drawnow limitrate

            if (mission.storage.plot_parameters.flag_save_video == 1) && (mission.flag_stop_sim == 0)
                open(obj.data.myVideo);
                writeVideo(obj.data.myVideo, getframe(plot_handle));
            end

            if (mission.storage.plot_parameters.flag_save_plots == 1) && (mission.flag_stop_sim == 1)
                saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Remote_Sensing',num2str(i_HW),'.png'])
            end

        end


    end
end

