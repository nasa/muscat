%% Class: Storage
% Helps store all the data from the simulation

classdef Storage < handle


    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        time_step_storage % [sec] : Storage time step
        time_step_storage_attitude % [sec] : Storage time step for attitude dynamics loop (Optional)

        flag_visualize_SC_attitude_orbit_during_sim % [Boolean] : 1 = Shows the attitude and position during simulation (Optional)
        wait_time_visualize_SC_attitude_orbit_during_sim % [sec] (Optional)
        flag_visualize_past_SC_orbit_during_sim % [Boolean] : 1 = Shows the entire orbit from the start of sim (Optional)

        %% [ ] Properties: Variables Computed Internally

        time_prev_storage % [sec] : Previous time when variables were stored #
        num_storage_steps % [integer] : Number of storage variables        #
        flag_store_this_time_step % [Boolean] : 1 = Store, else dont store #
        k_storage % [integer] : Storage counter variable                   #

        time_prev_storage_attitude % [sec] : Previous time when attitude variables were stored #
        num_storage_steps_attitude % [integer] : Number of storage variables for Attitude Dynamics Loop #
        flag_store_this_time_step_attitude % [Boolean] : 1 = Store, else dont store, for Attitude Dynamics Loop #
        k_storage_attitude % [integer] : Storage counter variable, for Attitude Dynamics Loop #


        prev_time_visualize_SC_attitude_orbit_during_sim % [sec]

        % Real-time plotting variables
        flag_realtime_plotting % [Boolean] : 1 = Enable real-time performance plotting
        realtime_plot_handle % Handle to the real-time plot figure
        realtime_plot_last_update % [sec] : Time of last real-time plot update
        realtime_plot_update_interval % [sec] : Minimum time between real-time plot updates
        realtime_plot_subhandles % Cell array of subplot handles for real-time plotting


        % Initialize real-time visualization settings
        last_viz_update_time % Time of last visualization update
        viz_update_interval  % Update visualization every 50 simulation seconds (reduced from 100)


        %% [ ] Properties: Other Useful Variables

        numerical_accuracy_factor % [float <= 1, but limit -> 1] : Used to take care of issues arising due to numerical accuracy of integer computations #

        plot_parameters % Parameters used for plotting
        % - color_array
        % - marker_array
        % - standard_font_size
        % - standard_font_type
        % - title_font_size
        % - flag_save_plots % [Boolean] 1: Save them (takes little time), 0: Doesnt save them
        % - flag_save_video % [Boolean] 1: Save them (takes a lot more time), 0: Doesnt save them
        % - quiver_auto_scale_factor % [float] : scale factor used for quiver3

        output_folder % [Boolean] Folder to store all outputs

        flag_stop_sim % flag to stop simulation

        last_mode % Cell array to store the last mode of each spacecraft

    end


    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Storage(init_data, mission)

            if init_data.time_step_storage == 0
                % Use 0 to use the mission.true_time.time_step value
                obj.time_step_storage = mission.true_time.time_step; % [sec]

            else
                obj.time_step_storage = init_data.time_step_storage; % [sec]
            end

            obj.flag_store_this_time_step = 1;
            obj.time_prev_storage = mission.true_time.time; % [sec]
            obj.num_storage_steps = ceil( (mission.true_time.t_final - mission.true_time.t_initial)/obj.time_step_storage ) + 1;
            obj.k_storage = 1;


            obj.last_viz_update_time = -inf; % Time of last visualization update
            obj.viz_update_interval = 1000;  % Update visualization every 50 simulation seconds (reduced from 100)

            obj.flag_stop_sim = 0;


            if isfield(init_data, 'time_step_storage_attitude')
                % time_step_storage_attitude has been specified

                if init_data.time_step_storage_attitude == 0
                    % Use 0 to use the mission.true_time.time_step_attitude value
                    obj.time_step_storage_attitude = mission.true_time.time_step_attitude; % [sec]

                else
                    obj.time_step_storage_attitude = init_data.time_step_storage_attitude; % [sec]
                end

            else
                obj.time_step_storage_attitude = obj.time_step_storage; % [sec]
            end

            obj.flag_store_this_time_step_attitude = 1;
            obj.time_prev_storage_attitude = mission.true_time.time; % [sec]
            obj.num_storage_steps_attitude = ceil( (mission.true_time.t_final - mission.true_time.t_initial)/obj.time_step_storage_attitude ) + 1;
            obj.k_storage_attitude = 1;

            obj.numerical_accuracy_factor = 0.99;

            % Set plot_parameters
            obj.plot_parameters = [];
            obj.plot_parameters.color_array = ['b' 'r' 'g' 'c' 'y' 'm' 'k'];
            % (Additional colors using rgb.m function from https://www.mathworks.com/matlabcentral/fileexchange/24497-rgb-triple-of-color-name-version-2)

            obj.plot_parameters.marker_array = ['o' 's' 'd' '^' 'v' '>' '<' 'p' 'h' '+'];
            obj.plot_parameters.standard_font_size = 20;
            obj.plot_parameters.standard_font_type = 'Times New Roman';
            obj.plot_parameters.title_font_size = 40;

            if isfield(init_data, 'flag_save_plots')
                obj.plot_parameters.flag_save_plots = init_data.flag_save_plots;
            else
                obj.plot_parameters.flag_save_plots = 1; % [Boolean] 1: Save them (takes little time), 0: Doesnt save them
            end

            if isfield(init_data, 'flag_save_video')
                obj.plot_parameters.flag_save_video = init_data.flag_save_video;
            else
                obj.plot_parameters.flag_save_video = 0; % [Boolean] 1: Save them (takes a lot more time), 0: Doesnt save them
            end

            if isfield(init_data, 'flag_visualize_SC_attitude_orbit_during_sim')
                obj.flag_visualize_SC_attitude_orbit_during_sim = init_data.flag_visualize_SC_attitude_orbit_during_sim;
            else
                obj.flag_visualize_SC_attitude_orbit_during_sim = 1;
            end

            if isfield(init_data, 'wait_time_visualize_SC_attitude_orbit_during_sim')
                obj.wait_time_visualize_SC_attitude_orbit_during_sim = init_data.wait_time_visualize_SC_attitude_orbit_during_sim; % [sec]
            else
                obj.wait_time_visualize_SC_attitude_orbit_during_sim = 0; % [sec]
            end
            obj.prev_time_visualize_SC_attitude_orbit_during_sim = -inf;

            if isfield(init_data, 'flag_visualize_past_SC_orbit_during_sim')
                obj.flag_visualize_past_SC_orbit_during_sim = init_data.flag_visualize_past_SC_orbit_during_sim;
            else
                obj.flag_visualize_past_SC_orbit_during_sim = 1;
            end

            % Initialize real-time plotting variables
            if isfield(init_data, 'flag_realtime_plotting')
                obj.flag_realtime_plotting = init_data.flag_realtime_plotting;
            else
                obj.flag_realtime_plotting = 1;
            end

            if isfield(init_data, 'quiver_auto_scale_factor')
                obj.plot_parameters.quiver_auto_scale_factor = init_data.quiver_auto_scale_factor;
            else
                obj.plot_parameters.quiver_auto_scale_factor = 0.1;
            end

            % Output Folder
            obj.output_folder = ['../Output/', mission.name, '_', char(datetime("now", "Format", "yyyy-MM-dd-HH'h'mm'm'ss's'")), ' (SimTime = ', char(string(mission.true_time.t_final/86400)), ' days)/'];
            %obj.output_folder = ['../Output/', mission.name, '_', char(datetime("now", "Format", "yyyy-MM-dd-HH'h'mm'm'ss's'")), ' (SimTime = ',char(string(mission.true_time.t_final/86400)), ' days)/'];
            mkdir(obj.output_folder)

            % Store video of func_visualize_SC_attitude_orbit_during_sim
            if (obj.plot_parameters.flag_save_video == 1) && (obj.flag_visualize_SC_attitude_orbit_during_sim == 1)
                obj.plot_parameters.video_filename = [obj.output_folder, mission.name,'_Attitude_Orbit.mp4'];
                obj.plot_parameters.myVideo = VideoWriter(obj.plot_parameters.video_filename, 'MPEG-4');
                obj.plot_parameters.myVideo.FrameRate = 30;  % Default 30
                obj.plot_parameters.myVideo.Quality = 100;    % Default 75
                open(obj.plot_parameters.myVideo);
            end


        end

        %% [ ] Methods: Update Storage Flag
        % Set the flag_store_this_time_step after sufficient time

        function obj = func_update_storage_flag(obj, mission)

            % Reset flags
            obj.flag_store_this_time_step = 0;

            if (mission.true_time.time - obj.time_prev_storage) >= (obj.time_step_storage * obj.numerical_accuracy_factor)
                obj.flag_store_this_time_step = 1;
            end

            if mission.true_time.k == mission.true_time.num_time_steps
                obj.flag_store_this_time_step = 1;
            end

            if obj.flag_store_this_time_step == 1
                obj.time_prev_storage = mission.true_time.time; % [sec]
                obj.k_storage = obj.k_storage + 1;
            end

        end

        %% [ ] Methods: Update Storage Flag Attitude
        % Set the flag_store_this_time_step_attitude after sufficient time

        function obj = func_update_storage_flag_attitude(obj, mission)

            % Reset flags
            obj.flag_store_this_time_step_attitude = 0;

            if (mission.true_time.time_attitude - obj.time_prev_storage_attitude) >= (obj.time_step_storage_attitude * obj.numerical_accuracy_factor)
                obj.flag_store_this_time_step_attitude = 1;
            end

            if (mission.true_time.k == mission.true_time.num_time_steps) && (mission.true_time.k_attitude == mission.true_time.num_time_steps_attitude)
                obj.flag_store_this_time_step_attitude = 1;
            end

            if obj.flag_store_this_time_step_attitude == 1
                obj.time_prev_storage_attitude = mission.true_time.time_attitude; % [sec]
                obj.k_storage_attitude = obj.k_storage_attitude + 1;
            end

        end


        %% [ ] Methods: Update Real-Time Plot
        % Update the real-time performance plot

        function obj = func_update_realtime_plot(obj, mission)
           if obj.flag_realtime_plotting
                % Initialize last_mode property if it doesn't exist
                if ~isfield(obj, 'last_mode')
                    obj.last_mode = {};
                    for i_SC = 1:mission.num_SC
                        obj.last_mode{i_SC} = mission.true_SC{i_SC}.software_SC_executive.this_sc_mode;
                    end
                end
                
                % Check for mode changes in any spacecraft
                mode_changed = false;
                for i_SC = 1:mission.num_SC
                    current_mode = mission.true_SC{i_SC}.software_SC_executive.this_sc_mode;
                    if ~strcmp(current_mode, obj.last_mode{i_SC})
                        mode_changed = true;
                        obj.last_mode{i_SC} = current_mode;
                    end
                end
                
                % Update visualization if time interval has elapsed OR mode has changed
                if (mission.true_time.time - obj.last_viz_update_time >= obj.viz_update_interval) || mode_changed
                    % Update visualization with attitude rotation and store the time
                    func_visualize_SC(obj, mission, true);
                    obj.last_viz_update_time = mission.true_time.time;
                    
                    % Force display update without blocking execution
                    drawnow limitrate;
                end
            end
        end

        %% [ ] Methods: Visualize Simulation Data
        % Visualize all simulation data

        function obj = func_visualize_simulation_data(obj, mission)
            % First, ensure we're not keeping any unnecessary figures open
            close all
            mission.flag_stop_sim = 1;

            % Close all video files
            if (obj.plot_parameters.flag_save_video == 1)
                if isfield(mission.storage.plot_parameters, 'myVideo')
                    close(obj.plot_parameters.myVideo);
                end

                for i_SC = 1:1:mission.num_SC
                    % Close Camera video files
                    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
                        if isfield(mission.true_SC{i_SC}.true_SC_camera{i_HW}.data, 'myVideo')
                            close(mission.true_SC{i_SC}.true_SC_camera{i_HW}.data.myVideo);
                        end
                    end

                    % Close Radar video files
                    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar
                        if isfield(mission.true_SC{i_SC}.true_SC_science_radar{i_HW}.data, 'myVideo')
                            close(mission.true_SC{i_SC}.true_SC_science_radar{i_HW}.data.myVideo);
                        end
                    end
                end
            end

            % Memory optimization: create a function to run between plots to cleanup memory
            function cleanup_memory()
                drawnow;  % Flush graphics queue and allow MATLAB to reclaim memory
                % More can be added here in case of need !
            end

            % Process one spacecraft at a time to limit memory usage
            for i_SC = 1:1:mission.num_SC
                % Basic plots for all spacecraft (minimal memory use)
                % Orbit Vizualization (shared plot)

                if i_SC == 1
                    % Shared for all spacecraft
                    fprintf('Plotting orbit visualization...\n');
                    func_plot_orbit_visualization(mission);
                    cleanup_memory();
                end

                % Spacecraft-specific plots
                fprintf('Plotting SC%d orbit estimator...\n', i_SC);
                func_plot_orbit_estimator(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d orbital control performance...\n', i_SC);
                func_plot_orbital_control_performance(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d attitude visualization...\n', i_SC);
                func_plot_attitude_visualization(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d attitude actuator performance...\n', i_SC);
                func_plot_attitude_actuator_performance(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d power visualization...\n', i_SC);
                func_plot_power_visualization(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d individual power consumption...\n', i_SC);
                func_plot_individual_power_consumption(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d data handling visualization...\n', i_SC);
                func_plot_data_handling_visualization(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting SC%d individual data generation...\n', i_SC);
                func_plot_individual_data_usage(mission, i_SC);
                cleanup_memory();

                fprintf('Plotting software executive visualization...\n');
                func_plot_software_executive_visualization(mission, i_SC);
                cleanup_memory();

                % Telecom Viz
                if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_communication_link > 0
                    fprintf('Plotting SC%d telecom...\n', i_SC);
                    func_plot_telecom(mission, i_SC);
                    cleanup_memory();
                end

                % Radar Viz
                for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar
                    fprintf('Plotting SC%d radar %d coverage...\n', i_SC, i_HW);
                    func_visualize_SC_radar_coverage_during_sim(mission.true_SC{i_SC}.true_SC_science_radar{i_HW}, mission, i_SC, i_HW);
                    cleanup_memory();
                end
            end
        end

    end
end

