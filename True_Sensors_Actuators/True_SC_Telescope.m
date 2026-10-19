%% Class: True_SC_Telescope
% SC's Telescope

classdef True_SC_Telescope < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_generated_per_pixel % [kb] : Data generated per pixel in telescope image

        instantaneous_data_compression % [float <= 1] : Telescope data compression (Optional)

        mode_true_SC_telescope_selector % [string]

        measurement_wait_time % [sec] How often can measurements be taken?

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        orientation_up % [unit vector] : Up Normal vector from location in telescope frame

        resolution % [x y] : Resolution of the telescope in pixels (e.g. [1024 1024])

        field_of_view % [deg] : Field of view (FOV) of the telescope in deg

        flag_show_telescope_plot % [Boolean] : 1 = Shows the telescope plot

        flag_show_stars % [Boolean] : 1 = Show stars upto mission.true_stars.maximum_magnitude. This takes a lot of time for wide FOV telescopes. (Optional)

        desired_RA % [deg] Desired RA that the telescope should point

        desired_Dec % [deg] Desired Dec that the telescope should point

        num_telescope_target % [integer] Number of Telescope Targets

        this_telescope_target_idx % [integer] Current Telescope Target Index

        obs_time_telescope_target % [array] Observation Time (in sec) per Telescope Target

        names_of_telescope_targets % [string cell] Names of all Telescope Targets

        RA_Dec_of_telescope_targets % [deg] RA and Dec of all Telescope Targets


        %% [ ] Properties: Variables Computed Internally

        name % [string] 'telescope i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [Image]

        measurement_time % [sec] SC time when this measurement was taken

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        plot_handle_telescope_image % Matlab plot handle for the image

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample 
                                    % used to avoid calling isprop() every time
                                    % step to determine if using instantaneous_data_rate_generated or
                                    % instantaneous_data_generated_per_sample, which is slow

        data % Other useful data

        mode_selector_name % [string] Alternative name of mode

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Telescope(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Telescope ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_generated_per_pixel = init_data.instantaneous_data_generated_per_pixel; % [kb]
            obj.instantaneous_data_generated_per_sample = 0; % [kb] Data is only generated when an image is captured

            obj.mode_true_SC_telescope_selector = init_data.mode_true_SC_telescope_selector; % [string]
            obj.measurement_wait_time = init_data.measurement_wait_time; % [sec]

            obj.flag_executive = 0;

            obj.measurement_time = 0; % [sec]

            obj.location = init_data.location; % [m]
            obj.orientation = init_data.orientation; % [unit vector]
            obj.orientation_up = init_data.orientation_up; % [unit vector]

            obj.resolution = init_data.resolution;  % [x y] pixel
            obj.field_of_view = init_data.field_of_view; % [deg]

            obj.flag_show_telescope_plot = init_data.flag_show_telescope_plot;

            if isfield(init_data, 'flag_show_stars')
                obj.flag_show_stars = init_data.flag_show_stars;
            else
                obj.flag_show_stars = 0;
            end

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
            obj.data.flag_take_image = 0;

            if isfield(init_data, 'mode_selector_name')
                obj.mode_selector_name = init_data.mode_selector_name;
            else
                obj.mode_selector_name = mission.name;
            end


            % Additional Mission-Specific Telescope Variables
            switch obj.mode_true_SC_telescope_selector

                case 'mission specific'
                    func_name = ['func_true_SC_telescope_constructor_', obj.mode_selector_name];     

                    if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                            disp('Additional Mission-Specific Telescope constructor function exists!');
                            this_func_handle = str2func(func_name);
                            obj = this_func_handle(obj, mission, i_SC, i_HW);

                        else
                            disp('Additional Mission-Specific Telescope constructor function does NOT exist.');
                            % Do nothing! 
                    end                   
                
                otherwise
                    % Default values
                    obj.desired_RA = 0; % [deg]
                    obj.desired_Dec = 0; % [deg]
                    obj.num_telescope_target = 1; % [integer]
                    obj.this_telescope_target_idx = 1; % [integer]

                    obj.names_of_telescope_targets{1} =  "Default Target"; % [string]
                    obj.RA_Dec_of_telescope_targets = zeros(obj.num_telescope_target,2); % [deg]

            end

            obj.obs_time_telescope_target = zeros(1,obj.num_telescope_target);


            % Initialize Variables to store:
            obj.store = [];
            obj.store.flag_take_image = zeros(mission.storage.num_storage_steps, length(obj.data.flag_take_image));
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.instantaneous_data_generated_per_sample = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_generated_per_sample)); % [kb]
            obj.store.instantaneous_power_consumed = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_consumed)); % [W]
            obj.store.desired_RA = zeros(mission.storage.num_storage_steps, length(obj.desired_RA)); % [deg]
            obj.store.desired_Dec = zeros(mission.storage.num_storage_steps, length(obj.desired_Dec)); % [deg]
            obj.store.this_telescope_target_idx = zeros(mission.storage.num_storage_steps, length(obj.this_telescope_target_idx)); % [integer]
            obj.store.obs_time_telescope_target = zeros(mission.storage.num_storage_steps, length(obj.obs_time_telescope_target)); % [array]


            % Update Storage
            obj = func_update_true_SC_telescope_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);


            % Store video
            if (mission.storage.plot_parameters.flag_save_video == 1) && (obj.flag_show_telescope_plot == 1)
                obj.data.video_filename = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Telescope',num2str(i_HW),'.mp4'];
                obj.data.myVideo = VideoWriter(obj.data.video_filename, 'MPEG-4');
                obj.data.myVideo.FrameRate = 30;  % Default 30
                obj.data.myVideo.Quality = 100;    % Default 75
                open(obj.data.myVideo);
            end


        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_telescope_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [Boolean]

                if obj.flag_executive == 1
                    obj.store.flag_take_image(mission.storage.k_storage,:) = obj.data.flag_take_image; % [Boolean]
                    obj.store.instantaneous_data_generated_per_sample(mission.storage.k_storage,:) = obj.instantaneous_data_generated_per_sample; % [kbps]
                    obj.store.instantaneous_power_consumed(mission.storage.k_storage,:) = obj.instantaneous_power_consumed; % [W]
                    obj.store.desired_RA(mission.storage.k_storage,:) = obj.desired_RA; % [deg]
                    obj.store.desired_Dec(mission.storage.k_storage,:) = obj.desired_Dec; % [deg]
                    obj.store.this_telescope_target_idx(mission.storage.k_storage,:) = obj.this_telescope_target_idx;
                    obj.store.obs_time_telescope_target(mission.storage.k_storage,:) = obj.obs_time_telescope_target;
                end
            end

        end

        %% [ ] Methods: Main
        % Update telescope

        function obj = func_main_true_SC_telescope(obj, mission, i_SC, i_HW)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time - obj.measurement_time) >= obj.measurement_wait_time

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time; % [sec]
                    obj.data.flag_take_image = 1;

                    switch obj.mode_true_SC_telescope_selector

                        case 'Simple'
                            obj = func_true_SC_telescope_Simple(obj, mission, i_SC, i_HW);

                        case 'mission specific'
                            func_name = ['func_true_SC_telescope_', obj.mode_selector_name];
                            this_func_handle = str2func(func_name);
                            obj = this_func_handle(obj, mission, i_SC, i_HW);

                        otherwise
                            error('Telescope mode not defined!')
                    end


                    % Update Data Generated
                    func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

                else
                    % Data not generated in this time step
                    obj.instantaneous_data_generated_per_sample = 0; % [kb] Data is only generated when an image is captured
                    obj.data.flag_take_image = 0;
                end

                % Update Power Consumed
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            else
                % Do nothing
                obj.data.flag_take_image = 0;
            end

            % Update Storage
            obj = func_update_true_SC_telescope_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

        %% [ ] Methods: Simple Telescope
        % Simple Telescope mode


        function obj = func_true_SC_telescope_Simple(obj, mission, i_SC, i_HW)

            obj.instantaneous_data_generated_per_sample = obj.instantaneous_data_compression * obj.instantaneous_data_generated_per_pixel * obj.resolution(1) * obj.resolution(2); % [kb]

            if obj.flag_show_telescope_plot == 1
                func_plot_telescope_image(obj, mission, i_SC, i_HW);
            end

            % Update Storage
            obj = func_update_true_SC_telescope_store(obj, mission);

        end


        %% [ ] Methods: Show Telescope Plot

        function obj = func_plot_telescope_image(obj, mission, i_SC, i_HW)

            % Show Telescope Plot

            obj.plot_handle_telescope_image = figure( (5*i_SC) + i_HW);
            clf
            set(obj.plot_handle_telescope_image,'Color',rgb('Black'));
            set(obj.plot_handle_telescope_image, 'Units', 'pixels', 'Position', [1 1 obj.resolution])
            %                 set(obj.plot_handle,'PaperPositionMode','auto');
            set(obj.plot_handle_telescope_image, 'Resize','off');
            hold on

            % field of view
            camva(obj.field_of_view);

            camproj('perspective');
            axis image
            axis off;


            % Light comming from sun
            %         camlight('headlight')
            h = light;
            h.Position = (mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_target{mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target}.position);
            h.Style = 'local';

            % Move SC away from Target
            campos(mission.true_SC{i_SC}.true_SC_orbit_propagation.position_relative_target);

            target_distance = norm(mission.true_SC{i_SC}.true_SC_orbit_propagation.position_relative_target);


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
                writeVideo(obj.data.myVideo, getframe(obj.plot_handle_telescope_image));
            end


        end
                


        %% [ ] Methods: Visualize Telescope Observations (writes to Report/Excel)
        % Visualize all telescope targets observed during simulation, and also
        % add the plots and underlying data to the mission Report/Excel output

        function obj = func_visualize_SC_telescope_observations_during_sim(obj, mission, i_SC, i_HW)

            plot_handle = figure((30*i_SC) + i_HW);
            clf
            set(plot_handle,'Name',['SC ',num2str(i_SC),' Telescope ',num2str(i_HW),' Observation'])
            set(plot_handle,'Color',[1 1 1]);
            set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
            set(plot_handle,'PaperPositionMode','auto');

            time_sim_elapsed = seconds(mission.true_time.time - mission.true_time.t_initial);
            time_sim_elapsed.Format = 'dd:hh:mm:ss';

            sgtitle(['SC ',num2str(i_SC),', ',obj.name,', Simulation Time = ',char(time_sim_elapsed)],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

            kd = mission.storage.k_storage;

            subplot(1,3,[1 2])

            func_true_SC_telescope_observations_subplot1(obj, mission, kd);

            subplot(1,3,3)

            func_true_SC_telescope_observations_subplot2(obj, mission, kd);

            plot_file_name = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Telescope_',num2str(i_HW),'_Observations.png'];

            if mission.storage.plot_parameters.flag_save_plots == 1
                saveas(plot_handle,plot_file_name)
            end

            %% Write Report

            if mission.storage.report_parameters.flag_write_report == 1

                import mlreportgen.report.*
                import mlreportgen.dom.*

                ch = Chapter;
                ch.Title = ['Results: SC ',num2str(i_SC),' ',obj.name,' Observations'];

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % % Selected Telescope Target vs Simulation Time % %
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                sec = Section;
                sec.Title = 'Selected Telescope Target vs Simulation Time';

                plot_handle = figure(110 + (30*i_SC) + i_HW);
                clf
                set(plot_handle,'Color',[1 1 1]);
                set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
                set(plot_handle,'PaperPositionMode','auto');

                func_true_SC_telescope_observations_subplot1(obj, mission, kd);

                set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
                ax = gca;
                ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
                ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
                ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

                drawnow limitrate

                fig = Figure(gcf);
                fig.Scaling = "auto";
                fig.Snapshot.Caption = ['Location: Subplot(1,3,[1 2]) inside ',plot_file_name];
                append(sec,fig);

                para = Paragraph('Explanation of this plot:');
                para.Bold = true;
                append(sec,para)

                para = Paragraph(['Shows which telescope target (or Standby) ',obj.name,' was pointed at, at every time step of the simulation.']);
                append(sec,para)

                para = Paragraph('Variables in Excel file:');
                para.Bold = true;
                append(sec,para)

                this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
                mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' ',obj.name,' Target Index'];
                this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

                for i_k = 1:1:kd
                    mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = obj.store.this_telescope_target_idx(i_k);
                end

                para = Paragraph(['- "SC ',num2str(i_SC),' ',obj.name,' Target Index" represents the index of the telescope target being observed at every time step. An index greater than ',num2str(obj.num_telescope_target),' (the number of defined targets) indicates Standby.']);
                append(sec,para)

                append(ch,sec)

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % % Observation Time for each Telescope Target % %
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                sec = Section;
                sec.Title = 'Observation Time for each Telescope Target';

                plot_handle = figure(140 + (30*i_SC) + i_HW);
                clf
                set(plot_handle,'Color',[1 1 1]);
                set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
                set(plot_handle,'PaperPositionMode','auto');

                func_true_SC_telescope_observations_subplot2(obj, mission, kd);

                set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
                ax = gca;
                ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
                ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
                ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

                drawnow limitrate

                fig = Figure(gcf);
                fig.Scaling = "auto";
                fig.Snapshot.Caption = ['Location: Subplot(1,3,3) inside ',plot_file_name];
                append(sec,fig);

                para = Paragraph('Explanation of this plot:');
                para.Bold = true;
                append(sec,para)

                para = Paragraph(['Total cumulative observation time (in hours) that ',obj.name,' spent on each telescope target over the entire simulation duration.']);
                append(sec,para)

                para = Paragraph('Variables in Excel file:');
                para.Bold = true;
                append(sec,para)

                for j = 1:1:obj.num_telescope_target

                    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
                    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' ',obj.name,' Cum. Obs. Time ',char(obj.names_of_telescope_targets{j}),' [hrs]'];
                    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

                    for i_k = 1:1:kd
                        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = obj.store.obs_time_telescope_target(i_k,j)/3600;
                    end

                    para = Paragraph(['- "SC ',num2str(i_SC),' ',obj.name,' Cum. Obs. Time ',char(obj.names_of_telescope_targets{j}),' [hrs]" represents the cumulative observation time (in hours) of target ',char(obj.names_of_telescope_targets{j}),' at every time step.']);
                    append(sec,para)

                end

                append(ch,sec)

                append(mission.storage.report_parameters.report_handle,ch);

            end

        end

    end
end

function func_true_SC_telescope_observations_subplot1(obj, mission, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Selected Telescope Target vs Simulation Time % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Define a colormap
cmap = jet(obj.num_telescope_target);

if any(obj.store.this_telescope_target_idx > obj.num_telescope_target)
    j = obj.num_telescope_target + 1;
    fill([mission.true_time.store.time(1) mission.true_time.store.time(kd) mission.true_time.store.time(kd) mission.true_time.store.time(1)], [(j-0.25) (j-0.25) (j+0.25) (j+0.25)], [.7 .7 .7], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Standby')
    ylim([-0.1 obj.num_telescope_target+2])
else
    ylim([-0.1 obj.num_telescope_target+1])
end

for j = 1:1:obj.num_telescope_target
    fill([mission.true_time.store.time(1) mission.true_time.store.time(kd) mission.true_time.store.time(kd) mission.true_time.store.time(1)], [(j-0.25) (j-0.25) (j+0.25) (j+0.25)], cmap(j,:), 'EdgeColor','none','FaceAlpha',0.5,'DisplayName',obj.names_of_telescope_targets{j})
end

plot(mission.true_time.store.time(1:kd), obj.store.this_telescope_target_idx(1:kd), '-k', 'LineWidth',2, 'DisplayName','Actual')

legend('Location','northeastoutside')
xlabel('Simulation Time [sec]')
ylabel('Telescope Target')
title('Selected Telescope Target vs Simulation Time','FontSize',mission.storage.plot_parameters.title_font_size)

xlim([0 mission.true_time.time])

grid on

set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

hold off

end

function func_true_SC_telescope_observations_subplot2(obj, mission, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Observation Time for each Telescope Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Define a colormap
cmap = jet(obj.num_telescope_target);

if any(obj.store.this_telescope_target_idx > obj.num_telescope_target)
    j = obj.num_telescope_target + 1;
    fill([0 obj.data.total_Standby_time obj.data.total_Standby_time 0]/(60*60), [(j-0.25) (j-0.25) (j+0.25) (j+0.25)], [.7 .7 .7], 'EdgeColor','none','FaceAlpha',0.5)
    ylim([-0.1 obj.num_telescope_target+2])
else
    ylim([-0.1 obj.num_telescope_target+1])
end

for j = 1:1:obj.num_telescope_target

    if obj.obs_time_telescope_target(j) > 0
        fill([0 obj.obs_time_telescope_target(j) obj.obs_time_telescope_target(j) 0]/(60*60), [(j-0.25) (j-0.25) (j+0.25) (j+0.25)], cmap(j,:), 'EdgeColor','none','FaceAlpha',0.5)
    end

end

xlabel('Observation Time [hours]')
ylabel('Telescope Target')
title('Observation Time for each Telescope Target','FontSize',mission.storage.plot_parameters.title_font_size)

grid on

set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

hold off

end

