%% [ ] Methods: Visualize All SC
% Visualize all SC Configuration

function obj = func_visualize_SC_v2(obj, mission, is_realtime_update)

% Handle optional parameter for real-time updates
if nargin < 3
    is_realtime_update = false;
end

plot_handle = figure(1);
clf
set(plot_handle, 'Name','Mission Dashboard')
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

% Add simulation time and attitude information to title if in real-time mode
elapsed_time = seconds(mission.true_time.time);
elapsed_time.Format = 'dd:hh:mm:ss';

sgtitle([mission.name, ', Simulation Time: ',char(elapsed_time)],'FontSize', obj.plot_parameters.title_font_size, 'FontName', obj.plot_parameters.standard_font_type)

%% For each spacecraft, attitude vizualization

for i_SC = 1:1:mission.num_SC
    subplot(1,mission.num_SC+2,i_SC+1)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % 3D Attitude Vizualization % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    func_plot_single_SC_attitude_v2(mission, i_SC);

    title(['SC ',num2str(i_SC), ', ConOps Mode = ',mission.true_SC{i_SC}.software_SC_executive.this_sc_mode], 'FontSize', mission.storage.plot_parameters.standard_font_size)
end

%% Add Dashboard

% Add mission dashboard on the left side but only if this is not a real-time update
% or if it's the first real-time update
try
    % Skip mission description text if this is a real-time update
    func_display_mission_dashboard(gca, mission, is_realtime_update);
catch dashboard_error
    warning('Could not create mission dashboard: %s', dashboard_error.message);
end

%% Save Plot and Video

drawnow limitrate

if (obj.plot_parameters.flag_save_video == 1) && (mission.flag_stop_sim == 0) && is_realtime_update
    open(obj.plot_parameters.myVideo);
    %     writeVideo(obj.plot_parameters.myVideo, getframe(plot_handle));

    % Suppose you generate a frame:
    frame = getframe(plot_handle);  % or however you capture your frame

    % Resize the frame to match required size
    %     targetSize = [1412, 3840];  % [height, width]
    targetSize = [3006, 6016];  % [height, width]
    frameResized = imresize(frame.cdata, targetSize);

    % Convert back to movie frame
    frameToWrite = im2frame(frameResized);

    writeVideo(obj.plot_parameters.myVideo, frameToWrite);
end

%% Save Image

if obj.plot_parameters.flag_save_plots == 1 && ~is_realtime_update
    plot_file_name = [obj.output_folder, mission.name,'_',num2str(mission.num_SC),'SC_Mission_Dashboard.png'];
    saveas(plot_handle,plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*
    
    ch = Chapter;
    ch.Title = 'Results: Spacecraft Vizualization';

    for i_SC = 1:1:mission.num_SC

        % SC Name
        sec = Section;
        sec.Title = mission.true_SC{i_SC}.true_SC_body.name;

        plot_handle = figure(101);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');        
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % 3D Attitude Vizualization % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        func_plot_single_SC_attitude_v2(mission, i_SC);

        title(['SC ',num2str(i_SC), ', ConOps Mode = ',mission.true_SC{i_SC}.software_SC_executive.this_sc_mode], 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        
        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.ZLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate 
        
        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('The SC configuration and attitude at the end of the simulation is shown.');
        append(sec,para)

        para = Paragraph('Most of the sensors and instruments (along with their pointing vectors) are shown. This includes:');
        append(sec,para)

        para = Paragraph('- SC Body');
        append(sec,para)

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel),' Solar Panel']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel),' Solar Panels']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera),' Camera']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera),' Cameras']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_telescope > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_telescope == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_telescope),' Telescope']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_telescope),' Telescopes']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker),' Star Tracker']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker),' Star Trackers']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor),' Sun Sensor']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor),' Sun Sensors']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar),' Science Radar']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar),' Science Radars']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing),' Remote Sensing Instrument']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing),' Remote Sensing Instruments']);
            end
            append(sec,para)
        end
        
        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna),' Radio Antenna']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna),' Radio Antennas']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel),' Reaction Wheel']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel),' Reaction Wheels']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_RCS_thruster > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_RCS_thruster == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_RCS_thruster),' Micro Thurster']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_RCS_thruster),' Micro Thursters']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster),' Chemical Thurster']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster),' Chemical Thursters']);
            end
            append(sec,para)
        end

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_ep_thruster > 0
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_ep_thruster == 1
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_ep_thruster),' Electirc Propulsion (EP) Thurster']);
            else
                para = Paragraph(['- ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_ep_thruster),' Electirc Propulsion (EP) Thursters']);
            end
            append(sec,para)
        end

        if mission.true_solar_system.index_Earth ~= 0
            para = Paragraph('The pointing vectors to Sun and Earth are also shown.');
        else
            if mission.num_target == 1
                para = Paragraph(['The pointing vectors to Sun, Earth, and Target (',mission.true_target{1}.name,') are also shown.']);
            else
                para = Paragraph(['The pointing vectors to Sun, Earth, and ',num2str(mission.num_target),' Targets are also shown.']);
            end
        end
        append(sec,para)

        append(ch,sec)

    end      

    append(mission.storage.report_parameters.report_handle,ch);   

end

end
