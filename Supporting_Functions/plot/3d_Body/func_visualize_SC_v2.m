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

if obj.plot_parameters.flag_save_plots == 1 && ~is_realtime_update
    saveas(plot_handle,[obj.output_folder, mission.name,'_',num2str(mission.num_SC),'SC_configuration.png'])
end
end
