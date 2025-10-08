%% [ ] Methods: Visualize All SC
% Visualize SC Configuration for NISAR

function obj = func_visualize_SC_NISAR(obj, mission, is_realtime_update)

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
    subplot(1,mission.num_SC+1,i_SC)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % 3D Attitude Vizualization % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    func_plot_single_SC_attitude_v2(mission, i_SC);

    title(['SC ',num2str(i_SC), ', ConOps Mode = ',mission.true_SC{i_SC}.software_SC_executive.this_sc_mode], 'FontSize', mission.storage.plot_parameters.standard_font_size)
end

%% Orbit Vizulaization

subplot(1,mission.num_SC+1,mission.num_SC+1)
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 3D Orbit Vizualization % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

kd = mission.storage.k_storage;

for i_SC = 1:1:mission.num_SC
    plot3(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(1), mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(2), mission.true_SC{i_SC}.true_SC_navigation.position_relative_target(3), 's','MarkerSize',15, 'MarkerFaceColor',rgb('Gray'), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)
    p_orbit = plot3(mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,1), mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,2), mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,3), '-','LineWidth',2, 'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name);
    set(get(get(p_orbit,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end

i_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target;
func_plot_target_shape(i_target, mission);

% Plot Ground Stations
for i_HW = 1:1:mission.true_ground_station.num_GS_radio_antenna
    this_GS_position = (mission.true_target{i_target}.rotation_matrix * mission.true_GS_radio_antenna{i_HW}.position')'; % [km]
    plot3(this_GS_position(1), this_GS_position(2), this_GS_position(3), 'ok', 'MarkerSize',10, 'MarkerFaceColor','r', 'DisplayName',mission.true_GS_radio_antenna{i_HW}.name)
end

flag_show_radar = 1;

if flag_show_radar > 0

    this_pos_points = mission.true_target{i_target}.radius * (mission.true_target{i_target}.rotation_matrix * mission.true_SC{i_SC}.true_SC_remote_sensing{flag_show_radar}.pos_points')'; % [km]

    % Define a colormap
    cmap = jet(max(mission.true_SC{i_SC}.true_SC_remote_sensing{flag_show_radar}.monostatic_observed_point)+1); % Use a colormap with max(obj.monostatic_observed_point)+1 colors
    cmap(1,:) = NaN*[1 1 1]; % Set 1 to transparent

    % Map the values to colors
    colors = cmap(1+mission.true_SC{i_SC}.true_SC_remote_sensing{flag_show_radar}.monostatic_observed_point', :);

    scatter3(this_pos_points(:,1), this_pos_points(:,2), this_pos_points(:,3), 10, colors, 'filled','DisplayName','Remote Sensing Points'); % 50 is the size of the markers

    title(['SC in ', mission.true_target{i_target}.name,'-centered Frame, with ',mission.true_SC{i_SC}.true_SC_remote_sensing{flag_show_radar}.name],'FontSize',mission.storage.plot_parameters.standard_font_size)

    % Add colorbar to show mapping
    colorbar;
    caxis([0 max(mission.true_SC{i_SC}.true_SC_remote_sensing{flag_show_radar}.monostatic_observed_point)+1]);
else
    title(['SC in ', mission.true_target{i_target}.name,'-centered Frame'],'FontSize',mission.storage.plot_parameters.standard_font_size)
end



grid on

view(3)

axis equal
legend('Location','southeast')
xlabel('X axis [km]')
ylabel('Y axis [km]')
zlabel('Z axis [km]')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off


hold off


%% Save Plot and Video

drawnow limitrate

if (obj.plot_parameters.flag_save_video == 1) && (mission.flag_stop_sim == 0) && is_realtime_update
    open(obj.plot_parameters.myVideo);
    % writeVideo(obj.plot_parameters.myVideo, getframe(plot_handle));

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
