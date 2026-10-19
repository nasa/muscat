%% [ ] Methods: Visualize Slew Angle

function func_visualize_SC_Slew_Angle(mission, i_SC)

kd = mission.storage.k_storage_attitude;

plot_handle = figure('Name', ['SC ',num2str(i_SC),' Slew Angle']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Slew Angle Error % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,1,1)

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_slew_angle_change(1:kd), '-k', 'LineWidth',2)

xlim([0 mission.true_time.time_attitude])

grid on
title('Angle between Desired Attitude and True Attitude','FontSize',mission.storage.plot_parameters.title_font_size)
xlabel('Time [sec]')
ylabel('Angle [rad]')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Slew State % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,1,2)

hold on

% Slew time
slew_procedure = mission.true_SC{i_SC}.software_SC_control_attitude.store.flag_slew(1:mission.storage.k_storage_attitude);
slew_time = mission.true_time.store.time_attitude(1:mission.storage.k_storage_attitude);

% Plot the desaturation procedure
band_height = 0.5; % Height of each desaturation band
band_position = 0;

fill([slew_time(1) slew_time(end) slew_time(end) slew_time(1)], [(band_position-band_height) (band_position-band_height) band_position band_position], 0.3*[1 1 1], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Slew State 0')
fill([slew_time(1) slew_time(end) slew_time(end) slew_time(1)], [band_position (band_position) (band_position+band_height) (band_position+band_height)], 0.7*[1 1 1], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Slew State 1')

% Plot the transition line in the middle of the band
plot(slew_time, slew_procedure * band_height + band_position - band_height/2, '-c', 'LineWidth',2, 'DisplayName','Slew Transition')

xlim([0 mission.true_time.time_attitude])

legend('Location','southeast')
xlabel('Time [sec]')
grid on

set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)



hold off

sgtitle(['SC ',num2str(i_SC),' Slew Angle and Mode'],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)


if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Slew_Angle.png'])
end

end
