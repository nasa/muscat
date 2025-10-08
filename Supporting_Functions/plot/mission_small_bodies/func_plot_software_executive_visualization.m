%% Executive SC Mode Visualization

function func_plot_software_executive_visualization(mission, i_SC)

kd = mission.storage.k_storage;

plot_handle = figure('Name',['SC ',num2str(i_SC),' Actual Modes']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Modes % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Define a colormap
cmap = jet(length(mission.true_SC{i_SC}.software_SC_executive.store.sc_modes)); % Use a colormap with max(obj.monostatic_observed_point)+1 colors

for j = 1:1:length(mission.true_SC{i_SC}.software_SC_executive.store.sc_modes)
    % plot(mission.true_time.store.time([1 kd]), [j j], '-', 'DisplayName', mission.true_SC{i_SC}.software_SC_executive.store.sc_modes{j})
    fill([mission.true_time.store.time(1) mission.true_time.store.time(kd) mission.true_time.store.time(kd) mission.true_time.store.time(1)], [(j-0.25) (j-0.25) (j+0.25) (j+0.25)], cmap(j,:), 'EdgeColor','none','FaceAlpha',0.5,'DisplayName',mission.true_SC{i_SC}.software_SC_executive.store.sc_modes{j})
end

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_executive.store.this_sc_mode_value(1:kd), '-k', 'LineWidth',2, 'DisplayName','Actual')

legend('Location','southeast')
xlabel('Time [sec]')
ylabel('SC Mode')

%xticks([mission.true_time.data.time_fraction])
%xticklabels([mission.true_time.data.days])

%xticks(mission.true_SC{i_SC}.software_SC_executive.data.this_side_duration(2,:))
%xticklabels(mission.true_SC{i_SC}.software_SC_executive.data.this_side_duration(2,:) /3600)

xlim([0 mission.true_time.time])

grid on

set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
%     title(['Actual Modes of SC ',num2str(i_SC)],'FontSize',mission.storage.plot_parameters.title_font_size)

sgtitle(['SC ',num2str(i_SC),' Actual Modes'],'fontsize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

% Slew time
slew_procedure = mission.true_SC{i_SC}.software_SC_control_attitude.store.flag_slew(1:mission.storage.k_storage_attitude);
slew_time = mission.true_time.store.time_attitude(1:mission.storage.k_storage_attitude);

% Plot the desaturation procedure
band_height = 0.25; % Height of each desaturation band
band_position = length(mission.true_SC{i_SC}.software_SC_executive.store.sc_modes) + 1;

fill([slew_time(1) slew_time(end) slew_time(end) slew_time(1)], [(band_position-band_height) (band_position-band_height) band_position band_position], 0.3*[1 1 1], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Slew State 0')
fill([slew_time(1) slew_time(end) slew_time(end) slew_time(1)], [band_position (band_position) (band_position+band_height) (band_position+band_height)], 0.7*[1 1 1], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Slew State 1')

% Plot the transition line in the middle of the band
plot(slew_time, slew_procedure * band_height + band_position - band_height/2, '-c', 'LineWidth',2, 'DisplayName','Slew Transition')

% Check if the SC has reaction wheels and if the desaturation procedure exists
has_reaction_wheels = (mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel) > 1;
if has_reaction_wheels && isfield(mission.true_SC{i_SC}.software_SC_control_attitude.store, 'desaturation_procedure')
    % Get the desaturation procedure data
    desaturation_procedure = mission.true_SC{i_SC}.software_SC_control_attitude.store.desaturation_procedure(1:mission.storage.k_storage_attitude);
    desaturation_time = mission.true_time.store.time_attitude(1:mission.storage.k_storage_attitude);

    % Plot the desaturation procedure
    band_height = 0.25; % Height of each desaturation band
    band_position = length(mission.true_SC{i_SC}.software_SC_executive.store.sc_modes) + 2;

    fill([desaturation_time(1) desaturation_time(end) desaturation_time(end) desaturation_time(1)], [(band_position-band_height) (band_position-band_height) band_position band_position], [0.8 0.8 1], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Desaturation State 0')
    fill([desaturation_time(1) desaturation_time(end) desaturation_time(end) desaturation_time(1)], [band_position (band_position) (band_position+band_height) (band_position+band_height)], [1 0.8 0.8], 'EdgeColor','none','FaceAlpha',0.5,'DisplayName','Desaturation State 1')

    % Plot the transition line in the middle of the band
    plot(desaturation_time, desaturation_procedure * band_height + band_position - band_height/2, '-m', 'LineWidth',2, 'DisplayName','Desaturation Transition')
end

yticks([1:1:band_position])

hold off

% end

if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_ConOps.png'])
end

