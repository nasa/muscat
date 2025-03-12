function func_plot_telecom(mission, i_SC)

kd = mission.storage.k_storage;

plot_handle = figure('Name',['SC ',num2str(i_SC),' Telecom Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

% Check if we should plot in days or seconds
plot_in_days = isfield(mission.true_time, 'data') && isfield(mission.true_time.data, 'time_fraction');

% Determine the number of subplots (4 if we have antenna data, 3 if not)
if isfield(mission.true_SC{i_SC}, 'true_SC_radio_antenna') && ~isempty(mission.true_SC{i_SC}.true_SC_radio_antenna)
    num_subplots = 4;
else
    num_subplots = 3;
end

% Plot Link Status
subplot(num_subplots,1,1);
hold on
grid on
xlabel(conditional_label(plot_in_days))
ylabel('Link Status')
if plot_in_days
    xlim([0 mission.true_time.time])
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
end
ylim([0 1.15])
yticks([0,1])
yticklabels({'Off', 'On'}); 
set(gca,'TickLabelInterpreter', 'tex')
legend()
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.store.flag_executive(1:kd),'-','LineWidth',2,'DisplayName',strcat(mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.name,": ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.TX_name," to ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.RX_name))
end
hold off

% Plot Antenna Mode if available
current_subplot = 1;
if num_subplots == 4
    current_subplot = current_subplot + 1;
    subplot(num_subplots,1,current_subplot);
    hold on
    grid on
    ylabel('Antenna Mode')
    xlabel(conditional_label(plot_in_days))
    if plot_in_days
        xticks(mission.true_time.data.time_fraction)
        xticklabels(mission.true_time.data.days)
        xlim([0 mission.true_time.time])
    end
    ylim([0 2.15])
    yticks([0,1,2])
    yticklabels({'Off', 'Tx', 'Rx'}); 
    set(gca,'TickLabelInterpreter', 'tex')
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    legend()

    for idx_radio_antenna = 1:1:length(mission.true_SC{i_SC}.true_SC_radio_antenna)
        plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_radio_antenna{idx_radio_antenna}.store.mode_TX_RX(1:kd),'-','LineWidth',2,'DisplayName',mission.true_SC{i_SC}.true_SC_radio_antenna{idx_radio_antenna}.name)
    end
    hold off
end

% Plot Data Rate
current_subplot = current_subplot + 1;
subplot(num_subplots,1,current_subplot);
hold on
grid on
if plot_in_days
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
    xlim([0 mission.true_time.time])
end
ylabel('Datarate [kbps]')
xlabel(conditional_label(plot_in_days))
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
legend()

for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.store.this_data_rate(1:kd),'-','LineWidth',2,'DisplayName',strcat(mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.name,": ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.TX_name," to ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.RX_name))
end
hold off

% Plot Visibility
current_subplot = current_subplot + 1;
subplot(num_subplots,1,current_subplot);
hold on
grid on
xlabel(conditional_label(plot_in_days))
ylabel('Visibility')
if plot_in_days
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
    xlim([0 mission.true_time.time])
end
yticks([0,1])
ylim([0 1.1])
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
legend()

for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.store.flag_TX_RX_visible(1:kd),'-','LineWidth',2,'DisplayName',strcat(mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.name,": ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.TX_name," to ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.RX_name))
end
hold off

sgtitle(['SC ',num2str(i_SC),' Telecom Performance'],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Telecom.png'])
end

end

function label = conditional_label(plot_in_days)
    if plot_in_days
        label = 'Time [days]';
    else
        label = 'Time [sec]';
    end
end