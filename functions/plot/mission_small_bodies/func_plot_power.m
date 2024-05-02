function func_plot_power(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix)

% ms = ms;
k = ms.true_time.counter;
plot_SC_number = ms.plot_parameters.plot_SC_number;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Power
fig_number = fig_number+1;
plot_handle = figure(fig_number);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');


subplot(2,3,1)
hold on

plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Power_Generated_Consumed_Energy_Unused(1:k,1),'-k','LineWidth',2)
plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Power_Generated_Consumed_Energy_Unused(1:k,2),'-r','LineWidth',2)

grid on
legend('Generated','Consumed')
xlabel('Time [sec]')
ylabel('Power [Watts]')
title('SC Power Generated and Consumed','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

subplot(2,3,2)
hold on

plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Power_Generated_Consumed_Energy_Unused(1:k,3),'-k','LineWidth',2)

grid on
legend('Energy Unused')
xlabel('Time [sec]')
ylabel('Energy [Watts * hr]')
title('SC Energy Unused','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

subplot(2,3,4)
hold on

for i = 1:1:mission_true_SC{plot_SC_number}.true_SC_solar_panel.num_solar_panels
    plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Solar_Panel(1:k,mission_true_SC{plot_SC_number}.true_SC_solar_panel.num_solar_panels+i),'--','LineWidth',1,'Color',ms.plot_parameters.color_array(i),'DisplayName',['Max ',mission_true_SC{plot_SC_number}.true_SC_solar_panel.solar_panel_data(i).name])
end

for i = 1:1:mission_true_SC{plot_SC_number}.true_SC_solar_panel.num_solar_panels
    plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Solar_Panel(1:k,i),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(i),'DisplayName',mission_true_SC{plot_SC_number}.true_SC_solar_panel.solar_panel_data(i).name)
end

grid on
legend
xlabel('Time [sec]')
ylabel('Power [Watts]')
title('Solar Panel Power Generated','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,3)
hold on

for i = 1:1:mission_true_SC{plot_SC_number}.true_SC_battery.num_battery
    plot([ms.true_time.time_array(1) ms.true_time.time_array(k)],mission_true_SC{plot_SC_number}.true_SC_battery.battery_data(1).maximum_capacity*[1 1],'--','LineWidth',1,'Color',ms.plot_parameters.color_array(i),'DisplayName',['Max ',mission_true_SC{plot_SC_number}.true_SC_battery.battery_data(i).name])
end

for i = 1:1:mission_true_SC{plot_SC_number}.true_SC_battery.num_battery
    plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Battery(1:k,i),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(i),'DisplayName',mission_true_SC{plot_SC_number}.true_SC_battery.battery_data(i).name)
end

grid on
legend
xlabel('Time [sec]')
ylabel('Energy Capacity [Watts * hrs]')
title('Battery Energy Capacity','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,6)
hold on

for i = 1:1:mission_true_SC{plot_SC_number}.true_SC_battery.num_battery
    plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Battery(1:k,mission_true_SC{plot_SC_number}.true_SC_battery.num_battery+i),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(i),'DisplayName',mission_true_SC{plot_SC_number}.true_SC_battery.battery_data(i).name)
end

grid on
legend
xlabel('Time [sec]')
ylabel('Percentage (%)')
title('Battery State of Charge','FontSize',ms.plot_parameters.title_font_size)
%             ylim([0 100])
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

subplot(2,3,5)
hold on

for i = 1:1:mission_true_SC{plot_SC_number}.true_SC_battery.num_battery
    plot(ms.true_time.time_array(1:k),ms.true_SC{plot_SC_number}.SC_Battery(1:k,2*mission_true_SC{plot_SC_number}.true_SC_battery.num_battery+i),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(i),'DisplayName',mission_true_SC{plot_SC_number}.true_SC_battery.battery_data(i).name)
end

grid on
legend
xlabel('Time [sec]')
ylabel('Power [Watts]')
title('Battery Instantaneous Power','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off



sgtitle(['Power onboard SC',num2str(plot_SC_number)],'FontSize',ms.plot_parameters.title_font_size,'FontName',ms.plot_parameters.standard_font_type)

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Power.png'])
end

end