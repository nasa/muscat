function func_plot_attitude_control(ms, mission_true_time, mission_true_small_body, mission_true_solar_system, mission_init_data, mission_true_SC, fig_number, plot_name_prefix)

% ms = ms;
k = ms.true_time.counter;
i_SC = ms.plot_parameters.plot_SC_number;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Attitude control

fig_number = fig_number+1;
plot_handle = figure(fig_number);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

tspan = ms.true_time.time_array;
true_SC_i = ms.true_SC{i_SC};
colors = ms.plot_parameters.color_array;
rpm_per_radps = 60/2/pi;
font_size = ms.plot_parameters.standard_font_size;

if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
    
    subplot(2,3,1)
    hold on
    
    for i=1:1:mission_true_SC{i_SC}.true_SC_rwa_actuator.num_reaction_wheel
        plot(tspan(1:k),true_SC_i.rwa_actuator_angular_velocity(1:k,i) * rpm_per_radps,'-','LineWidth',2,'Color',colors(i),'DisplayName',['RW ',num2str(i)])
    end
    plot([tspan(1) tspan(k)],+mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(1).maximum_angular_velocity* rpm_per_radps * [1 1],'--r','LineWidth',2,'DisplayName','Max RPM')
    plot([tspan(1) tspan(k)],-mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(1).maximum_angular_velocity* rpm_per_radps * [1 1],'-.r','LineWidth',2,'DisplayName','Min RPM')
    
    grid on
    legend
    xlabel('Time [sec]')
    ylabel('Angular Velocity [RPM]')
    title('RWA Angular Velocity','FontSize',ms.plot_parameters.title_font_size)
    set(gca, 'FontSize',font_size,'FontName',ms.plot_parameters.standard_font_type)
    hold off
    
end


subplot(2,3,2)
hold on

fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'r','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Desat (both)')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','RWA only')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','MT only')

plot(tspan(1:k),true_SC_i.actuator_to_use(1:k),'-k','LineWidth',2,'DisplayName','Actuator in Use')

grid on
legend('Location','southeast')
xlabel('Time [sec]')
ylabel('MT-RWA Selection')
title('Actuator in Use','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,3)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i.desired_desat_control_torque(1:k,i), ...
        '-','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{desat} ',num2str(i)])
end

grid on
legend
xlabel('Time [sec]')
ylabel('Torque [Nm]')
title('Desired Desat Control Torque','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,4)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i .desired_control_torque(1:k,i), ...
        '-','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{des} ',num2str(i)])
end

for i=1:1:3
    plot(tspan(1:k),true_SC_i.control_torque(1:k,i), ...
        '-.','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{true} ',num2str(i)])
end

grid on
legend
xlabel('Time [sec]')
ylabel('Torque [Nm]')
title('Desired and True Control Torque','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,5)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i .desired_control_torque_MT(1:k,i), ...
        '-','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{des} ',num2str(i)])
end

for i=1:1:3
    plot(tspan(1:k),true_SC_i.control_torque_MT(1:k,i), ...
        '-.','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{true} ',num2str(i)])
end

grid on
legend
xlabel('Time [sec]')
ylabel('Torque [Nm]')
title('Desired and True Control Torque (MT only)','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

subplot(2,3,6)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i .desired_control_torque_RWA(1:k,i), ...
        '-','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{des} ',num2str(i)])
end

for i=1:1:3
    plot(tspan(1:k),true_SC_i.control_torque_RWA(1:k,i), ...
        '-.','LineWidth',2,'Color',colors(i), ...
        'DisplayName',['\tau_{true} ',num2str(i)])
end

grid on
legend
xlabel('Time [sec]')
ylabel('Torque [Nm]')
title('Desired and True Control Torque (RWA only)','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

sgtitle(['SC',num2str(i_SC),' Reaction Wheel and Desaturation',],'FontSize',ms.plot_parameters.title_font_size,'FontName',ms.plot_parameters.standard_font_type)

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Control_attitude.png']);
end

end