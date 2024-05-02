function func_plot_attitude_control( ...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_true_small_body, ...
    mission_true_solar_system, ...
    mission_init_data, ...
    mission_true_SC, ...
    fig_number, ...
    plot_name_prefix, ...
    xlimits)

if nargin < 9
    xlimits = [];
end

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

tspan = ms.true_time.time_array / 3600;
true_SC_i = ms.true_SC{i_SC};
colors = num2cell(ms.plot_parameters.color_array);
% colors = func_get_plot_colors();
rpm_per_radps = 60/2/pi;
font_size = ms.plot_parameters.standard_font_size;
N_RW = mission_true_SC{i_SC}.true_SC_rwa_actuator.num_reaction_wheel;

% ******************* Actuator in use *******************
subplot(2,3,1)
hold on

fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'r','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Desat (both)')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','RWA only')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','MT only')

plot(tspan(1:k),true_SC_i.actuator_to_use(1:k),'-k','LineWidth',2,'DisplayName','Actuator in Use')

grid on
legend('Location','southeast')
xlabel('Time [hours]')
ylabel('MT-RWA Selection')
title('Actuator in Use')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end

% ******************* RW Torque *******************
subplot(2,3,2)
hold on

for i=1:N_RW
    J_xx = mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(i).J_xx;
    acc = true_SC_i.rwa_actuator_angular_acceleration(1:k,i);
    torque = J_xx * acc;
    plot(tspan(1:k),torque, ...
        '-','LineWidth',2,'Color',colors{i},'DisplayName',['RW ',num2str(i)])
end

% Limits
max_torque = mission_true_SC{i_SC}.true_SC_rwa_actuator.rw_maximum_torque;
plot([tspan(1) tspan(k)],+max_torque * [1 1],'--k','LineWidth',1,'DisplayName','Max')
plot([tspan(1) tspan(k)],-max_torque * [1 1],'--k','LineWidth',1,'HandleVisibility','off')

grid on
legend
xlabel('Time [hours]')
ylabel('Torque [Nm]')
title('RWA Torque')
ylim([-max_torque max_torque])
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end

% ******************* RW Momentum *******************
subplot(2,3,3)
hold on

for i=1:N_RW
    J_xx = mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(i).J_xx;
    vel = true_SC_i.rwa_actuator_angular_velocity(1:k,i);
    momentum = J_xx * vel;
    plot(tspan(1:k),momentum, ...
        '-','LineWidth',2,'Color',colors{i},'DisplayName',['RW ',num2str(i)])
end

% Limits
J_xx = mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(1).J_xx;
vel_max = mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(1).maximum_angular_velocity;
max_momentum = J_xx * vel_max;
plot([tspan(1) tspan(k)],+max_momentum * [1 1],'--k','LineWidth',1,'DisplayName','Max')
plot([tspan(1) tspan(k)],-max_momentum * [1 1],'--k','LineWidth',1,'HandleVisibility','off')

grid on
legend
xlabel('Time [hours]')
ylabel('Momentum [Nms]')
title('RWA Momentum')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end

% ******************* Desired and True Control Torque *******************
subplot(2,3,4)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i .desired_control_torque(1:k,i), ...
        '-','LineWidth',2,'Color',colors{i}, ...
        'DisplayName',['\tau_{des} ',num2str(i)])
end

for i=1:1:3
    plot(tspan(1:k),true_SC_i.control_torque(1:k,i), ...
        '-.','LineWidth',2,'Color',colors{i}, ...
        'DisplayName',['\tau_{true} ',num2str(i)])
end

grid on
legend
xlabel('Time [hours]')
ylabel('Torque [Nm]')
title('Desired and True Control Torque')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end


subplot(2,3,5)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i .desired_control_torque_MT(1:k,i), ...
        '-','LineWidth',2,'Color',colors{i}, ...
        'DisplayName',['\tau_{des} ',num2str(i)])
end

for i=1:1:3
    plot(tspan(1:k),true_SC_i.control_torque_MT(1:k,i), ...
        '-.','LineWidth',2,'Color',colors{i}, ...
        'DisplayName',['\tau_{true} ',num2str(i)])
end

grid on
legend
xlabel('Time [hours]')
ylabel('Torque [Nm]')
title('Desired and True Control Torque (MT only)')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end


subplot(2,3,6)
hold on

for i=1:1:3
    plot(tspan(1:k),true_SC_i .desired_control_torque_RWA(1:k,i), ...
        '-','LineWidth',2,'Color',colors{i}, ...
        'DisplayName',['\tau_{des} ',num2str(i)])
end

for i=1:1:3
    plot(tspan(1:k),true_SC_i.control_torque_RWA(1:k,i), ...
        '-.','LineWidth',2,'Color',colors{i}, ...
        'DisplayName',['\tau_{true} ',num2str(i)])
end

grid on
legend
xlabel('Time [hours]')
ylabel('Torque [Nm]')
title('Desired and True Control Torque (RWA only)')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end

sgtitle(['Reaction Wheel and Desaturation',' / SC',num2str(i_SC)], ...
    'fontsize',ms.plot_parameters.title_font_size,'FontName','Times New Roman')

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Control_attitude.png']);
end

end