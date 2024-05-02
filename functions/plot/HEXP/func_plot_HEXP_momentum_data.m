function fig = func_plot_HEXP_momentum_data( ...
    kd, ...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_init_data, ...
    mission_true_small_body, ...
    mission_true_stars, ...
    mission_true_solar_system, ...
    mission_true_SC, ...
    fig_number)

if kd == -1
    idx_array = find(ms.true_time.time_array_dynamics);
    kd = idx_array(end);
end
k = floor((kd-1) * ms.true_time.time_step_dynamics / ms.true_time.time_step) + 1;

colors = func_get_plot_colors();
font_size = 20; % ms.plot_parameters.standard_font_size;

% Time
tspan = ms.true_time.time_array(1:k) / (24 * 3600); % [days]
i_SC = 1;
true_SC_i = ms.true_SC{i_SC};
% colors = ms.plot_parameters.color_array;

% **************************** Figure ****************************
% figure(fig_number);
% close(fig_number)
fig = figure(fig_number);
clf;
set(fig, 'units', 'normalized', 'outerposition', [0 0 0.5 0.5]);
set(fig,'PaperPositionMode','auto');
set(fig,'Color',[1 1 1]);

% **************************** Reaction wheel torque and momentum ****************************

set(fig,'defaultAxesColorOrder', zeros(2,3));
subplot(2,1,1, 'Parent', fig)
hold on
N_RW = mission_true_SC{i_SC}.true_SC_rwa_actuator.num_reaction_wheel;
for i=1:N_RW
    J_xx = mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(i).J_xx;
    acc = true_SC_i.rwa_actuator_angular_acceleration(1:k,i);
    vel = true_SC_i.rwa_actuator_angular_velocity(1:k,i);
    momentum = J_xx * vel;
    torque = J_xx * acc;
    
    % yyaxis left
    % plot(tspan(1:k),torque, ...
    %     '-','LineWidth',2,'Color',colors(i),'DisplayName',['RW ',num2str(i)])
    
    % yyaxis right
    plot(tspan(1:k),momentum, ...
        '-','LineWidth',2,'Color',colors{i}, 'LineWidth',2,'DisplayName',['RW ',num2str(i)])
end

% Limits
max_torque = mission_true_SC{i_SC}.true_SC_rwa_actuator.rw_maximum_torque;
max_momentum = mission_true_SC{i_SC}.true_SC_rwa_actuator.rw_momentum_capacity;
% plot([tspan(1) tspan(k)],+max_torque * [1 1],'--k','LineWidth',1,'DisplayName','Max')
% plot([tspan(1) tspan(k)],-max_torque * [1 1],'--k','LineWidth',1,'HandleVisibility','off')
plot([tspan(1) tspan(k)],+max_momentum * [1 1],'--k','LineWidth',1,'DisplayName','Max')
plot([tspan(1) tspan(k)],-max_momentum * [1 1],'--k','LineWidth',1,'HandleVisibility','off')

grid on
legend('Location','northwest')
xlabel('Time [days]')

% yyaxis left
% ylabel('Torque [Nm]')
% ylim([-max_torque max_torque])

% yyaxis left
ylabel('Momentum [Nms]')
ylim([-max_momentum max_momentum])

title('Reaction Wheel Stored Momentum')
% xlim([0, 360])
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

% **************************** Data and Power ****************************

subplot(2,1,2, 'Parent', fig)
hold on

% Data volume
yyaxis left
n = mission_true_SC{i_SC}.software_SC_communication.number_communication;
for i =1:mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory
    plot( ...
        tspan,true_SC_i.SC_Data_Volume(1:k,i) * 8, ...
        '-k','LineWidth',2,'DisplayName',...
        ['Data Stored'],'Color',colors{i})
    hold on
end
if false
for i =1:mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory
    plot([tspan(1) tspan(k)], ...
        mission_true_SC{i_SC}.true_SC_onboard_memory.memory_data(i).maximum_data_storage*[1 1] * 8, ...
        '--r','LineWidth',2,'DisplayName', ...
        ['Data Capacity'],'Color',colors{i})
end
end
for i =1:n
    if mission_true_SC{i_SC}.software_SC_communication.comm_direction(i) == 1
        plot( ...
            tspan,true_SC_i.SC_communication_data(1:k,3*n+i) * 8, ...
            '-k','LineWidth',1,'DisplayName', ...
            ['Received from SC ',num2str(mission_true_SC{i_SC}.software_SC_communication.comm_interlocutor(i))],'Color',colors{i})
    end
end
ylabel('On-board Data Volume [Gb]')

% Power
yyaxis right
% plot(tspan(1:k),true_SC_i.SC_Power_Generated_Consumed_Energy_Unused(1:k,2),'-r','LineWidth',1, 'DisplayName','Power Consumed')
plot(tspan(1:k),true_SC_i.SC_Power_Generated_Consumed_Energy_Unused(1:k,1),...
    '-g','LineWidth',2, 'DisplayName','Power Generated', 'Color',colors{5})
ylabel('Power [Watts]')

grid on
legend('Location','northwest')
xlabel('Time [days]')

title('Data Volume and Power')
% xlim([0, 360])
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

if ms.plot_parameters.storage_save_plots == 1
    saveas(fig,[mission_init_data.output_folder,'HEXP_mission.png']);
end

end