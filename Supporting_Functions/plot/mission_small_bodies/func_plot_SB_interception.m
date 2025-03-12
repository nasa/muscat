function func_plot_SB_interception(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix)

% ms = ms;
kd = ms.true_time.counter_dynamics;
plot_SC_number = ms.plot_parameters.plot_SC_number;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SB interception
fig_number = fig_number+1;
plot_handle = figure(fig_number);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

subplot(2,3,1)
hold on

plot3(ms.true_SC{plot_SC_number}.position_array(kd,1), ms.true_SC{plot_SC_number}.position_array(kd,2), ms.true_SC{plot_SC_number}.position_array(kd,3), 'or','MarkerSize',10)
plot3(ms.true_small_body.position_array(kd,1), ms.true_small_body.position_array(kd,2), ms.true_small_body.position_array(kd,3), 'ok','MarkerSize',10)

plot3(ms.true_SC{plot_SC_number}.position_array(1:kd,1), ms.true_SC{plot_SC_number}.position_array(1:kd,2), ms.true_SC{plot_SC_number}.position_array(1:kd,3), '-r','LineWidth',2)
plot3(ms.true_small_body.position_array(1:kd,1), ms.true_small_body.position_array(1:kd,2), ms.true_small_body.position_array(1:kd,3), '-k','LineWidth',2)

grid on
legend('SC','SB')
xlabel('X Axis [km]')
ylabel('Y Axis [km]')
zlabel('Z Axis [km]')
title('Position of SC and SB','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,2)
hold on

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_SB_Distance(1:kd,1) ,'-r','LineWidth',2)
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_SB_Distance(1:kd,2) ,'--b','LineWidth',2)
plot(ms.true_time.time_array_dynamics(kd),ms.true_SC{plot_SC_number}.SC_SB_Distance(kd,1),'or','MarkerSize',10)

plot(ms.true_time.time_array_dynamics(1:kd), ones(length(ms.true_time.time_array_dynamics(1:kd)), 1).*mission_true_small_body.radius*mission_true_SC{1}.true_SC_navigation.ratio_orbit_radius_SB_radius, '--k','LineWidth',2)

%plot(ms.true_time.time_array_dynamics(1:kd), vecnorm(ms.true_SC{plot_SC_number}.desired_distance(1:kd, 1:3), 2,2), 'k','LineWidth',2);

grid on
xlabel('Time [sec]')
ylabel('Distance [km]')
legend('True','Estimated by SC')
title('Distance between SC and SB','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,3)
semilogy(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimated_Desired_Intercept_Distance(1:kd,4),'-m','LineWidth',2)
hold on
semilogy(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimated_Desired_Intercept_Distance(1:kd,1),'-k','LineWidth',2)
semilogy(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimated_Desired_Intercept_Distance(1:kd,2),'--b','LineWidth',2)
semilogy([ms.true_time.time_array_dynamics(1) ms.true_time.time_array_dynamics(kd)],mission_true_small_body.radius*[1 1],'-.r','LineWidth',2)

grid on
legend('True','Estimated','Desired','SB Radius')
xlabel('Time [sec]')
ylabel('Intercept Distance [km]')
title('Intercept Distance between SC and SB','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,4)
hold on
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Desired_Control_DeltaV(1:kd,1),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Desired_Control_DeltaV(1:kd,2),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Desired_Control_DeltaV(1:kd,3),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('Estimated','Desired')
xlabel('Time [sec]')
ylabel('Velocity [m/sec]')
title('Desired DeltaV','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,5)
hold on
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_True_Command_Thrust(1:kd,1),'-k','LineWidth',2)
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_True_Command_Thrust(1:kd,2),'-.b','LineWidth',2)

grid on
legend('True','SC Estimated')
xlabel('Time [sec]')
ylabel('Thrust [N]')
title('Chemical Thrusters Thrust','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,6)
hold on
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimated_Desired_Intercept_Distance(1:kd,3),'--b','LineWidth',2)

grid on
legend('Desired DeltaV needs to be executed')
xlabel('Time [sec]')
ylabel('Boolean')
title('Desired DeltaV needs to be executed Flag','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


sgtitle(['Distance between SC',num2str(plot_SC_number),' and SB, Thruster Performance'],'FontSize',ms.plot_parameters.title_font_size,'FontName',ms.plot_parameters.standard_font_type)

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Interception_SB.png'])
end
end