function func_plot_attitude_visualization(mission, i_SC)

kd = mission.storage.k_storage_attitude;

%% Attitude Visualization
plot_handle = figure('Name',['SC ',num2str(i_SC),' Attitude Estimator Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


subplot(2,4,1)
hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_adc')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\beta_1 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\beta_2 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\beta_3 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,4), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\beta_s (True)')
end

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_attitude')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,1),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\beta_1 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,2),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\beta_2 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,3),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\beta_3 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,4),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\beta_4 (Estimated)')
end

ylim([-1 1])

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Quaternion')
title('Quaternion','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off


if isfield(mission.true_SC{i_SC}, 'true_SC_adc')

   [roll,pitch,yaw] = func_quat2euler(mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,:));
  
  % Plot Euler 
  subplot(2, 4, 2)  
  hold on
  plot(mission.true_time.store.time_attitude(1:kd), roll(:)*180/pi, '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Roll (True)')
  plot(mission.true_time.store.time_attitude(1:kd), pitch(:)*180/pi, '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pitch (True)')
  plot(mission.true_time.store.time_attitude(1:kd), yaw(:)*180/pi, '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Yaw (True)')
  grid on
  legend('Location','southwest')
  xlabel('Time [sec]')
  ylabel('Angle (deg)')
  title('Euler Angles','FontSize',mission.storage.plot_parameters.title_font_size)
  set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
  hold off
end


if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_attitude')

    subplot(2,4,3)
    hold on

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \beta_1 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \beta_2 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \beta_3 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,4) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,4), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\Delta \beta_4 (True)')

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,1),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \beta_1 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,2),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \beta_2 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,3),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \beta_3 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,4),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\Delta \beta_4 (Estimated)')

    %     semilogy(tspan, abs(sc_quaternion(:,1)-sc_est_quaternion(:,4)),'-','LineWidth',2,'Color',colors(1))
    
    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Estimation Error')
    title('Estimation Error in Attitude','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end



if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

    subplot(2,4,4)
    hold on

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\beta_1 (Desired)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\beta_2 (Desired)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\beta_3 (Desired)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,4), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\beta_4 (Desired)')

    ylim([-1 1])
    
    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Quaternion')
    title('Desired Quaternion','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Angular Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


subplot(2,4,5)
hold on
if isfield(mission.true_SC{i_SC}, 'true_SC_adc')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\omega_1 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\omega_2 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\omega_3 (True)')
end

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_attitude')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,1),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\omega_1 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,2),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\omega_2 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,3),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\omega_3 (Estimated)')      
end

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('\omega [rad/sec]')
title('Angular Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_attitude')

    subplot(2,4,6)
    hold on

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \omega_1 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \omega_2 (True)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \omega_3 (True)')

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \omega_1 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \omega_2 (Estimated)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \omega_3 (Estimated)')

    %     semilogy(tspan, abs(true_SC_i.SC_Angular_Velocity(1:k,1)-sc_est_quaternion(:,1)),'-','LineWidth',2,'Color',colors(1))
    
    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Estimation Error [rad/sec]')
    title('Estimation Error in Angular Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude Control % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

    subplot(2,4,7)
    hold on

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\omega_1 (Desired)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\omega_2 (Desired)')
    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\omega_3 (Desired)')    

    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('\omega [rad/sec]')
    title('Desired Angular Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Angle Error between Desired Attitude and Estimated Attitude % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

    subplot(2,4,8)
    hold on

    plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.error_angle_desired_attitude(1:kd), '-','LineWidth',2,'Color','k')

    grid on
    %     legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Error angle [rad]')
    title('Angle between {\beta}_{desired} and {\beta}_{estimated}','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end


sgtitle(['SC ',num2str(i_SC),' Attitude Estimator Performance'],'fontsize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Attitude_Estimator.png'])
end

end
