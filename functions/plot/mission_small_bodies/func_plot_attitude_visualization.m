function func_plot_attitude_visualization(...
    ms, ...
    mission_true_time, ...
    mission_true_small_body, ...
    mission_true_solar_system, ...
    mission_init_data, ...
    mission_true_SC, ...
    fig_number, ...
    plot_name_prefix)

% ms = ms;
k = ms.true_time.counter;
kd = ms.true_time.counter_dynamics;

%% Attitude Visualization
fig_number = fig_number+1;
plot_handle = figure(fig_number);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

i_SC = ms.plot_parameters.plot_SC_number;
true_SC_i = ms.true_SC{i_SC};
tspan = ms.true_time.time_array(1:k);
sc_quaternion = true_SC_i.SC_Quaternion(1:k,:);
sc_est_quaternion = true_SC_i.SC_Estimate_Omega_Quaternion(1:k,:);
colors = ms.plot_parameters.color_array;

subplot(2,3,1)
hold on
plot(tspan,sc_quaternion(:,1),'LineWidth',2,'Color',colors(1))
plot(tspan,sc_quaternion(:,2),'LineWidth',2,'Color',colors(2))
plot(tspan,sc_quaternion(:,3),'LineWidth',2,'Color',colors(3))
plot(tspan,sc_quaternion(:,4),'LineWidth',2,'Color',colors(4))

plot(tspan,sc_est_quaternion(:,4),'--','LineWidth',2,'Color',colors(1))
plot(tspan,sc_est_quaternion(:,5),'--','LineWidth',2,'Color',colors(2))
plot(tspan,sc_est_quaternion(:,6),'--','LineWidth',2,'Color',colors(3))
plot(tspan,sc_est_quaternion(:,7),'--','LineWidth',2,'Color',colors(4))

ylim([-1 1])

grid on
legend('\beta_1 (True)','\beta_2 (True)', '\beta_3 (True)', '\beta_4 (Scalar) (True)','\beta_1 (Estimated)','\beta_2 (Estimated)', '\beta_3 (Estimated)', '\beta_4 (Scalar) (Estimated)')
xlabel('Time [sec]')
ylabel('Quaternion')
title('Quaternion: True vs Estimated','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,2)

semilogy(tspan, abs(sc_quaternion(:,1)-sc_est_quaternion(:,4)),'-','LineWidth',2,'Color',colors(1))
hold on
semilogy(tspan, abs(sc_quaternion(:,2)-sc_est_quaternion(:,5)),'-','LineWidth',2,'Color',colors(2))
semilogy(tspan, abs(sc_quaternion(:,3)-sc_est_quaternion(:,6)),'-','LineWidth',2,'Color',colors(3))
semilogy(tspan, abs(sc_quaternion(:,4)-sc_est_quaternion(:,7)),'-','LineWidth',2,'Color',colors(4))

semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,4),'--','LineWidth',2,'Color',colors(1))
semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,5),'--','LineWidth',2,'Color',colors(2))
semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,6),'--','LineWidth',2,'Color',colors(3))
semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,7),'--','LineWidth',2,'Color',colors(4))

grid on
legend('True error in \beta_1', 'True error in \beta_2', 'True error in \beta_3', 'True error in \beta_4', 'Estimated 1-\sigma error in \beta_1','Estimated 1-\sigma error in \beta_2', 'Estimated 1-\sigma error in \beta_3', 'Estimated 1-\sigma error in \beta_4 (Scalar)')
xlabel('Time [sec]')
ylabel('Estimation Error')
title('Estimation Error in Quaternion','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,3)
hold on

plot(tspan,true_SC_i.SC_Desired_Omega_Quaternion(1:k,4),'-.','LineWidth',2,'Color',colors(1))
plot(tspan,true_SC_i.SC_Desired_Omega_Quaternion(1:k,5),'-.','LineWidth',2,'Color',colors(2))
plot(tspan,true_SC_i.SC_Desired_Omega_Quaternion(1:k,6),'-.','LineWidth',2,'Color',colors(3))
plot(tspan,true_SC_i.SC_Desired_Omega_Quaternion(1:k,7),'-.','LineWidth',2,'Color',colors(4))

ylim([-1 1])

grid on
legend('\beta_1 (Desired)','\beta_2 (Desired)', '\beta_3 (Desired)', '\beta_4 (Scalar) (Desired)')
xlabel('Time [sec]')
ylabel('Quaternion')
title('Desired Quaternion','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,4)
hold on
plot(tspan,true_SC_i.SC_Angular_Velocity(1:k,1),'LineWidth',2,'Color',colors(1))
plot(tspan,true_SC_i.SC_Angular_Velocity(1:k,2),'LineWidth',2,'Color',colors(2))
plot(tspan,true_SC_i.SC_Angular_Velocity(1:k,3),'LineWidth',2,'Color',colors(3))

plot(tspan,sc_est_quaternion(:,1),'--','LineWidth',2,'Color',colors(1))
plot(tspan,sc_est_quaternion(:,2),'--','LineWidth',2,'Color',colors(2))
plot(tspan,sc_est_quaternion(:,3),'--','LineWidth',2,'Color',colors(3))

grid on
legend('\omega_1 (True)','\omega_2 (True)', '\omega_3 (True)', '\omega_1 (Estimated)','\omega_2 (Estimated)', '\omega_3 (Estimated)')
xlabel('Time [sec]')
ylabel('Angular Velocity [rad/sec]')
title('Angular Velocity: True vs Estimated','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

subplot(2,3,5)

semilogy(tspan, abs(true_SC_i.SC_Angular_Velocity(1:k,1)-sc_est_quaternion(:,1)),'-','LineWidth',2,'Color',colors(1))
hold on
semilogy(tspan, abs(true_SC_i.SC_Angular_Velocity(1:k,2)-sc_est_quaternion(:,2)),'-','LineWidth',2,'Color',colors(2))
semilogy(tspan, abs(true_SC_i.SC_Angular_Velocity(1:k,3)-sc_est_quaternion(:,3)),'-','LineWidth',2,'Color',colors(3))

semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,1),'--','LineWidth',2,'Color',colors(1))
semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,2),'--','LineWidth',2,'Color',colors(2))
semilogy(tspan,true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,3),'--','LineWidth',2,'Color',colors(3))

grid on
legend('True error in \omega_1', 'True error in \omega_2', 'True error in \omega_3', 'Estimated 1-\sigma error in \omega_1','Estimated 1-\sigma error in \omega_2', 'Estimated 1-\sigma error in \omega_3')
xlabel('Time [sec]')
ylabel('Estimation Error')
title('Estimation Error in Angular Velocity','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off



subplot(2,3,6)
hold on

fill([tspan(1) tspan(k) tspan(k) tspan(1)],[4.75 4.75 5.25 5.25],'m','EdgeColor','none','FaceAlpha',0.5,'DisplayName','INTERSAT comm')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[3.75 3.75 4.25 4.25],'r','EdgeColor','none','FaceAlpha',0.5,'DisplayName','DTE comm')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','\Delta V')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Sun')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'c','EdgeColor','none','FaceAlpha',0.5,'DisplayName','SB')

plot(tspan,true_SC_i.desired_SC_attitude_mode(1:k),'-k','LineWidth',2,'DisplayName','Mode')

grid on
legend('Location','southeast')
xlabel('Time [sec]')
ylabel('Attitude Mode')
title('Attitude Mode Selection','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


sgtitle(['SC',num2str(i_SC),' Attitude Estimator Performance'],'fontsize',ms.plot_parameters.title_font_size,'FontName',ms.plot_parameters.standard_font_type)

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Attitude_estimator',date,'.png'])
end

end