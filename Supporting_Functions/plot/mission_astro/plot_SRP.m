
% Solar Radiation Pressure
Rot = eye(3);
k = mission_storage.true_time.counter;
colors = func_get_plot_colors();
t_srp = mission_storage.true_time.time_array(2:k);
srp_torque_body = mission_storage.true_SC{1}.disturbance_torque_SRP(2:k,1:3) * Rot';
srp_force_J2000 = mission_storage.true_SC{1}.disturbance_force_SRP(2:k,1:3);
srp_force_body = zeros(length(t_srp),3);
for ti = 1:length(t_srp)
    rot = squeeze(mission_storage.true_SC{1}.rotation_matrix_SC(ti,:,:))';
    srp_force_body(ti,:)  = Rot * rot * srp_force_J2000(ti,:)';
end

subplot(2,2,2)
hold on
plot(t_srp,srp_torque_body(:,1),'LineWidth',2,'Color',colors{1})
plot(t_srp,srp_torque_body(:,2),'LineWidth',2,'Color',colors{2})
plot(t_srp,srp_torque_body(:,3),'LineWidth',2,'Color',colors{3})

grid on
legend('T_{SRP,x}', 'T_{SRP,y}', 'T_{SRP,z}')
xlabel('Time [sec]')
ylabel('SRP Torque [Nm]')
title('SRP Torque (Body Frame)')
set(gca, 'fontsize',mission_storage.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

subplot(2,2,4)
hold on
plot(t_srp,srp_force_body(:,1),'LineWidth',2,'Color',colors{1})
plot(t_srp,srp_force_body(:,2),'LineWidth',2,'Color',colors{2})
plot(t_srp,srp_force_body(:,3),'LineWidth',2,'Color',colors{3})

grid on
legend('T_{SRP,x}', 'T_{SRP,y}', 'T_{SRP,z}')
xlabel('Time [sec]')
ylabel('SRP Force [N]')
title('SRP Force (Body Frame)')
set(gca, 'fontsize',mission_storage.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off
