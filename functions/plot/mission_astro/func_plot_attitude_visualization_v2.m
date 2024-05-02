function func_plot_attitude_visualization(...
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
tspan = ms.true_time.time_array(1:k) / 3600;
sc_quaternion = true_SC_i.SC_Quaternion(1:k,:);
sc_est_quaternion = true_SC_i.SC_Estimate_Omega_Quaternion(1:k,4:7);
sc_est_omega = true_SC_i.SC_Estimate_Omega_Quaternion(1:k,1:3);
colors = ms.plot_parameters.color_array;


% eul = quat2eul(sc_quaternion) * 180/pi;
% eul_est = quat2eul(sc_est_quaternion) * 180/pi;

% True vs Estimated Roll/Pitch/Yaw ***********************************
subplot(2,3,1)
hold on

% plot(tspan,eul(:,1),'LineWidth',2,'Color',colors(1), 'DisplayName', '\alpha')
% plot(tspan,eul(:,2),'LineWidth',2,'Color',colors(2), 'DisplayName', '\beta')
% plot(tspan,eul(:,3),'LineWidth',2,'Color',colors(3), 'DisplayName', '\gamma')

% plot(tspan,eul_est(:,1),'--','LineWidth',2,'Color',colors(1), 'HandleVisibility','off');
% plot(tspan,eul_est(:,2),'--','LineWidth',2,'Color',colors(2), 'HandleVisibility','off');
% plot(tspan,eul_est(:,3),'--','LineWidth',2,'Color',colors(3), 'HandleVisibility','off');
% plot([], [], 'k--', 'LineWidth', 2, 'DisplayName', 'Estimated', 'HandleVisibility', 'on');

ps = [];
for i = 1:4
    kwargs = {'HandleVisibility', 'off', 'Color', colors(i)};
    if i == 4
        label = ['\beta_', num2str(i) , ' (Scalar)'];
    else
        label = ['\beta_', num2str(i)];
    end
    ps(i) = plot(tspan, true_SC_i.SC_Quaternion(1:k,i), '-', 'LineWidth', 1, ...
        'Color', colors(i), 'HandleVisibility', 'on', 'DisplayName', label);
    plot(tspan, true_SC_i.SC_Estimate_Omega_Quaternion(1:k,3+i), '--', 'LineWidth', 1.5, kwargs{:});
    plot(tspan, true_SC_i.SC_Desired_Omega_Quaternion_Control(1:k,3+i), '-.', 'LineWidth', 2, kwargs{:})
    plot(tspan, true_SC_i.SC_Desired_Omega_Quaternion(1:k,3+i), ':', 'LineWidth', 1, kwargs{:})
end

ps(5) = plot(NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'True', 'HandleVisibility', 'on');
ps(6) = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Estimate', 'HandleVisibility', 'on');
ps(7) = plot(NaN, NaN, 'k-.', 'LineWidth', 2, 'DisplayName', 'Control', 'HandleVisibility', 'on');
ps(8) = plot(NaN, NaN, 'k:', 'LineWidth', 2, 'DisplayName', 'Desired', 'HandleVisibility', 'on');

ylim([-1,1])

grid on
legend(ps, 'Location', 'southwest')
xlabel('Time [hours]')
ylabel('Quaternion')
title('Quaternion')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end

% Quaternion Uncertainty ***********************************
subplot(2,3,2)

hold on
sigma_fill = 3 * true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,:);
for i = 4:7
    % args ={'FaceColor',colors(i),'FaceAlpha',0.08,'EdgeColor', 'none', 'HandleVisibility','off'};
    args = {':', 'LineWidth', 2, 'Color', colors(i), 'HandleVisibility', 'off'};
    plot(tspan,+sigma_fill(:,i),args{:})
    plot(tspan,-sigma_fill(:,i),args{:})
end

for i = 1:3
    plot(tspan, sc_quaternion(:,i)-sc_est_quaternion(:,i),'-','LineWidth',2,'Color',colors(i))
end

ylim([-1,1])
grid on
legend('\beta_1', '\beta_2', '\beta_3', '\beta_4')
xlabel('Time [hours]')
ylim(1.5 * mean(sigma_fill(:,4:6), 'all') * [-1,1])
ylabel('Estimation Error')
title('Estimation Error in Quaternion')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end


subplot(2,3,3)
hold on

for i = 1:4
    kwargs = {'HandleVisibility', 'off', 'Color', colors(i)};
    plot(tspan, true_SC_i.SC_Desired_Omega_Quaternion_Control(1:k,3+i), '-.', 'LineWidth', 1, kwargs{:})
    plot(tspan, true_SC_i.SC_Desired_Omega_Quaternion(1:k,3+i), ':', 'LineWidth', 1, kwargs{:})
end

ylim([-1,1])

grid on
% legend('\beta_1 (Desired)','\beta_2 (Desired)', '\beta_3 (Desired)', '\beta_4 (Scalar) (Desired)')
xlabel('Time [hours]')
ylabel('Quaternion')
title('Desired Quaternion')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end

% Angular Velocity ***********************************

subplot(2,3,4)
hold on
ps = [];
for i = 1:3
    kwargs = {'HandleVisibility', 'off', 'Color', colors(i)};
    label = ['\omega_', num2str(i) , ' (Scalar)'];
    ps(i) = plot(tspan, true_SC_i.SC_Angular_Velocity(1:k,i), '-', 'LineWidth', 1, ...
        'Color', colors(i), 'HandleVisibility', 'on', 'DisplayName', label);
    plot(tspan, true_SC_i.SC_Estimate_Omega_Quaternion(1:k,i), '--', 'LineWidth', 1.5, kwargs{:});
    plot(tspan, true_SC_i.SC_Desired_Omega_Quaternion_Control(1:k,i), '-.', 'LineWidth', 2, kwargs{:})
    plot(tspan, true_SC_i.SC_Desired_Omega_Quaternion(1:k,i), ':', 'LineWidth', 1, kwargs{:})
end

ps(4) = plot(NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'True', 'HandleVisibility', 'on');
ps(5) = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Estimate', 'HandleVisibility', 'on');
ps(6) = plot(NaN, NaN, 'k-.', 'LineWidth', 2, 'DisplayName', 'Control', 'HandleVisibility', 'on');
ps(7) = plot(NaN, NaN, 'k:', 'LineWidth', 2, 'DisplayName', 'Desired', 'HandleVisibility', 'on');

grid on
legend(ps, 'Location', 'southwest')
xlabel('Time [hours]')
ylabel('Angular Velocity [rad/s]')
title('Angular Velocity')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

% Angular velocity error ***********************************
subplot(2,3,5)

hold on
sigma_fill = 3 * true_SC_i.SC_Estimate_Omega_Quaternion_Uncertainty(1:k,:);
for i = 1:3
    % args ={'FaceColor',colors(i),'FaceAlpha',0.0,'EdgeColor', colors(i), 'HandleVisibility','off'};
    args = {':', 'LineWidth', 2, 'Color', colors(i), 'HandleVisibility', 'off'};
    plot(tspan,+sigma_fill(:,i),args{:})
    plot(tspan,-sigma_fill(:,i),args{:})
end

for i = 1:3
    plot(tspan, true_SC_i.SC_Angular_Velocity(1:k,i)-sc_est_omega(:,i),'-','LineWidth',2,'Color',colors(i))
end

grid on
legend('\omega_1', '\omega_2', '\omega_3')
xlabel('Time [hours]')
ylim(1.5 * mean(sigma_fill(:,1:3), 'all') * [-1,1])
ylabel('Estimation Error')
title('Estimation Error in Angular Velocity')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end



subplot(2,3,6)
hold on

fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','DTE')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Target + DTE')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'c','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Target')

plot(tspan(2:end),true_SC_i.desired_SC_attitude_mode(2:k) - 5,'-k','LineWidth',2,'DisplayName','Mode')

grid on
legend('Location','southeast')
xlabel('Time [hours]')
ylabel('Attitude Mode')
title('Attitude Mode Selection')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

if ~isempty(xlimits)
    xlim(xlimits)
end


sgtitle(['Attitude Estimator Performance',' / SC',num2str(i_SC)],'fontsize',ms.plot_parameters.title_font_size,'FontName','Times New Roman')

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Attitude_estimator',date,'.png'])
end
end