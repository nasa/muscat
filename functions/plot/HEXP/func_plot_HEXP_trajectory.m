function func_plot_HEXP_trajectory( ...
    mission_init_data, ...
    mission_true_SC, ...
    mission_true_time, ...
    mission_true_solar_system, ...
    flag_save_plot)

spice_name = mission_true_SC{1}.true_SC_navigation.spice_name;
% Timespan vector: 10 years with 1 week steps
tspan = mission_true_time.t_initial_date + (0:1:365*10)*24*3600;
SC_pos_vel = cspice_spkezr(spice_name,tspan,'J2000','NONE','SUN')';
Earth_pos_vel = cspice_spkezr('EARTH',tspan,'J2000','NONE','SUN')';
Moon_pos_vel = cspice_spkezr('MOON',tspan(1:30),'J2000','NONE','SUN')';

% Find L1 position
R = vecnorm(Earth_pos_vel(:,1:3), 2, 2); % Distance Earth-Sun
mu_1 = mission_true_solar_system.mu_Sun;  % Sun gravitational parameter
mu_2 = mission_true_solar_system.mu_Earth; % Earth gravitational parameter
r = R * (mu_2 / (3*mu_1))^(1/3); % Distance Earth-L1
L1_pos = Earth_pos_vel(:,1:3) .* (1 - r./R); % L1 position
L1_pos_vel = [L1_pos, zeros(size(L1_pos))];

SC_pos_plot = Rv2Rtn(Earth_pos_vel, SC_pos_vel);
L1_pos_plot = Rv2Rtn(Earth_pos_vel(1,:)', L1_pos_vel(1,:)');
Moon_pos_plot = Rv2Rtn(Earth_pos_vel(1:30,:), Moon_pos_vel);

% Plot 3D trajectory
fig = figure(2);
set(fig,'Color',[1 1 1]);
set(fig,'units','normalized','outerposition',[0 0 0.7 1])
set(fig,'PaperPositionMode','auto');
clf;

subplot(1,1,1)
plot3(SC_pos_plot(:,1),SC_pos_plot(:,2),SC_pos_plot(:,3),'linewidth',1.5,'DisplayName', 'HEXP trajectory');
hold on;
plot3(Moon_pos_plot(:,1),Moon_pos_plot(:,2),Moon_pos_plot(:,3),'linewidth',1.5,'DisplayName', 'Moon orbit');
scatter3(0,0,0, 100, 'filled', 'DisplayName', 'Earth', 'HandleVisibility','off');
scatter3(L1_pos_plot(1),L1_pos_plot(2),L1_pos_plot(3), 100, 'filled', 'DisplayName', 'Sun-Earth L1', 'HandleVisibility','off');

legend('Position', [0.7,0.75,0.15,0.05])
axis equal; grid on;
xlabel("X [km]")
ylabel("Y [km]")
zlabel("Z [km]")
set(gca, 'fontsize',20,'FontName','Times New Roman')

if flag_save_plot == 1
    filename = [mission_init_data.output_folder filesep 'SC_trajectory.png'];
    exportgraphics(fig, filename)
end
