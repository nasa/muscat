function fig = func_plot_HEXP( ...
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
time_current = ms.true_time.time_array_dynamics(kd);

colors = func_get_plot_colors();
font_size = ms.plot_parameters.standard_font_size;

% Preprocess data
spice_name = mission_true_SC{1}.true_SC_navigation.spice_name;
launch_epoch = cal2sec('01-JUL-2031 12:00:00');

% Time
dt = 24 * 3600;
n_moon = 30 * 24 * 3600 / dt;
tspan = [launch_epoch : dt : mission_true_time.t_initial_date - dt, mission_true_time.t_initial_date + ms.true_time.time_array_dynamics(1:kd)'];

% Spice
SC_pos_vel = cspice_spkezr(spice_name,tspan,'J2000','NONE','SUN')';
Earth_pos_vel = cspice_spkezr('EARTH',tspan,'J2000','NONE','SUN')';
Moon_pos_vel = cspice_spkezr('MOON',tspan,'J2000','NONE','SUN')';

% L1 position
R = vecnorm(Earth_pos_vel(:,1:3), 2, 2); % Distance Earth-Sun
mu_1 = mission_true_solar_system.mu_Sun;  % Sun gravitational parameter
mu_2 = mission_true_solar_system.mu_Earth; % Earth gravitational parameter
r = R * (mu_2 / (3*mu_1))^(1/3); % Distance Earth-L1
L1_pos = Earth_pos_vel(:,1:3) .* (1 - r./R); % L1 position
L1_pos_vel = [L1_pos, zeros(size(L1_pos))];

% Relative position
SC_pos_plot = Rv2Rtn(Earth_pos_vel, SC_pos_vel);
L1_pos_plot = Rv2Rtn(Earth_pos_vel(1,:)', L1_pos_vel(1,:)');
Moon_pos_plot = Rv2Rtn(Earth_pos_vel(1:n_moon, :), Moon_pos_vel(1:n_moon, :));

% Sun and Earth
i_SC = 1;
rotFrame = RotationEci2Rtn(Earth_pos_vel(end,:)');
direction_sun = rotFrame * ( ...
    ms.true_solar_system.position_array_Sun(kd, :) ...
    - ms.true_SC{i_SC}.position_array(kd,:))';
direction_earth = rotFrame * ( ...
    ms.true_solar_system.position_array_Earth(kd, :) ...
    - ms.true_SC{i_SC}.position_array(kd,:))';
direction_earth = direction_earth / norm(direction_earth);
direction_sun = direction_sun / norm(direction_sun);

% Target
target_idx = ms.true_SC{1}.SC_target_index(k);
target_ra = ms.target_data.ra(target_idx); % [deg]
target_dec = ms.target_data.dec(target_idx); % [deg]
direction_target = rotFrame * Planetocentric2Cartesian([1, deg2rad(target_dec), deg2rad(target_ra)]');

% **************************** Figure ****************************
% figure(fig_number);
% close(fig_number)
fig = figure(fig_number);
clf;
set(fig,'outerposition',[0 0 1600 900])
set(fig,'PaperPositionMode','auto');
set(fig,'Color',[1 1 1]);

% **************************** Trajectory ****************************
subplot(3,2,[1, 3])
et = plot3(SC_pos_plot(:,1),SC_pos_plot(:,2),SC_pos_plot(:,3), ...
    'color',colors{1},'linewidth',1.5,'DisplayName', 'HEXP Trajectory');
hold on
scatter3(SC_pos_plot(end,1),SC_pos_plot(end,2),SC_pos_plot(end,3), ...
    100,colors{1}, 'filled', 'DisplayName', 'HEXP', 'HandleVisibility','off');

mo = plot3(Moon_pos_plot(:,1),Moon_pos_plot(:,2),Moon_pos_plot(:,3), ...
    'color',colors{2},'linewidth',1.5,'DisplayName', 'Moon Orbit');
e = scatter3(0,0,0, 100, 'b', 'filled', 'DisplayName', 'Earth');
% scatter3(L1_pos_plot(1),L1_pos_plot(2),L1_pos_plot(3), ...
%     100,'k','filled', 'DisplayName', 'Sun-Earth L1', 'HandleVisibility','off');
legend_lines = [e, et, mo];

args = {'color', NaN, 'facealpha', 0.5, 'stemWidth', 0.1e5, 'tipWidth', 0.2e5, 'HandleVisibility','off'};
args2 = {'LineWidth', 2};
start_point = SC_pos_plot(end,1:3)';

args{2} = [246,190,0] / 255;
mArrow3(start_point,start_point+direction_sun*5e5, args{:});
p1 = quiver3([],[],[],[],[],[], 'color', args{2}, args2{:}, 'DisplayName', 'Sun');
args{2} = [139,69,19] / 255;
mArrow3(start_point,start_point+direction_earth*5e5, args{:});
p2 = quiver3([],[],[],[],[],[], 'color', args{2}, args2{:}, 'DisplayName', 'Earth');
args{2} = [0.75, 0, 0.75];
mArrow3(start_point,start_point+direction_target*5e5, args{:});
p3 = quiver3([],[],[],[],[],[], 'color', args{2}, args2{:}, 'DisplayName', 'Target');
legend_lines = [legend_lines, p1, p2, p3];

flag_body_frame = false;
rot_matrix_SC = rotFrame * squeeze(ms.true_SC{1}.rotation_matrix_SC(kd,:,:));
if flag_body_frame
    args{2} = 'r';
    mArrow3(start_point,start_point + rot_matrix_SC * [1,0,0]' * 3e5, args{:});
    p4 = quiver3([],[],[],[],[],[], 'color', args{2}, args2{:}, 'DisplayName', 'Sun');
    args{2} = 'g';
    mArrow3(start_point,start_point + rot_matrix_SC * [0,1,0]' * 3e5, args{:});
    p5 = quiver3([],[],[],[],[],[], 'color', args{2}, args2{:}, 'DisplayName', 'Earth');
    args{2} = 'b';
    mArrow3(start_point,start_point + rot_matrix_SC * [0,0,1]' * 3e5, args{:});
    p6 = quiver3([],[],[],[],[],[], 'color', args{2}, args2{:}, 'DisplayName', 'Target');
    legend_lines = [legend_lines, p4, p5, p6];
end

% Plot spacecraft
scale_sc = 0.25e5;
pos = SC_pos_plot(end,1:3)';
func_plot_spacecraft_attitude( ...
    kd, ms, mission_true_SC, i_SC, scale_sc, pos, rotFrame);

% legend('Position', [0.35,0.85,0.1,0.05])
legend(legend_lines, 'Location', 'northeast')
title('Position and Attitude')
axis equal; grid on;
xlim([-25, 5] * 1e5)
ylim([-10, 10] * 1e5)
zlim([-9, 9] * 1e5)
view([-60, 5])
xlabel("X [km]")
ylabel("Y [km]")
zlabel("Z [km]")
set(gca,'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')

% **************************** Camera image ****************************

panel1 = uipanel('Parent', fig, ...
    'Units', 'norm', ...
    'Position', [0.49, 0.47, 0.4*9/16, 0.4], ...
    'BorderType', 'none');
subplot(1,1,1, 'Parent', panel1)
func_plot_image(kd, ms, mission_true_solar_system, mission_true_stars, mission_true_SC)
title('HEXP instrument view')

% Legend
ps = [];
cs = ['y', 'g', 'r'];
labels = {'Current target', 'Observed targets', 'Unobserved targets'};
for i = 1:3
    p = plot3(0, 0, 0,['o',cs(i)],'MarkerSize',5,'MarkerFaceColor',cs(i),'MarkerEdgeColor','k', 'DisplayName', labels{i}');
    ps = [ps, p];
end
% Legend with transparency
l = legend(ps, labels, 'Location', 'northeast');
set(l, 'Color', [1,1,1]*0.8, 'EdgeColor', 'None');
set(panel1, 'BackgroundColor', [12, 22, 79]/255*0.5);

subplot(2,4,3, 'Parent', fig)
title('Instrument FOV', 'Position', [0.38, 0.88])
axis off
set(gca,'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')

% **************************** Desired vs true attitude ****************************

tspan = ms.true_time.time_array(1:k) / 3600;
true_SC_i = ms.true_SC{i_SC};
sc_quaternion = true_SC_i.SC_Quaternion(1:k,:);
colors = ms.plot_parameters.color_array;
subplot(3,3,7, 'Parent', fig)
hold on
for i = 1:4
    plot(tspan, sc_quaternion(:,i), 'LineWidth', 1, 'Color', colors(i), 'DisplayName', ['q_', num2str(i)])
    plot(tspan(2:end),true_SC_i.SC_Desired_Omega_Quaternion(2:k,3+i),'--','LineWidth',2,'Color',colors(i), 'HandleVisibility','off')
end

plot([],[],'-k','LineWidth',1.5,'DisplayName','True')
plot([],[],'--k','LineWidth',1.5,'DisplayName','Desired')

grid on
legend('Location','northwest')
xlabel('Time [hours]')
ylabel('Quaternion')
title('Desired and True Attitude')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

% **************************** Reaction wheel torque and momentum ****************************

set(fig,'defaultAxesColorOrder', zeros(2,3));
subplot(3,3,8, 'Parent', fig)
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
        '-','LineWidth',2,'Color',colors(i),'LineWidth',1,'DisplayName',['RW ',num2str(i)])
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
xlabel('Time [hours]')

% yyaxis left
% ylabel('Torque [Nm]')
% ylim([-max_torque max_torque])

% yyaxis left
ylabel('Momentum [Nms]')
ylim([-max_momentum max_momentum])

title('Reaction Wheel Stored Momentum')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

% **************************** Data and Power ****************************

subplot(3,3,9, 'Parent', fig)
hold on

% Data volume
yyaxis left
n = mission_true_SC{i_SC}.software_SC_communication.number_communication;
for i =1:mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory
    plot(tspan,true_SC_i.SC_Data_Volume(1:k,i),'-k','LineWidth',1,'DisplayName',['Data Stored'],'Color',ms.plot_parameters.color_array(i))
    hold on
end
% for i =1:mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory
%     plot([tspan(1) tspan(k)], mission_true_SC{i_SC}.true_SC_onboard_memory.memory_data(i).maximum_data_storage*[1 1],'--r','LineWidth',2,'DisplayName',['Data Capacity'],'Color',ms.plot_parameters.color_array(i))
% end
for i =1:n
    if mission_true_SC{i_SC}.software_SC_communication.comm_direction(i) == 1
        plot(tspan,true_SC_i.SC_communication_data(1:k,3*n+i),'-k','LineWidth',1,'DisplayName',['Received from SC ',num2str(mission_true_SC{i_SC}.software_SC_communication.comm_interlocutor(i))],'Color',ms.plot_parameters.color_array(i))
    end
end
ylabel('Data Volume [GB]')

% Power
yyaxis right
plot(tspan(1:k),true_SC_i.SC_Power_Generated_Consumed_Energy_Unused(1:k,2),'-r','LineWidth',1, 'DisplayName','Power Consumed')
plot(tspan(1:k),true_SC_i.SC_Power_Generated_Consumed_Energy_Unused(1:k,1),'-g','LineWidth',1, 'DisplayName','Power Generated')
ylabel('Power [Watts]')

grid on
legend('Location','northwest')
xlabel('Time [hours]')

title('Data Volume and Power')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off

% **************************** Attitude Mode ****************************

subplot(3,4,4, 'Parent', fig)
hold on
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','DTE')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Target + DTE')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'c','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Target')

plot(tspan(2:end),true_SC_i.desired_SC_attitude_mode(2:k) - 5,'-k','LineWidth',2,'DisplayName','Mode')

grid on
legend('Location','northwest')
xlabel('Time [hours]')
ylabel('Attitude Mode')
title('Attitude Mode Selection')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off


% **************************** Actuator in Use ****************************

subplot(3,4,8, 'Parent', fig)
hold on
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'r','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Desat (both)')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','RWA only')
fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','MT only')

plot(tspan(2:k),true_SC_i.actuator_to_use(2:k),'-k','LineWidth',2,'DisplayName','Actuator in Use')

grid on
legend('Location','northwest')
xlabel('Time [hours]')
ylabel('Actuator')
title('Actuator in Use')
set(gca, 'fontsize',font_size,'FontName','Times New Roman')
hold off


% **************************** Title ****************************

% 1 = Point camera to SB
% 2 = Maximize SP Power
% 3 = Point Thurster along DeltaV direction
% 4 = Point Antenna to Earth
% 5 = Point Antenna for InterSat comm
% 6 = Point to target (e.g., exoplanet)
attitude_mode_desc = {
    'Point camera to SB'
    'Maximize SP Power'
    'Point Thurster along DeltaV direction'
    'Point Antenna to Earth'
    'Point Antenna for InterSat comm'
    'Point Instrument to Target'
    };
target_name = ms.target_data.name{target_idx};
target_name = strrep(target_name, '_', '\\_');

t = tspan(k);
d = floor(t / 24);
h = floor(t - d * 24);
m = round((t - d * 24 - h) * 60);

args = {'fontsize', 20, 'FontName','Times New Roman'};
sgtitle(sprintf([
    '\\fontsize{20}HEX-P Simulation time\n', num2str(d), 'd ', num2str(h), 'h ', num2str(m), 'm', '\n\\fontsize{14}', ...
    ...     'Attitude mode: ', attitude_mode_desc{ms.true_SC{1}.desired_SC_attitude_mode(k)}, '\n', ...
    'Target: ', target_name, ' (',num2str(target_idx), '/', num2str(length(ms.target_data.name)), ')\n',  ...
    'RA = ', num2str(target_ra), '°, Dec = ', num2str(target_dec), '°', ...
    ]), args{:})
text

if ms.plot_parameters.storage_save_plots == 1
    saveas(fig,[mission_init_data.output_folder,'HEXP_mission.png']);
end

end