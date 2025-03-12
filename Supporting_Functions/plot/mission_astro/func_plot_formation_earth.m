function plot_handle = func_plot_formation_earth( ...
    kd, ...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_init_data, ...
    mission_true_small_body, ...
    mission_true_stars, ...
    mission_true_solar_system, ...
    mission_true_SC)

k = floor((kd-1) * ms.true_time.time_step_dynamics / ms.true_time.time_step) + 1;

% Calculate absolute and position and velocity
num_SC = mission_init_data.num_SC;
rv = cell(1, num_SC);
rtn = cell(1, num_SC);
labels = cell(1, num_SC);
% Retrieve positions (ECI)
for i_SC = 1:num_SC
    rv{i_SC} = [
        ms.true_SC{i_SC}.position_array(1:kd,:) - ms.true_small_body.position_array(1:kd,:), ...
        ms.true_SC{i_SC}.velocity_array(1:kd,:) - ms.true_small_body.velocity_array(1:kd,:)
        ];
    labels{i_SC} = mission_true_SC{i_SC}.true_SC_body.name;
end
% Compute positions (RTN)
for i_SC = 1:num_SC
    rtn{i_SC} = Rv2Rtn(rv{1}, rv{i_SC});
end

% Configure plot
scale_sc = 1.5;
max_x = 5;

% Initialize figure
plot_handle = figure(1); clf;
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'Position',[0 0 1300 900]);
% set(plot_handle,'PaperPositionMode','auto');

% Plot config
linewidth = 1.5;
colors = func_get_plot_colors();

% Formation plot (RTN)
subplot(1,2,1);
grid on; axis equal;

for i_SC = 1:num_SC
    plot3(rtn{i_SC}(:, 1), rtn{i_SC}(:, 2), rtn{i_SC}(:, 3), ...
        'k', 'DisplayName', labels{i_SC}, 'LineWidth', linewidth);
    hold on;
    
    % Plot spacecraft
    Reci2rtn = RotationEci2Rtn(rv{1}(end, :));
    func_plot_spacecraft_attitude( ...
        kd, ms, mission_true_SC, i_SC, scale_sc, rtn{i_SC}(end, 1:3)', Reci2rtn);
    
    % Plot projections
    args = {':', 'Color', [0.5, 0.5, 0.5], 'DisplayName', labels{i_SC}, 'LineWidth', 1};
    plot3(-max_x * ones(size(rtn{i_SC},1),1), rtn{i_SC}(:, 2), rtn{i_SC}(:, 3), args{:});
    plot3(rtn{i_SC}(:, 1), +max_x * ones(size(rtn{i_SC},1),1), rtn{i_SC}(:, 3), args{:});
    plot3(rtn{i_SC}(:, 1), rtn{i_SC}(:, 2), -max_x * ones(size(rtn{i_SC},1),1), args{:});
end

direction_sun = Reci2rtn * (...
    ms.true_solar_system.position_array_Sun(kd, :) - ms.true_SC{i_SC}.position_array(kd,:))';
direction_sun = direction_sun / norm(direction_sun);
direction_earth = [-1, 0, 0];

% Target
target_idx = ms.true_SC{1}.SC_target_index(k);
target_ra = ms.target_data.ra(target_idx); % [deg]
target_dec = ms.target_data.dec(target_idx); % [deg]
direction_target = Reci2rtn * Planetocentric2Cartesian([1, deg2rad(target_dec), deg2rad(target_ra)]');

pos_scale = 0.7*max_x;
args = {'color', 'X', "LineWidth", 3, 'DisplayName','X', "AutoScaleFactor", 0.6*scale_sc, 'MaxHeadSize', 100};
args{2} = [246,190,0] / 255;
args{6} = 'Sun';
p1 = plot_arrow3(pos_scale*direction_sun, direction_sun, args);
args{2} = [139,69,19] / 255;
args{6} = 'Earth';
p2 = plot_arrow3(pos_scale*direction_earth, direction_earth, args);
args{2} = [0.75, 0, 0.75];
args{6} = 'Target';
p3 = plot_arrow3(pos_scale*direction_target, direction_target, args);

legend([p1, p2, p3],'Position',[0.4 0.7 0.05 0.05])
grid on; axis equal
view([47, 15])
xlabel('R [km]')
ylabel('T [km]')
zlabel('N [km]')
title('Spacecraft Position (RTN)')
xlim([-max_x, max_x])
ylim([-max_x, max_x])
zlim([-max_x, max_x])
xticks(-max_x:max_x/2:max_x)
yticks(-max_x:max_x/2:max_x)
zticks(-max_x:max_x/2:max_x)

set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')

% Earth plot (ECI)
subplot(1,2,2);
grid on; hold on; axis equal;

% Plot orbits
for i_SC = 1:1:num_SC
    plot3(rv{i_SC}(:, 1), rv{i_SC}(:, 2), rv{i_SC}(:, 3), ...
        '-','LineWidth',2,'Color',colors{2},'DisplayName',labels{i_SC})
    hold on
    scatter3(rv{i_SC}(end, 1), rv{i_SC}(end, 2), rv{i_SC}(end, 3), ...
        50, colors{2}, 'filled', 'DisplayName', labels{i_SC})
end

direction_sun = Reci2rtn' * direction_sun;
direction_target = Reci2rtn' * direction_target;

SB_radius = mission_true_small_body.radius;
args = {'color', 'X', "LineWidth", 3, 'DisplayName','X', "AutoScaleFactor", 0.3*SB_radius, 'MaxHeadSize', 100};
args{2} = [246,190,0] / 255;
args{6} = 'Sun';
plot_arrow3(1.4 * SB_radius * direction_sun, direction_sun, args);
args{2} = [0.75, 0, 0.75];
args{6} = 'Target';
plot_arrow3(1.4 * SB_radius * direction_target, direction_target, args);

shape_model = mission_true_small_body.shape_model;
switch mission_true_small_body.shape_model_type
    case 'trisurf'
        % Trisurf body
        shape_model.Vertices = (1e-3)*(mission_true_small_body.rotation_matrix * shape_model.Vertices')';
        patch(shape_model, 'FaceColor', 0.7*[1 1 1], 'EdgeColor', 'k', 'DisplayName', mission_true_small_body.name)
        %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)
    case 'sphere'
        [xP,yP,zP] = sphere(50);
        SB_radius = mission_true_small_body.radius;
        hPlanet = surf(xP*SB_radius, yP*SB_radius, zP*SB_radius,'FaceColor','blue','EdgeColor','none', 'DisplayName', '');
        cdata = fliplr(imread(shape_model.img));
        time_earth_example = ms.true_time.time_array_dynamics(kd);
        cdata_time = cdata;
        idx_time_earth_example = mod( floor(size(cdata,2)*time_earth_example/86400), size(cdata,2)) + 1;
        if idx_time_earth_example == 1
            cdata_time = cdata;
        else
            cdata_time(:,1:idx_time_earth_example,:) = cdata(:,size(cdata,2)-idx_time_earth_example+1:size(cdata,2),:);
            cdata_time(:,idx_time_earth_example+1:size(cdata,2),:) = cdata(:,1:size(cdata,2)-idx_time_earth_example,:);
        end
        set(hPlanet,'facecolor','texture','cdata',cdata_time,'edgecolor','none');
    otherwise
        error('Shape model type not recognized')
end

axis equal;
view([47, 15])
xlabel('X [km]')
ylabel('Y [km]')
zlabel('Z [km]')
max_x = round(1.7 * SB_radius, -4);
xlim([-max_x, max_x])
ylim([-max_x, max_x])
zlim([-max_x, max_x])
xticks(-max_x:max_x/2:max_x)
yticks(-max_x:max_x/2:max_x)
zticks(-max_x:max_x/2:max_x)


% Simulation time hours, minutes, seconds
tspan = ms.true_time.time_array_dynamics(1:kd);
h = floor(tspan(kd)/3600);
m = floor((tspan(kd) - h*3600)/60);
s = floor(tspan(kd) - h*3600 - m*60);
title('Spacecraft Position (ECI)')

args = {'fontsize', ms.plot_parameters.standard_font_size,'FontName','Times New Roman'};
set(gca, args{:})

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
args = {'fontsize', 20, 'FontName','Times New Roman'};
sgtitle(sprintf([
    '\\fontsize{20} Simulation time: ', num2str(h), 'h ', num2str(m), 'm ', num2str(s), 's', '\n', ...
    '\\fontsize{14} Attitude mode: ', attitude_mode_desc{ms.true_SC{1}.desired_SC_attitude_mode(k)}, '\n', ...
    'Target: ', target_name, ' (',num2str(target_idx), '/', num2str(length(ms.target_data.name)), ')\n',  ...
    'RA = ', num2str(target_ra), '°, Dec = ', num2str(target_dec), '°', ...
    ]), args{:})
text
hold off
end