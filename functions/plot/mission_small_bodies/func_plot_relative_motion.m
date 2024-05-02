function func_plot_relative_motion( ...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_true_small_body, ...
    mission_true_solar_system, ...
    mission_init_data, ...
    mission_true_SC, ...
    fig_number, ...
    plot_name_prefix)

kd = ms.true_time.counter_dynamics;

% Calculate absolute and position and velocity
num_SC = mission_init_data.num_SC;

tspan = ms.true_time.time_array_dynamics(1:kd);

rv = cell(1, num_SC);
rtn = cell(1, num_SC);
for i_SC = 1:num_SC
    rv{i_SC} = [
        ms.true_SC{i_SC}.position_array(1:kd,:) - ms.true_small_body.position_array(1:kd,:), ...
        ms.true_SC{i_SC}.velocity_array(1:kd,:) - ms.true_small_body.velocity_array(1:kd,:)
        ];
end
for i_SC = 1:num_SC
    rtn{i_SC} = 1e3 * Rv2Rtn(rv{1}, rv{i_SC});
end


% Configure plot
scale_sc = 1e3;
max_x = 5e3;
gray_color = [0.5, 0.5, 0.5];

% Initialize figure
plot_handle = figure(); clf;
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

lw = 1.5; marker_size = 75;

n_colors = 10;
cmap = colormap(lines(n_colors));
colors = cell(n_colors);
for i = 1:n_colors
    colors{i} = cmap(i, :);
end
labels = cell(1, num_SC);
for i = 1:num_SC
    labels{i} = ['SC ', num2str(i)];
end

subplot(1,2,1);
grid on; hold on; axis equal;
for i_SC = 1:num_SC
    plot3(rtn{i_SC}(:, 1), rtn{i_SC}(:, 2), rtn{i_SC}(:, 3), 'Color', colors{i_SC}, 'DisplayName', labels{i_SC}, 'LineWidth', lw);
    funct_plot_spacecraft(mission_storage, mission_true_SC, i_SC, scale_sc, rtn{i_SC}(end, 1:3)');
    % p(i_SC) = scatter3(rtn{i_SC}(end, 1), rtn{i_SC}(end, 2), rtn{i_SC}(end, 3), marker_size, 'filled', 'MarkerFaceColor', colors{i_SC});
    
    plot3(-max_x * ones(size(rtn{i_SC},1),1), rtn{i_SC}(:, 2), rtn{i_SC}(:, 3), 'Color', gray_color, 'DisplayName', labels{i_SC}, 'LineWidth', lw);
    plot3(rtn{i_SC}(:, 1), +max_x * ones(size(rtn{i_SC},1),1), rtn{i_SC}(:, 3), 'Color', gray_color, 'DisplayName', labels{i_SC}, 'LineWidth', lw);
    plot3(rtn{i_SC}(:, 1), rtn{i_SC}(:, 2), -max_x * ones(size(rtn{i_SC},1),1), 'Color', gray_color, 'DisplayName', labels{i_SC}, 'LineWidth', lw);
end

view([47, 15])
xlabel('R [m]')
ylabel('T [m]')
zlabel('N [m]')
xlim([-max_x, max_x])
ylim([-max_x, max_x])
zlim([-max_x, max_x])
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')

subplot(1,2,2)

for i_SC = 1:1:num_SC
    
    plot3(rv{i_SC}(1:kd,1), rv{i_SC}(1:kd,2), rv{i_SC}(1:kd,3), '-','LineWidth',5, 'Color',ms.plot_parameters.color_array(i_SC), 'DisplayName',mission_true_SC{i_SC}.true_SC_body.name);
    hold on
end

this_SB_Shape_Model = mission_true_small_body.shape_model;
switch mission_true_small_body.shape_model_type
    case 'trisurf'
        this_SB_Shape_Model.Vertices = (1e-3)*(mission_true_small_body.rotation_matrix * this_SB_Shape_Model.Vertices')';
        patch(this_SB_Shape_Model, 'FaceColor', 0.7*[1 1 1], 'EdgeColor', 'k', 'DisplayName', mission_true_small_body.name)
        %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)
    case 'sphere'
        % TODO: Add rotation
        
        [xP,yP,zP] = sphere(50);
        SB_radius = mission_true_small_body.radius;
        hPlanet = surf(xP*SB_radius, yP*SB_radius, zP*SB_radius,'FaceColor','blue','EdgeColor','none', 'DisplayName', '');
        
        % image_file = 'http://upload.wikimedia.org/wikipedia/commons/thumb/c/cd/Land_ocean_ice_2048.jpg/1024px-Land_ocean_ice_2048.jpg';
        % cdata = imread(image_file);
        cdata = fliplr(imread(this_SB_Shape_Model.img));
        time_earth_example = 12 * 60 * 60;
        cdata_time = cdata;
        idx_time_earth_example = mod( floor(size(cdata,2)*time_earth_example/86400), size(cdata,2)) + 1;
        if idx_time_earth_example == 1
            cdata_time = cdata;
        else
            %     cdata_time(:,1:size(cdata,2)-idx_time_earth_example,:) = cdata(:,idx_time_earth_example+1:size(cdata,2),:);
            %     cdata_time(:,size(cdata,2)-idx_time_earth_example+1:size(cdata,2),:) = cdata(:,1:idx_time_earth_example,:);
            
            cdata_time(:,1:idx_time_earth_example,:) = cdata(:,size(cdata,2)-idx_time_earth_example+1:size(cdata,2),:);
            cdata_time(:,idx_time_earth_example+1:size(cdata,2),:) = cdata(:,1:size(cdata,2)-idx_time_earth_example,:);
        end
        set(hPlanet,'facecolor','texture','cdata',cdata_time,'edgecolor','none');
    otherwise
        error('Shape model type not recognized')
end

axis equal
xlabel('X axis [km]')
ylabel('Y axis [km]')
zlabel('Z axis [km]')
title('SC in SB-centered Frame')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

end