function func_plot_science_camera_mothership(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix)

i_SC = ms.plot_parameters.plot_SC_number;
obj = mission_true_SC{i_SC}.true_SC_science_camera;

k = ms.true_time.counter;
kd = ms.true_time.counter_dynamics;
time_k = ms.true_time.time_array(k);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Science Camera
% Only for DROID

fig_number = fig_number + 1;
plot_handle = figure(fig_number);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

subplot(2,3,1)

for i_SC = 1:1:mission_init_data.num_SC
    plot3(ms.true_SC{i_SC}.position_array(1:kd,1) - ms.true_small_body.position_array(1:kd,1), ms.true_SC{i_SC}.position_array(1:kd,2) - ms.true_small_body.position_array(1:kd,2), ...
        ms.true_SC{i_SC}.position_array(1:kd,3) - ms.true_small_body.position_array(1:kd,3), '-','LineWidth',5, 'Color',ms.plot_parameters.color_array(i_SC), 'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
end
hold on

this_SB_Shape_Model = mission_true_small_body.shape_model;
switch mission_true_small_body.shape_model_type
    case 'trisurf'
        this_SB_Shape_Model.Vertices = (1e-3)*(mission_true_small_body.rotation_matrix * this_SB_Shape_Model.Vertices')';
        patch(this_SB_Shape_Model, 'FaceColor', 0.7*[1 1 1], 'EdgeColor', 'k', 'DisplayName', mission_true_small_body.name)
        %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)
    case 'sphere'
        % TODO: Add rotation
        topo = fliplr(imread(this_SB_Shape_Model.img));
        [xP,yP,zP] = sphere(50);
        SB_radius = mission_true_small_body.radius;
        hPlanet = surf(xP*SB_radius, yP*SB_radius, zP*SB_radius,'FaceColor','blue','EdgeColor','none', 'DisplayName', '');
        set(hPlanet,'facecolor','texture','cdata',topo,'edgecolor','none');
    otherwise
        error('Shape model type not recognized')
end



axis equal
legend('Location','southwest')
xlabel('X axis [km]')
ylabel('Y axis [km]')
zlabel('Z axis [km]')
title('SC in SB-centered Frame','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,2)
hold on

histogram_data = zeros(obj.num_points,1);


for pt_i = 1:1:obj.num_points

    for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)

        exisiting_data_points = obj.observed_camera_data{pt_i,i_ang};

        if isempty(exisiting_data_points)
            % Do nothing
            % histogram_data(pt_i) = 0;

        else
            histogram_data(pt_i) = histogram_data(pt_i) + sum(exisiting_data_points(:,1) < time_k);
        end
    end
end

plot([1:1:obj.num_points], histogram_data, 'LineWidth',2.0)

xlabel('Points') 
ylabel('Number of Images') 
title(['Number of Images, Coverage = ',num2str(sum(histogram_data ~= 0)),' of ',num2str(obj.num_points)],'FontSize',ms.plot_parameters.title_font_size) 
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,3)
hold on


for pt_i = 1:1:obj.num_points

    resolution_data = [];

    for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)

        exisiting_data_points = obj.observed_camera_data{pt_i,i_ang};

        if isempty(exisiting_data_points)
            % Do nothing

        else
            resolution_data = [resolution_data; exisiting_data_points((exisiting_data_points(:,1) < time_k),3)];
        end
    end

    if ~isempty(resolution_data)
        plot(pt_i*ones(length(resolution_data),1),resolution_data,'Marker',ms.plot_parameters.marker_array(1+mod(pt_i,length(ms.plot_parameters.marker_array))),'MarkerSize',5,'MarkerFaceColor',ms.plot_parameters.color_array(1+mod(pt_i,length(ms.plot_parameters.color_array))),'MarkerEdgeColor','k')
    end
end

xlim([0 obj.num_points])
xlabel('Points')
ylabel('Spatial Resolution [meters / pixel]')
title('Spatial Resolution of Images','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,4)
hold on


for pt_i = 1:1:obj.num_points

    incidence_angle_data = [];

    for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)

        exisiting_data_points = obj.observed_camera_data{pt_i,i_ang};

        if isempty(exisiting_data_points)
            % Do nothing

        else
            incidence_angle_data = [incidence_angle_data; exisiting_data_points((exisiting_data_points(:,1) < time_k),4)];
        end
    end

    if ~isempty(incidence_angle_data)
        plot(pt_i*ones(length(incidence_angle_data),1),incidence_angle_data,'Marker',ms.plot_parameters.marker_array(1+mod(pt_i,length(ms.plot_parameters.marker_array))),'MarkerSize',5,'MarkerFaceColor',ms.plot_parameters.color_array(1+mod(pt_i,length(ms.plot_parameters.color_array))),'MarkerEdgeColor','k')
    end
end

xlim([0 obj.num_points])
xlabel('Points')
ylabel('Incidence Angle [deg]')
title('Sun-Normal Incidence Angle','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,5)
hold on


for pt_i = 1:1:obj.num_points

    emergence_angle_data = [];

    for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)

        exisiting_data_points = obj.observed_camera_data{pt_i,i_ang};

        if isempty(exisiting_data_points)
            % Do nothing

        else
            emergence_angle_data = [emergence_angle_data; exisiting_data_points((exisiting_data_points(:,1) < time_k),5)];
        end
    end

    if ~isempty(emergence_angle_data)
        plot(pt_i*ones(length(emergence_angle_data),1),emergence_angle_data,'Marker',ms.plot_parameters.marker_array(1+mod(pt_i,length(ms.plot_parameters.marker_array))),'MarkerSize',5,'MarkerFaceColor',ms.plot_parameters.color_array(1+mod(pt_i,length(ms.plot_parameters.color_array))),'MarkerEdgeColor','k')
    end
end

xlim([0 obj.num_points])
xlabel('Points')
ylabel('Emergence Angle [deg]')
title('SC-Normal Emergence Angle','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off


subplot(2,3,6)
hold on


for pt_i = 1:1:obj.num_points

    phase_angle_data = [];

    for i_ang = 1:1:length(obj.camera_normal_Sun_angle_array)

        exisiting_data_points = obj.observed_camera_data{pt_i,i_ang};

        if isempty(exisiting_data_points)
            % Do nothing

        else
            phase_angle_data = [phase_angle_data; exisiting_data_points((exisiting_data_points(:,1) < time_k),6)];
        end
    end

    if ~isempty(phase_angle_data)
        plot(pt_i*ones(length(phase_angle_data),1),phase_angle_data,'Marker',ms.plot_parameters.marker_array(1+mod(pt_i,length(ms.plot_parameters.marker_array))),'MarkerSize',5,'MarkerFaceColor',ms.plot_parameters.color_array(1+mod(pt_i,length(ms.plot_parameters.color_array))),'MarkerEdgeColor','k')
    end
end

xlim([0 obj.num_points])
xlabel('Points')
ylabel('Phase Angle [deg]')
title('SC-Sun Phase Angle','FontSize',ms.plot_parameters.title_font_size)
set(gca, 'FontSize',ms.plot_parameters.standard_font_size,'FontName',ms.plot_parameters.standard_font_type)
hold off

sgtitle(['Science Camera onboard SC',num2str(i_SC)],'FontSize',ms.plot_parameters.title_font_size,'FontName',ms.plot_parameters.standard_font_type)

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Science_Camera.png'])
end


