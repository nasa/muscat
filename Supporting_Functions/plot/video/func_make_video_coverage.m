function func_make_video_coverage( ...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_init_data, ...
    mission_true_small_body, ...
    mission_true_stars, ...
    mission_true_solar_system, ...
    mission_true_SC)

close all

idx_array = find(ms.true_time.time_array);
k_end = idx_array(end);

video_filename = [mission_init_data.output_folder,ms.plot_parameters.plot_type,'_Video','.mp4'];
myVideo = VideoWriter(video_filename, 'MPEG-4');
myVideo.FrameRate = 30;  % Default 30
myVideo.Quality = 100;    % Default 75

% compute 3D coverage
observed_point = zeros(ms.true_time.num_storage_steps,6); %[x_sc2, y_sc2, z_sc2, x_sc3, y_sc3, z_sc3]
for i_SC=2:3
    
    for j=1:ms.true_time.num_storage_steps
        index_observed_point = ms.true_SC{i_SC}.SC_altimeter(j,3);
        if index_observed_point ~= 0
            radius = mission_true_SC{i_SC}.true_SC_altimeter.estimated_SB_radius(index_observed_point);
            observed_point(j,1+(i_SC-2)*3:3+(i_SC-2)*3) = (mission_true_small_body.rotation_matrix * (mission_true_SC{i_SC}.true_SC_altimeter.points(index_observed_point,:))'*radius)';
        end
    end
    
end

open(myVideo);

for k = 1:50:k_end
    
    plot_handle = figure(1);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'Position',[0 0 1300 600]);
    
    hold on
    
    subplot(1,3,1)
    
    for i_SC = 1:1:mission_init_data.num_SC
        plot3(ms.true_SC{i_SC}.position_array(1:kd,1) - ms.true_small_body.position_array(1:kd,1), ms.true_SC{i_SC}.position_array(1:kd,2) - ms.true_small_body.position_array(1:kd,2), ...
            ms.true_SC{i_SC}.position_array(1:kd,3) - ms.true_small_body.position_array(1:kd,3), '-','LineWidth',2, 'Color',ms.plot_parameters.color_array(i_SC), 'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
    end
    hold on
    
    this_SB_Shape_Model = mission_true_small_body.shape_model;
    switch mission_true_small_body.shape_model_type
        case 'trisurf'
            this_SB_Shape_Model.Vertices = (1e-3)*(mission_true_small_body.rotation_matrix * this_SB_Shape_Model.Vertices')';
            patch(this_SB_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'k', 'DisplayName',mission_true_small_body.name) %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)
        case 'sphere'
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
    title('SC in SB-centered Frame')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    
    subplot(1,3,2)
    for i_SC = 1:1:mission_init_data.num_SC
        plot(ms.true_SC{i_SC}.SC_groundtrack(1:k,1),ms.true_SC{i_SC}.SC_groundtrack(1:k,2),'LineWidth',2,'DisplayName',['(SC',num2str(i_SC),')'],'Color',ms.plot_parameters.color_array(i_SC))
        hold on
    end
    legend()
    xlim([-180 180])
    ylim([-90 90])
    xlabel('Azimuth [deg]')
    ylabel('Elevation [deg]')
    title(['2D coverage / groundtrack'])
    grid on
    
    subplot(1,3,3)
    scatter3(observed_point(1:k,1),observed_point(1:k,2),observed_point(1:k,3),'filled','DisplayName',['(SC',num2str(2),')'],'Color',ms.plot_parameters.color_array(2))
    hold on
    scatter3(observed_point(1:k,4),observed_point(1:k,5),observed_point(1:k,6),'filled','DisplayName',['(SC',num2str(3),')'],'Color',ms.plot_parameters.color_array(3))
    
    this_SB_Shape_Model = mission_true_small_body.shape_model;
    switch mission_true_small_body.shape_model_type
        case 'trisurf'
            this_SB_Shape_Model.Vertices = (1e-3)*(mission_true_small_body.rotation_matrix * this_SB_Shape_Model.Vertices')';
            patch(this_SB_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'k', 'DisplayName',mission_true_small_body.name,'FaceAlpha', 0.5) %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)
        case 'sphere'
            topo = fliplr(imread(this_SB_Shape_Model.img));
            [xP,yP,zP] = sphere(50);
            SB_radius = mission_true_small_body.radius;
            hPlanet = surf(xP*SB_radius, yP*SB_radius, zP*SB_radius,'FaceColor','blue','EdgeColor','none', 'DisplayName', '');
            set(hPlanet,'facecolor','texture','cdata',topo,'edgecolor','none');
        otherwise
            error('Shape model type not recognized')
    end
    view([-5 45]);
    title('Altimeter : 3D coverage')
    legend()
    
    grid on
    axis equal
    
    
    
    sgtitle(['Mission Simulation, Time = ',num2str(round(ms.true_time.time_array(k))),' sec'],'fontsize',20,'FontName','Times New Roman')
    
    % Store video
    F = getframe(plot_handle);
    writeVideo(myVideo, F);
    
end


close(myVideo);
end