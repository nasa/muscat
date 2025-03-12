function func_plot_orbit_visualization_all( ...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_true_small_body, ...
    mission_true_solar_system, ...
    mission_init_data, ...
    mission_true_SC,  ...
    fig_number, ...
    plot_name_prefix)

kd = ms.true_time.counter_dynamics;
num_SC = mission_init_data.num_SC;
colors = ms.plot_parameters.color_array;

%% Orbit Vizualization
plot_handle = figure(fig_number);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

for i_SC = 1:1:num_SC
    
    % Position and Velocity
    tspan = ms.true_time.time_array_dynamics(1:kd);
    r_true = ms.true_SC{i_SC}.position_array(1:kd,:) - ms.true_small_body.position_array(1:kd,:);
    v_true = ms.true_SC{i_SC}.velocity_array(1:kd,:) - ms.true_small_body.velocity_array(1:kd,:);
    rv_est = ms.true_SC{i_SC}.SC_Estimate_Position_Velocity(1:kd,:);
    r_error = rv_est(:,1:3) - r_true;
    v_error = rv_est(:,4:6) - v_true;
    r_sigma = 3*sqrt(ms.true_SC{i_SC}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,1:3));
    v_sigma = 3*sqrt(ms.true_SC{i_SC}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,4:6));
    
    coords = {'X','Y','Z'};
    error_args = {'-','LineWidth',1.5};
    area_args = {'FaceAlpha',0.08,'EdgeColor', 'none'};
    
    % Position
    subplot(3,4,4*(i_SC-1)+1)
    hold on
    for i_coord = 1:1:3
        plot(tspan,r_error(:,i_coord),error_args{:},'Color',colors(i_coord), 'DisplayName', ['r_',coords{i_coord}, ' error'])
    end
    for i_coord = 1:1:3
        area(tspan,r_sigma(:,i_coord),area_args{:},'FaceColor',colors(i_coord), 'DisplayName', ['r_',coords{i_coord}, ' 3-\sigma'])
    end
    grid on
    legend('Location','southwest', 'NumColumns', 2)
    xlabel('Time [sec]')
    ylabel('Position error [km]')
    title(['Estimation Error SC', num2str(i_SC)])
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    % Velocity
    subplot(3,4,4*(i_SC-1)+2)
    hold on
    for i_coord = 1:1:3
        plot(tspan,v_error(:,i_coord),error_args{:},'Color',colors(i_coord), 'DisplayName', ['v_',coords{i_coord}, ' error'])
    end
    for i_coord = 1:1:3
        area(tspan,v_sigma(:,i_coord),area_args{:},'FaceColor',colors(i_coord), 'DisplayName', ['v_',coords{i_coord}, ' 3-\sigma'])
    end
    grid on
    legend('Location','southwest', 'NumColumns', 2)
    xlabel('Time [sec]')
    ylabel('Velocity error [km/sec]')
    title(['Estimation Error SC', num2str(i_SC)])
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
end

subplot(1,2,2)

for i_SC = 1:1:num_SC
    plot3(ms.true_SC{i_SC}.position_array(1:kd,1) - ms.true_small_body.position_array(1:kd,1), ms.true_SC{i_SC}.position_array(1:kd,2) - ms.true_small_body.position_array(1:kd,2), ...
        ms.true_SC{i_SC}.position_array(1:kd,3) - ms.true_small_body.position_array(1:kd,3), '-','LineWidth',5, 'Color',ms.plot_parameters.color_array(i_SC), 'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
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
legend('Location','southwest')
xlabel('X axis [km]')
ylabel('Y axis [km]')
zlabel('Z axis [km]')
title('SC in SB-centered Frame')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off


return
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.position_array(1:kd,1),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.position_array(1:kd,2),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.position_array(1:kd,3),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,1),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,2),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,3),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))
grid on
legend('Pos X (True)', 'Pos Y (True)', 'Pos Z (True)','Pos X (Estimated)', 'Pos Y (Estimated)', 'Pos Z (Estimated)')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SC Position [km]')
title('SC Position: True vs Estimated')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

subplot(2,4,5)

semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_SC{plot_SC_number}.position_array(1:kd,1)-ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,1)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
hold on
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_SC{plot_SC_number}.position_array(1:kd,2)-ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,2)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_SC{plot_SC_number}.position_array(1:kd,3)-ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,3)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,1),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,2),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,3),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('True error in Pos X', 'True error in Pos Y', 'True error in Pos Z', 'Estimated 1-\sigma error in Pos X', 'Estimated 1-\sigma error in Pos Y', 'Estimated 1-\sigma error in Pos Z')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SC Position Error [km]')
title('Estimation error in SC Position')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

subplot(2,4,2)
hold on

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.velocity_array(1:kd,1),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.velocity_array(1:kd,2),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.velocity_array(1:kd,3),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,4),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,5),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,6),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('Vel X (True)', 'Vel Y (True)', 'Vel Z (True)','Vel X (Estimated)', 'Vel Y (Estimated)', 'Vel Z (Estimated)')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SC Velocity [km/sec]')
title('SC Velocity: True vs Estimated')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

subplot(2,4,6)

semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_SC{plot_SC_number}.velocity_array(1:kd,1)-ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,4)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
hold on
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_SC{plot_SC_number}.velocity_array(1:kd,2)-ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,5)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_SC{plot_SC_number}.velocity_array(1:kd,3)-ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,6)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,4),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,5),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity_Uncertainty(1:kd,6),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('True error in Vel X', 'True error in Vel Y', 'True error in Vel Z', 'Estimated 1-\sigma error in Vel X', 'Estimated 1-\sigma error in Vel Y', 'Estimated 1-\sigma error in Vel Z')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SC Velocity Error [km/sec]')
title('Estimation error in SC Velocity')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off



subplot(2,4,3)
hold on

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_small_body.position_array(1:kd,1),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_small_body.position_array(1:kd,2),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_small_body.position_array(1:kd,3),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,1),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,2),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,3),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))
grid on
legend('Pos X (True)', 'Pos Y (True)', 'Pos Z (True)','Pos X (Estimated)', 'Pos Y (Estimated)', 'Pos Z (Estimated)')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SB Position [km]')
title('SB Position: True vs Estimated')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

subplot(2,4,7)

semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_small_body.position_array(1:kd,1)-ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,1)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
hold on
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_small_body.position_array(1:kd,2)-ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,2)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_small_body.position_array(1:kd,3)-ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,3)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity_Uncertainty(1:kd,1),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity_Uncertainty(1:kd,2),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity_Uncertainty(1:kd,3),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('True error in Pos X', 'True error in Pos Y', 'True error in Pos Z', 'Estimated 1-\sigma error in Pos X', 'Estimated 1-\sigma error in Pos Y', 'Estimated 1-\sigma error in Pos Z')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SB Position Error [km]')
title('Estimation error in SB Position')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off


subplot(2,4,4)
hold on

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_small_body.velocity_array(1:kd,1),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_small_body.velocity_array(1:kd,2),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_small_body.velocity_array(1:kd,3),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,4),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,5),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
plot(ms.true_time.time_array_dynamics(1:kd),ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,6),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('Vel X (True)', 'Vel Y (True)', 'Vel Z (True)','Vel X (Estimated)', 'Vel Y (Estimated)', 'Vel Z (Estimated)')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SB Velocity [km/sec]')
title('SB Velocity: True vs Estimated')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

subplot(2,4,8)

semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_small_body.velocity_array(1:kd,1)-ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,4)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
hold on
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_small_body.velocity_array(1:kd,2)-ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,5)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), abs(ms.true_small_body.velocity_array(1:kd,3)-ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,6)),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity_Uncertainty(1:kd,4),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity_Uncertainty(1:kd,5),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
semilogy(ms.true_time.time_array_dynamics(1:kd), ms.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity_Uncertainty(1:kd,6),'--','LineWidth',2,'Color',ms.plot_parameters.color_array(3))

grid on
legend('True error in Vel X', 'True error in Vel Y', 'True error in Vel Z', 'Estimated 1-\sigma error in Vel X', 'Estimated 1-\sigma error in Vel Y', 'Estimated 1-\sigma error in Vel Z')
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('SC Velocity Error [km/sec]')
title('Estimation error in SB Velocity')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off


sgtitle(['SC and SB Position Estimator Performance',' / SC',num2str(plot_SC_number)],'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')


subplot(1,2,2)

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
title('SC in SB-centered Frame')
set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
hold off

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Orbit.png'])
end
end