function func_plot_science( ...
    ms, ... % mission_storage
    ission_true_time, ...
    ission_true_small_body, ...
    ission_true_solar_system, ...
    ission_init_data, ...
    ission_true_SC, ...
    fig_number, ...
    plot_name_prefix)

k = ms.true_time.counter;
kd = ms.true_time.counter_dynamics;
plot_SC_number = ms.plot_parameters.plot_SC_number;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Science
% Only for DROID
if mission_init_data.mission_type == 3
    fig_number = fig_number+1;
    plot_handle = figure(fig_number);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');
    
    subplot(2,4,7)
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
    grid on
    axis equal
    
    total_number_point_altimeter = zeros(ms.true_time.num_storage_steps,1);
    total_number_point_radar = zeros(ms.true_time.num_storage_steps,1);
    total_number_point_bistatic = zeros(ms.true_time.num_storage_steps,1);
    total_number_point_antipodal = zeros(ms.true_time.num_storage_steps,6);
    for i=1:mission_init_data.num_SC
        if mission_true_SC{i}.true_SC_body.flag_hardware_exists.science_altimeter
            total_number_point_altimeter = total_number_point_altimeter + ms.true_SC{i}.SC_altimeter(1:k,2);
        end
        if mission_true_SC{i}.true_SC_body.flag_hardware_exists.science_radar
            total_number_point_radar = total_number_point_radar + ms.true_SC{i}.SC_radar_monostatic(1:k,1);
            total_number_point_bistatic = total_number_point_bistatic + ms.true_SC{i}.SC_radar_bistatic(1:k,1);
            total_number_point_antipodal = total_number_point_antipodal + ms.true_SC{i}.SC_radar_antipodal;
        end
    end
    for i=1:mission_init_data.num_SC
        
        if (mission_true_SC{i}.true_SC_body.flag_hardware_exists.science_altimeter == 1)
            
            subplot(2,4,5)
            plot(ms.true_time.time_array(1:k),ms.true_SC{i}.SC_altimeter(1:k,2),'DisplayName',['(SC',num2str(i),')'],'Color',ms.plot_parameters.color_array(i),'LineWidth',2)
            hold on
            xlabel('Time [sec]')
            ylabel('number')
            title(['Altimeter : points observed = ',num2str(total_number_point_altimeter(k)/mission_true_SC{i}.true_SC_altimeter.num_points*100),'%'])
            legend()
            grid on
            
            subplot(2,4,6)
            plot(ms.true_time.time_array(1:k),ms.true_SC{i}.SC_altimeter(1:k,1),'DisplayName',['Measured distance from surface (SC',num2str(i),')'],'Color',ms.plot_parameters.color_array(i),'LineWidth',2)
            hold on
            plot(ms.true_time.time_array(1:kd), ms.true_SC{i}.SC_SB_Distance(1:kd,1),'--','DisplayName',['Distance from SB center (SC',num2str(i),')'],'Color',ms.plot_parameters.color_array(i),'LineWidth',2)
            xlabel('Time [sec]')
            ylabel('Distance [km]')
            title('Altimeter : Measured distance from SB surface')
            legend()
            grid on
            
            
            % compute 3D measurement
            observed_point = zeros(mission_true_SC{i}.true_SC_altimeter.num_point_observed,3);
            index_observed_points = find(mission_true_SC{i}.true_SC_altimeter.observed_points(:,1) ~= 0);
            id = 1;
            for j=1:length(index_observed_points)
                radius = mission_true_SC{i}.true_SC_altimeter.estimated_SB_radius(index_observed_points(id));
                observed_point(id,:) = (mission_true_small_body.rotation_matrix * (mission_true_SC{i}.true_SC_altimeter.points(index_observed_points(id),:))'*radius)';
                id = id+1;
            end
            subplot(2,4,7)
            hold on
            scatter3(observed_point(:,1),observed_point(:,2),observed_point(:,3),'filled','DisplayName',['(SC',num2str(i),')'],'Color',ms.plot_parameters.color_array(i))
            title('Altimeter : 3D coverage')
            legend()
        end
        
        if (mission_true_SC{i}.true_SC_body.flag_hardware_exists.science_radar == 1)
            subplot(2,4,1)
            plot(ms.true_time.time_array(1:k),ms.true_SC{i}.SC_radar_monostatic(1:k,1),'DisplayName',['(SC',num2str(i),')'],'Color',ms.plot_parameters.color_array(i),'LineWidth',2)
            hold on
            xlabel('Time [sec]')
            ylabel('number')
            title(['Radar Monostatic : points observed = ',num2str(total_number_point_radar(k)/mission_true_SC{i}.true_SC_radar.num_points*100),'%'])
            legend()
            grid on
            
            
        end
        
        % Ground track 2D
        subplot(2,4,8)
        plot(ms.true_SC{i}.SC_groundtrack(1:k,1),ms.true_SC{i}.SC_groundtrack(1:k,2),'LineWidth',2,'DisplayName',['(SC',num2str(i),')'],'Color',ms.plot_parameters.color_array(i))
        hold on
        
    end
    
    
    subplot(2,4,5)
    plot(ms.true_time.time_array(1:k),total_number_point_altimeter,'DisplayName','Total','Color','g','LineWidth',2)
    
    subplot(2,4,1)
    plot(ms.true_time.time_array(1:k),total_number_point_radar,'DisplayName','Total','Color','g','LineWidth',2)
    
    
    subplot(2,4,2)
    for a=1:length(mission_true_SC{3}.true_SC_radar.antipodal_angle_array)
        if a == length(mission_true_SC{3}.true_SC_radar.antipodal_angle_array)
            plot(ms.true_time.time_array(1:k),total_number_point_antipodal(1:k,a),'DisplayName',['\pm',num2str(mission_true_SC{3}.true_SC_radar.antipodal_angle_array(a)),' deg (limb)'],'Color',ms.plot_parameters.color_array(a),'LineWidth',2)
        else
            plot(ms.true_time.time_array(1:k),total_number_point_antipodal(1:k,a),'DisplayName',['\pm ',num2str(mission_true_SC{3}.true_SC_radar.antipodal_angle_array(a)),' deg'],'Color',ms.plot_parameters.color_array(a),'LineWidth',2)
        end
        hold on
    end
    
    xlabel('Time [sec]')
    ylabel('number')
    title('Radar Antipodal : number of points observed')
    legend()
    grid on
    
    subplot(2,4,4)
    plot(ms.true_time.time_array(1:k),ms.true_SC{1}.radar_interSC_angle(1:k,1),'Color',ms.plot_parameters.color_array(1),'LineWidth',2)
    title("Radar Antipodal : inter spacecraft angular separation")
    grid on
    xlabel('Time [sec]')
    ylabel('angle [deg]')
    
    
    subplot(2,4,3)
    plot(ms.true_time.time_array(1:k),total_number_point_bistatic,'DisplayName','Total','Color','g','LineWidth',2)
    xlabel('Time [sec]')
    ylabel('number')
    title(['Radar Bistatic : number of points observed : ',num2str(mission_true_SC{3}.true_SC_radar.bistatic_num_point_observed/(mission_true_SC{3}.true_SC_radar.num_points^2)*100),'%'])
    legend()
    grid on
    
    subplot(2,4,8)
    legend()
    xlim([-180 180])
    ylim([-90 90])
    xlabel('Azimuth [deg]')
    ylabel('Elevation [deg]')
    title(['2D coverage'])
    grid on
    
    sgtitle(['Science onboard'],'fontsize',ms.plot_parameters.title_font_size,'FontName','Times New Roman')
    
end

if ms.plot_parameters.storage_save_plots == 1
    saveas(plot_handle,[plot_name_prefix,'Science.png'])
end

end