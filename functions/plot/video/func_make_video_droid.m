function func_make_video_droid( ...
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

% Process few data for analysis
%                         cumulative_delta_V = zeros(ms.true_time.num_storage_steps,3);
total_number_point_altimeter = zeros(ms.true_time.num_storage_steps,1);
total_number_point_radar = zeros(ms.true_time.num_storage_steps,1);
total_number_point_bistatic = zeros(ms.true_time.num_storage_steps,1);
total_number_point_antipodal = zeros(ms.true_time.num_storage_steps,6);
for i=1:mission_init_data.num_SC
    if mission_true_SC{i}.true_SC_body.flag_hardware_exists.science_altimeter
        total_number_point_altimeter = total_number_point_altimeter + ms.true_SC{i}.SC_altimeter(:,2);
    end
    if mission_true_SC{i}.true_SC_body.flag_hardware_exists.science_radar
        total_number_point_radar = total_number_point_radar + ms.true_SC{i}.SC_radar_monostatic(:,1);
        total_number_point_bistatic = total_number_point_bistatic + ms.true_SC{i}.SC_radar_bistatic(:,1);
        total_number_point_antipodal = total_number_point_antipodal + ms.true_SC{i}.SC_radar_antipodal;
    end
    %                             if mission_true_SC{i}.true_SC_body.flag_hardware_exists.navigation_chemical_thruster
    %                                 cumulative_delta_V(:,i) =
    %                             end
end

open(myVideo);

for k = 1:50:k_end
    
    kd = floor(k * mission_true_time.time_step / mission_true_time.time_step_dynamics);
    
    plot_handle = figure(1);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'Position',[0 0 1300 900]);
    
    hold on
    
    % Orbit
    subplot(3,4,1)
    for i_SC = 1:1:mission_init_data.num_SC
        plot3(ms.true_SC{i_SC}.position_array(1:kd,1) - ms.true_small_body.position_array(1:kd,1), ms.true_SC{i_SC}.position_array(1:kd,2) - ms.true_small_body.position_array(1:kd,2), ms.true_SC{i_SC}.position_array(1:kd,3) - ms.true_small_body.position_array(1:kd,3), '-','LineWidth',1.5, 'Color',ms.plot_parameters.color_array(i_SC), 'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
        hold on
    end
    
    this_SB_Shape_Model = mission_true_small_body.shape_model;
    this_SB_Shape_Model.Vertices = (1e-3)*(mission_true_small_body.rotation_matrix * this_SB_Shape_Model.Vertices')';
    patch(this_SB_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'k', 'DisplayName',mission_true_small_body.name) %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)
    %                             view([0 0]);
    axis equal
    legend('Location','southwest')
    xlabel('X axis [km]')
    ylabel('Y axis [km]')
    zlabel('Z axis [km]')
    title('SC in SB-centered Frame')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    % Ground track
    subplot(3,4,2)
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
    
    % monostatic
    subplot(3,4,3)
    for i_SC = 1:1:mission_init_data.num_SC
        if (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_radar == 1)
            plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_radar_monostatic(1:k,1),'DisplayName',['(SC',num2str(i_SC),')'],'Color',ms.plot_parameters.color_array(i_SC),'LineWidth',2)
            hold on
        end
    end
    xlabel('Time [sec]')
    ylabel('number')
    title(['Radar Monostatic : points observed = ',num2str(total_number_point_radar(k)/mission_true_SC{2}.true_SC_radar.num_points*100),'%'])
    legend()
    grid on
    
    % antipodal
    subplot(3,4,4)
    for a=1:length(mission_true_SC{3}.true_SC_radar.antipodal_angle_array)
        if a == length(mission_true_SC{3}.true_SC_radar.antipodal_angle_array)
            plot(ms.true_time.time_array(1:k),total_number_point_antipodal(1:k,a),'DisplayName',['\pm ',num2str(mission_true_SC{3}.true_SC_radar.antipodal_angle_array(a)),' deg (limb)'],'Color',ms.plot_parameters.color_array(a),'LineWidth',2)
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
    
    
    % angular separation radar
    subplot(3,4,5)
    plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.radar_interSC_angle(1:k,1),'Color',ms.plot_parameters.color_array(1),'LineWidth',2)
    title("Antipodal angular separation")
    grid on
    xlabel('Time [sec]')
    ylabel('angle [deg]')
    
    
    % Power
    %                             subplot(3,4,6)
    %                             for i_SC = 1:mission_init_data.num_SC
    %                                 plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_Power_Generated_Consumed_Energy_Unused(1:k,1),'-','LineWidth',1.5,'Color',ms.plot_parameters.color_array(i_SC),'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
    %                                 hold on
    %                                 plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_Power_Generated_Consumed_Energy_Unused(1:k,2),'--','LineWidth',1.5,'Color',ms.plot_parameters.color_array(i_SC),'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
    %                                 hold on
    %                             end
    %                             grid on
    %                             legend()
    %                             xlabel('Time [sec]')
    %                             ylabel('Power [Watts]')
    %                             title('SC Power Generated and Consumed')
    %                             set(gca, 'fontsize',13,'FontName','Times New Roman')
    %                             hold off
    
    % Altimeter SB distance
    subplot(3,4,6)
    for i_SC = 1:mission_init_data.num_SC
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_altimeter == 1
            plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_altimeter(1:k,1),'DisplayName',['Altimeter Measure (SC',num2str(i_SC),')'],'Color',ms.plot_parameters.color_array(i_SC),'LineWidth',2)
        end
        hold on
        plot(ms.true_time.time_array(1:k), ms.true_SC{i_SC}.SC_SB_Distance(1:kd,1),'--','DisplayName',['Distance from SB center (SC',num2str(i_SC),')'],'Color',ms.plot_parameters.color_array(i_SC),'LineWidth',2)
        hold on
    end
    xlabel('Time [sec]')
    ylabel('Distance [km]')
    title('Altimeter : Measured distance from SB surface')
    legend()
    grid on
    
    
    % Battery
    subplot(3,4,7)
    hold on
    
    for i_SC = 1:mission_init_data.num_SC
        for i = 1:1:mission_true_SC{i_SC}.true_SC_battery.num_battery
            plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_Battery(1:k,mission_true_SC{i_SC}.true_SC_battery.num_battery+i),'-','LineWidth',1.5,'Color',ms.plot_parameters.color_array(i_SC),'DisplayName',mission_true_SC{i_SC}.true_SC_body.name)
            hold on
        end
    end
    
    grid on
    legend
    xlabel('Time [sec]')
    ylabel('Percentage (%)')
    title('Battery State of Charge')
    %             ylim([0 100])
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    % Data onboard
    subplot(3,4,8)
    for i_SC = 1:mission_init_data.num_SC
        for i =1:mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory
            semilogy(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_Data_Volume(1:k,i),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
            hold on
        end
    end
    
    grid on
    legend
    xlabel('Time [sec]')
    ylabel('Data Volume [GB]')
    title('Data onboard')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    % interlocutor
    subplot(3,4,9)
    hold on
    
    fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[-1.25 -1.25 -0.75 -0.75],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','DTE')
    for i=1:mission_init_data.num_SC
        fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[(i-0.25) (i-0.25) (i+0.25) (i+0.25)],ms.plot_parameters.color_array(i),'EdgeColor','none','FaceAlpha',0.5,'DisplayName',['SC',num2str(i)])
    end
    n=mission_true_SC{1}.software_SC_communication.number_communication;
    plot(ms.true_time.time_array(1:k),ms.true_SC{1}.SC_communication_data(1:k,4*n+1),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
    grid on
    legend()
    title('Mothership active communication')
    
    % Attitude mode
    subplot(3,4,10)
    hold on
    
    fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[4.75 4.75 5.25 5.25],'m','EdgeColor','none','FaceAlpha',0.5,'DisplayName','INTERSAT comm')
    fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[3.75 3.75 4.25 4.25],'r','EdgeColor','none','FaceAlpha',0.5,'DisplayName','DTE comm')
    fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[2.75 2.75 3.25 3.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','\Delta V')
    fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[1.75 1.75 2.25 2.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Sun')
    fill([ms.true_time.time_array(1) ms.true_time.time_array(k) ms.true_time.time_array(k) ms.true_time.time_array(1)],[0.75 0.75 1.25 1.25],'c','EdgeColor','none','FaceAlpha',0.5,'DisplayName','SB')
    
    for i_SC=1:mission_init_data.num_SC
        plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.desired_SC_attitude_mode(1:k),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
    end
    
    grid on
    legend('Location','southeast')
    xlabel('Time [sec]')
    ylabel('Attitude Mode')
    title('SC Attitude pointing mode')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    subplot(3,4,11)
    for i_SC=1:mission_init_data.num_SC
        hold on
        plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_True_Command_Thrust(1:k,1),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
    end
    grid on
    legend()
    xlabel('Time [sec]')
    ylabel('Thrust [N]')
    title('Chemical Thrusters Thrust')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    subplot(3,4,12)
    for i_SC=1:mission_init_data.num_SC
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
            plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_Propelant_mass(1:k,2),'--','LineWidth',2,'DisplayName',['CT-SC',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
            hold on;
        end
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
            plot(ms.true_time.time_array(1:k),ms.true_SC{i_SC}.SC_Propelant_mass(1:k,1),'-','LineWidth',2,'DisplayName',['MT-SC',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
            hold on;
        end
    end
    grid on
    legend()
    xlabel('Time [sec]')
    ylabel('mass [kg]')
    title('Propelant mass used')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    
    sgtitle(['Mission Simulation, Time = ',num2str(round(ms.true_time.time_array(k))),' sec'],'fontsize',20,'FontName','Times New Roman')
    
    % Store video
    F = getframe(plot_handle);
    writeVideo(myVideo, F);
    
    
end
end