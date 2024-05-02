function func_make_video_dart(...
    ms, ... % mission_storage
    mission_true_time, ...
    mission_init_data, ...
    mission_true_small_body, ...
    mission_true_stars, ...
    mission_true_solar_system, ...
    mission_true_SC)

i_SC = 1;

close all

idx_array = find(tspan);
k_end = idx_array(end);

video_filename = [mission_init_data.output_folder,ms.plot_parameters.plot_type,'_Video','.mp4'];
myVideo = VideoWriter(video_filename, 'MPEG-4');
myVideo.FrameRate = 30;  % Default 30
myVideo.Quality = 100;    % Default 75

open(myVideo);

for k = 1:200:k_end
    
    kd = 1 + floor((k-1) * mission_true_time.time_step / mission_true_time.time_step_dynamics);
    
    plot_handle = figure(1);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'Position',[0 0 1300 900]);
    
    hold on
    
    % Orbit
    subplot(3,3,1)
    hold on
    
    plot3(ms.true_SC{i_SC}.position_array(kd,1), ms.true_SC{i_SC}.position_array(kd,2), ms.true_SC{i_SC}.position_array(kd,3), 'or','MarkerSize',10)
    plot3(ms.true_small_body.position_array(kd,1), ms.true_small_body.position_array(kd,2), ms.true_small_body.position_array(kd,3), 'ok','MarkerSize',10)
    
    plot3(ms.true_SC{i_SC}.position_array(1:kd,1), ms.true_SC{i_SC}.position_array(1:kd,2), ms.true_SC{i_SC}.position_array(1:kd,3), '-r','LineWidth',2)
    plot3(ms.true_small_body.position_array(1:kd,1), ms.true_small_body.position_array(1:kd,2), ms.true_small_body.position_array(1:kd,3), '-k','LineWidth',2)
    
    grid on
    legend('SC','SB')
    xlabel('X Axis [km]')
    ylabel('Y Axis [km]')
    zlabel('Z Axis [km]')
    title('Position of SC and SB')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    % Distance SC SB
    subplot(3,3,2)
    hold on
    
    plot(tspan_dynamics(1:kd),ms.true_SC{i_SC}.SC_SB_Distance(1:kd,1) ,'-r','LineWidth',2)
    plot(tspan_dynamics(1:kd),ms.true_SC{i_SC}.SC_SB_Distance(1:kd,2) ,'--b','LineWidth',2)
    plot(tspan(kd),ms.true_SC{i_SC}.SC_SB_Distance(kd,1),'or','MarkerSize',10)
    
    grid on
    xlabel('Time [sec]')
    ylabel('Distance [km]')
    legend('True','Estimated by SC')
    title('Distance between SC and SB')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    % Intercept distance SC SB
    subplot(3,3,3)
    semilogy(tspan(1:k),ms.true_SC{i_SC}.SC_Estimated_Desired_Intercept_Distance(1:k,4),'-m','LineWidth',2)
    hold on
    semilogy(tspan(1:k),ms.true_SC{i_SC}.SC_Estimated_Desired_Intercept_Distance(1:k,1),'-k','LineWidth',2)
    semilogy(tspan(1:k),ms.true_SC{i_SC}.SC_Estimated_Desired_Intercept_Distance(1:k,2),'--b','LineWidth',2)
    semilogy([tspan(1) tspan(k)],mission_true_small_body.radius*[1 1],'-.r','LineWidth',2)
    
    grid on
    legend('True','Estimated','Desired','SB Radius')
    xlabel('Time [sec]')
    ylabel('Intercept Distance [km]')
    title('Intercept Distance between SC and SB')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    
    % Battery
    subplot(3,3,4)
    hold on
    
    for i = 1:1:mission_true_SC{i_SC}.true_SC_battery.num_battery
        plot(tspan(1:k),ms.true_SC{i_SC}.SC_Battery(1:k,mission_true_SC{i_SC}.true_SC_battery.num_battery+i),'-','LineWidth',1.5,'Color',ms.plot_parameters.color_array(i),'DisplayName',mission_true_SC{i_SC}.true_SC_battery.battery_data(i).name)
        hold on
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
    subplot(3,3,5)
    for i =1:mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory
        semilogy(tspan(1:k),ms.true_SC{i_SC}.SC_Data_Volume(1:k,i),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color',ms.plot_parameters.color_array(i_SC))
        hold on
    end
    
    grid on
    legend
    xlabel('Time [sec]')
    ylabel('Data Volume [GB]')
    title('Data onboard')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    
    % Attitude mode
    subplot(3,3,6)
    hold on
    
    tspan = ms.true_time.time_array;
    fill([tspan(1) tspan(k) tspan(k) tspan(1)],[4.75 4.75 5.25 5.25],'m','EdgeColor','none','FaceAlpha',0.5,'DisplayName','INTERSAT comm')
    fill([tspan(1) tspan(k) tspan(k) tspan(1)],[3.75 3.75 4.25 4.25],'r','EdgeColor','none','FaceAlpha',0.5,'DisplayName','DTE comm')
    fill([tspan(1) tspan(k) tspan(k) tspan(1)],[2.75 2.75 3.25 3.25],'g','EdgeColor','none','FaceAlpha',0.5,'DisplayName','\Delta V')
    fill([tspan(1) tspan(k) tspan(k) tspan(1)],[1.75 1.75 2.25 2.25],'b','EdgeColor','none','FaceAlpha',0.5,'DisplayName','Sun')
    fill([tspan(1) tspan(k) tspan(k) tspan(1)],[0.75 0.75 1.25 1.25],'c','EdgeColor','none','FaceAlpha',0.5,'DisplayName','SB')
    
    plot(tspan(1:k),ms.true_SC{i_SC}.desired_SC_attitude_mode(1:k),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color','black')
    
    grid on
    legend('Location','southeast')
    xlabel('Time [sec]')
    ylabel('Attitude Mode')
    title('SC Attitude pointing mode')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    subplot(3,3,7)
    plot(tspan(1:k),ms.true_SC{i_SC}.SC_True_Command_Thrust(1:k,1),'-k','LineWidth',2,'DisplayName',['SC ',num2str(i_SC)],'Color','blue')
    
    grid on
    legend()
    xlabel('Time [sec]')
    ylabel('Thrust [N]')
    title('Chemical Thrusters Thrust')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    subplot(3,3,8)
    if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
        plot(tspan(1:k),ms.true_SC{i_SC}.SC_Propelant_mass(1:k,2),'r','LineWidth',2,'DisplayName',"CT")
        hold on;
    end
    if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
        plot(tspan(1:k),ms.true_SC{i_SC}.SC_Propelant_mass(1:k,1),'b','LineWidth',2,'DisplayName',"MT")
        hold on;
    end
    grid on
    legend()
    xlabel('Time [sec]')
    ylabel('mass [kg]')
    title('Propelant mass used')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    
    subplot(3,3,9)
    hold on
    plot(tspan(1:k),ms.true_SC{i_SC}.SC_Desired_Control_DeltaV(1:k,1),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(1))
    plot(tspan(1:k),ms.true_SC{i_SC}.SC_Desired_Control_DeltaV(1:k,2),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(2))
    plot(tspan(1:k),ms.true_SC{i_SC}.SC_Desired_Control_DeltaV(1:k,3),'-','LineWidth',2,'Color',ms.plot_parameters.color_array(3))
    
    grid on
    legend('Estimated','Desired')
    xlabel('Time [sec]')
    ylabel('Velocity [m/sec]')
    title('Desired DeltaV')
    set(gca, 'fontsize',ms.plot_parameters.standard_font_size,'FontName','Times New Roman')
    hold off
    
    sgtitle(['Mission Simulation, Time = ',num2str(round(tspan(k))),' sec'],'fontsize',20,'FontName','Times New Roman')
    
    % Store video
    F = getframe(plot_handle);
    writeVideo(myVideo, F);
    
    
end
end