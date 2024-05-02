function func_make_video_camera(...
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

open(myVideo);

for k = 2:2:k_end
    
    disp([num2str(k),"/",num2str(k_end)])
    clear plot_handle
    plot_handle = figure(1);
    clf
    set(plot_handle,'Color',[12, 22, 79]/255*0.5,'Position',[10 100 mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_camera_sensor.camera_data(1).resolution]);
    
    hold on
    
    % update sim state
    mission_true_time.time = ms.true_time.time_array(k);                                                                   % time
    mission_true_time.date = mission_true_time.t_initial_date + mission_true_time.time;                                     % date
    mission_true_small_body = func_update_SB_position_velocity_rot_matrix(mission_true_small_body,mission_true_time);       % SB position
    mission_true_solar_system = func_update_Sun_Earth_position_velocity(mission_true_solar_system,mission_true_time);       % SUN position
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_navigation.position = ms.true_SC{ms.plot_parameters.plot_SC_number}.position_array(kd,:)';
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_navigation.position_relative_SB = mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_navigation.position - mission_true_small_body.position_SB;
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_adc.attitude = ms.true_SC{ms.plot_parameters.plot_SC_number}.SC_Quaternion(k,:)';
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_adc = func_update_rotation_matrix(mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_adc);
    
    % plot camera view
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_camera_sensor.camera_data(1).take_measurement = 1;
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_camera_sensor.camera_data(1).flag_show_camera_plot = 1;
    mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_camera_sensor = func_get_camera_image(mission_true_SC{ms.plot_parameters.plot_SC_number}.true_SC_camera_sensor,1,mission_true_time,mission_true_small_body,mission_true_SC,ms.plot_parameters.plot_SC_number,mission_true_solar_system,mission_true_stars,mission_init_data);
    
    writeVideo(myVideo, getframe(plot_handle));
end

close(myVideo);
end