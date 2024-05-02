function func_make_video_earth( ...
    mission_storage, ...
    mission_true_time, ...
    mission_init_data, ...
    mission_true_small_body, ...
    mission_true_stars, ...
    mission_true_solar_system, ...
    mission_true_SC)

close all

max_time = 2*60*60; % 2 hours
kd_max = floor(max_time / mission_storage.true_time.time_step_dynamics);

idx_array = find(mission_storage.true_time.time_array_dynamics);
kd_end = min(idx_array(end), kd_max);

video_filename = [mission_init_data.output_folder,mission_storage.plot_parameters.plot_type,'_Video','.mp4'];
myVideo = VideoWriter(video_filename, 'MPEG-4');
myVideo.FrameRate = 50;  % Default 30
myVideo.Quality = 100;    % Default 75

open(myVideo);

tic
for kd = 2:1:kd_end
    
    % Print progress
    if mod(kd,10) == 0
        disp([num2str(kd), ' of ',num2str(kd_end), ' (', num2str(round(kd / kd_end * 100)), '%)'])
        % Expected time left
        time_per_loop = toc / kd;
        time_left = time_per_loop * (kd_end - kd);
        disp(['Time left: ', num2str(round(time_left)), ' seconds (', num2str(round(time_left / 60)), ' minutes)'])
    end
    
    plot_handle = func_plot_formation_earth( ...
        kd, ...
        mission_storage, ...
        mission_true_time, ...
        mission_init_data, ...
        mission_true_small_body, ...
        mission_true_stars, ...
        mission_true_solar_system, ...
        mission_true_SC);
    
    % delay 1 sec
    pause(1)
    
    % Store video
    F = getframe(plot_handle);
    writeVideo(myVideo, F);
    
end
end