function func_make_video_HEXP( ...
    mission_storage, ...
    mission_true_time, ...
    mission_init_data, ...
    mission_true_small_body, ...
    mission_true_stars, ...
    mission_true_solar_system, ...
    mission_true_SC, ...
    output_folder)

close all

max_time = 15*24*60*60; % 2 months
kd_max = floor(max_time / mission_storage.true_time.time_step_dynamics);

idx_array = find(mission_storage.true_time.time_array_dynamics);
kd_end = min(idx_array(end), kd_max);

video_filename = [output_folder,mission_storage.plot_parameters.plot_type,'_Video','.mp4'];
myVideo = VideoWriter(video_filename, 'MPEG-4');
myVideo.FrameRate = 50;  % Default 30
myVideo.Quality = 100;    % Default 75

open(myVideo);

tic
for kd = 1:10:kd_end
    
    % Print progress
    if mod(kd-1,10) == 0
        disp([num2str(kd), ' of ',num2str(kd_end), ' (', num2str(round(kd / kd_end * 100)), '%)'])
        % Expected time left
        time_per_loop = toc / kd;
        time_left = seconds(time_per_loop * (kd_end - kd));
        format = 'dd:hh:mm:ss';
        time_left.Format = format;
        disp(['Time left: ', char(time_left)])
    end
    
    plot_handle = func_plot_HEXP( ...
        kd, ...
        mission_storage, ...
        mission_true_time, ...
        mission_init_data, ...
        mission_true_small_body, ...
        mission_true_stars, ...
        mission_true_solar_system, ...
        mission_true_SC, ...
        1);
    
    % delay 1 sec
    pause(1)
    
    % Store video
    F = getframe(plot_handle);
    writeVideo(myVideo, F);
    
end
end