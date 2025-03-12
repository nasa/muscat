function func_make_video(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC)

if mission_storage.plot_parameters.storage_save_mission_video == 1
    
    %% DROID video plot
    switch mission_storage.plot_parameters.plot_type
        %% DROID coverage video plot
        case 'coverage'
            func_make_video_coverage(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC);
            %% DROID video plot
        case 'DROID'
            func_make_video_droid(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC);
            %% DART video plot
        case 'DART'
            func_make_video_dart(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC);
        case "camera"
            func_make_video_camera(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC);
        otherwise
            % do nothing
    end
    % close all
end

end