function func_plot_storage_data_structures(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC)

close all

% ms = ms;
k = ms.true_time.counter;
plot_name_prefix = [mission_init_data.output_folder 'Time_'];
if ms.true_time.time_array(k) < 60
    plot_name_prefix = append(plot_name_prefix,[num2str(round(ms.true_time.time_array(k),2)),'sec_']);
elseif ms.true_time.time_array(k) < 60*60
    plot_name_prefix = append(plot_name_prefix,[num2str(round(ms.true_time.time_array(k)/60,2)),'min_']);
elseif ms.true_time.time_array(k) < 24*60*60
    plot_name_prefix = append(plot_name_prefix,[num2str(round(ms.true_time.time_array(k)/3600,2)),'hr_']);
else
    plot_name_prefix = append(plot_name_prefix,[num2str(round(ms.true_time.time_array(k)/86400,2)),'day_']);
end

plot_name_prefix = append(plot_name_prefix,['factor_X','_']);
if mission_true_SC{1}.software_SC_control_orbit.time_horizon_data_cutoff > 0
    plot_name_prefix = append(plot_name_prefix,['DCO_',num2str(round(mission_true_SC{1}.software_SC_control_orbit.time_horizon_data_cutoff/3600,2)),'_']);
end

fig_number = 0;

% Orbit Vizualization
fig_number = fig_number + 1;
func_plot_orbit_visualization(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);

% SB Interception
fig_number = fig_number + 1;
func_plot_SB_interception(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);

% Attitude Vizualization
fig_number = fig_number + 1;
func_plot_attitude_visualization(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);

% Attitude control
fig_number = fig_number + 1;
func_plot_attitude_control(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);

% Data
fig_number = fig_number + 1;
func_plot_data(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);

% Power
fig_number = fig_number + 1;
func_plot_power(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);

% % Science
% fig_number = fig_number + 1;
% % func_plot_science(mission_storage,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);
% func_plot_science_mothership(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix);
% 
% % Camera
% fig_number = fig_number + 1;
% func_plot_science_camera_mothership(ms,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC, fig_number, plot_name_prefix)

end