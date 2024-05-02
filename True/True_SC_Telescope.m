classdef True_SC_Telescope < handle
    %TRUE_TIME Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        time
        time_prev
        
        health
        temperature
        
        % power & data
        instantaneous_power_consumed
        instantaneous_power_consumption
        instantaneous_data_volume
        instantaneous_data_generated
        
        measurement_frequency
        measurement_wait_time
        measurement_time
        accumulated_wait_time
        take_measurement
        
        orientation
        
        measurement_vector
        measurement_available
        
        num_targets
        observed_targets
        num_observed_targets
        targets
        
        desired_attitude_for_target_achieved
        
        % Target
        AU_distance
        target_data
        current_target_obs_time
        current_target_position
        current_target_idx
        current_target_exposure
        visibility_labels
        visibility_values
        target_pointing_error
        
        % Visibility
        visibility_target_sun
        visibility_target_moon
        visibility_target_earth
        visibility_sun_earth
        
        t_i_prob_distro
        this_target_solar_angle
    end
    
    methods
        
        function obj = True_SC_Telescope(sc_data,mission_true_time)
            
            obj.time_prev = 0;
            obj.time = mission_true_time.time;
            
            obj.health = 1; % 1 = ok
            obj.temperature = 20;  % [deg]
            
            % power & data
            obj.instantaneous_power_consumed = 0;
            obj.instantaneous_power_consumption = sc_data.telescope_power_consumption;
            obj.instantaneous_data_volume = sc_data.telescope_data_volume;
            obj.instantaneous_data_generated = 0;
            
            
            obj.measurement_frequency = sc_data.telescope_measurement_frequency;
            obj.measurement_wait_time = 1/obj.measurement_frequency;
            obj.accumulated_wait_time = 0;
            obj.take_measurement = 0;
            obj.measurement_time = 0;
            
            obj.orientation = sc_data.telescope_orientation;
            
            % Constants
            obj.AU_distance = 1.5e8; % [km]
            
            obj.desired_attitude_for_target_achieved = 0;
            
            % Target data
            nustar_obs = readtable(sc_data.path_target_data);
            % idxs = all(cell2mat(nustar_obs.obs_type) ~= 'TOO', 2); % Filter out TOO
            idxs = true(size(nustar_obs.obs_type));
            nustar_obs = nustar_obs(idxs, :);
            obj.target_data = [];
            obj.target_data.name = nustar_obs.name;
            obj.target_data.ra = dms2degrees(cell2mat(cellfun(@(x) str2double(strsplit(x)), nustar_obs.ra, 'UniformOutput', false)));
            obj.target_data.dec = dms2degrees(cell2mat(cellfun(@(x) str2double(strsplit(x)), nustar_obs.dec, 'UniformOutput', false)));
            obj.target_data.exposure = nustar_obs.exposure_a;
            
            obj.current_target_obs_time = 0;
            obj.current_target_idx = 1;
            obj.current_target_position = zeros(3,1);
            obj.current_target_exposure = Inf;
        end
        
        function obj = func_update_target(obj, mission_true_SC)
            % HEXP Operations
            % - Every 24h, point antenna to downlink to Earth
            % - Queuing based on a target list
            % - If next target is with 60 deg of the Sun, skip it
            % - Enough power for downlink, heating up thrusters and performin observations
            
            % Obstruction Variables
            keep_out_angle_Sun = deg2rad(30); % [deg]
            keep_out_angle_Moon = deg2rad(0); % [deg]
            
            % Target
            flag_target_selected = false;
            target_idx_ant = obj.current_target_idx;
            n_targets_str = num2str(length(obj.target_data.ra));
            while ~flag_target_selected
                
                if obj.current_target_obs_time >= obj.current_target_exposure
                    % Target finished
                    disp(['Finished observing target ', num2str(obj.current_target_idx), '/', n_targets_str]);
                    obj.current_target_idx = obj.current_target_idx + 1;
                    obj.current_target_obs_time = 0;
                    obj.desired_attitude_for_target_achieved = 0;
                else
                    % Target being observed
                    ra = obj.target_data.ra(obj.current_target_idx); % [deg]
                    dec = obj.target_data.dec(obj.current_target_idx); % [deg]
                    obj.current_target_position = 100 * obj.AU_distance * Planetocentric2Cartesian(...
                        [1, deg2rad(dec), deg2rad(ra)]' ...
                        );
                    
                    % Retrieve positions
                    sw_est = mission_true_SC.software_SC_estimate_SC_SB_orbit;
                    pos_sun = sw_est.Sun_position;
                    pos_moon = sw_est.Moon_position;
                    pos_earth = sw_est.Earth_position;
                    pos_sc = sw_est.SC_position;
                    pos_target = obj.current_target_position;
                    
                    R_Earth = 6378.137; % [km]
                    
                    % Compute visibility
                    obj.this_target_solar_angle = func_angle_between_vectors(pos_sun - pos_sc, pos_target - pos_sc);
                    obj.visibility_target_sun = obj.this_target_solar_angle >= keep_out_angle_Sun;
                    obj.visibility_target_moon = func_angle_between_vectors(pos_moon - pos_sc, pos_target - pos_sc) ...
                        >= keep_out_angle_Moon;
                    obj.visibility_target_earth = ~calculate_eclipse(pos_earth, pos_target, pos_sc, R_Earth);
                    obj.visibility_sun_earth = ~calculate_eclipse(pos_earth, pos_sun, pos_sc, R_Earth);
                    
                    if obj.visibility_target_sun
                        % If target is visible (only checking Sun), select it
                        flag_target_selected = true;
                    else
                        % If target is not visible, select next target
                        disp(['Skipping target ', num2str(obj.current_target_idx), '/', n_targets_str]);
                        obj.current_target_idx = obj.current_target_idx + 1;
                        obj.current_target_obs_time = 0;
                    end
                    % Calculate visibility
                    % visibility_target = (obj.visibility_values(1) && obj.visibility_values(2) && obj.visibility_values(3));
                end
            end
            if obj.current_target_idx ~= target_idx_ant
                disp(['Selected target ', num2str(obj.current_target_idx), '/', n_targets_str])
            end
            
            this_target_exposure = obj.target_data.exposure(obj.current_target_idx);
            this_target_intensity = generate_target_intensity(this_target_exposure);
            this_target_accrued_data = generate_timestep_datavol(obj.measurement_wait_time, this_target_intensity);
            
            obj.current_target_exposure = this_target_exposure;
            obj.instantaneous_data_volume = this_target_accrued_data;
            
        end
        
        function obj = func_update_accumulated_wait_time(obj, True_time)
            % Update current time
            % Update accumulated time since last measurement
            
            obj.time = True_time.time;
            
            if obj.health == 1
                obj.accumulated_wait_time = obj.time - obj.measurement_time;
            else
                obj.accumulated_wait_time = 0;
            end
            
        end
        
        function obj = func_check_wait_time(obj)
            % check if a measurement must be taken or not accroding to
            % accumulated wait time
            
            if obj.accumulated_wait_time >= obj.measurement_wait_time
                obj.take_measurement = 1;
            else
                obj.take_measurement = 0;
            end
            
        end
        
        function obj = func_check_attitude(obj,SC_control_attitude,SC_estimate_attitude)
            
            
            achieved_prev = obj.desired_attitude_for_target_achieved;
            obj.desired_attitude_for_target_achieved = 0;
            
            % Check desired attitude for SB pointing is achieved
            if ismember(SC_control_attitude.desired_SC_attitude_mode, [6,7]) ...
                    && norm(SC_control_attitude.desired_attitude - SC_estimate_attitude.attitude) <= 0.1
                obj.desired_attitude_for_target_achieved = 1;
            else
                obj.desired_attitude_for_target_achieved = 0;
            end
            
            if obj.desired_attitude_for_target_achieved && ~achieved_prev
                disp('Desired attitude for target achieved')
            end
            
        end
        
        function obj = func_get_telescope_measurement(obj, true_time, true_small_body, true_SC_navigation)
            % update the measurement if take_measurement is up
            % set/reset flags and accumulated wait time
            
            obj = func_update_accumulated_wait_time(obj, true_time);    % update time
            obj = func_check_wait_time(obj);                            % check for measurement trigerring
            
            if (obj.take_measurement == 1) && (obj.desired_attitude_for_target_achieved == 1)
                
                obj.current_target_obs_time = obj.current_target_obs_time + obj.measurement_wait_time;
                
                % Update routine flag, time, power, data
                obj.measurement_time = obj.time;
                obj.measurement_available = 1;
                obj.take_measurement = 0;
                obj.instantaneous_data_generated = obj.instantaneous_data_volume; % [kb, kbits]
                obj.instantaneous_power_consumed = obj.instantaneous_power_consumption;
            else
                obj.measurement_available = 0;
                obj.instantaneous_data_generated = 0;
                obj.instantaneous_power_consumed = 0;
            end
        end
    end
end


% Randomly generates target intensity based on NuSTAR distribution
function intensity = generate_target_intensity(this_target_exposure)
% From nustar slews NB file
exp_scale_bins = [0.47712125,  1.31861945,  2.16011765,  3.00161585,  3.84311404, 4.68461224,  5.52611044];

scaled_exp = log10(this_target_exposure);
i = 1;
while exp_scale_bins(i) < scaled_exp
    i = i + 1;
end
i_bin_0 = i - 1;

intensity = generate_intensity(i_bin_0); % Calling the previous function
end


function intensity = generate_intensity(bin0)
% Source Incidence Fraction
source_intensity_fraction = [[2, 0.09]; [10, 0.76]; [20, 0.05]; [100, 0.07]; [200, 0.01]; [1000, 0.015]; [2000, 0.004]];

avgb = flip(source_intensity_fraction(:,1));
avgb = avgb(bin0);

stdVal = 0.3 * avgb;

intensity = normrnd(avgb, stdVal);
end

function tdv = generate_timestep_datavol(ts, tint)
% ts: target exposure
% tint: target intensity

% mCrab vs kb/s per Data Rates 2/1/2023
leda_data_rates = [[2, 27]; [10, 54]; [20, 108]; [100, 541]; [200, 1082]; [1000, 5410]];
% Not deadtime corrected values. Needs to be checked against Kristin
% For two HEDAs mCrab vs kb/s per Data Rates 2/1/2023.
heda_data_rates = [[0, 5 * 2]; [10, 9 * 2]; [20, 19 * 2]; [100, 92 * 2]; [200, 184 * 2]; [1000, 920 * 2]];
% 0.62 Gb/day for two lasers converted to kbps
met_data_rate = 0.62 * 1e6 / 3600 / 24;
% for LEDA + 2x HEDAs in kbps
hk_data_rate = 0.7 + 0.6 * 2;

ldrg = @(x) interp1(leda_data_rates(:, 1), leda_data_rates(:, 2), x, 'linear', 'extrap');
hdrg = @(x) interp1(heda_data_rates(:, 1), heda_data_rates(:, 2), x, 'linear', 'extrap');

tdv = ts * (ldrg(tint) + hdrg(tint) + hk_data_rate + met_data_rate);
end
