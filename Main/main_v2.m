% Main File to Execute Missions

addpath(genpath('../MuSCAT_Supporting_Files'))

init_data = [];
init_data.flag_store_video = 0;
init_data.flag_show_video = 0;
if (init_data.flag_store_video == 1)
    init_data.video_filename = ['Camera_Video','.mp4'];
    myVideo = VideoWriter(init_data.video_filename, 'MPEG-4');
    myVideo.FrameRate = 30;  % Default 30
    myVideo.Quality = 100;    % Default 75
    open(myVideo);
end

%% Time Loop

this_loop_data = [];

tic

disp('Starting Simulation')
for k=2:1:mission_true_time.num_time_steps
    % Print progress
    if mod(k-2,200) == 0
        % Expected time left
        time_elap = seconds(toc);
        time_per_loop = time_elap / k;
        time_left = time_per_loop * (mission_true_time.num_time_steps - k);
        
        time_sim_elapsed = seconds(mission_true_time.time - mission_true_time.t_initial);
        time_sim_total = seconds(mission_true_time.t_final - mission_true_time.t_initial);
        
        perc = round(time_sim_elapsed / time_sim_total * 100, 1);
        
        format = 'dd:hh:mm:ss';
        time_elap.Format = format;
        time_left.Format = format;
        time_sim_elapsed.Format = format;
        time_sim_total.Format = format;
        
        disp(['- Simulation: ', char(time_sim_elapsed), ' / ', char(time_sim_total), ' (', num2str(perc),'%),' ...
            ' Elapsed: ', char(time_elap), ', Left: ', char(time_left), ', Total: ', char(time_elap + time_left)])
    end
    
    % Update Dynamics Flag
    update_dynamics_flag = rem(k, round(mission_true_time.time_step_dynamics / mission_true_time.time_step)) == 0;

    % Update time
    mission_true_time = func_update_true_time_date(mission_true_time);

    %% Update Environmental Variables
    if update_dynamics_flag
        % Update Sun and Earth Position
        mission_true_solar_system = func_update_Sun_Earth_position_velocity(mission_true_solar_system,mission_true_time);

        % Update SB position
        mission_true_small_body.func_update_SB_position_velocity_rot_matrix(mission_true_time);
    end
    
 
    %% Update SC Variables
    for i_SC = 1:mission_init_data.num_SC
        % Update SC Mass
        mission_true_SC{i_SC}.software_SC_executive.mass = mission_true_SC{i_SC}.true_SC_body.mass;
        
        % Update & Reset Data/Power
        mission_true_SC{i_SC}.true_SC_power.time = mission_true_time.time;
        mission_true_SC{i_SC}.true_SC_data_handling.time = mission_true_time.time;
        mission_true_SC{i_SC}.true_SC_power.instantaneous_power_consumed = 0;
        mission_true_SC{i_SC}.true_SC_power.instantaneous_power_generated = 0;
        mission_true_SC{i_SC}.true_SC_data_handling.instantaneous_data_generated = 0;
        
        %% External Disturbance torque and force
        mission_true_SC{i_SC}.true_SC_gravity_gradient = func_update_disurbance_torque_G2( ...
            mission_true_SC{i_SC}.true_SC_gravity_gradient, ...
            mission_true_SC{i_SC}.true_SC_adc, ...
            mission_true_SC{i_SC}.true_SC_navigation, ...
            mission_true_small_body);
        mission_true_SC{i_SC}.true_SC_srp = func_update_disturbance_force_torque_SRP(...
            mission_true_SC{i_SC}.true_SC_srp, ...
            mission_true_SC{i_SC}.true_SC_body, ...
            mission_true_SC{i_SC}.true_SC_adc, ...
            mission_true_SC{i_SC}.true_SC_navigation, ...
            mission_true_solar_system);
        mission_true_SC{i_SC}.true_SC_navigation.disturbance_force = mission_true_SC{i_SC}.true_SC_srp.disturbance_force_SRP;
        mission_true_SC{i_SC}.true_SC_adc = func_update_disturbance_torque( ...
            mission_true_SC{i_SC}.true_SC_adc, ...
            mission_true_SC{i_SC}.true_SC_srp, ...
            mission_true_SC{i_SC}.true_SC_gravity_gradient);
        
        %% Turth Position Dynamics
        if update_dynamics_flag
            % Control thurst
            mission_true_SC{i_SC}.true_SC_navigation = func_update_control_force(mission_true_SC{i_SC}.true_SC_navigation,mission_true_SC{i_SC});
            % Simulate Position+velocity dynamics
            mission_true_SC{i_SC}.true_SC_navigation.func_true_position_velocity_dynamics( ...
                mission_init_data, ...
                mission_true_time, ...
                mission_true_small_body, ...
                mission_true_solar_system, ...
                mission_true_SC{i_SC}.true_SC_body);
        end
        
        %% Truth Attitude Dynamics
        % Control Torque and Momentum from Reaction Wheel
        mission_true_SC{i_SC}.true_SC_adc = func_update_control_torque(mission_true_SC{i_SC}.true_SC_adc, mission_true_SC{i_SC});
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
            true_rw_momentum = mission_true_SC{i_SC}.true_SC_rwa_actuator.func_calc_true_rw_momentum();
        else
            true_rw_momentum = zeros(3,1);
        end
        % Simulate Attitude dynamics
        mission_true_SC{i_SC}.true_SC_adc = func_true_attitude_angular_velocity_dynamics(mission_true_SC{i_SC}.true_SC_adc, mission_true_time, true_rw_momentum);
        % Compute Rotation Matrix
        mission_true_SC{i_SC}.true_SC_adc = func_update_rotation_matrix(mission_true_SC{i_SC}.true_SC_adc);
        % Simulate true Reaction Wheel dynamics
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
            mission_true_SC{i_SC}.true_SC_rwa_actuator = func_dynamics_reaction_wheel(mission_true_SC{i_SC}.true_SC_rwa_actuator, mission_true_SC{i_SC}.true_SC_adc);
        end




    end
    
    
    for i_SC = 1:mission_init_data.num_SC
        sc = mission_true_SC{i_SC};
        
        %% Executive Layer (Layer 5)
        
        % Get Time
        sc.software_SC_executive.get_time = 1;
        
        %% Reactive Layer (Layer 6)
        
        %% Get Time
        
        % On board clock
        sc.true_SC_onboard_clock_sensor = func_get_time(sc.true_SC_onboard_clock_sensor,mission_true_time);
        
        if (sc.software_SC_executive.get_time == 1) ...
                && (sc.true_SC_onboard_clock_sensor.onboard_clock_data(1).measurement_available == 1)
            
            sc.software_SC_executive.time = sc.true_SC_onboard_clock_sensor.time;
            sc.software_SC_executive.time_step = sc.software_SC_executive.time - sc.software_SC_executive.prev_time;
            sc.software_SC_executive.prev_time = sc.software_SC_executive.time;
            
            sc.software_SC_executive.date = sc.software_SC_executive.t_initial_date + sc.software_SC_executive.time;
            
            % Update Power and Data
            for j=1:sc.true_SC_onboard_clock_sensor.num_onboard_clock
                sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                    sc.true_SC_power, ...
                    sc.true_SC_onboard_clock_sensor.onboard_clock_data(j));
                sc.true_SC_data_handling = func_update_data_generated( ...
                    sc.true_SC_data_handling,sc.true_SC_onboard_clock_sensor.onboard_clock_data(j));
            end
        end
        
        %% Executive Layer (Layer 5)
        
        sc.software_SC_executive = func_schedule_main_tasks(sc.software_SC_executive, sc);
        
        
        %% Reactive Layer (6)
        
        if sc.software_SC_executive.estimate_SC_attitude == 1
            
            % Update Time
            sc.software_SC_estimate_attitude.time = sc.software_SC_executive.time;
            
            %% Sensors update routine
            % Sun Sensor Sensor Measurements
            if sc.true_SC_body.flag_hardware_exists.adc_sun_sensor == 1
                
                sc.true_SC_sun_sensor.func_get_sun_sensor_measurement( ...
                    sc.true_SC_adc, ...
                    mission_true_time);
                sc.software_SC_estimate_attitude.measurement_available_sun_sensor = ...
                    sc.true_SC_sun_sensor.sun_sensor_data(1).measurement_available;
                % Update Power and Data
                for j=1:sc.true_SC_sun_sensor.num_sun_sensor
                    sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                        sc.true_SC_power, ...
                        sc.true_SC_sun_sensor.sun_sensor_data(j));
                    sc.true_SC_data_handling = func_update_data_generated( ...
                        sc.true_SC_data_handling, ...
                        sc.true_SC_sun_sensor.sun_sensor_data(j));
                end
            end
            
            % Star Tracker Sensor Measurements
            if sc.true_SC_body.flag_hardware_exists.adc_star_tracker == 1
                sc.true_SC_star_tracker_sensor = func_get_star_tracker_measurement( ...
                    sc.true_SC_star_tracker_sensor, ...
                    sc.true_SC_adc, ...
                    mission_true_time);
                sc.software_SC_estimate_attitude.measurement_available_star_tracker = ...
                    sc.true_SC_star_tracker_sensor.star_tracker_data(1).measurement_available;
                % Update Power and Data
                for j=1:sc.true_SC_star_tracker_sensor.num_star_tracker
                    sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                        sc.true_SC_power, ...
                        sc.true_SC_star_tracker_sensor.star_tracker_data(j));
                    sc.true_SC_data_handling = func_update_data_generated( ...
                        sc.true_SC_data_handling, ...
                        sc.true_SC_star_tracker_sensor.star_tracker_data(j));
                end
            end
            
            % IMU Sensor Measurements
            if sc.true_SC_body.flag_hardware_exists.adc_imu == 1
                sc.true_SC_imu_sensor = func_get_imu_sensor_measurement(...
                    sc.true_SC_imu_sensor, ...
                    sc.true_SC_adc, ...
                    mission_true_time);
                sc.software_SC_estimate_attitude.measurement_available_imu = ...
                    sc.true_SC_imu_sensor.imu_sensor_data(1).measurement_available;
                % Update Power and Data
                for j=1:sc.true_SC_imu_sensor.num_imu_sensor
                    sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                        sc.true_SC_power, ...
                        sc.true_SC_imu_sensor.imu_sensor_data(j));
                    sc.true_SC_data_handling = func_update_data_generated( ...
                        sc.true_SC_data_handling, ...
                        sc.true_SC_imu_sensor.imu_sensor_data(j));
                end
            end
            
            switch mission_init_data.attitude_estimation_type
                case 'truth'
                    % Use true data bypass
                    sc.software_SC_estimate_attitude = func_update_attitude_with_truth_data(...
                        sc.software_SC_estimate_attitude,...
                        sc.true_SC_adc);
                    
                case 'KF'
                    % KF Propagation Step
                    sc.software_SC_estimate_attitude = func_KF_propagation_step(...
                        sc.software_SC_estimate_attitude, ...
                        sc.software_SC_control_attitude, ...
                        sc.true_SC_micro_thruster_actuator, ...
                        sc.software_SC_executive);
                    
                    % KF Update Step
                    sc.software_SC_estimate_attitude = func_KF_update_step(...
                        sc.software_SC_estimate_attitude, ...
                        sc.true_SC_imu_sensor, ...
                        sc.true_SC_sun_sensor, ...
                        sc.true_SC_star_tracker_sensor);
                otherwise
                    error('Wrong attitude estimation method')
            end
            
            % Update estimated rotation matrix
            if sc.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
                meas_rw_momentum = sc.true_SC_rwa_actuator.func_calc_meas_rw_momentum();
            else
                meas_rw_momentum = zeros(3,1);
            end
            sc.software_SC_estimate_attitude = func_compute_rotation_matrix(...
                sc.software_SC_estimate_attitude, ...
                sc.software_SC_control_attitude, ...
                meas_rw_momentum);
        end
        % end % end parfor
        
        % for i_SC = 1:mission_init_data.num_SC
        %% Control SC attitude
        
        if sc.software_SC_executive.control_SC_attitude == 1
            
            % Update Time
            sc.software_SC_control_attitude.time = sc.software_SC_executive.time;
            
            % Decide to Compute Desired Attitude
            sc.software_SC_control_attitude = func_start_selected_desired_attitude( ...
                sc.software_SC_control_attitude,sc.software_SC_executive);
            
            % Compute Desired Attitude
            if sc.software_SC_control_attitude.compute_desired_attitude == 1
                switch sc.software_SC_control_attitude.desired_SC_attitude_mode
                    
                    case 1 % 1 = Point to SB
                        if (sc.true_SC_body.flag_hardware_exists.navigation_camera == 1)
                            % Point camera if you have one

                         
                            pointing = sc.true_SC_camera_sensor.camera_data.rotation_matrix*[1 0 0]';
                          
                        elseif (sc.true_SC_body.flag_hardware_exists.science_radar == 1)
                            % Point radar if you have one
                            pointing = sc.true_SC_radar.orientation;
                        elseif (sc.true_SC_body.flag_hardware_exists.science_altimeter == 1)
                            % Point altimeter if you have one
                            pointing = sc.true_SC_altimeter.orientation;
                        else
                            % else point along x axis
                            pointing = [1 0 0];
                        end


                    
                        SP_pointing =  sc.true_SC_solar_panel.solar_panel_data(1).shape_model.orientation_solar_cell_side;  % Point solar panel
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_SB_maximize_SP( ...
                            sc.software_SC_control_attitude, ...
                            sc.software_SC_estimate_SC_SB_orbit, ...
                            pointing, ...
                            SP_pointing);
                        %                         sc.software_SC_control_attitude = func_compute_desired_attitude_for_SB(sc.software_SC_control_attitude,sc.software_SC_estimate_SC_SB_orbit, pointing);
                        
                    case 2 % 2 = Maximize SP Power
                        pointing =  sc.true_SC_solar_panel.solar_panel_data(1).shape_model.orientation_solar_cell_side;  % Point solar panel
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_Solar_Panels_To_Sun( ...
                            sc.software_SC_control_attitude,sc.software_SC_estimate_SC_SB_orbit, pointing);
                        
                    case 3 % 3 = Point Thurster along DeltaV direction
                        SP_pointing =  sc.true_SC_solar_panel.solar_panel_data(1).shape_model.orientation_solar_cell_side;  % Point solar panel
                        pointing = sc.true_SC_chemical_thruster_actuator.chemical_thruster_data(1).orientation;
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_DeltaV_maximize_SP( ...
                            sc.software_SC_control_attitude ,sc.software_SC_control_orbit, sc.software_SC_estimate_SC_SB_orbit, pointing, SP_pointing);
                        
                    case 4 % 4 = Point Antenna to Earth
                        index_dte_comm = find(sc.software_SC_communication.comm_interlocutor == -1);                     % id of DTE comm
                        antenna_used_for_dte = sc.software_SC_communication.commu_use_antenna_index(index_dte_comm);     % antenna used for dte
                        pointing = sc.true_SC_radio.antenna_data(antenna_used_for_dte).antenna_pointing;                 % pointing of antenna used for dte
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_DTE( ...
                            sc.software_SC_control_attitude,sc.software_SC_estimate_SC_SB_orbit,pointing);
                        
                    case 5 % 5 = Point Antenna for InterSat comm
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_intersat_comm( ...
                            sc.software_SC_control_attitude,mission_true_SC,i_SC);
                        
                    case 6 % 6 = Point to target (e.g., exoplanet) and maximize SP Power
                        % Update target
                        sc.true_SC_telescope = func_update_target( ...
                            sc.true_SC_telescope, ...
                            sc);
                        
                        % Retrieve instrument pointing
                        pointing_science = sc.true_SC_telescope.orientation;
                        pointing_sun =  sc.true_SC_solar_panel.solar_panel_data(1).shape_model.orientation_solar_cell_side';
                        
                        % Compute attitude
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_target_SP( ...
                            sc.software_SC_control_attitude, ...
                            sc.true_SC_telescope.current_target_position, ...
                            sc.software_SC_estimate_SC_SB_orbit, ...
                            pointing_science, ...
                            pointing_sun);
                        
                    case 7 % 7 = Point to Earth and target (if possible) and maximize SP Power
                        % Update target
                        sc.true_SC_telescope = func_update_target( ...
                            sc.true_SC_telescope, ...
                            sc);
                        
                        % Retrieve instrument pointing
                        antenna_gimbal = deg2rad(180); % TODO: this is a temporary value
                        index_dte_comm = find(sc.software_SC_communication.comm_interlocutor == -1); % id of DTE comm
                        antenna_used_for_dte = sc.software_SC_communication.commu_use_antenna_index(index_dte_comm);     % antenna used for dte
                        pointing_science = sc.true_SC_telescope.orientation;
                        pointing_sun = sc.true_SC_solar_panel.solar_panel_data(1).shape_model.orientation_solar_cell_side';
                        pointing_dte = sc.true_SC_radio.antenna_data(antenna_used_for_dte).antenna_pointing;
                        
                        % Compute attitude
                        sc.software_SC_control_attitude = func_compute_desired_attitude_for_DTE_target_SP( ...
                            sc.software_SC_control_attitude, ...
                            sc.true_SC_telescope.current_target_position, ...
                            sc.software_SC_estimate_SC_SB_orbit, ...
                            pointing_science, ...
                            pointing_sun, ...
                            pointing_dte, ...
                            antenna_gimbal);
                        
                    otherwise
                        error('Desired SC attitude mode not defined')
                end
            end
            
            % Compute Contol Torque
            sc.software_SC_control_attitude = function_compute_desired_control_torque( ...
                sc.software_SC_control_attitude, ...
                sc.software_SC_estimate_attitude, ...
                meas_rw_momentum, ...
                sc.true_SC_adc);
            
            % Desaturate the RWA
            if (sc.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1) ...
                    && (sc.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1)      % only if SC has RWA and MT
                
                if (sc.software_SC_executive.desat_procedure == 1) ...
                        || (sc.true_SC_rwa_actuator.flag_desat_in_process == 1)
                    sc.true_SC_rwa_actuator = function_compute_RWA_DeSat_torque( ...
                        sc.true_SC_rwa_actuator,sc.software_SC_estimate_attitude);
                else
                    sc.true_SC_rwa_actuator.desired_desat_control_torque = zeros(3,1);
                end
                
            end
            
            % Decide which actuator to use
            sc.software_SC_control_attitude = func_actuator_selection( ...
                sc.software_SC_control_attitude,sc);
            
            % Command RWA
            if sc.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
                
                if (sc.true_SC_rwa_actuator.flag_desat_in_process ~= 1)
                    sc.true_SC_rwa_actuator = func_compute_rw_command(...
                        sc.true_SC_rwa_actuator,...
                        sc.software_SC_control_attitude, ...
                        sc.software_SC_estimate_attitude);
                end
                
                sc.software_SC_control_attitude.control_torque = ...
                    sc.software_SC_control_attitude.control_torque ...
                    + sum([sc.true_SC_rwa_actuator.RW_data.commanded_control_torque_RW]')';
                
                % Update Power and Data
                for j=1:sc.true_SC_rwa_actuator.num_reaction_wheel
                    sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                        sc.true_SC_power, ...
                        sc.true_SC_rwa_actuator.RW_data(j));
                    sc.true_SC_data_handling = func_update_data_generated( ...
                        sc.true_SC_data_handling, ...
                        sc.true_SC_rwa_actuator.RW_data(j));
                end
            end
            
            % Command MT
            if sc.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
                
                switch sc.software_SC_control_attitude.select_micro_thruster_decomposition
                    case 1 % 1 = Optimization (slow)
                        sc.true_SC_micro_thruster_actuator = func_decompose_control_torque_into_thrusters_optimization_kkt( ...
                            sc.true_SC_micro_thruster_actuator,sc.software_SC_control_attitude);
                    case 2 % 2 = Adhoc (Fast, but doesnt check resultant forces)
                        sc.true_SC_micro_thruster_actuator = func_decompose_control_torque_into_thrusters_adhoc( ...
                            sc.true_SC_micro_thruster_actuator,sc.software_SC_control_attitude);
                    otherwise
                        % Should not reach here!
                end
                
                % Send thruster commands to spacecraft
                sc.true_SC_micro_thruster_actuator = func_command_thrust( ...
                    sc.true_SC_micro_thruster_actuator,sc.software_SC_executive,mission_true_time);
                sc.software_SC_control_attitude.control_torque = ...
                    sc.software_SC_control_attitude.control_torque ...
                    + sum([sc.true_SC_micro_thruster_actuator.MT_data.commanded_torque]')';
                
                % Update Power and Data
                for j=1:sc.true_SC_micro_thruster_actuator.num_micro_thruster
                    sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                        sc.true_SC_power, ...
                        sc.true_SC_micro_thruster_actuator.MT_data(j));
                    sc.true_SC_data_handling = func_update_data_generated( ...
                        sc.true_SC_data_handling,sc.true_SC_micro_thruster_actuator.MT_data(j));
                end
            end
            
        end
        
        
        %% Get Camera Image
        
        if (sc.true_SC_body.flag_hardware_exists.navigation_camera == 1) && (sc.software_SC_executive.get_camera_image == 1)
            
            % Update Time
            sc.true_SC_camera_sensor.time = sc.software_SC_executive.time;
            
            % Update only when executive decides it
            for index_camera = 1:sc.true_SC_camera_sensor.num_camera
                
                %                 sc.true_SC_camera_sensor = func_update_accumulated_wait_time(sc.true_SC_camera_sensor, mission_true_time);    % update time
                %                 sc.true_SC_camera_sensor = func_check_wait_time(sc.true_SC_camera_sensor);                                    % check for measurement trigerring
                
                if (sc.true_SC_camera_sensor.camera_data(index_camera).flag_show_camera_plot == 1)
                    
                    clear plot_handle
                    plot_handle = figure(index_camera);
                    clf
                    set(plot_handle,'Color',[12, 22, 79]/255*0.5,'Position',[10 100 sc.true_SC_camera_sensor.camera_data(index_camera).resolution]);
                    
                    sc.true_SC_camera_sensor = func_get_camera_image(sc.true_SC_camera_sensor,index_camera,mission_true_time,mission_true_small_body,mission_true_SC,i_SC,mission_true_solar_system,mission_true_stars,mission_init_data);
                    
                    if (init_data.flag_store_video == 1)
                        writeVideo(myVideo, getframe(plot_handle));
                    end
                    
                else
                    sc.true_SC_camera_sensor = func_get_camera_image(sc.true_SC_camera_sensor,index_camera,mission_true_time,mission_true_small_body,mission_true_SC,i_SC,mission_true_solar_system,mission_true_stars,mission_init_data);
                end
                
                
                % Process image
                sc.true_SC_camera_sensor = func_process_image(sc.true_SC_camera_sensor,sc.true_SC_navigation, mission_true_small_body,sc.true_SC_adc);
                
                % Update power and data
                sc.true_SC_power = fun_update_instantaneous_power_consumed(sc.true_SC_power,sc.true_SC_camera_sensor.camera_data(index_camera));
                
                sc.true_SC_data_handling = func_update_data_generated(sc.true_SC_data_handling,sc.true_SC_camera_sensor.camera_data(index_camera));
                
                % Science Camera Measurements
                if isfield(sc.true_SC_body.flag_hardware_exists, 'science_camera') && ...
                        sc.true_SC_body.flag_hardware_exists.science_camera == 1
                    sc.true_SC_science_camera = func_get_science_camera_measurements(sc.true_SC_science_camera, mission_true_time, mission_true_small_body, sc.true_SC_navigation, sc.true_SC_camera_sensor,index_camera, mission_true_solar_system, i_SC);
                end
                
            end
            
        end
        
        % Get GNSS measurement
        if isfield(sc.true_SC_body.flag_hardware_exists, 'gnss_receiver') && ...
                sc.true_SC_body.flag_hardware_exists.gnss_receiver == 1
            sc.true_SC_gnss_receiver.func_get_measurement(mission_true_SC, mission_true_time);
        end
        
        %% Estimate SC and SB Orbits
        
        if sc.software_SC_executive.estimate_SC_SB_orbits == 1
            
            % Update Time
            sc.software_SC_estimate_SC_SB_orbit.time = sc.software_SC_executive.time;
            
            switch mission_init_data.state_estimation_type
                case 'truth'
                    sc.software_SC_estimate_SC_SB_orbit.func_update_orbit_with_truth_data( ...
                        sc.true_SC_navigation, ...
                        mission_true_small_body, ...
                        mission_true_solar_system);
                    
                    % sc.software_SC_estimate_SC_SB_orbit.SC_position_relative_SB = sc.true_SC_navigation.position_relative_SB;
                    % sc.software_SC_estimate_SC_SB_orbit.SC_velocity_relative_SB = sc.true_SC_navigation.velocity_relative_SB;
                    % sc.software_SC_estimate_SC_SB_orbit.time_prev = sc.software_SC_estimate_SC_SB_orbit.time;
                    % sc.software_SC_estimate_SC_SB_orbit = update_true_bodies( ...
                    %     sc.software_SC_estimate_SC_SB_orbit, ...
                    %     mission_true_small_body, ...
                    %     mission_true_solar_system);
                    
                case 'swarm'
                    sc.software_SC_estimate_SC_SB_orbit.func_get_orbit_measurement(sc);
                    sc.software_SC_estimate_SC_SB_orbit.step(mission_true_small_body, mission_true_solar_system);
                    
                otherwise
                    error('Wrong state estimation method')
            end
        end
        
        %% Control Orbits
        
        if sc.software_SC_executive.control_SC_orbit == 1
            
            % Update Time
            sc.software_SC_control_orbit.time = sc.software_SC_executive.time;
            sc.software_SC_control_orbit.date = sc.software_SC_executive.date;
            
            if sc.software_SC_control_orbit.time > (sc.software_SC_control_orbit.time_DeltaV + 20)
                % Compute new TCM, old  iTCMs not valid anymore
                sc.software_SC_control_orbit.desired_DeltaV_computed = 0;
            end
            
            if sc.software_SC_control_orbit.desired_DeltaV_computed == 0
                
                switch mission_init_data.mission_type
                    
                    % case 1 % : Approach SB and Orbit SB
                    
                    case 2 % : DART mission profile
                        
                        % Compute SC-SB Intercept Point
                        sc.software_SC_control_orbit = func_estimate_SB_intercept_location_time( ...
                            sc.software_SC_control_orbit,sc.software_SC_estimate_SC_SB_orbit);
                        
                        % Compute SC-SB Intercept Point using Turth (SC can't access this!)
                        sc.true_SC_navigation = func_true_SB_intercept_distance( ...
                            sc.true_SC_navigation,mission_true_small_body,mission_true_solar_system,sc.software_SC_control_orbit);
                        
                        if sc.software_SC_control_orbit.intercept_distance > sc.software_SC_estimate_SC_SB_orbit.SB_radius
                            % Compute TCM using Lambert Manuver
                            sc.software_SC_control_orbit = func_compute_TCM_Lambert_Battin_v2( ...
                                sc.software_SC_control_orbit,sc.software_SC_estimate_SC_SB_orbit);
                            sc.true_SC_chemical_thruster_actuator.chemical_thruster_data.command_time = sc.software_SC_control_orbit.time; % reset time for warmup to begin
                        end
                        
                    case 3 % : Orbit SB
                        
                        % Compute SC Distance to Desired Trajectory
                        sc.software_SC_control_orbit = func_compute_distance_desired_circular_trajectory( ...
                            sc.software_SC_control_orbit,sc.software_SC_estimate_SC_SB_orbit);
                        
                        % Compute SC Distance to Desired Trajectory using Turth (SC can't access this!)
                        sc.true_SC_navigation = func_compute_true_distance_desired_circular_trajectory( ...
                            sc.true_SC_navigation,sc.software_SC_control_orbit);
                        
                        if sc.software_SC_control_orbit.intercept_distance > 0*(1e-3)*sc.software_SC_estimate_SC_SB_orbit.SB_radius
                            % Compute TCM using Lambert Manuver
                            sc.software_SC_control_orbit = func_compute_Lambert_intercept_desired_circular_trajectory( ...
                                sc.software_SC_control_orbit, ...
                                sc.software_SC_estimate_SC_SB_orbit, ...
                                mission_true_small_body, ...
                                sc.software_SC_executive);
                            sc.true_SC_chemical_thruster_actuator.chemical_thruster_data.command_time = sc.software_SC_control_orbit.time; % reset time for warmup to begin
                        end
                        
                        
                end
                
            end
            
            if sc.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed == 1
                
                % Update time fo chemical thruster for warming up delay
                sc.true_SC_chemical_thruster_actuator = func_update_accumulated_warm_up_time( ...
                    sc.true_SC_chemical_thruster_actuator, mission_true_time);
                sc.true_SC_chemical_thruster_actuator = func_check_warmup_time( ...
                    sc.true_SC_chemical_thruster_actuator);
                
                % Check Desired Attitude for DeltaV
                sc.software_SC_control_orbit = func_check_attitude_for_DeltaV( ...
                    sc.software_SC_control_orbit,sc.software_SC_control_attitude,sc.software_SC_estimate_attitude);
                
                % Command DeltaV for this time step
                sc.software_SC_control_orbit = func_command_DeltaV( ...
                    sc.software_SC_control_orbit,sc.true_SC_chemical_thruster_actuator,sc.software_SC_estimate_SC_SB_orbit,sc.software_SC_executive);
                
                % Command Chemical Thurster
                if sum([sc.true_SC_chemical_thruster_actuator.chemical_thruster_data.health]) == 1
                    
                    sc.true_SC_chemical_thruster_actuator = func_command_thurst( ...
                        sc.true_SC_chemical_thruster_actuator,sc.software_SC_control_orbit,sc.true_SC_adc,mission_true_time);
                    
                    for j=1:sc.true_SC_chemical_thruster_actuator.num_chemical_thruster
                        % Update Power and Data
                        sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                            sc.true_SC_power, ...
                            sc.true_SC_chemical_thruster_actuator.chemical_thruster_data(j));
                        sc.true_SC_data_handling = func_update_data_generated( ...
                            sc.true_SC_data_handling,sc.true_SC_chemical_thruster_actuator.chemical_thruster_data(j));
                    end
                    
                end
                
            else
                % Switch off thruster
                sc.true_SC_chemical_thruster_actuator = func_switch_off_thurst( ...
                    sc.true_SC_chemical_thruster_actuator, sc.software_SC_executive);
            end
            
        else
            sc.software_SC_control_orbit.desired_DeltaV_computed = 0;
            sc.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed = 0;
            
            % Switch off thruster
            sc.true_SC_chemical_thruster_actuator = func_switch_off_thurst( ...
                sc.true_SC_chemical_thruster_actuator, sc.software_SC_executive);
            
        end
        
        %% Real time plot
        % Attitude
        if (mod(k,10) == 0) && (init_data.flag_show_video == 1)
            clear plot_handle
            plot_handle = figure(index_camera + 1);
            if(i_SC==mission_init_data.num_SC) %if the last SC, store frame and clear figure
                if (init_data.flag_store_video == 1)
                    this_loop_data.F = getframe(plot_handle);
                    writeVideo(myVideo, this_loop_data.F);
                end
                clf
            end
            set(plot_handle,'units','normalized','Position',[0.1 0.1 0.8 0.8])
            subplot(1,mission_init_data.num_SC,i_SC)
            sc.true_SC_adc = func_visualize_attitude( ...
                sc.true_SC_adc, ...
                mission_storage, ...
                sc.true_SC_body, ...
                sc.true_SC_solar_panel, ...
                sc.software_SC_control_attitude, ...
                sc.true_SC_micro_thruster_actuator, ...
                sc.true_SC_chemical_thruster_actuator, ...
                mission_true_time);
            
        end
        
        %% Science measurement
        if sc.software_SC_executive.science_mode_enabled == 1
            
            % Altimeter
            if sc.true_SC_body.flag_hardware_exists.science_altimeter == 1
                
                % check if SC pointing towards SB
                sc.true_SC_altimeter.func_check_attitude_for_comm( ...
                    sc.software_SC_control_attitude, ...
                    sc.software_SC_estimate_attitude);
                
                % perform measurement
                sc.true_SC_altimeter.func_get_altimeter_measurement( ...
                    mission_true_time, mission_true_small_body, sc.true_SC_navigation);
                
                % Update Power and Data
                sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                    sc.true_SC_power, ...
                    sc.true_SC_altimeter);
                sc.true_SC_data_handling = func_update_data_generated( ...
                    sc.true_SC_data_handling,sc.true_SC_altimeter);
            end
            
            % Radar
            if sc.true_SC_body.flag_hardware_exists.science_radar == 1
                
                % check if SC pointing towards SB
                sc.true_SC_radar.func_check_attitude_for_comm( ...
                    sc.software_SC_control_attitude,sc.software_SC_estimate_attitude);
                
                % perform measurement
                sc.true_SC_radar.func_get_radar_measurement( ...
                    mission_true_time, mission_true_small_body, sc.true_SC_navigation, mission_init_data.num_SC, i_SC, mission_true_SC);
                
                % Update Power and Data
                for j=1:sc.true_SC_star_tracker_sensor.num_star_tracker
                    sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                        sc.true_SC_power,sc.true_SC_radar);
                    sc.true_SC_data_handling = func_update_data_generated( ...
                        sc.true_SC_data_handling,sc.true_SC_radar);
                end
            end
            
            % Telescope
            if isfield(sc.true_SC_body.flag_hardware_exists, 'science_telescope') ...
                    && sc.true_SC_body.flag_hardware_exists.science_telescope
                
                % check if SC pointing towards target
                sc.true_SC_telescope = func_check_attitude( ...
                    sc.true_SC_telescope, ...
                    sc.software_SC_control_attitude, ...
                    sc.software_SC_estimate_attitude);
                
                % perform measurement
                sc.true_SC_telescope = func_get_telescope_measurement( ...
                    sc.true_SC_telescope, ...
                    mission_true_time, ...
                    mission_true_small_body, ...
                    sc.true_SC_navigation);
                
                % Update Power and Data
                sc.true_SC_power = fun_update_instantaneous_power_consumed(...
                    sc.true_SC_power, ...
                    sc.true_SC_telescope);
                
                sc.true_SC_data_handling = func_update_data_generated( ...
                    sc.true_SC_data_handling, ...
                    sc.true_SC_telescope);
            end

            % Mid-/long-wave infrared point spectrometer (MLPS)
            if isfield(mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists, 'science_mlps') ...
                && mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_mlps
            
                % check if SC pointing towards SB
                 mission_true_SC{i_SC}.true_SC_mlps = func_check_attitude_for_comm( ...
                    mission_true_SC{i_SC}.true_SC_mlps,mission_true_SC{i_SC}.software_SC_control_attitude,mission_true_SC{i_SC}.software_SC_estimate_attitude);
                
                % perform measurement
                mission_true_SC{i_SC}.true_SC_mlps = func_get_mlps_measurement(mission_true_SC{i_SC}.true_SC_mlps, mission_true_time);
                
                % Update Power and Data
                mission_true_SC{i_SC}.true_SC_power = fun_update_instantaneous_power_consumed( ...
                    mission_true_SC{i_SC}.true_SC_power,mission_true_SC{i_SC}.true_SC_mlps);
                mission_true_SC{i_SC}.true_SC_data_handling = func_update_data_generated( ...
                    mission_true_SC{i_SC}.true_SC_data_handling,mission_true_SC{i_SC}.true_SC_mlps);
            end

            % X-ray spectrometer (XRS)
            if isfield(mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists, 'science_xrs') ...
                    && mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_xrs
                
                % check if SC pointing towards SB
                mission_true_SC{i_SC}.true_SC_xrs = func_check_attitude_for_comm( ...
                    mission_true_SC{i_SC}.true_SC_xrs,mission_true_SC{i_SC}.software_SC_control_attitude,mission_true_SC{i_SC}.software_SC_estimate_attitude);
                
                % perform measurement
                mission_true_SC{i_SC}.true_SC_xrs = func_get_xrs_measurement(mission_true_SC{i_SC}.true_SC_xrs, mission_true_time);

                % Update Power and Data
                mission_true_SC{i_SC}.true_SC_power = fun_update_instantaneous_power_consumed( ...
                    mission_true_SC{i_SC}.true_SC_power,mission_true_SC{i_SC}.true_SC_xrs);
                mission_true_SC{i_SC}.true_SC_data_handling = func_update_data_generated( ...
                    mission_true_SC{i_SC}.true_SC_data_handling,mission_true_SC{i_SC}.true_SC_xrs);
            end

        end
        
        
        %% Data and Comms (Layer 6)
        
        % Compute antenna pointing loss
        sc.true_SC_radio = func_attitude_pointing_loss( ...
            sc.true_SC_radio, mission_true_SC, i_SC,mission_true_solar_system);
        
        sc.true_SC_communication = func_link_budget( ...
            sc.true_SC_communication, mission_true_time, mission_true_SC, i_SC, mission_true_solar_system);
        
        % Update SC Data and Comms
        sc.software_SC_communication = update_communication_routine( ...
            sc.software_SC_communication, sc.true_SC_onboard_memory, sc.true_SC_communication);
        
        if any(sc.software_SC_executive.send_data_comm == 1) ...
                || (sc.software_SC_executive.send_data_to_Earth == 1)
            
            if (sum(sc.software_SC_executive.send_data_comm) > 1)
                % multiple comm happends only at first triggering from executive
                sc.software_SC_communication.time_interSC_comm = sc.software_SC_executive.time; % Update Time Inter SC comm
            else
                if sc.software_SC_communication.comm_interlocutor(find(sc.software_SC_executive.send_data_comm == 1)) > 0
                    % if it's not DTE
                    sc.software_SC_communication.time_interSC_comm = sc.software_SC_executive.time; % Update Time Inter SC comm
                end
            end
            if (sc.software_SC_executive.send_data_to_Earth == 1)
                sc.software_SC_communication.time_dte_comm = sc.software_SC_executive.time; % Update Time DTE
            end
            
            % Check Desired Attitude for comm
            sc.software_SC_communication = func_check_attitude_for_comm( ...
                sc.software_SC_communication, ...
                sc.software_SC_control_attitude, ...
                sc.software_SC_estimate_attitude);
            % Exchange data
            sc.software_SC_communication.func_exchange_data(mission_true_SC,i_SC,mission_true_time,sc.software_SC_executive);
            
        else
            sc.software_SC_communication.data_exchanged = zeros(1,sc.software_SC_communication.number_communication); % [kb]
            sc.software_SC_communication.desired_attitude_for_comm_achieved = zeros(1,sc.software_SC_communication.number_communication);
            sc.software_SC_communication.data_needs_to_be_exchanged = zeros(1,sc.software_SC_communication.number_communication);
            sc.software_SC_communication.flag_all_data_sent = zeros(1,sc.software_SC_communication.number_communication);
        end
        
        % Update memory and executive flag
        for i = 1:sc.software_SC_communication.number_communication
            %memory
            if sc.software_SC_communication.data_exchanged(i) > 0
                if sc.software_SC_communication.comm_direction(i) == -1
                    % data sent
                    sc.true_SC_onboard_memory = func_delete_from_memory( ...
                        sc.true_SC_onboard_memory,sc.software_SC_communication.data_exchanged(i)); % delete from memory
                    sc.true_SC_power.instantaneous_power_consumed = ...
                        sc.true_SC_power.instantaneous_power_consumed ...
                        + sc.true_SC_radio.antenna_data( ...
                        sc.software_SC_communication.commu_use_antenna_index(i)).TX_power; % update TX power with the antenna used for this comm
                else
                    % data received
                    sc.true_SC_onboard_memory = func_write_to_memory( ...
                        sc.true_SC_onboard_memory,sc.software_SC_communication.data_exchanged(i));    % write to memory
                end
            end
            if sc.software_SC_communication.data_needs_to_be_exchanged(i) == 0
                sc.software_SC_executive.send_data_comm(i) = 0;
            end
        end
        
        
        
        
        %% Physical-Sim Layer (Layer 7)
        
        % Update Memory
        sc.true_SC_onboard_memory = func_write_to_memory( ...
            sc.true_SC_onboard_memory,sc.true_SC_data_handling.instantaneous_data_generated);
        
        % Update Solar panel Power consumed and generated
        sc.true_SC_solar_panel = func_update_SP_instantaneous_power( ...
            sc.true_SC_solar_panel, sc.true_SC_adc, sc.true_SC_navigation, mission_true_solar_system);
        for j=1:sc.true_SC_solar_panel.num_solar_panels
            % consumed power
            sc.true_SC_power = fun_update_instantaneous_power_consumed( ...
                sc.true_SC_power, ...
                sc.true_SC_solar_panel.solar_panel_data(j));
            % generated power
            sc.true_SC_power = fun_update_power_generated( ...
                sc.true_SC_power,sc.true_SC_solar_panel.solar_panel_data(j));
        end
        
        % Update battery
        sc.true_SC_battery = func_update_battery( ...
            sc.true_SC_battery,sc.true_SC_power);
        
        % Update energy
        sc.true_SC_power = fun_update_instantaneous_energy( ...
            sc.true_SC_power, sc.true_SC_battery);
        
        
    end
    
    %% Update Storage
    mission_storage.func_update_storage_data_structures( ...
        mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC);
    
    
    %% Fix Warnings
    this_loop_data.w = warning('query','last');
    if  ~isempty(this_loop_data.w)
        this_loop_data.id = this_loop_data.w.identifier;
        warning('off',this_loop_data.id);
    end
    
    %% Check for SC-SB Crash
    
    if norm(mission_true_SC{1}.true_SC_navigation.position - mission_true_small_body.position_SB) <= mission_true_small_body.radius
        
        disp('SC Crashed into SB!')
        
        mission_storage.flag_must_store_data_for_this_time_step = 1;
        mission_storage.func_update_storage_data_structures( ...
            mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC);
        
        break
        
    end
    
end

if (init_data.flag_store_video == 1)
    close(myVideo);
    close all
end

toc

%% Save data
save([mission_init_data.output_folder 'mission_storage_',char(datetime('today')),'.mat'],'mission_storage')
save([mission_init_data.output_folder 'all_data.mat'])
