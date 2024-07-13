% General Mission Initialization File

close all, clear all, clc

addpath(genpath('../MuSCAT_Supporting_Files'))
addpath(genpath('.'))

%% Define Mission

mission_init_data = [];
mission_init_data.name = 'DART';
mission_init_data.num_SC = 1; % Number of Spacecraft

% Output Folder
mission_init_data.output_folder = ['Output/' char(datetime("now", "Format", "yyyy-MM-dd-HH'h'mm'm'")) '/'];
mkdir(mission_init_data.output_folder)

mission_init_data.small_body_type = 1; % Select Small Body Type
% 1. Bennu
% 2. Apophis
% 3. Toutatis
% 4. Itokawa
% 5. 1996HW1

mission_init_data.mission_type = 2; % Select Mission Type
% 1. Approach SB and Orbit SB
% 2. Approach and Crash into SB (DART mission)
% 3. Orbit SB

mission_init_data.autonomy_type = 1; % Select if this is a Mission Concept simulation, or if the Mission uses Autonomy, or is carried out from the ground
% 0. Only Mission Concept Simulation (runs faster)
% 1. Mission uses Autonomy
% 2. Mission carried out from the ground, major decisions need DCO time
% • DCO_time_horizon [sec] : Data Cut Off (DCO) Time Horizon
mission_init_data.DCO_time_horizon = 0; % [sec]

% Estimation (truth, KF)
mission_init_data.state_estimation_type = 'truth';
mission_init_data.attitude_estimation_type = 'truth';

%% Initialize Environmental Variables

% True_Time
time_init_data = [];
time_init_data.t_initial = 0; % [sec]
time_init_data.t_final = 190000; % [sec]
time_init_data.time_step = 0.1; % [sec]
time_init_data.t_initial_date_string = '02-NOV-2018 00:00:00'; % [string] Format = [DD-MMM(words)-YYYY HH-MM-SS]
mission_true_time = True_Time(time_init_data)

% True_Stars
mission_true_stars = True_Stars(mission_true_time)
mission_true_stars.maximum_magnitude = 10;

% True_Solar_System
mission_true_solar_system = True_Solar_System(mission_true_time)

% True_Small_Body
mission_true_small_body = True_Small_Body(mission_init_data,mission_true_time)

clear time_init_data

%% Initialize Each Spacecraft
mission_true_SC = cell(mission_init_data.num_SC, 1);
for i_SC = 1:1:mission_init_data.num_SC
    mission_true_SC{i_SC} = True_Spacecraft(i_SC);
end

for i_SC = 1:1:mission_init_data.num_SC
    
    sc_body_init_data = [];
    
    switch i_SC
        
        case 1
            
            % mass
            sc_body_init_data.mass = [];
            sc_body_init_data.mass.dry_mass = 11; % [kg]
            sc_body_init_data.mass.propellant_mass = 0; % [kg]
            sc_body_init_data.mass.supplement_mass = 0; % [kg]
            sc_body_init_data.mass.location_propellant_mass = [0 0 0]'; % [m]
            sc_body_init_data.mass.location_supplement_mass = [0 0 0]'; % [m]
            
            % Body shape
            sc_body_init_data.shape_model = [];
            sc_body_init_data.shape_model.Vertices = [0 0 0;
                0.3 0 0;
                0.3 0 0.1;
                0 0 0.1;
                0 0.2 0;
                0.3 0.2 0;
                0.3 0.2 0.1;
                0 0.2 0.1]; % [m]
            sc_body_init_data.shape_model.Faces = [1 2 3;
                1 4 3;
                2 3 7;
                2 6 7;
                3 4 8;
                3 7 8;
                1 4 8;
                1 5 8;
                1 2 6;
                1 5 6;
                5 6 7;
                5 8 7];
            
            sc_body_init_data.flag_given_inertia_matrix = 0;     % [boolean] : flag to use given inertia matrix instead of internally computed one
            sc_body_init_data.given_inertia_matrix = [];          % Inertia of body shape in body center of mass
            
            sc_body_init_data.shape_model.reflectance_factor = 0.6*ones(size(sc_body_init_data.shape_model.Faces,1),1);
            %             sc_body_init_data.shape_model.reflectance
            %% Solar panels
            sc_body_init_data.number_solar_panel = 2;
            sc_body_init_data.solar_panel(1).shape_model.Vertices = [0 0 0; 0.2 0 0; 0.2 0 -0.6; 0 0 -0.6];             % [m] vertices
            sc_body_init_data.solar_panel(1).shape_model.Faces = [1 2 3; 1 4 3];                                        % faces
            sc_body_init_data.solar_panel(1).shape_model.reflectance_factor_solar_cell_side = [0.01;0.01];              % reflectance factor of solar cell side
            sc_body_init_data.solar_panel(1).shape_model.reflectance_factor_opposite_side = [0.5;0.5];                  % reflectance factor of solar cell side
            sc_body_init_data.solar_panel(1).shape_model.orientation_solar_cell_side = [cosd(45) sind(45) 0];%[0 -1 0];           % orientation normal vector of solar cell side
            sc_body_init_data.solar_panel(1).solar_panel_instantaneous_power_consumption = 0.01; % [W] (irrespective of whether it is generating power or not)
            sc_body_init_data.solar_panel(1).solar_panel_mass = 0.24; % [kg] ~ 2 kg/m^2
            sc_body_init_data.solar_panel(1).type = 2;
            sc_body_init_data.solar_panel(1).packing_fraction = 0.74; % Packing fraction of solar cells in solar panel
            sc_body_init_data.solar_panel(1).solar_cell_efficiency = 0.28; % Efficiency of each solar cell
            
            sc_body_init_data.solar_panel(2) = sc_body_init_data.solar_panel(1);
            sc_body_init_data.solar_panel(2).shape_model.Vertices = [0 0 0.1;0.2 0 0.1;0.2 0 0.7;0 0 0.7];                % [m] vertices
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % HW exists
            sc_body_init_data.flag_hardware_exists = [];
            sc_body_init_data.flag_hardware_exists.adc_sun_sensor = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.adc_star_tracker = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.adc_imu = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.adc_micro_thruster = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.adc_reaction_wheel_assembly = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.navigation_camera = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.navigation_chemical_thruster = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.navigation_ep_thruster = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.power_solar_panel = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.power_battery = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.power_pdc = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.dte_communication = 1; % [Boolean]
            sc_body_init_data.flag_hardware_exists.intersat_communication = 0; % [Boolean]
            sc_body_init_data.flag_hardware_exists.science_altimeter = 0; % [Boolean]
            sc_body_init_data.flag_hardware_exists.science_radar = 0; % [Boolean]
            sc_body_init_data.flag_hardware_exists.gnss_receiver = 0; % [Boolean]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial Position and Velocity
            
            sc_body_init_data.ratio_orbit_radius_SB_radius = 9;
            
            cspice_furnsh('../MuSCAT_Supporting_Files/SC_data/traj_daresim_simple.bsp')
            
            % bandyopa@MT-319257 exe % ./brief ../../../SC_data/traj_daresim_simple.bsp
            %
            % BRIEF -- Version 4.0.0, September 8, 2010 -- Toolkit Version N0066
            %
            %
            % Summary for: ../../../SC_data/traj_daresim_simple.bsp
            %
            % Body: -110
            %       Start of Interval (ET)              End of Interval (ET)
            %       -----------------------------       -----------------------------
            %       2018 OCT 27 21:36:30.000            2018 NOV 03 21:36:00.000
            
            sc_body_init_data.spice_name = '-110'; % [string] : SC s SPICE Name
            
            % Sun centered - J2000 frame (inertial)
            SC_pos_vel_t0 = cspice_spkezr(sc_body_init_data.spice_name,mission_true_time.date,'J2000','NONE','SUN');
            sc_body_init_data.position = SC_pos_vel_t0(1:3); % [km]
            sc_body_init_data.velocity = SC_pos_vel_t0(4:6); % [km/sec]
            
            % SB centered - J2000 frame (inertial)
            SC_pos_vel_t0_SBcentered = cspice_spkezr(sc_body_init_data.spice_name,mission_true_time.date,'J2000','NONE',mission_true_small_body.spice_name);
            sc_body_init_data.position_relative_SB = SC_pos_vel_t0_SBcentered(1:3); % [km]
            sc_body_init_data.velocity_relative_SB = SC_pos_vel_t0_SBcentered(4:6); % [km/sec]
            
            sc_body_init_data.flag_use_precomputed_spice_trajectory = 0; % [Boolean] : 1 = Use SPICE trajectory, Else compute ODE dynamics
            sc_body_init_data.spice_filename = '../MuSCAT_Supporting_Files/SC_data/traj_daresim_simple.bsp'; % [string] : SC s SPICE FileName
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial Attitude
            sc_body_init_data.SC_MRP_init = [0.1 0.2 0.3]'; % MRP
            sc_body_init_data.SC_omega_init = [0 0 0]'; % [rad/sec]
            
            sc_body_init_data.SC_e_init = sc_body_init_data.SC_MRP_init/norm(sc_body_init_data.SC_MRP_init);
            sc_body_init_data.SC_Phi_init = 4*atand(sc_body_init_data.SC_MRP_init(1)/sc_body_init_data.SC_e_init(1)); % [deg]
            sc_body_init_data.SC_beta_v_init = sc_body_init_data.SC_e_init * sind(sc_body_init_data.SC_Phi_init/2);
            sc_body_init_data.SC_beta_4_init = cosd(sc_body_init_data.SC_Phi_init/2);
            
            sc_body_init_data.attitude = [sc_body_init_data.SC_beta_v_init; sc_body_init_data.SC_beta_4_init];
            % sc_body_init_data.attitude = [0.15    0   -0.2    0.6]';
            sc_body_init_data.attitude = sc_body_init_data.attitude/norm(sc_body_init_data.attitude);
            sc_body_init_data.angular_velocity = sc_body_init_data.SC_omega_init;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Attitude controller tuning
            sc_body_init_data.control_gain_MT = [1 0.2];    % [Kr ; Lambda_r] for MT control
            sc_body_init_data.control_gain_RWA = [0.1 1];   % [Kr ; Lambda_r] for RWA control
            sc_body_init_data.actuator_selection_threshold_torque = 0.05;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % External disturbance
            sc_body_init_data.enable_SRP    = 1;    % enable Solar Radiation Pressure disturbance torque and force of SC
            sc_body_init_data.enable_G2     = 1;    % enable Gravity Gradient disturbance torque
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Battery
            sc_body_init_data.num_battery = 2;
            sc_body_init_data.battery_maximum_capacity = [40 40]; % [Watts * hrs]
            sc_body_init_data.charging_efficiency = [0.96 0.96];
            sc_body_init_data.discharging_efficiency = [0.96 0.96];
            sc_body_init_data.battery_instantaneous_power_consumption = [1e-4 1e-4];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Onboard clock
            sc_body_init_data.num_onboard_clock = 1;
            sc_body_init_data.onboard_clock_orientation = [1 0 0];
            sc_body_init_data.onboard_clock_location = [0 0 0];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Onboard memory
            sc_body_init_data.num_memory = 1;
            sc_body_init_data.memory_instantaneous_power_consumption = [0.1]; % [W]
            sc_body_init_data.memory_maximum_data_storage = (1e3)*[1]; % [GB]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Antennas
            sc_body_init_data.number_antenna = 1;
            sc_body_init_data.antenna_type = ["dipole"];      % antenna type
            sc_body_init_data.antenna_gain = [28.1];          % [dB] Gain of DTE antenna SC
            sc_body_init_data.antenna_axis = [[0 1 0]];       % [unit vector] antenna physical axis in Body frame
            sc_body_init_data.antenna_pointing = [[0 0 1]];   % [unit vector] antenna pointing direction for nominal gain in Body frame
            sc_body_init_data.antenna_frequency = [8450];     % [MHz]
            sc_body_init_data.tx_power = [50];                % [W]
            sc_body_init_data.tx_line_loss = [1];             % [dB]
            sc_body_init_data.SC_noise_temperature = [100];   % [K]
            sc_body_init_data.SC_energy_bit_required = [4.2]; % [dB]
            sc_body_init_data.SC_coding_gain = [7.3];         % [dB]
            sc_body_init_data.SC_beamwidth = [0.1];           % [MHz]
            sc_body_init_data.SC_maximum_data_rate = [1000];  % [kbps]
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Communication (general)
            sc_body_init_data.communication_interlocutor = [-1];                % -1 : DTE / n>0 : index of spacecraft to communicate with
            sc_body_init_data.communication_direction = [-1];                   % -1 : send / 1 : receive / 0 : bidirectionnal
            sc_body_init_data.communication_needs_attitude_pointing = [1];      % [boolean] define if SC needs to orient itself before communication to maximize antenna gain
            sc_body_init_data.communication_use_antenna_index = [1];            % index of antenna to use for each communication
            sc_body_init_data.minimum_required_link_margin = 3;                         % [dB] required minimum link margin
            sc_body_init_data.compute_data_rate = 1;                                    % [boolean] 1= compute data rate / otherwise assume 'given_data_rate'
            sc_body_init_data.given_data_rate = 0;                                      % [kbps] data rate assumed if 'compute_data_rate' = 0
            
            
        otherwise
            error('Only 1 SC data provided!')
            
    end
    
    %% Sensors
    % Sun Sensor
    if sc_body_init_data.flag_hardware_exists.adc_sun_sensor == 1
        sc_body_init_data.num_sun_sensor = 1;
        sc_body_init_data.sun_sensor_location = [[0 0 0]];
        sc_body_init_data.sun_sensor_orientation = [[1 0 0]];
        sc_body_init_data.sun_sunsor_instantaneous_power_consumption = 36e-3; % [W] https://www.cubesatshop.com/wp-content/uploads/2016/06/SSOCA60-Technical-Specifications.pdf
        sc_body_init_data.sun_sunsor_instantaneous_data_volume = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        sc_body_init_data.sun_sunsor_measurement_noise = deg2rad(0.5); % [rad] 0.5 degrees
        sc_body_init_data.sun_sunsor_measurement_frequency = 10; % [Hz]
        mission_true_SC{i_SC}.true_SC_sun_sensor = True_SC_Sun_Sensor(mission_true_time,sc_body_init_data);
    end
    
    % Star Tracker
    if sc_body_init_data.flag_hardware_exists.adc_star_tracker == 1
        sc_body_init_data.num_star_tracker = 1;
        sc_body_init_data.star_tracker_location = [[0 0 0]];
        sc_body_init_data.star_tracker_orientation = [[1 0 0]];
        sc_body_init_data.star_tracker_instantaneous_power_consumption = 1.5; % [W] https://www.bluecanyontech.com/static/datasheet/BCT_DataSheet_Components_StarTrackers.pdf
        sc_body_init_data.star_tracker_instantaneous_data_volume = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        sc_body_init_data.star_tracker_measurement_noise = 2e-4; % [rad] 0.5 degrees
        sc_body_init_data.star_tracker_measurement_frequency = 10; % [Hz]sc_body_init_data.sun_sunsor_instantaneous_power_consumption = 36e-3; % [W] https://www.cubesatshop.com/wp-content/uploads/2016/06/SSOCA60-Technical-Specifications.pdf
        mission_true_SC{i_SC}.true_SC_star_tracker_sensor = True_SC_Star_Tracker_Sensor(mission_true_time,sc_body_init_data);
    end
    
    % IMU Sensor
    if sc_body_init_data.flag_hardware_exists.adc_imu == 1
        sc_body_init_data.num_imu_sensor = 1;
        sc_body_init_data.imu_sensor_location = [[0 0 0]];
        sc_body_init_data.imu_sensor_orientation = eye(3);
        sc_body_init_data.imu_sensor_instantaneous_power_consumption = 0.6; % [W] https://www.micro-a.net/imu-tmpl.html
        sc_body_init_data.imu_sensor_instantaneous_data_volume = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        sc_body_init_data.imu_sensor_measurement_noise = 9.7e-5; % [rad/sec]
        sc_body_init_data.imu_sensor_measurement_frequency = 10; % [Hz] between to measurement
        mission_true_SC{i_SC}.true_SC_imu_sensor = True_SC_IMU_Sensor(mission_true_time,sc_body_init_data);
    end
    
    % Camera
    if sc_body_init_data.flag_hardware_exists.navigation_camera == 1
        sc_body_init_data.num_camera = 1;
        sc_body_init_data.camera_instantaneous_power_consumption = [10];% [Watts] https://dragonflyaerospace.com/products/caiman/
        sc_body_init_data.camera_measurement_frequency = 1/10; % [Hz]
        sc_body_init_data.camera_angle_array = [0 0 0];
        sc_body_init_data.camera_location = [0 0 0];
        sc_body_init_data.camera_measurement_noise = [0];
        sc_body_init_data.camera_resolution = [1024 1024];  % [x y] pixel
        sc_body_init_data.camera_FOV = [10];                 % [deg]
        sc_body_init_data.camera_flag_show_stars = [1];     % [boolean] : show star. slow for WFOV
        sc_body_init_data.flag_show_camera_plot=[0];        % [boolean] : show plot during loop
        mission_true_SC{i_SC}.true_SC_camera_sensor = True_SC_Camera_Sensor(sc_body_init_data, mission_true_time);
    end
    
    
    %%%%%%% True component
    mission_true_SC{i_SC}.true_SC_body = True_SC_Body(mission_true_time,i_SC,sc_body_init_data);
    
    mission_true_SC{i_SC}.true_SC_navigation = True_SC_Navigation(mission_true_time,sc_body_init_data);
    
    mission_true_SC{i_SC}.true_SC_adc = True_SC_ADC(mission_true_time,sc_body_init_data,mission_true_SC{i_SC}.true_SC_body);
    
    mission_true_SC{i_SC}.true_SC_onboard_clock_sensor = True_SC_Onboard_Clock_Sensor(sc_body_init_data);
    
    mission_true_SC{i_SC}.true_SC_data_handling = True_SC_Data_handling(mission_true_time);
    
    mission_true_SC{i_SC}.true_SC_solar_panel = True_SC_Solar_Panel(sc_body_init_data);
    
    mission_true_SC{i_SC}.true_SC_power = True_SC_Power(mission_true_time);
    
    mission_true_SC{i_SC}.true_SC_gravity_gradient = True_SC_Gravity_Gradient(sc_body_init_data);
    
    mission_true_SC{i_SC}.true_SC_srp = True_SC_SRP(sc_body_init_data, mission_true_SC{i_SC}.true_SC_body);
    
    mission_true_SC{i_SC}.true_SC_battery = True_SC_Battery(sc_body_init_data,mission_true_time);
    
    mission_true_SC{i_SC}.true_SC_onboard_memory = True_SC_Onboard_Memory(mission_true_time,sc_body_init_data);
    
    mission_true_SC{i_SC}.true_SC_radio = True_SC_Radio(sc_body_init_data, mission_true_solar_system);
    
    mission_true_SC{i_SC}.true_SC_communication = True_SC_Communication(sc_body_init_data,mission_true_time);
    
    % DTE Communication
    if sc_body_init_data.flag_hardware_exists.dte_communication == 1
        
        sc_body_init_data.ground_station_antenna_gain = 90;         % [dB] gain of Earth receiver
        sc_body_init_data.ground_station_temperature_noise = 100;   % [K] temperature noise
        sc_body_init_data.ground_station_beamwidth = 0.1;           % [MHz] receiver beamwwidth
        sc_body_init_data.ground_station_Energy_bit_required = 4.2; % [dB] Minimum energy bit required
        sc_body_init_data.ground_station_loss_pointing = 0;         % [dB] Loss due to pointing
        sc_body_init_data.ground_station_coding_gain = 7.3;         % [dB] Coding gain
        mission_true_SC{i_SC}.true_earth_dte_communication = True_Earth_DTE_communication(sc_body_init_data);
    end
    
    % Science altimeter instrument
    if sc_body_init_data.flag_hardware_exists.science_altimeter == 1
        
        sc_body_init_data.altimeter_power_consumption = 2;      % [W]
        sc_body_init_data.altimeter_data_volume = 0.1;          % [kb]
        sc_body_init_data.altimeter_measurement_frequency = 1; % [Hz]
        sc_body_init_data.altimeter_orientation = [1 0 0]';     % [Unit vector]
        sc_body_init_data.altimeter_mesh_selection = 9;         % 1 = 500 points, 2 = 1000 points, 3 = 2500 points, 4 = 5000 points, 5 = 10000 points, 6 = 50000 points, 7 = 100000 points, 8 = 200000 points, 9 = 400000 points, 10 = 500000 points, 11 = 1000000 points, 12 = 50 points, 13 = 100 points, 14 = 200 points, 15 = 300 points, 16 = 400 points
        mission_true_SC{i_SC}.true_SC_altimeter = True_SC_Altimeter(sc_body_init_data,mission_true_time);
    end
    % Science altimeter instrument
    if sc_body_init_data.flag_hardware_exists.science_radar == 1
        
        sc_body_init_data.radar_power_consumption = 20;         % [W]
        sc_body_init_data.radar_data_volume = 20;               % [kb]
        sc_body_init_data.radar_measurement_frequency = 1;      % [Hz]
        sc_body_init_data.radar_orientation = [1 0 0]';         % [Unit vector]
        sc_body_init_data.radar_mesh_selection = 9;             % 1 = 500 points, 2 = 1000 points, 3 = 2500 points, 4 = 5000 points, 5 = 10000 points, 6 = 50000 points, 7 = 100000 points, 8 = 200000 points, 9 = 400000 points, 10 = 500000 points, 11 = 1000000 points, 12 = 50 points, 13 = 100 points, 14 = 200 points, 15 = 300 points, 16 = 400 points
        mission_true_SC{i_SC}.true_SC_radar = True_SC_Radar(sc_body_init_data,mission_true_time,mission_true_SC{i_SC}.true_SC_navigation);
    end
    
    %% Actuators
    % Micro Trhuster Actuator
    if sc_body_init_data.flag_hardware_exists.adc_micro_thruster == 1
        sc_body_init_data.num_micro_thruster = 12;
        sc_body_init_data.micro_thruster_position_array = [0.3 0.2/2 0.1/2;
            0.3 0.2/2 0.1/2;
            0.3 0.2/2 0.1/2;
            0.3 0.2/2 0.1/2;
            0 0.2/2 0.1/2;
            0 0.2/2 0.1/2;
            0 0.2/2 0.1/2;
            0 0.2/2 0.1/2;
            0.3/2 0.2 0.1/2;
            0.3/2 0.2 0.1/2;
            0.3/2 0 0.1/2;
            0.3/2 0 0.1/2];
        sc_body_init_data.micro_thruster_direction_array = [0 1 0;
            0 -1 0;
            0 0 1;
            0 0 -1;
            0 1 0;
            0 -1 0;
            0 0 1;
            0 0 -1;
            0 0 1;
            0 0 -1;
            0 0 1;
            0 0 -1];
        sc_body_init_data.micro_thruster_maximum_thrust = 10*(1e-3) * ones(12,1);    % [N]
        sc_body_init_data.micro_thruster_thruster_noise = 100*(1e-6) * ones(12,1);   % [N]
        sc_body_init_data.micro_thruster_ISP = 50 * ones(12,1);                      % [s]
        sc_body_init_data.micro_thruster_instantaneous_power_consumption = 2 * ones(12,1); % [W]
        sc_body_init_data.micro_thruster_instantaneous_data_volume = (1e-3)*16; % [kb]
        mission_true_SC{i_SC}.true_SC_micro_thruster_actuator = True_SC_Micro_Thruster_Actuator(sc_body_init_data, mission_true_time,mission_true_SC{i_SC}.true_SC_body);
    end
    
    % RWA actuator
    if sc_body_init_data.flag_hardware_exists.adc_reaction_wheel_assembly == 1
        sc_body_init_data.num_reaction_wheel = 3;
        sc_body_init_data.rw_position = repmat([0 0 0],3,1);
        % 3 RW / trihedral conf. : 1 on each axis
        sc_body_init_data.rw_angle_array = [
            0 0 0;
            pi/2 0 -pi;
            0 -pi/2 -pi/2];
        % https://nanoavionics.com/cubesat-components/cubesat-reaction-wheels-control-system-satbus-4rw/
        %         sc_body_init_data.rw_angle_array = [-pi/2 -pi/4 0;
        %                   pi/2 pi/4 0;
        %                   0 0 -pi/4;
        %                   -pi 0 pi/4];
        sc_body_init_data.rw_radius = (43e-3)/2;    % [m] radius of 1 RW
        sc_body_init_data.rw_mass = 0.137;          % [kg] mass of 1 RW
        sc_body_init_data.rw_maximum_angular_velocity = 6500*2*pi/60; % [rad/s] 6500 RPM
        sc_body_init_data.rw_angular_velocity_noise = 1*2*pi/60;      % [rad/s] velocity noise
        sc_body_init_data.rw_power_consumed_angular_velocity_array = [1e-3*[180 600 6000]; [0 1000*2*pi/60 sc_body_init_data.rw_maximum_angular_velocity]]; % [power_array ; velocity_array]
        sc_body_init_data.rw_instantaneous_data_volume = (3)*16e-3; % [kb] : velocity + health + temperature, each of 16-bit depth;
        mission_true_SC{i_SC}.true_SC_rwa_actuator = True_SC_RWA_Actuator(sc_body_init_data, mission_true_time,mission_true_SC{i_SC});
    end
    
    % Chemical Thruster
    if sc_body_init_data.flag_hardware_exists.navigation_chemical_thruster == 1
        sc_body_init_data.num_chemical_thruster = 1;
        sc_body_init_data.chemical_thruster_instantaneous_power_consumption = 10;       % [Watts] % https://satsearch.co/products/ecaps-1n-hpgp-thruster
        sc_body_init_data.chemical_thruster_instantaneous_data_volume = (1e-3)*16;      % [kilo bits]
        sc_body_init_data.chemical_thruster_warmup_time = 0*30*60;                      % [sec] Warmup for 30min before the DeltaV
        sc_body_init_data.chemical_thruster_warmup_power = 50;                          % [Watts] to heat the propellant
        sc_body_init_data.chemical_thruster_direction_array = [1 0 0];                  % [unit vector]
        sc_body_init_data.chemical_thruster_position_array = [0 0.2/2 0.1/2];           % [m]
        sc_body_init_data.chemical_thruster_maximum_thrust = 1;                         % [N]
        sc_body_init_data.chemical_thruster_minimum_thrust = 0.25;                      % [N]
        sc_body_init_data.chemical_thruster_noise = 70e-3;                              % [N]
        sc_body_init_data.chemical_thruster_ISP = 200;                                  % [s]
        mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator = True_SC_Chemical_Thruster_Actuator(mission_true_time,sc_body_init_data);
    end
    
    %%%%%%% Software component
    mission_true_SC{i_SC}.software_SC_communication = Software_SC_Communication(sc_body_init_data,mission_true_SC{i_SC},mission_true_time);
    
    mission_true_SC{i_SC}.software_SC_executive = Software_SC_Executive(mission_true_SC{i_SC}.true_SC_body,mission_true_time,mission_true_SC{i_SC}.software_SC_communication);
    
    mission_true_SC{i_SC}.software_SC_estimate_attitude = Software_SC_Estimate_Attitude(mission_true_SC{i_SC},mission_true_time);
    
    mission_true_SC{i_SC}.software_SC_estimate_SC_SB_orbit = Software_SC_Estimate_SC_SB_Orbit(mission_true_SC{i_SC}.true_SC_body,mission_true_SC{i_SC}.true_SC_navigation,mission_true_time,mission_true_small_body,mission_true_solar_system);
    
    mission_true_SC{i_SC}.software_SC_control_attitude = Software_SC_Control_Attitude(sc_body_init_data, mission_true_time);
    
    mission_true_SC{i_SC}.software_SC_control_orbit = Software_SC_Control_Orbit(mission_init_data, mission_true_time, sc_body_init_data, mission_true_SC{i_SC}.software_SC_executive, mission_true_small_body);
    
    clear sc_body_init_data
    
end

%% Visualize Spacecrafts
flag_visualize_SC = 1;
if flag_visualize_SC == 1
    figure()
    for i_SC=1:mission_init_data.num_SC
        subplot(1,mission_init_data.num_SC,i_SC)
        trisurf(mission_true_SC{i_SC}.true_SC_body.shape_model{1}.Faces, mission_true_SC{i_SC}.true_SC_body.shape_model{1}.Vertices(:,1),mission_true_SC{i_SC}.true_SC_body.shape_model{1}.Vertices(:,2),mission_true_SC{i_SC}.true_SC_body.shape_model{1}.Vertices(:,3),'FaceColor','white','DisplayName','Body')
        hold on;
        for sp=1:mission_true_SC{i_SC}.true_SC_solar_panel.num_solar_panels
            trisurf(mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Faces, mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Vertices(:,1),mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Vertices(:,2),mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Vertices(:,3),'FaceColor','blue',"DisplayName",['SP ',num2str(sp)])
        end
        hold on;
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
            for i=1:mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.num_micro_thruster
                quiver3(mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i).location(1),mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i).location(2),mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i).location(3) , mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i).orientation(1),mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i).orientation(2),mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i).orientation(3),"LineWidth",3,"DisplayName",['MT ',num2str(i)],"AutoScaleFactor",0.1)
            end
        end
        if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
            for i=1:mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.num_chemical_thruster
                quiver3(mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i).location(1),mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i).location(2),mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i).location(3) , -mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i).orientation(1),-mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i).orientation(2),-mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i).orientation(3),"LineWidth",10,"DisplayName",['CT ',num2str(i)],"AutoScaleFactor",0.15)
            end
        end
        axis equal
        legend
        xlabel("X_{SC}")
        ylabel("Y_{SC}")
        zlabel("Z_{SC}")
        title(["Spacecraft ",num2str(i_SC)])
    end
end
clear flag_visualize_SC

%% Setup Storage

mission_storage = Storage();
mission_storage.true_time.time_step = 1; % [sec] Use 0 to use the True_time.time_step value
mission_storage.plot_parameters.storage_save_plots = 1;
mission_storage.func_create_storage_data_structures(mission_true_time,mission_init_data,mission_true_SC);
mission_storage.func_update_storage_data_structures(mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC);

%% Execute Main File
save();

run main_v2.m

%% Save data
save([mission_init_data.output_folder 'mission_storage_',char(datetime('today')),'.mat'],'mission_storage')
save([mission_init_data.output_folder 'all_data.mat'])

%% Plot Storage
mission_storage.plot_parameters.plot_SC_number = 1;
func_plot_storage_data_structures(mission_storage,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC);

return
%% Plot video
mission_storage.plot_parameters.plot_type = 'DART'; % "DROID" / "coverage" / "camera" / "DART"
func_make_video(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC);

