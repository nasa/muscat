%% DART Mission
% Initialization File for DART (Double Asteroid Redirection Test) mission simulation

% Clear workspace
clear 
close all  
clc

% Change workspace folder to this file location
mfile_name          = mfilename('fullpath');
[pathstr,name,ext]  = fileparts(mfile_name);
cd(pathstr);
clear mfile_name pathstr name ext

% Add required paths
addpath(genpath('../../MuSCAT_Supporting_Files'))
addpath(genpath('../.'))


%% Mission Definition

mission = [];
mission.name = 'DART';         % Name of the Mission
mission.num_SC = 1;            % Number of Spacecraft
mission.num_target = 1;        % Number of Target bodies
mission.frame = 'Absolute';    % Frame type: 'Absolute', 'Relative', or 'Combined'
mission.flag_stop_sim = 0;     % Boolean flag to stop simulation if needed

%% Time Configuration

init_data = [];
init_data.t_initial = 0;                                    % [sec] Initial time
init_data.t_final = 200000;                                 % [sec] Final time 
init_data.time_step = 5;                                    % [sec] Simulation time step
init_data.t_initial_date_string = '02-NOV-2018 00:00:00';   % Format = [DD-MMM-YYYY HH:MM:SS]
init_data.time_step_attitude = 0.1;                         % [sec] Time step for attitude dynamics
mission.true_time = True_Time(init_data);

%% Storage Configuration

init_data = [];
init_data.time_step_storage = 0;
init_data.time_step_storage_attitude = 0;
init_data.flag_visualize_SC_attitude_orbit_during_sim = 0;  % [Boolean] Show attitude during sim
init_data.flag_realtime_plotting = 1;      % [Boolean] Show mission data and attitude during sim
init_data.flag_save_plots = 1;             % [Boolean] 1: Save them (takes little time), 0: Doesnt save them
init_data.flag_save_video = 0;             % [Boolean] 1: Save them (takes more time), 0: Doesnt save them
mission.storage = Storage(init_data, mission);

% Set font size for plots
mission.storage.plot_parameters.standard_font_size = 15;

% Initialize time storage
func_initialize_time_store(mission.true_time, mission);

%% Star Catalog Configuration

mission.true_stars = True_Stars(mission);
mission.true_stars.maximum_magnitude = 10;  % Maximum star magnitude to include

%% Solar System Configuration

init_data = [];
init_data.SS_body_names = ["Sun", "Earth"];  % Solar system bodies to include
mission.true_solar_system = True_Solar_System(init_data, mission);

%% Target Body Configuration

for i_target = 1:1:mission.num_target
    init_data = [];
    init_data.target_name = 'Bennu';        % Target asteroid name
    mission.true_target{i_target} = True_Target_SPICE(init_data, mission);
end

%% Ground Station Configuration

init_data = [];
init_data.num_GS_radio_antenna = 1;         % Number of ground station antennas
mission.true_ground_station = True_Ground_Station(init_data, mission);

%% Ground Station Radio Antenna Configuration

for i_HW = 1:1:mission.true_ground_station.num_GS_radio_antenna
    init_data = [];
    init_data.antenna_type = 'High Gain';
    init_data.mode_true_GS_radio_antenna_selector = 'RX';

    % Link Margin Calculation Parameters
    init_data.antenna_gain = 90;            % [dB]
    init_data.noise_temperature = 100;      % [K]
    init_data.beamwidth = 0.1;              % [MHz]
    init_data.energy_bit_required = 4.2;    % [dB]
    init_data.line_loss = 0;                % [dB]
    init_data.coding_gain = 7.3;            % [dB]

    mission.true_GS_radio_antenna{i_HW} = True_GS_Radio_Antenna(init_data, mission, i_HW);
end

%% Spacecraft Initialization

for i_SC = 1:1:mission.num_SC
    mission.true_SC{i_SC} = [];
end

%% Spacecraft Body Configuration

i_SC = 1;  % First spacecraft

init_data = [];
init_data.i_SC = i_SC;

% Body shape model
init_data.shape_model{1} = [];
init_data.shape_model{1}.Vertices = [0 0 0;
    0.3 0 0;
    0.3 0 0.1;
    0 0 0.1;
    0 0.2 0;
    0.3 0.2 0;
    0.3 0.2 0.1;
    0 0.2 0.1]; % [m]
init_data.shape_model{1}.Faces = [1 2 3;
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

init_data.shape_model{1}.Face_reflectance_factor = 0.6*ones(size(init_data.shape_model{1}.Faces,1),1);
init_data.shape_model{1}.type = 'cuboid';
init_data.shape_model{1}.mass = 11; % [kg] Dry mass

% Additional mass components
init_data.mass.supplement{1}.mass = 0.5; % [kg]
init_data.mass.supplement{1}.location = [0.1 0 0]; % [m]
init_data.mass.supplement{1}.MI_over_m = zeros(3,3); % [m^2]

init_data.mass.supplement{2}.mass = 0.5; % [kg]
init_data.mass.supplement{2}.location = [0 0 0.1]; % [m]
init_data.mass.supplement{2}.MI_over_m = zeros(3,3); % [m^2]

init_data.mode_COM_selector = 'update';  % Compute Center of Mass dynamically
init_data.mode_MI_selector = 'update';   % Compute Moment of Inertia dynamically

% Initialize hardware configuration
run init_num_hardware_exists
init_data.num_hardware_exists = num_hardware_exists;
clear num_hardware_exists

% Define hardware complement
init_data.num_hardware_exists.num_onboard_clock = 1;
init_data.num_hardware_exists.num_camera = 1;
init_data.num_hardware_exists.num_solar_panel = 3;
init_data.num_hardware_exists.num_battery = 2;
init_data.num_hardware_exists.num_onboard_memory = 2;
init_data.num_hardware_exists.num_sun_sensor = 6;
init_data.num_hardware_exists.num_star_tracker = 3;
init_data.num_hardware_exists.num_imu = 1;
init_data.num_hardware_exists.num_micro_thruster = 12;
init_data.num_hardware_exists.num_chemical_thruster = 1;
init_data.num_hardware_exists.num_reaction_wheel = 3;
init_data.num_hardware_exists.num_communication_link = 2;
init_data.num_hardware_exists.num_radio_antenna = 1;
init_data.num_hardware_exists.num_fuel_tank = 1;
init_data.num_hardware_exists.num_onboard_computer = 2;

mission.true_SC{i_SC}.true_SC_body = True_SC_Body(init_data, mission);

%% Initialize First Spacecraft's Position and Velocity

init_data = [];
init_data.spice_filename = '../../MuSCAT_Supporting_Files/SC_data/DART/traj_daresim_simple.bsp'; % [string] : SC s SPICE FileName
cspice_furnsh(init_data.spice_filename)

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

init_data.spice_name = '-110'; % [string] : SC's SPICE Name

% Sun centered - J2000 frame (inertial)
init_data.SC_pos_vel = cspice_spkezr(init_data.spice_name,mission.true_time.date,'J2000','NONE','SUN');
init_data.position = init_data.SC_pos_vel(1:3)'; % [km]
init_data.velocity = init_data.SC_pos_vel(4:6)'; % [km/sec]

init_data.mode_true_SC_navigation_dynamics_selector = 'Absolute Dynamics';

mission.true_SC{i_SC}.true_SC_navigation = True_SC_Navigation(init_data, mission);


%% Initialize First Spacecraft's Attitude

init_data = [];
init_data.SC_MRP_init = [0.1 0.2 0.3]; % MRP
init_data.SC_omega_init = [0 0 0.001]; % [rad/sec]

init_data.SC_e_init = init_data.SC_MRP_init/norm(init_data.SC_MRP_init);
init_data.SC_Phi_init = 4*atand(init_data.SC_MRP_init(1)/init_data.SC_e_init(1)); % [deg]
init_data.SC_beta_v_init = init_data.SC_e_init * sind(init_data.SC_Phi_init/2);
init_data.SC_beta_4_init = cosd(init_data.SC_Phi_init/2);

init_data.attitude = [init_data.SC_beta_v_init, init_data.SC_beta_4_init]; % [quaternion]
init_data.attitude = func_quaternion_properize(init_data.attitude); % [quaternion] properized
init_data.angular_velocity = init_data.SC_omega_init;

init_data.mode_true_SC_attitude_dynamics_selector = 'Rigid';

mission.true_SC{i_SC}.true_SC_adc = True_SC_ADC(init_data, mission);

%% Initialize First Spacecraft's Power

init_data = [];
init_data.power_loss_rate = 0.05; % [float] 5% power loss in distribution and conversion
mission.true_SC{i_SC}.true_SC_power = True_SC_Power(init_data, mission);

%% Initialize First Spacecraft's Data

init_data = [];
init_data.mode_true_SC_data_handling_selector = 'Generic';
mission.true_SC{i_SC}.true_SC_data_handling = True_SC_Data_Handling(init_data, mission);


%% Initialize First Spacecraft's Radio Antenna
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna

    init_data = [];
    init_data.location = [0 1 0];   % [unit vector] antenna physical axis in Body frame
    init_data.orientation = [0 0 1];   % [unit vector] antenna pointing direction for nominal gain in Body frame

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Antennas
    init_data.antenna_type = "dipole";      % antenna type
    init_data.antenna_gain = 28.1;          % [dB] Gain of DTE antenna SC
    init_data.antenna_axis = [0 1 0];       % [unit vector] antenna physical axis in Body frame
    init_data.antenna_pointing = [0 0 1];   % [unit vector] antenna pointing direction for nominal gain in Body frame
    init_data.antenna_frequency = 8450;     % [MHz]
    init_data.tx_line_loss = 1;             % [dB]
    init_data.noise_temperature = 100;   % [K]
    init_data.energy_bit_required = 4.2; % [dB]
    init_data.coding_gain = 7.3;         % [dB]
    init_data.beamwidth = 0.1;           % [MHz]

    init_data.maximum_data_rate = 1000;  % [kbps]
    init_data.base_data_rate_generated = 10; % [kbps]

    init_data.TX_power_consumed = 50;  % [W] Based on 50 W RF output and 50% efficiency
    init_data.RX_power_consumed = 25;   % [W] Typical for spacecraft receivers

    mission.true_SC{i_SC}.true_SC_radio_antenna{i_HW} = True_SC_Radio_Antenna(init_data, mission, i_SC, i_HW);
end


%% Spacecraft Fuel Tank Configuration
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
    init_data = [];
    
    % Basic properties
    init_data.name = ['Fuel Tank ', num2str(i_HW)];
    init_data.instantaneous_power_consumed = 2.0;    % [W] for heaters, valves, etc.
    init_data.instantaneous_data_rate_generated = 0.1;  % [kbps] for telemetry
    
    % Fuel properties
    init_data.maximum_capacity = 5.0;    % [kg] total fuel capacity
    init_data.initial_fuel_mass = 5.0;   % [kg] initial fuel mass (full tank)
    init_data.fuel_density = 1000;       % [kg/m^3] typical hydrazine density
    
    % Physical properties
    init_data.location = [0.15, 0.1, 0.05];  % [m] tank location in body frame
    
    % Shape model - simplified cuboid
    init_data.shape_model = [];
    init_data.shape_model.Vertices = [
        0.1, 0.05, 0.0;
        0.2, 0.05, 0.0;
        0.2, 0.15, 0.0;
        0.1, 0.15, 0.0;
        0.1, 0.05, 0.1;
        0.2, 0.05, 0.1;
        0.2, 0.15, 0.1;
        0.1, 0.15, 0.1
    ];
    init_data.shape_model.Faces = [
        1, 2, 3;
        1, 3, 4;
        5, 6, 7;
        5, 7, 8;
        1, 2, 6;
        1, 6, 5;
        2, 3, 7;
        2, 7, 6;
        3, 4, 8;
        3, 8, 7;
        4, 1, 5;
        4, 5, 8
    ];
    init_data.shape_model.type = 'cuboid';
    
    % Create fuel tank object
    mission.true_SC{i_SC}.true_SC_fuel_tank{i_HW} = True_SC_Fuel_Tank(init_data, mission, i_SC, i_HW);
    
    % Update spacecraft body with fuel mass properties
    mission.true_SC{i_SC}.true_SC_body.mass.propellant{i_HW}.mass = init_data.initial_fuel_mass;
    mission.true_SC{i_SC}.true_SC_body.mass.propellant{i_HW}.location = init_data.location;
    mission.true_SC{i_SC}.true_SC_body.mass.propellant{i_HW}.MI_over_m = zeros(3,3);  % Simple approximation
    
    % Trigger mass properties update
    mission.true_SC{i_SC}.true_SC_body.flag_update_SC_body_total_mass_COM_MI = 1;
end

%% Initialize First Spacecraft's Solar Panels

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel

    init_data = [];
    init_data.instantaneous_power_consumed = 0.01; % [W] (irrespective of whether it is generating power or not)
    init_data.instantaneous_data_rate_generated = (1e-3)*8; % [kbps] i.e. 1 Byte per sec

    init_data.shape_model = [];
    init_data.shape_model.Vertices = [0 0 0; 0.2 0 0; 0.2 0 -0.6; 0 0 -0.6];             % [m] vertices
    init_data.shape_model.Faces = [1 2 3; 1 4 3];                                        % faces
    init_data.shape_model.Face_reflectance_factor_solar_cell_side = [0.01; 0.01];        % reflectance factor of solar cell side
    init_data.shape_model.Face_reflectance_factor_opposite_side = [0.5; 0.5];            % reflectance factor of solar cell side
    init_data.shape_model.Face_orientation_solar_cell_side = [0 -1 0];                   % orientation normal vector of solar cell side
    init_data.shape_model.type = 'cuboid';


    init_data.mass = 0.24; % [kg] ~ 2 kg/m^2

    init_data.type = 'passive_deployed';
    % 'body_mounted' : Stuck to SC side (only solar cell side is used for SRP)
    % 'passive_deployed' : Passively deployed (orientation in body frame B does not change, i.e. it is static)
    % 'active_deployed_gimballed' : Actively gimballed (orientation in body frame B changes)

    init_data.packing_fraction = 0.74; % Packing fraction of solar cells in solar panel
    init_data.solar_cell_efficiency = 0.28; % Efficiency of each solar cell

    if i_HW == 2
        init_data.shape_model.Vertices = [0 0 0.1; 0.2 0 0.1; 0.2 0 0.7; 0 0 0.7];       % [m] vertices
    end

    if i_HW == 3
        init_data.shape_model.Vertices = mission.true_SC{i_SC}.true_SC_body.shape_model{1}.Vertices;
        init_data.shape_model.Faces = mission.true_SC{i_SC}.true_SC_body.shape_model{1}.Faces(1:2,:);
        init_data.type = 'body_mounted';
    end

    mission.true_SC{i_SC}.true_SC_solar_panel{i_HW} = True_SC_Solar_Panel(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Battery

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

    init_data = [];
    init_data.maximum_capacity = 40; % [W hr]
    init_data.charging_efficiency = 0.96; % [float <= 1]
    init_data.discharging_efficiency = 0.96; % [float <= 1]
    init_data.instantaneous_power_consumed = 1e-4; % [W]
    init_data.instantaneous_data_rate_generated = (1e-3)*8; % [kbps] i.e. 1 Byte per sec

    mission.true_SC{i_SC}.true_SC_battery{i_HW} = True_SC_Battery(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Onboard Memory

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory

    init_data = [];
    init_data.maximum_capacity = 1e6; % [kb]
    init_data.instantaneous_power_consumed = 1; % [W]
    init_data.instantaneous_data_rate_generated = (1e-3)*8; % [kbps] i.e. 1 Byte per sec

    mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW} = True_SC_Onboard_Memory(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Onboard Clock

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_clock

    init_data = [];
    init_data.instantaneous_power_consumed = 0.1; % [W]
    init_data.instantaneous_data_rate_generated = (1e-3)*16; % [kbps] i.e. 2 Bytes per sec
    init_data.mode_true_SC_onboard_clock_selector = 'Simple';
    init_data.measurement_wait_time = 0; % [sec]

    mission.true_SC{i_SC}.true_SC_onboard_clock{i_HW} = True_SC_Onboard_Clock(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Cameras

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera

    init_data = [];
    init_data.instantaneous_power_consumed = 10; % [W] https://dragonflyaerospace.com/products/caiman/
    init_data.mode_true_SC_camera_selector = 'Simple';
    init_data.measurement_wait_time = 60; % [sec] -> This was 10x60 but in the v1 is 60 only and that is how switch mode is implemented

    init_data.location = [0.3 0.1 0.05]; % [m] 
    init_data.orientation = [1 0 0]; % [unit vector]
    init_data.orientation_up = [0 0 1]; % [unit vector]

    init_data.resolution = [512 512];  % [x y] pixel
    init_data.field_of_view = 10; % [deg]
    init_data.flag_show_camera_plot = 0;
    init_data.flag_show_stars = 1;

    init_data.instantaneous_data_generated_per_pixel = (1e-3)* 8; % [kb]

    mission.true_SC{i_SC}.true_SC_camera{i_HW} = True_SC_Camera(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Sun Sensors

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor

    init_data = [];
    init_data.instantaneous_power_consumed = 36e-3; % [W] https://www.cubesatshop.com/wp-content/uploads/2016/06/SSOCA60-Technical-Specifications.pdf
    init_data.instantaneous_data_generated_per_sample = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
    init_data.mode_true_SC_sun_sensor_selector = 'Simple with Sun in FOV';
    init_data.measurement_wait_time = 0.1; % [sec]
    init_data.measurement_noise = deg2rad(0.5); % [rad] 0.5 degrees
    init_data.field_of_view = 60; % [deg]

    switch i_HW

        case 1
            init_data.location = [0.3 0.05 0.05]; % [m]
            init_data.orientation = [1 0 0]; % [unit vector]

        case 2
            init_data.location = [0 0.05 0.05]; % [m]
            init_data.orientation = [-1 0 0]; % [unit vector]

        case 3
            init_data.location = [0.15 0.2 0.05]; % [m]
            init_data.orientation = [0 1 0]; % [unit vector]

        case 4
            init_data.location = [0.15 0 0.05]; % [m]
            init_data.orientation = [0 -1 0]; % [unit vector]

        case 5
            init_data.location = [0.15 0.1 0.1]; % [m]
            init_data.orientation = [0 0 1]; % [unit vector]

        case 6
            init_data.location = [0.15 0.1 0]; % [m]
            init_data.orientation = [0 0 -1]; % [unit vector]

        otherwise
            error('Should not reach here!')
    end

    mission.true_SC{i_SC}.true_SC_sun_sensor{i_HW} = True_SC_Sun_Sensor(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Star Tracker

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker

    init_data = [];

    init_data.instantaneous_power_consumed = 1.5; % [W] https://www.bluecanyontech.com/static/datasheet/BCT_DataSheet_Components_StarTrackers.pdf
    init_data.instantaneous_data_generated_per_sample = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
    init_data.mode_true_SC_star_tracker_selector = 'Simple with Sun outside FOV';
    init_data.measurement_wait_time = 0.1; % [sec]
    init_data.measurement_noise = 2e-4; % [rad]
    init_data.field_of_view = 90; % [deg]

    switch i_HW

        case 1
            init_data.location = [0.3 0.15 0.05]; % [m]
            init_data.orientation = [1 0 0]; % [unit vector]

        case 2
            init_data.location = [0 0.15 0.05]; % [m]
            init_data.orientation = [-1 0 0]; % [unit vector]

        case 3
            init_data.location = [0.10 0.2 0.05]; % [m]
            init_data.orientation = [0 1 0]; % [unit vector]

        otherwise
            error('Should not reach here!')
    end

    mission.true_SC{i_SC}.true_SC_star_tracker{i_HW} = True_SC_Star_Tracker(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's IMU

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_imu

    init_data = [];

    init_data.instantaneous_power_consumed = 0.6; % [W] https://www.micro-a.net/imu-tmpl.html
    init_data.instantaneous_data_generated_per_sample = (3 + 1)*16e-3; % [kb] : 3 angular velocity + 1 time vector, each of 16-bit depth
    init_data.mode_true_SC_imu_selector = 'Simple';
    init_data.measurement_wait_time = 0.1; % [sec]
    init_data.measurement_noise = 9.7e-5; % [rad/sec]

    init_data.location = [0 0 0]; % [m]
    init_data.orientation = [1 0 0]; % [unit vector]

    mission.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data, mission, i_SC, i_HW);

end

%% Initialize First Spacecraft's Micro Thrusters

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster

    init_data = [];

    init_data.instantaneous_power_consumption = 10; % Watts
    init_data.instantaneous_data_generated_per_sample = (3)*16e-3; % [kb] : thrust + health + temperature, each of 16-bit depth
    init_data.mode_true_SC_micro_thruster_selector = 'Simple'; % Mode (Truth/Simple)
    init_data.thruster_noise = 100*(1e-6); % Noise level [N](unit depends on implementation)

    init_data.micro_thruster_ISP = 700; % [sec]

    init_data.minimum_thrust = 0.001; % [N]
    init_data.maximum_thrust = 10*(1e-2); % [N]
    
    init_data.command_wait_time = 0.5; % Seconds between commands
    
    switch i_HW
        case 1
            init_data.location = [0.3 0.1 0.05]; % [m]
            init_data.orientation = [0 1 0]; % [unit vector]
    
        case 2
            init_data.location = [0.3 0.1 0.05]; % [m]
            init_data.orientation = [0 -1 0]; % [unit vector]
    
        case 3
            init_data.location = [0.3 0.1 0.05]; % [m]
            init_data.orientation = [0 0 1]; % [unit vector]
    
        case 4
            init_data.location = [0.3 0.1 0.05]; % [m]
            init_data.orientation = [0 0 -1]; % [unit vector]
    
        case 5
            init_data.location = [0 0.1 0.05]; % [m]
            init_data.orientation = [0 1 0]; % [unit vector]
    
        case 6
            init_data.location = [0 0.1 0.05]; % [m]
            init_data.orientation = [0 -1 0]; % [unit vector]
    
        case 7
            init_data.location = [0 0.1 0.05]; % [m]
            init_data.orientation = [0 0 1]; % [unit vector]
    
        case 8
            init_data.location = [0 0.1 0.05]; % [m]
            init_data.orientation = [0 0 -1]; % [unit vector]
    
        case 9
            init_data.location = [0.15 0.2 0.05]; % [m]
            init_data.orientation = [0 0 1]; % [unit vector]
    
        case 10
            init_data.location = [0.15 0.2 0.05]; % [m]
            init_data.orientation = [0 0 -1]; % [unit vector]
    
        case 11
            init_data.location = [0.15 0 0.05]; % [m]
            init_data.orientation = [0 0 1]; % [unit vector]
    
        case 12
            init_data.location = [0.15 0 0.05]; % [m]
            init_data.orientation = [0 0 -1]; % [unit vector]
    
        otherwise
            error('Should not reach here!')
    end

    mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW} = True_SC_Micro_Thruster(init_data, mission, i_SC, i_HW);

end



%% Initialize the Reaction Wheels 
% Dart can be simulated using 3 or 4 wheels as an example !
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel

    init_data = [];
    init_data.location = [0,0,0];
    init_data.radius = (43e-3)/2; % [m] radius of 1 RW
    init_data.mass = 0.137; % [kg] mass of 1 RW
    init_data.max_angular_velocity = 6500*2*pi/60; % [rad/s] 6500 RPM
    init_data.angular_velocity_noise = 0.001*2*pi/60;      % [rad/s] velocity noise (reduced from 0.01)
    init_data.instantaneous_data_generated_per_sample = (3)*16e-3; % [kb] : thrust + health + temperature, each of 16-bit depth
    init_data.max_torque = 3.2*1e-3; % Nm 
    
    % Calculate and set maximum acceleration
    % This is redundant with the calculation in the True_SC_Reaction_Wheel constructor,
    % but makes it explicit and easier to adjust
    moment_of_inertia = 0.5 * init_data.mass * init_data.radius^2;
    init_data.maximum_acceleration = init_data.max_torque / moment_of_inertia; % rad/s^2
        
    init_data.power_consumed_angular_velocity_array = [1e-3*[180 600 6000]; [0 1000*2*pi/60 init_data.max_angular_velocity]]; % [power_array ; velocity_array]

    % 3 wheel configuration
    if(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel == 3)
        switch i_HW
            case 1
                init_data.orientation = [1, 0, 0];  % X-axis
            case 2
                init_data.orientation = [0, 1, 0];  % Y-axis
            case 3
                init_data.orientation = [0, 0, 1];  % Z-axis
        end
    end

    % 4 wheel configuration
    if(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel == 4)
        switch i_HW
            case 1
                init_data.orientation = [1, 1, 0]/sqrt(2);  % Diagonal in XY-plane
            case 2
                init_data.orientation = [1, -1, 0]/sqrt(2); % Diagonal in XY-plane
            case 3
                init_data.orientation = [0, 1, 1]/sqrt(2);  % Diagonal in YZ-plane
            case 4
                init_data.orientation = [0, 1, -1]/sqrt(2); % Diagonal in YZ-plane
        end
    end

    mission.true_SC{i_SC}.true_SC_reaction_wheel{i_HW} = True_SC_Reaction_Wheel(init_data, mission, i_SC, i_HW);

end


%% Chemical Thruster Configuration
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster

    init_data = [];

    % Power and data parameters
    init_data.instantaneous_power_consumption = 2.0;        % [W] Base power draw (standby)
    init_data.thruster_warm_up_power_consumed = 5.0;        % [W] Power during warm-up phase
    init_data.command_actuation_power_consumed = 15.0;      % [W] Power during active thrust
    init_data.instantaneous_data_generated_per_sample = 10; % [kb] per sample
    init_data.chemical_thruster_noise = 10e-4;              % [N] Thrust noise level

    % Thruster properties
    init_data.chemical_thruster_ISP = 200;                 % [s] Specific impulse
    init_data.command_wait_time = 1;                       % [s] Minimum time between commands
    init_data.location = [0.3, 0.2/2, 0.1/2];                % [m] Thruster location in body frame
    init_data.orientation = [-1, 0, 0];                     % Thrust direction (unit vector)

    init_data.maximum_thrust = 1;                          % [N] Maximum thrust level
    init_data.minimum_thrust = 0.01;                       % [N] Minimum thrust level

    % Create chemical thruster object
    mission.true_SC{i_SC}.true_SC_chemical_thruster{i_HW} = True_SC_Chemical_Thruster(init_data, mission, i_SC, i_HW);
end

%% Onboard Computer Configuration
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_computer
    init_data = [];
    
    % Basic properties
    init_data.name = ['Onboard Computer ', num2str(i_HW)];
    
    % Set different properties for primary and backup computers
    if i_HW == 1
        % Primary computer
        init_data.instantaneous_power_consumed = 8.0;     % [W] Main flight computer
        init_data.instantaneous_data_rate_generated = 2.0;  % [kbps] for telemetry and logs
        init_data.processor_utilization = 25;             % [%] Fixed CPU usage
    else
        % Backup computer
        init_data.instantaneous_power_consumed = 4.0;     % [W] Backup in standby mode
        init_data.instantaneous_data_rate_generated = 0.5;  % [kbps] minimal telemetry
        init_data.processor_utilization = 5;              % [%] Fixed low utilization in standby
    end
    
    % Create onboard computer object
    mission.true_SC{i_SC}.true_SC_onboard_computer{i_HW} = True_SC_Onboard_Computer(init_data, mission, i_SC, i_HW);
end

%% Spacecraft Communication Links Configuration

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_communication_link
    init_data = [];

    % Configure appropriate link based on index
    if i_HW == 1
        % Downlink: Spacecraft to Earth
        init_data.TX_spacecraft = i_SC;            % Transmitter is this spacecraft
        init_data.TX_spacecraft_Radio_HW = 1;      % Using antenna 1
 
        init_data.RX_spacecraft = 0;               % Receiver is Ground Station (0)
        init_data.RX_spacecraft_Radio_HW = 1;      % Using GS antenna 1

        init_data.flag_compute_data_rate = 0;      % Use given data rate instead of computing
        init_data.given_data_rate = 360;            % [kbps] Downlink data rate
    else
        % Uplink: Earth to Spacecraft
        init_data.TX_spacecraft = 0;               % Transmitter is Ground Station (0)
        init_data.TX_spacecraft_Radio_HW = 1;      % Using GS antenna 1

        init_data.RX_spacecraft = i_SC;            % Receiver is this spacecraft
        init_data.RX_spacecraft_Radio_HW = 1;      % Using antenna 1

        init_data.flag_compute_data_rate = 0;      % Use given data rate instead of computing
        init_data.given_data_rate = 0;             % [kbps] Set low to avoid overfilling memory
    end

    % Create communication link object
    mission.true_SC{i_SC}.true_SC_communication_link{i_HW} = True_SC_Communication_Link(init_data, mission, i_SC, i_HW);
end


%% Initialize Solar Radiation Pressure
init_data = [];
init_data.enable_SRP = 1; % Enable SRP calculations

mission.true_SC{i_SC}.true_SC_SRP = True_SC_SRP(init_data, mission, i_SC);


%% Initialize Gravity Gradient for Earth
init_data = [];
init_data.enable_G2 = 0; % Disable gravity gradient because Dart is an interceptor

mission.true_SC{i_SC}.true_SC_gravity_gradient = True_SC_Gravity_Gradient(init_data, mission, i_SC);


%% Spacecraft Software: Executive Configuration

init_data = [];
init_data.sc_modes = {'Point camera to Target', 'Maximize SP Power', 'Point Thruster along DeltaV direction', 'DTE Comm'};
init_data.mode_software_SC_executive_selector = 'DART';

mission.true_SC{i_SC}.software_SC_executive = Software_SC_Executive(init_data, mission, i_SC);

%% Spacecraft Software: Attitude Estimation Configuration

init_data = [];
init_data.mode_software_SC_estimate_attitude_selector = 'Truth';  % Use true attitude values

mission.true_SC{i_SC}.software_SC_estimate_attitude = Software_SC_Estimate_Attitude(init_data, mission, i_SC);

%% Spacecraft Software: Orbit Estimation Configuration

init_data = [];
init_data.mode_software_SC_estimate_orbit_selector = 'TruthWithErrorGrowth';  % Use true orbit values with error growth when target not visible

mission.true_SC{i_SC}.software_SC_estimate_orbit = Software_SC_Estimate_Orbit(init_data, mission, i_SC);

%% Spacecraft Software: Orbit Control Configuration

init_data = [];
init_data.max_time_before_control = 0.5*60*60 + 900;  % 45 minutes 
init_data.mode_software_SC_control_orbit_selector = 'DART';

mission.true_SC{i_SC}.software_SC_control_orbit = Software_SC_Control_Orbit(init_data, mission, i_SC);

%% Spacecraft Software: Attitude Control Configuration

init_data = [];
init_data.mode_software_SC_control_attitude_selector = 'DART Control Asymptotically Stable send to actuators'; 
init_data.control_gain = [1 0.2];    % Controller gain parameters

mission.true_SC{i_SC}.software_SC_control_attitude = Software_SC_Control_Attitude(init_data, mission, i_SC);

%% Spacecraft Software: Communication Configuration

init_data = [];
init_data.mode_software_SC_communication_selector = 'DART';
init_data.instantaneous_data_generated_per_sample = (1e-3)*8*2;  % [kb] 2 Bytes per sample
init_data.attitude_error_threshold_deg = 1;  % [deg] Max attitude error for communication

init_data.data = [];
init_data.data.last_communication_time = 0;
init_data.data.wait_time_comm_dte = 0.7*60*60;  % [sec] 42 minutes between DTE comms

mission.true_SC{i_SC}.software_SC_communication = Software_SC_Communication(init_data, mission, i_SC);

%% Spacecraft Software: Power Management Configuration

init_data = [];
init_data.mode_software_SC_power_selector = 'Generic';
init_data.instantaneous_data_generated_per_sample = (1e-3)*8*2;  % [kb] 2 Bytes per sample

mission.true_SC{i_SC}.software_SC_power = Software_SC_Power(init_data, mission, i_SC);

%% Spacecraft Software: Data Handling Configuration

init_data = [];
init_data.mode_software_SC_data_handling_selector = 'Generic';
init_data.instantaneous_data_generated_per_sample = (1e-3)*8*2;  % [kb] 2 Bytes per sample

mission.true_SC{i_SC}.software_SC_data_handling = Software_SC_Data_Handling(init_data, mission, i_SC);

%% Final Things to Do Before Running the Simulation

% Initialize mass, COM, MI
func_update_SC_body_total_mass_COM_MI(mission.true_SC{i_SC}.true_SC_body);

% Initialize store of Power
func_initialize_store_HW_power_consumed_generated(mission.true_SC{i_SC}.true_SC_power, mission);

% Initialize store of Data Handling
func_initialize_store_HW_data_generated_removed(mission.true_SC{i_SC}.true_SC_data_handling, mission);

% Initialize onboard computers
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_computer
    mission.true_SC{i_SC}.true_SC_onboard_computer{i_HW}.flag_executive = 1; % Start active
end

%% Save All Data
clear init_data i_SC i_HW i_target

% Vizualise the SC in 3D + Dashboard
func_visualize_SC(mission.storage, mission, true);
save([mission.storage.output_folder, 'all_data.mat'], '-v7.3')

%% Execute Main File
run main_v3.m

%% Save All Data
close all
disp ('----------------------------------------')
disp('Simulation Over. Starting saving data to disk...');

save([mission.storage.output_folder, 'all_data.mat'], '-v7.3')

disp(['Finished writing to file "all_data.mat" in folder : ', mission.storage.output_folder])
disp ('----------------------------------------')


%% Plots
% Use our memory-optimized visualization
fprintf('Starting memory-optimized visualization...');
memoryInfo = evalc('dispmemory()');
disp(['Current memory before visualisation - ', memoryInfo(1:end-1), ])

func_visualize_simulation_data(mission.storage, mission);

memoryInfo = evalc('dispmemory()');
fprintf('Visualization complete.');
disp(['Current memory after visualisation - ', memoryInfo(1:end-1), ])
disp ('----------------------------------------')


