%% NISAR Mission with 2 SC
% Initialization File for NISAR (NASA-ISRO Synthetic Aperture Radar) mission simulation

% Clear workspace
clear all
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
mission.name = 'NISAR 2SC';         % Name of the Mission
mission.num_SC = 2;            % Number of Spacecraft
mission.num_target = 1;        % Number of Target bodies
mission.frame = 'Relative';    % Frame type: 'Absolute', 'Relative', or 'Combined'
mission.flag_stop_sim = 0;     % Boolean flag to stop simulation if needed

mission.this_phase = 'Hurricane Helene';
% 1: 'Hurricane Helene' 0 UTC, 24 September 2024 to 0 UTC, 1 October 2024 (Hurricane Helene)
% 2: 'Texas heavy rain' 0 UTC, 3 July 2025 to 12 UTC, 5 July, 2025 (Texas heavy rain)

%% Time Configuration

init_data = [];
init_data.t_initial = 0;                                    % [sec] Initial time
init_data.time_step = 60;                                    % [sec] Simulation time step

switch mission.this_phase

    case 'Hurricane Helene'
        init_data.t_initial_date_string = '24-SEP-2024 00:00:00';   % Format = [DD-MMM-YYYY HH:MM:SS]
        init_data.t_final = 8*86400;                                 % [sec] Final time

    case 'Texas heavy rain'
        init_data.t_initial_date_string = '03-JUL-2025 00:00:00';   % Format = [DD-MMM-YYYY HH:MM:SS]
        init_data.t_final = 4*86400;                                 % [sec] Final time

    otherwise
        init_data.t_initial_date_string = '07-AUG-2025 00:00:00';   % Format = [DD-MMM-YYYY HH:MM:SS]
        init_data.t_final = 1*86400;                                 % [sec] Final time

end

% init_data.time_step_attitude = 5;                         % [sec] Time step for attitude dynamics
mission.true_time = True_Time(init_data);

%% Storage Configuration

init_data = [];
init_data.time_step_storage = 0; % [sec] Use 0 to use the mission.true_time.time_step value
init_data.time_step_storage_attitude = 0; % [sec] Use 0 to use the mission.true_time.time_step_attitude value
init_data.flag_realtime_plotting = 0;      % [Boolean] Show mission data and attitude during sim
init_data.wait_time_realtime_plotting = 60*60; % [sec] Wait time between plots (Optional)
init_data.flag_save_plots = 1; % [Boolean] 1: Save them (takes little time), 0: Doesnt save them (Optional)
init_data.flag_save_video = 0; % [Boolean]1 1: Save them (takes a lot more time), 0: Doesnt save them (Optional)
init_data.quiver_auto_scale_factor = 1;    % (Optional)
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
init_data.SS_body_names = ["Sun", "Moon"];  % Solar system bodies to include
mission.true_solar_system = True_Solar_System(init_data, mission);

%% Target Body Configuration

for i_target = 1:1:mission.num_target
    init_data = [];
    init_data.target_name = 'Earth';        % Target asteroid name
    mission.true_target{i_target} = True_Target_SPICE(init_data, mission);
end

% mission.true_target{1}.prime_meridian = 360;

%% Ground Station Configuration

init_data = [];
init_data.num_GS_radio_antenna = 17;         % Number of ground station antennas
mission.true_ground_station = True_Ground_Station(init_data, mission);

%% Ground Station Radio Antenna Configuration

for i_HW = 1:1:mission.true_ground_station.num_GS_radio_antenna
    init_data = [];
    init_data.antenna_type = 'High Gain';
    init_data.mode_true_GS_radio_antenna_selector = 'RX';

    switch i_HW

        case 1
            init_data.name = 'Bangalore (India)';
            init_data.latitude = 13.034; % [deg]
            init_data.longitude = 76.608333; % [deg]

        case 2
            init_data.name = 'Lucknow (India)';
            init_data.latitude = 26.913; % [deg]
            init_data.longitude = 80.957; % [deg]

        case 3
            init_data.name = 'Sriharikota (India)';
            init_data.latitude = 13.673; % [deg]
            init_data.longitude = 80.19599972; % [deg]

        case 4
            init_data.name = 'Alaska (Fairbanks, Alaska Satellite Facility, NEN)';
            init_data.latitude = 64.8587; % [deg]
            init_data.longitude = -147.8576; % [deg]

        case 5
            init_data.name = 'Alaska (SSC, North Pole, NEN)';
            init_data.latitude = 64.8042; % [deg]
            init_data.longitude = -147.5002; % [deg]

        case 6
            init_data.name = 'Florida (Launch Communications Stations, NEN)';
            init_data.latitude = 28.542064; % [deg]
            init_data.longitude = -80.642953; % [deg]

        case 7
            init_data.name = 'South Africa (Hartebeesthoek, SANSA, NEN)';
            init_data.latitude = -25.8870; % [deg]
            init_data.longitude = 27.7120; % [deg]

        case 8
            init_data.name = 'Hawaii (South Point, SSC, NEN)';
            init_data.latitude = 19.0140; % [deg]
            init_data.longitude = -155.6633; % [deg]

        case 9
            init_data.name = 'Australia (Dongara, SSC, NEN)';
            init_data.latitude = -29.0457; % [deg]
            init_data.longitude = 115.3487; % [deg]

        case 10
            init_data.name = 'Sweden (Kiruna, SSC, NEN)';
            init_data.latitude = 67.8896; % [deg]
            init_data.longitude = 21.0657; % [deg]

        case 11
            init_data.name = 'Antarctica (McMurdo, NASA, NEN)';
            init_data.latitude = -77.8391; % [deg]
            init_data.longitude = 166.6671; % [deg]

        case 12
            init_data.name = 'Antarctica (Troll, KSAT, NEN)';
            init_data.latitude = -72.0022; % [deg]
            init_data.longitude = 2.0575; % [deg]

        case 13
            init_data.name = 'Norway (Svalbard, KSAT, NEN)';
            init_data.latitude = 78.231; % [deg]
            init_data.longitude = 15.389; % [deg]

        case 14
            init_data.name = 'Chile (Santiago, SSC, NEN)';
            init_data.latitude = -33.1511; % [deg]
            init_data.longitude = -70.6664; % [deg]

        case 15
            init_data.name = 'Singapore (KSAT, NEN)';
            init_data.latitude = 1.3962; % [deg]
            init_data.longitude = 103.8343; % [deg]

        case 16
            init_data.name = 'Virginia (Wallops Island, NASA, NEN)';
            init_data.latitude = 37.9249; % [deg]
            init_data.longitude = -75.4765; % [deg]

        case 17
            init_data.name = 'New Mexico (White Sands Complex, NASA, NEN)';
            init_data.latitude = 32.5047; % [deg]
            init_data.longitude = -106.6108; % [deg]


        otherwise
            error('GS antenna not defined!')
    end

    mission.true_GS_radio_antenna{i_HW} = True_GS_Radio_Antenna(init_data, mission, i_HW);
end

%% Spacecraft Initialization

for i_SC = 1:1:mission.num_SC
    mission.true_SC{i_SC} = [];


    %% Spacecraft Body Configuration
    % i_SC = 1;  % First spacecraft
    init_data = [];
    init_data.i_SC = i_SC;

    % Payload body shape model
    init_data.shape_model{1} = [];
    init_data.shape_model{1}.Vertices = [
        [4.0, 0, 0],
        [5.5, 0, 0],
        [5.5, 2, 0],
        [4.0, 2, 0],
        [4.0, 0, 2],
        [5.5, 0, 2],
        [5.5, 2, 2],
        [4.0, 2, 2]
        ];              % 1.5x2x2 [m]
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
    init_data.shape_model{1}.mass = 2142; % [kg] Dry mass
    init_data.shape_model{1}.name = 'NISAR';

    % Computations required to have a regular octagon

    t   = 1 - sqrt(2)/2;
    ymin = 0.5; ymax = 1.5;
    zmin = 0.5; zmax = 1.5;

    yL = ymin + t;  % 0.5 + 0.292893 ≈ 0.7929
    yH = ymax - t;  % 1.5 - 0.292893 ≈ 1.2071
    zL = zmin + t;  % 0.5 + 0.292893 ≈ 0.7929
    zH = zmax - t;  % 1.5 - 0.292893 ≈ 1.2071

    % Bus (bottom) body shape model

    % Vertices (x = 4 face)
    init_data.shape_model{4} = [];
    init_data.shape_model{4}.Vertices = [
        4, yL,   zmin;
        4, yH,   zmin;
        4, ymax, zL;
        4, ymin, zL;
        0, yL,   zmin;
        0, yH,   zmin;
        0, ymax, zL;
        0, ymin, zL
        ];

    init_data.shape_model{4}.Faces = [1 2 3;
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

    init_data.shape_model{4}.Face_reflectance_factor = 0.6*ones(size(init_data.shape_model{1}.Faces,1),1);
    init_data.shape_model{4}.type = 'cuboid';
    init_data.shape_model{4}.mass = 2142; % [kg] Dry mass
    init_data.shape_model{4}.name = 'NISAR';

    % Bus (middle) body shape model
    init_data.shape_model{5} = [];
    init_data.shape_model{5}.Vertices = [
        4, ymin, zL;
        4, ymax, zL;
        4, ymax, zH;
        4, ymin, zH;
        0, ymin, zL;
        0, ymax, zL;
        0, ymax, zH;
        0, ymin, zH
        ];              % 1.5x2x2 [m]
    init_data.shape_model{5}.Faces = [1 2 3;
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
    init_data.shape_model{5}.Face_reflectance_factor = 0.6*ones(size(init_data.shape_model{1}.Faces,1),1);
    init_data.shape_model{5}.type = 'cuboid';
    init_data.shape_model{5}.mass = 2142; % [kg] Dry mass
    init_data.shape_model{5}.name = 'NISAR';

    % Bus (top) body shape model
    init_data.shape_model{6} = [];
    init_data.shape_model{6}.Vertices = [
        4, ymin, zH;
        4, ymax, zH;
        4, yH,   zmax;
        4, yL,   zmax;
        0, ymin, zH;
        0, ymax, zH;
        0, yH,   zmax;
        0, yL,   zmax
        ];              % 1.5x2x2 [m]
    init_data.shape_model{6}.Faces = [1 2 3;
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
    init_data.shape_model{6}.Face_reflectance_factor = 0.6*ones(size(init_data.shape_model{1}.Faces,1),1);
    init_data.shape_model{6}.type = 'cuboid';
    init_data.shape_model{6}.mass = 2142; % [kg] Dry mass
    init_data.shape_model{6}.name = 'NISAR';


    % Boom base at top middle of the spacecraft (fixed point)
    boom_base_position = [4.2, 1.0, 2];  % [m]
    boom_length = 9;                         % total boom length
    boom_width = 0.1;                        % boom thickness

    % Define the unrotated boom vertices as a cuboid along +Z starting from boom_base_position
    boom_vertices_unrot = [
        boom_base_position(1)-boom_width/2, boom_base_position(2)-boom_width/2, boom_base_position(3);
        boom_base_position(1)+boom_width/2, boom_base_position(2)-boom_width/2, boom_base_position(3);
        boom_base_position(1)+boom_width/2, boom_base_position(2)+boom_width/2, boom_base_position(3);
        boom_base_position(1)-boom_width/2, boom_base_position(2)+boom_width/2, boom_base_position(3);
        boom_base_position(1)-boom_width/2, boom_base_position(2)-boom_width/2, boom_base_position(3)+boom_length;
        boom_base_position(1)+boom_width/2, boom_base_position(2)-boom_width/2, boom_base_position(3)+boom_length;
        boom_base_position(1)+boom_width/2, boom_base_position(2)+boom_width/2, boom_base_position(3)+boom_length;
        boom_base_position(1)-boom_width/2, boom_base_position(2)+boom_width/2, boom_base_position(3)+boom_length
        ];

    % Define boom faces as before
    boom_faces = [
        1 2 3; 1 3 4;  % bottom
        5 6 7; 5 7 8;  % top
        1 2 6; 1 6 5;  % sides
        2 3 7; 2 7 6;
        3 4 8; 3 8 7;
        4 1 5; 4 5 8
        ];

    % Tilt angle in radians
    theta_deg = 37;
    theta_rad = deg2rad(theta_deg);

    % Rotation matrix about Y-axis
    Ry = [cos(theta_rad) 0 sin(theta_rad);
        0             1 0;
        -sin(theta_rad) 0 cos(theta_rad)];

    % Translate boom vertices to origin for rotation (subtract boom base)
    boom_vertices_centered = boom_vertices_unrot - boom_base_position;

    % Rotate vertices
    boom_vertices_rotated = (Ry * boom_vertices_centered')';

    % Translate back to original boom base position
    boom_vertices_tilted = boom_vertices_rotated + boom_base_position;

    % Assign to spacecraft shape model for the boom
    init_data.shape_model{2} = [];
    init_data.shape_model{2}.Vertices = boom_vertices_tilted;
    init_data.shape_model{2}.Faces = boom_faces;
    init_data.shape_model{2}.Face_reflectance_factor = 0.6 * ones(size(boom_faces,1),1);
    init_data.shape_model{2}.type = 'cuboid';
    init_data.shape_model{2}.mass = 40;
    init_data.shape_model{2}.name = 'Boom';

    % Calculate the new antenna origin as the centroid of the top four boom vertices (boom end after tilt)
    antenna_origin = mean(boom_vertices_tilted(5:8,:), 1) + [-5, 0, 3];

    % Define reflector vertices without rotation (relative to antenna_origin)
    reflector_size = 12; % antenna side length in meters for square reflector
    reflector_thickness = 0.1;

    reflector_vertices_unrot = [
        antenna_origin(1)-reflector_size/2, antenna_origin(2)-reflector_size/2, antenna_origin(3)-reflector_thickness/2;
        antenna_origin(1)+reflector_size/2, antenna_origin(2)-reflector_size/2, antenna_origin(3)-reflector_thickness/2;
        antenna_origin(1)+reflector_size/2, antenna_origin(2)+reflector_size/2, antenna_origin(3)-reflector_thickness/2;
        antenna_origin(1)-reflector_size/2, antenna_origin(2)+reflector_size/2, antenna_origin(3)-reflector_thickness/2;
        antenna_origin(1)-reflector_size/2, antenna_origin(2)-reflector_size/2, antenna_origin(3)+reflector_thickness/2;
        antenna_origin(1)+reflector_size/2, antenna_origin(2)-reflector_size/2, antenna_origin(3)+reflector_thickness/2;
        antenna_origin(1)+reflector_size/2, antenna_origin(2)+reflector_size/2, antenna_origin(3)+reflector_thickness/2;
        antenna_origin(1)-reflector_size/2, antenna_origin(2)+reflector_size/2, antenna_origin(3)+reflector_thickness/2
        ];

    reflector_faces = [
        1 2 3; 1 3 4;  % bottom
        5 6 7; 5 7 8;  % top
        1 2 6; 1 6 5;  % sides
        2 3 7; 2 7 6;
        3 4 8; 3 8 7;
        4 1 5; 4 5 8
        ];

    % Apply the same tilt to the reflector vertices about antenna origin
    reflector_vertices_centered = reflector_vertices_unrot - antenna_origin;
    reflector_vertices_tilted = (Ry * reflector_vertices_centered')' + antenna_origin;

    % Assign to spacecraft shape model for the radio antenna
    init_data.shape_model{3} = [];
    init_data.shape_model{3}.Vertices = reflector_vertices_tilted;
    init_data.shape_model{3}.Faces = reflector_faces;
    init_data.shape_model{3}.Face_reflectance_factor = 0.7 * ones(size(reflector_faces,1),1);
    init_data.shape_model{3}.type = 'cuboid';
    init_data.shape_model{3}.mass = 80;
    init_data.shape_model{3}.name = 'Reflector';

    % Additional mass components
    init_data.mass.supplement{1}.mass = 40; % [kg]
    init_data.mass.supplement{1}.location = [ (5.5/2) + (9/2), 0, 0 ]; % [m]
    init_data.mass.supplement{1}.MI_over_m = zeros(3,3); % [m^2]
    init_data.mass.supplement{2}.mass = 80; % [kg]
    init_data.mass.supplement{2}.location = [ (5.5/2) + 9, 0, 0 ]; % [m]
    init_data.mass.supplement{2}.MI_over_m = zeros(3,3); % [m^2]


    init_data.mode_COM_selector = 'update';  % Compute Center of Mass dynamically
    init_data.mode_MI_selector = 'update';   % Compute Moment of Inertia dynamically

    % Initialize hardware configuration
    run init_num_hardware_exists
    init_data.num_hardware_exists = num_hardware_exists;
    clear num_hardware_exists

    % Define hardware complement
    init_data.num_hardware_exists.num_onboard_clock = 1;
    init_data.num_hardware_exists.num_camera = 0;
    init_data.num_hardware_exists.num_solar_panel = 2;
    init_data.num_hardware_exists.num_battery = 1;
    init_data.num_hardware_exists.num_onboard_memory = 1;
    init_data.num_hardware_exists.num_sun_sensor = 0;
    init_data.num_hardware_exists.num_star_tracker = 0;
    init_data.num_hardware_exists.num_imu = 0;
    init_data.num_hardware_exists.num_micro_thruster = 0; % Failed attempt
    init_data.num_hardware_exists.num_chemical_thruster = 0;
    init_data.num_hardware_exists.num_reaction_wheel = 0;
    init_data.num_hardware_exists.num_communication_link = 2*mission.true_ground_station.num_GS_radio_antenna;
    init_data.num_hardware_exists.num_radio_antenna = 1;
    init_data.num_hardware_exists.num_fuel_tank = 1;
    init_data.num_hardware_exists.num_onboard_computer = 1;
    init_data.num_hardware_exists.num_remote_sensing = 2;

    mission.true_SC{i_SC}.true_SC_body = True_SC_Body(init_data, mission);

    %% Initialize First Spacecraft's Position and Velocity

    init_data = [];

    % Earth centered - J2000 frame (inertial): ECI frame
    % init_data.position_relative_target = [ 0.0,  -0.0,  7125.0]; % [km] Altitude 747 km
    % init_data.velocity_relative_target = [ 7.48,  0.0,  0.0 ]; % [km/sec]

    a = 747 + mission.true_target{1}.radius; % [km]
    e = 0;
    i = deg2rad(98.4); % [rad]
    % RAAN = deg2rad(12); % [rad]
    w = 0; % [rad]
    M = 0; % [rad]

    if i_SC == 1

        switch mission.this_phase

            case 'Hurricane Helene'
                % 6AM-6PM orbit
                RAAN = deg2rad(91); % [rad]

            case 'Texas heavy rain'
                % 6AM-6PM orbit
                RAAN = deg2rad(12); % [rad]

            otherwise  
                % 6AM-6PM orbit
                RAAN = deg2rad(47.5); % [rad]

        end

    elseif i_SC == 2

        switch mission.this_phase

            case 'Hurricane Helene'
                % 12AM-12PM orbit
                % RAAN = deg2rad(91+90); % [rad]

                % 6AM-6PM orbit
                RAAN = deg2rad(91); % [rad]
                M = deg2rad(180); % [rad]

            case 'Texas heavy rain'
                % 12AM-12PM orbit
                % RAAN = deg2rad(12+90); % [rad]

                % 6AM-6PM orbit
                RAAN = deg2rad(12); % [rad]
                M = deg2rad(180); % [rad]

            otherwise
                % 12AM-12PM orbit
                % RAAN = deg2rad(135); % [rad]

                % 6AM-6PM orbit
                RAAN = deg2rad(47.5); % [rad]
                M = deg2rad(180); % [rad]

        end


    else

    end



    oe = [a; e; i; RAAN; w; M];
    rv = Coe2Rv(oe, mission.true_target{1}.mu); % [km, km/sec]
    init_data.position_relative_target = rv(1:3)'; % [km]
    init_data.velocity_relative_target = rv(4:6)'; % [km/sec]

    Sun_pos = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_target{1}.position); % [km]
    SC_pos = func_normalize_vec(rv(1:3)'); % [km]
    Sun_Earth_SC_angle = rad2deg(func_angle_between_vectors(Sun_pos', SC_pos')) % [deg]

    SC_vel = func_normalize_vec(rv(4:6)'); % [km]
    SC_orbit_normal = cross(SC_pos, SC_vel);
    Sun_SC_orbit_normal_angle = rad2deg(func_angle_between_vectors(Sun_pos', SC_orbit_normal')) % [deg]

    init_data.name_relative_target = 'Earth';

    init_data.mode_true_SC_navigation_dynamics_selector = 'Relative Dynamics';

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
    init_data.power_loss_rate = 0.1; % [float] 10% power loss in distribution and conversion
    mission.true_SC{i_SC}.true_SC_power = True_SC_Power(init_data, mission);

    %% Initialize First Spacecraft's Data

    init_data = [];
    init_data.mode_true_SC_data_handling_selector = 'Generic';
    mission.true_SC{i_SC}.true_SC_data_handling = True_SC_Data_Handling(init_data, mission);

    %% Initialize First Spacecraft's Radio Antenna

    pointing_theta = 37; % Antenna's pointing direction [deg]

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna

        init_data = [];
        init_data.location = antenna_origin;   % [unit vector] antenna physical axis in Body frame
        init_data.orientation = -[sind(pointing_theta) 0 cosd(pointing_theta)];   % [unit vector] antenna pointing direction for nominal gain in Body frame

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Antennas
        init_data.antenna_type = "High Gain";      % antenna type

        init_data.base_data_rate_generated = 10; % [kbps]

        init_data.TX_power_consumed = 10;  % [W] Based on 10 W RF output
        init_data.RX_power_consumed = 5;   % [W] Typical for spacecraft receivers

        mission.true_SC{i_SC}.true_SC_radio_antenna{i_HW} = True_SC_Radio_Antenna(init_data, mission, i_SC, i_HW);
    end


    %% Spacecraft Fuel Tank Configuration
    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
        init_data = [];

        % Basic properties
        init_data.name = ['Fuel Tank ', num2str(i_HW)];
        init_data.instantaneous_power_consumed = 5.0;    % [W] for heaters, valves, etc.
        init_data.instantaneous_data_rate_generated = 0.1;  % [kbps] for telemetry

        % Fuel properties
        init_data.maximum_capacity = 90.0;    % [kg] total fuel capacity
        init_data.initial_fuel_mass = 0.0;   % [kg] initial fuel mass (full tank)
        init_data.fuel_density = 1011;       % [kg/m^3] typical hydrazine density

        % Physical properties
        init_data.location = [2.0, 1.0, 0.5];  % [m] tank location in body frame

        % Shape model - simplified cuboid
        init_data.shape_model = [];
        init_data.shape_model.Vertices = [
            0,    0,    0;
            0.5,  0,    0;
            0.5,  0.3,  0;
            0,    0.3,  0;
            0,    0,    0.6;
            0.5,  0,    0.6;
            0.5,  0.3,  0.6;
            0,    0.3,  0.6
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
        init_data.instantaneous_power_consumed = 10; % [W]
        init_data.instantaneous_data_rate_generated = 1.5e9; % [kbps]
        init_data.shape_model = [];

        panel_length = 5.5;        % Length along Y
        panel_thickness = 2.0;     % Thickness along Z

        if i_HW == 1
            % Attach first solar panel with origin at (0, 0, 1)
            x_origin = 5;
            y_origin = -2.75;
            z_origin = 2;
            init_data.shape_model.Vertices = [
                x_origin,           y_origin + panel_length/2,   z_origin;
                x_origin,           y_origin - panel_length/2,   z_origin;
                x_origin,           y_origin - panel_length/2,   z_origin - panel_thickness;
                x_origin,           y_origin + panel_length/2,   z_origin - panel_thickness
                ];

        elseif i_HW == 2
            % Attach second solar panel with origin at (2, 0, 1)
            x_origin = 5;
            y_origin = 2.75+2;
            z_origin = 2;
            init_data.shape_model.Vertices = [
                x_origin,           y_origin + panel_length/2,   z_origin;
                x_origin,           y_origin - panel_length/2,   z_origin;
                x_origin,           y_origin - panel_length/2,   z_origin - panel_thickness;
                x_origin,           y_origin + panel_length/2,   z_origin - panel_thickness
                ];

        end

        init_data.shape_model.Faces = [1 2 3; 1 4 3];
        init_data.shape_model.Face_reflectance_factor_solar_cell_side = [0.01; 0.01];
        init_data.shape_model.Face_reflectance_factor_opposite_side = [0.5; 0.5];
        % Face normal points along ±X depending on side

        init_data.shape_model.Face_orientation_solar_cell_side = [1 0 0]; % Facing inward from left side

        init_data.shape_model.type = 'cuboid';
        init_data.mass = 0.24; % Approximate mass

        init_data.type = 'passive_deployed';
        init_data.packing_fraction = 0.74;
        init_data.solar_cell_efficiency = 0.28;

        mission.true_SC{i_SC}.true_SC_solar_panel{i_HW} = True_SC_Solar_Panel(init_data, mission, i_SC, i_HW);
    end




    %% Initialize First Spacecraft's Battery

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

        init_data = [];
        init_data.maximum_capacity = 11000; % [W hr] (known)
        init_data.charging_efficiency = 0.96; % [float <= 1]
        init_data.discharging_efficiency = 0.96; % [float <= 1]
        init_data.instantaneous_power_consumed = 10; % [W]
        init_data.instantaneous_data_rate_generated = 7.4e6; % [kbps] i.e. 1 Byte per sec

        mission.true_SC{i_SC}.true_SC_battery{i_HW} = True_SC_Battery(init_data, mission, i_SC, i_HW);

    end

    %% Initialize First Spacecraft's Onboard Memory

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory

        init_data = [];
        init_data.maximum_capacity = 100e9; % [kb] 100 Tb (assumed value)
        init_data.instantaneous_power_consumed = 10; % [W]
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

        init_data.flag_show_coverage_plot = 0;
        init_data.wait_time_visualize_SC_camera_coverage_during_sim = 10*60; % [sec]
        init_data.num_points = 5000; % [integer]

        init_data.instantaneous_data_generated_per_pixel = (1e-3)* 8; % [kb]

        mission.true_SC{i_SC}.true_SC_camera{i_HW} = True_SC_Camera(init_data, mission, i_SC, i_HW);

    end

    %% Initialize First Spacecraft's Sun Sensors

    % Everything presumed

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor

        init_data = [];
        init_data.instantaneous_power_consumed = 0.05; % [W]
        init_data.instantaneous_data_generated_per_sample = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        init_data.mode_true_SC_sun_sensor_selector = 'Simple with Sun in FOV';
        init_data.measurement_wait_time = 0.1; % [sec]
        init_data.measurement_noise = deg2rad(0.5); % [rad] 0.5 degrees
        init_data.field_of_view = 60; % [deg]

        switch i_HW

            case 1
                init_data.location = [0, yL, zmin];
                init_data.orientation = [0 0 -1];
            case 2
                init_data.location = [0, ymax, zL];
                init_data.orientation = [-1 0 0];
            case 3
                init_data.location = [4, ymax, zL];
                init_data.orientation = [0 0 -1];
            case 4
                init_data.location = [4, ymin, zL];
                init_data.orientation = [0 -1 0];
            case 5
                init_data.location = [2, ymin, zH];
                init_data.orientation = [0 0 1];
            case 6
                init_data.location = [2, ymax, zH];
                init_data.orientation = [0 1 0];

            otherwise
                error('Should not reach here!')
        end

        mission.true_SC{i_SC}.true_SC_sun_sensor{i_HW} = True_SC_Sun_Sensor(init_data, mission, i_SC, i_HW);

    end

    %% Initialize First Spacecraft's Star Tracker

    % Everything presumed - star trackers that could fit in the I-3K bus

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker

        init_data = [];

        init_data.instantaneous_power_consumed = 1.5; %
        init_data.instantaneous_data_generated_per_sample = (4 + 1)*16e-3; % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        init_data.mode_true_SC_star_tracker_selector = 'Simple with Sun outside FOV';
        init_data.measurement_wait_time = 0.1; % [sec]
        init_data.measurement_noise = 1e-4; % [rad]
        init_data.field_of_view = 19; % [deg]

        switch i_HW

            case 1
                init_data.location = [0.3, ymin, zH];
                init_data.orientation = [0 -1 0];

            case 2
                init_data.location = [3.8, ymin, zL];
                init_data.orientation = [0 0 -1];

            case 3
                init_data.location = [2, ymax, zL];
                init_data.orientation = [0 1 0];

            otherwise
                error('Should not reach here!')
        end

        mission.true_SC{i_SC}.true_SC_star_tracker{i_HW} = True_SC_Star_Tracker(init_data, mission, i_SC, i_HW);

    end

    %% Initialize First Spacecraft's IMU

    % Everything presumed

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_imu

        init_data = [];

        init_data.instantaneous_power_consumed = 2.5; % [W]
        init_data.instantaneous_data_generated_per_sample = (3 + 1)*16e-3; % [kb] : 3 angular velocity + 1 time vector, each of 16-bit depth
        init_data.mode_true_SC_imu_selector = 'Simple';
        init_data.measurement_wait_time = 0.1; % [sec]
        init_data.measurement_noise = 2e-5; % [rad/sec]

        init_data.location = [2.75 1.0 1.0]; % [m]
        init_data.orientation = [1 0 0]; % [unit vector]

        mission.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data, mission, i_SC, i_HW);

    end

    %% Initialize First Spacecraft's Micro Thrusters (to be finished later)

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster

        init_data = [];

        init_data.instantaneous_power_consumption = 50; % Watts
        init_data.instantaneous_data_generated_per_sample = (3)*16e-3; % [kb] : thrust + health + temperature, each of 16-bit depth
        init_data.mode_true_SC_micro_thruster_selector = 'Simple'; % Mode (Truth/Simple)
        init_data.thruster_noise = 100*(1e-6); % Noise level [N](unit depends on implementation)

        init_data.micro_thruster_ISP = 220; % [sec]

        if i_HW <= 11
            init_data.minimum_thrust = 0.1;  % [N] minimum for 11 N thrusters
            init_data.maximum_thrust = 11;   % [N] max thrust for these thrusters
        else
            init_data.minimum_thrust = 0.01; % [N] min for 1 N thrusters
            init_data.maximum_thrust = 1;    % [N] max thrust for these thrusters
        end

        init_data.command_wait_time = 0.5; % Seconds between commands

        switch i_HW
            case 1
                init_data.location = [1.0, 0.5, 0.2];
                init_data.orientation = [1 0 0];
            case 2
                init_data.location = [1.0, -0.5, 0.2];
                init_data.orientation = [-1 0 0];
            case 3
                init_data.location = [-1.0, 0.5, 0.2];
                init_data.orientation = [0 1 0];
            case 4
                init_data.location = [-1.0, -0.5, 0.2];
                init_data.orientation = [0 -1 0];
            case 5
                init_data.location = [0.5, 1.0, 0.5];
                init_data.orientation = [0 0 1];
            case 6
                init_data.location = [0.5, -1.0, 0.5];
                init_data.orientation = [0 0 -1];
            case 7
                init_data.location = [0.0, 0.5, -0.5];
                init_data.orientation = [1 0 0];
            case 8
                init_data.location = [0.0, -0.5, -0.5];
                init_data.orientation = [-1 0 0];
            case 9
                init_data.location = [0.5, 0.0, -0.5];
                init_data.orientation = [0 1 0];
            case 10
                init_data.location = [-0.5, 0.0, -0.5];
                init_data.orientation = [0 -1 0];
            case 11
                init_data.location = [0.0, 0.0, 1.0];
                init_data.orientation = [0 0 1];
                % Last four thrusters (1 N)
            case 12
                init_data.location = [0.25, 0.25, 0.2];
                init_data.orientation = [1 0 0];
            case 13
                init_data.location = [0.25, -0.25, 0.2];
                init_data.orientation = [-1 0 0];
            case 14
                init_data.location = [-0.25, 0.25, 0.2];
                init_data.orientation = [0 1 0];
            case 15
                init_data.location = [-0.25, -0.25, 0.2];
                init_data.orientation = [0 -1 0];
            otherwise
                error('Should not reach here!')
        end

        mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW} = True_SC_Micro_Thruster(init_data, mission, i_SC, i_HW);

    end


    %% Initialize the Reaction Wheels  (finish later - positions/orientations to be completed)

    % Everything presumed

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel

        init_data = [];
        init_data.location = [2.75, 1.0, zmin];
        init_data.radius = 0.3; % [m] radius of 1 RW
        init_data.mass = 10; % [kg] mass of 1 RW
        init_data.max_angular_velocity = 6000*2*pi/60; % [rad/s] 6000 RPM
        init_data.angular_velocity_noise = 0.001*2*pi/60;      % [rad/s] velocity noise
        init_data.instantaneous_data_generated_per_sample = (3)*16e-3; % [kb] : thrust + health + temperature, each of 16-bit depth
        init_data.max_torque = 0.2; % Nm

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
                    init_data.orientation = [0, 0, -1];  % Z-axis
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
        init_data.instantaneous_power_consumption = 5.0;        % [W] Base power draw (standby)
        init_data.thruster_warm_up_power_consumed = 20.0;        % [W] Power during warm-up phase
        init_data.command_actuation_power_consumed = 50.0;      % [W] Power during active thrust
        init_data.instantaneous_data_generated_per_sample = 10; % [kb] per sample
        init_data.chemical_thruster_noise = 1e-4;              % [N] Thrust noise level

        % Thruster properties
        init_data.chemical_thruster_ISP = 200;                 % [s] Specific impulse
        init_data.command_wait_time = 1;                       % [s] Minimum time between commands
        init_data.location = [2.75, 0, 0];                % [m] Thruster location in body frame
        init_data.orientation = [-1, 0, 0];                     % Thrust direction (unit vector)

        init_data.maximum_thrust = 11;                          % [N] Maximum thrust level
        init_data.minimum_thrust = 0.1;                       % [N] Minimum thrust level

        % Create chemical thruster object
        mission.true_SC{i_SC}.true_SC_chemical_thruster{i_HW} = True_SC_Chemical_Thruster(init_data, mission, i_SC, i_HW);
    end

    %% Onboard Computer Configuration

    % Everything presumed

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_computer
        init_data = [];

        % Basic properties
        init_data.name = ['Onboard Computer ', num2str(i_HW)];

        % Set different properties for primary and backup computers
        if i_HW == 1
            % Primary computer
            init_data.instantaneous_power_consumed = 10.0;     % [W] Main flight computer
            init_data.instantaneous_data_rate_generated = 4.0;  % [kbps] for telemetry and logs
            init_data.processor_utilization = 40;             % [%] Fixed CPU usage
        else
            % Backup computer
            init_data.instantaneous_power_consumed = 3.5;     % [W] Backup in standby mode
            init_data.instantaneous_data_rate_generated = 0.7;  % [kbps] minimal telemetry
            init_data.processor_utilization = 8;              % [%] Fixed low utilization in standby
        end

        % Create onboard computer object
        mission.true_SC{i_SC}.true_SC_onboard_computer{i_HW} = True_SC_Onboard_Computer(init_data, mission, i_SC, i_HW);
    end

    %% Initialize First Spacecraft's Remote Sensing Instruments for NISAR

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing
        init_data = [];

        switch i_HW

            case 1
                % L-band SAR near spacecraft center (origin/mid X)
                init_data.name = 'L-band SAR';
                init_data.instantaneous_power_consumed = 3000; % [W] realistic L-band power consumption
                init_data.instantaneous_data_rate_generated = 4e5; % [kbps] 35 Tb in 1 day
                init_data.mode_true_SC_remote_sensing_selector = 'Generic';
                init_data.measurement_wait_time = 0; % [sec]

                % Taken from reflector
                init_data.location = antenna_origin;   % [unit vector] antenna physical axis in Body frame
                init_data.orientation = -[sind(pointing_theta) 0 cosd(pointing_theta)];   % [unit vector] antenna pointing direction for nominal gain in Body frame
                %             init_data.location = [3 (ymax+yH)/2 (zmax+zL)/2]; % [m] near spacecraft center (X=0, Y=0, Z=1m height)
                %             init_data.orientation = [0 0 1]; % [unit vector] pointing forward along +X

                init_data.field_of_view = 30; % [deg], narrow beam typical for SAR
                init_data.flag_show_coverage_plot = 0;
                init_data.wait_time_visualize_SC_remote_sensing_coverage_during_sim = 0; % [sec]
                init_data.num_points = 50000; % points for coverage visualization

            case 2
                % S-band SAR on front of spacecraft (X=about 2.75 m, forward)
                init_data.name = 'S-band SAR';
                init_data.instantaneous_power_consumed = 13000; % [W] higher power S-band transmitter
                init_data.instantaneous_data_rate_generated = 6.43e6; % [kbps] ~1 Gbps approx.
                init_data.mode_true_SC_remote_sensing_selector = 'Generic';
                init_data.measurement_wait_time = 0; % [sec]

                % Taken from reflector
                init_data.location = antenna_origin;   % [unit vector] antenna physical axis in Body frame
                init_data.orientation = -[sind(pointing_theta) 0 cosd(pointing_theta)];   % [unit vector] antenna pointing direction for nominal gain in Body frame
                %             init_data.location = [0 (yL+yH)/2 (zL+zH)/2]; % [m] front center along X
                %             init_data.orientation = [-1 0 0]; % [unit vector] pointing forward +X

                init_data.field_of_view = 30; % [deg]
                init_data.flag_show_coverage_plot = 0;
                init_data.wait_time_visualize_SC_remote_sensing_coverage_during_sim = 10*60; % [sec] 10 min
                init_data.num_points = 50000;
            otherwise
                error('Should not reach here!')
        end
        mission.true_SC{i_SC}.true_SC_remote_sensing{i_HW} = True_SC_Remote_Sensing(init_data, mission, i_SC, i_HW);
    end


    %% Spacecraft Communication Links Configuration

    for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_communication_link
        init_data = [];

        % Configure appropriate link based on index
        if mod(i_HW,2) == 1
            % Downlink: Spacecraft to Earth
            init_data.TX_spacecraft = i_SC;            % Transmitter is this spacecraft
            init_data.TX_spacecraft_Radio_HW = 1;      % Using antenna 1

            init_data.RX_spacecraft = 0;               % Receiver is Ground Station (0)
            init_data.RX_spacecraft_Radio_HW = ceil(i_HW/2);  % Using GS antenna 1

            if ~isempty(strfind(mission.true_GS_radio_antenna{init_data.RX_spacecraft_Radio_HW}.name, 'India'))
                init_data.given_data_rate = 2.9e9;         % [kbps] Downlink data rate at India is 2.9Gbps
            elseif ~isempty(strfind(mission.true_GS_radio_antenna{init_data.RX_spacecraft_Radio_HW}.name, 'NEN'))
                init_data.given_data_rate = 4e9;         % [kbps] Downlink data rate at NEN is 4Gbps
            else
                error('Data rate not defined!')
            end

        else
            % Uplink: GS to Spacecraft
            init_data.TX_spacecraft = 0;               % Transmitter is Ground Station (0)
            init_data.TX_spacecraft_Radio_HW = i_HW/2;      % Using GS antenna 1

            init_data.RX_spacecraft = i_SC;            % Receiver is this spacecraft
            init_data.RX_spacecraft_Radio_HW = 1;      % Using antenna 1

            init_data.given_data_rate = 4;             % [kbps] Set low to avoid overfilling memory
        end

        init_data.flag_compute_data_rate = 0;      % Use given data rate instead of computing
        init_data.mode_true_SC_communication_link_selector = 'Multi-SC per GS';

        % Create communication link object
        mission.true_SC{i_SC}.true_SC_communication_link{i_HW} = True_SC_Communication_Link(init_data, mission, i_SC, i_HW);
    end


    %% Initialize Solar Radiation Pressure
    init_data = [];
    init_data.enable_SRP = 1; % Enable SRP calculations

    mission.true_SC{i_SC}.true_SC_SRP = True_SC_SRP(init_data, mission, i_SC);


    %% Initialize Gravity Gradient for Earth
    init_data = [];
    init_data.enable_G2 = 1; % Enable gravity gradient

    mission.true_SC{i_SC}.true_SC_gravity_gradient = True_SC_Gravity_Gradient(init_data, mission, i_SC);


    %% Spacecraft Software: Executive Configuration

    init_data = [];
    init_data.sc_modes = {'Science Mode', 'Maximize SP Power', 'Telecom'};
    init_data.mode_software_SC_executive_selector = 'NISAR';

    mission.true_SC{i_SC}.software_SC_executive = Software_SC_Executive(init_data, mission, i_SC);

    %% Spacecraft Software: Attitude Estimation Configuration

    init_data = [];
    init_data.mode_software_SC_estimate_attitude_selector = 'Truth';  % Use true attitude values

    mission.true_SC{i_SC}.software_SC_estimate_attitude = Software_SC_Estimate_Attitude(init_data, mission, i_SC);

    %% Spacecraft Software: Orbit Estimation Configuration

    init_data = [];
    init_data.mode_software_SC_estimate_orbit_selector = 'Truth';  % Use true orbit values with error growth when target not visible

    mission.true_SC{i_SC}.software_SC_estimate_orbit = Software_SC_Estimate_Orbit(init_data, mission, i_SC);

    %% Spacecraft Software: Orbit Control Configuration

    init_data = [];
    init_data.max_time_before_control = 0.5*60*60 + 900;  % 45 minutes
    init_data.mode_software_SC_control_orbit_selector = 'Inactive';

    mission.true_SC{i_SC}.software_SC_control_orbit = Software_SC_Control_Orbit(init_data, mission, i_SC);

    %% Spacecraft Software: Attitude Control Configuration

    init_data = [];
    init_data.mode_software_SC_control_attitude_selector = 'NISAR Oracle';
    init_data.control_gain = [1 0.2];    % Controller gain parameters

    mission.true_SC{i_SC}.software_SC_control_attitude = Software_SC_Control_Attitude(init_data, mission, i_SC);

    %% Spacecraft Software: Communication Configuration

    init_data = [];
    init_data.mode_software_SC_communication_selector = 'NISAR';
    init_data.instantaneous_data_generated_per_sample = (1e-3)*8*2;  % [kb] 2 Bytes per sample
    init_data.attitude_error_threshold_deg = 1;  % [deg] Max attitude error for communication

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

end

%% Save All Data
% clear init_data i_HW i_target
clearvars -except mission i_SC

% Vizualise the SC in 3D + Dashboard
% func_visualize_SC_NISAR(mission.storage, mission, false);
func_visualize_SC_NISAR_2SC(mission.storage, mission, false);

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

fprintf('Plotting SC%d ground track visualization...\n', i_SC);
func_post_process_NISAR_Ground_Tracks(mission, i_SC);
drawnow limitrate

for i_SC = 1:1:mission.num_SC
    func_post_process_NISAR_output_to_Excel(mission, i_SC)
end

