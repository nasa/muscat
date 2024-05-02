% General Mission Initialization File
close all, clear all, clc
addpath(genpath('../MuSCAT_Supporting_Files'))
addpath(genpath('.'))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Mission

mission_init_data = [];
mission_init_data.name = 'DARE';
mission_init_data.num_SC = 1;  % Number of Spacecraft

% Output Folder
mission_init_data.output_folder = ['Output/' char(datetime("now", "Format", "yyyy-MM-dd-HH'h'mm'm'")) '/'];
mkdir(mission_init_data.output_folder)

% Select Small Body Type
mission_init_data.small_body_type = 1; 
% 1. Bennu
% 2. Apophis
% 3. Toutatis
% 4. Itokawa
% 5. 1996HW1

% Select Mission Type
mission_init_data.mission_type = 2; 
% 1. Approach SB and Orbit SB
% 2. Approach and Crash into SB (DART mission)
% 3. Orbit SB

% Select if this is a Mission Concept simulation, or if the Mission uses Autonomy, or is carried out from the ground
mission_init_data.autonomy_type = 1; 
% 0. Only Mission Concept Simulation (runs faster)
% 1. Mission uses Autonomy
% 2. Mission carried out from the ground, major decisions need DCO time

% DCO_time_horizon [sec] : Data Cut Off (DCO) Time Horizon
mission_init_data.DCO_time_horizon = 0;  % [sec]

% Estimation (truth, KF)
mission_init_data.state_estimation_type = 'truth';
mission_init_data.attitude_estimation_type = 'truth';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Environment Variables

% True_Time
time_init_data = [];
time_init_data.t_initial = 0;     % [sec]
time_init_data.t_final = 500;   % [sec] Cruise-approach.
% time_init_data.t_final = 190000;  % [sec] To SB collision.
time_init_data.time_step = 0.1;   % [sec]
time_init_data.t_initial_date_string = '02-NOV-2018 00:00:00';  % [string] Format = [DD-MMM(words)-YYYY HH-MM-SS]
mission_true_time = True_Time(time_init_data);

% True_Stars
mission_true_stars = True_Stars(mission_true_time);
mission_true_stars.maximum_magnitude = 10;

% True_Solar_System
mission_true_solar_system = True_Solar_System(mission_true_time);

% True_Small_Body
mission_true_small_body = True_Small_Body(mission_init_data,mission_true_time);

clear time_init_data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Spacecraft

mission_true_SC = [];

for i_SC = 1:1:mission_init_data.num_SC
    
    sc_body_init_data = [];
            
    % Mass
    sc_body_init_data.mass = [];
    sc_body_init_data.mass.dry_mass = 178;       % [kg]
    sc_body_init_data.mass.propellant_mass = 0;  % [kg]
    sc_body_init_data.mass.supplement_mass = 0;  % [kg]
    sc_body_init_data.mass.location_propellant_mass = [0, 0, 0]';  % [m]
    sc_body_init_data.mass.location_supplement_mass = [0, 0, 0]';  % [m]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Spacecraft Shape Model

    % CubeSat standard dimensions
    length = 0.3;  % [m]
    width = 0.3;   % [m]
    height = 0.3;  % [m]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Structural Shape Models

    % Primary structure: cuboid
    primary_struct_shape = create_shape_cuboid(...
        length,... 
        width,... 
        height,... 
        [length/2, width/2, height/2]',... 
        eul2rotm([0, 0, 0])...
    );
    primary_struct_max_z = height;

    % SEP connector: cylinder
    sep_connector_shape = create_shape_cylinder(...
        length/6,...  % Visual estimate: sixth of the length of the primary structure
        0.02,...      % Visual estimate: 2cm separation of SEP structure from primary structure
        [length/2, width/2, primary_struct_max_z+0.02/2]',...
        eul2rotm([0, 0, 0]),...
        length/6-0.02...
    );
    sep_connector_max_z = primary_struct_max_z + 0.02;

    % SEP structure: cuboid
    sep_struct_shape = create_shape_cuboid(...
        length,...
        width,...
        0.12,...  % Visual estimate
        [length/2, width/2, sep_connector_max_z+0.12/2]',...
        eul2rotm([0, 0, 0])...
    );
    sep_struct_max_z = sep_connector_max_z + 0.12;
    
    % Store structural shape models and data
    structural_shape_models = {...
        primary_struct_shape;...
        sep_connector_shape;...
        sep_struct_shape;...
    };
    structural_shape_data = {
        "Primary Structure", [200, 200, 200], 0.6;
        "SEP Connector",     [200, 200, 200], 0.6;
        "SEP Structure",     [200, 200, 200], 0.6;
    };

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Propulsion & ACS Shape Models

    % SEP thuster: cyclinder
    sep_thruster_position = [length/2, width/2, sep_struct_max_z+0.06/2];
    sep_thruster_orientation = [0, 0, pi];
    sep_thruster_shape = create_shape_cylinder(...
        length/8,...  % Visual estimate: eigth of the length of the SEP structure
        0.06,...      % Visual estimate
        sep_thruster_position',...
        eul2rotm(sep_thruster_orientation),...
        length/8-0.02...
    );
    sep_thruster_direction = [0, 0, 1] * sep_thruster_shape.rot';
    sep_thruster_max_z = sep_struct_max_z + 0.06;

    % Microthrusters: cylinder
    num_micro_thruster = 12;
    micro_thruster_position_array = [
        -0.02, width/2, height/2;...        % Post-track (back), nadir pointed
        -0.02, width/2, height/2;...        % Post-track (back), zenith pointed
        length+0.02, width/2, height/2;...  % Along-track (front), nadir pointed
        length+0.02, width/2, height/2;...  % Along-track (front), zenith pointed
        length/2, -0.02, height/2;...       % Cross-track (left), post-track pointed
        length/2, -0.02, height/2;...       % Cross-track (left), along-track pointed
        length/2, -0.02, height/2;...       % Cross-track (left), nadir pointed
        length/2, -0.02, height/2;...       % Cross-track (left), zenith pointed
        length/2, width+0.02, height/2;...  % Cross-track (right), post-track pointed
        length/2, width+0.02, height/2;...  % Cross-track (right), along-track pointed
        length/2, width+0.02, height/2;...  % Cross-track (right), nadir pointed
        length/2, width+0.02, height/2;...  % Cross-track (right), zenith pointed
    ];
    micro_thruster_orientation_array = [
        0, 0, 0;...      % Post-track (back), nadir pointed
        0, 0, pi;...     % Post-track (back), zenith pointed
        0, 0, 0;...      % Along-track (front), nadir pointed
        0, 0, pi;...     % Along-track (front), zenith pointed
        0, pi/2, 0;...   % Cross-track (left), post-track pointed
        0, -pi/2, 0;...  % Cross-track (left), along-track pointed
        0, 0, 0;...      % Post-track (back), nadir pointed
        0, 0, pi;...     % Post-track (back), zenith pointed
        0, pi/2, 0;...   % Cross-track (right), post-track pointed
        0, -pi/2, 0;...  % Cross-track (right), along-track pointed
        0, 0, 0;...      % Post-track (back), nadir pointed
        0, 0, pi;...     % Post-track (back), zenith pointed
    ];
    micro_thruster_direction_array = zeros(num_micro_thruster, 3);
    micro_thruster_shape_models = cell(num_micro_thruster, 1);
    for i_micro_thruster = 1:1:num_micro_thruster
        micro_thruster_shape_models{i_micro_thruster} = create_shape_cylinder(...
            0.02,...
            0.055,...
            micro_thruster_position_array(i_micro_thruster, :)',...
            eul2rotm(micro_thruster_orientation_array(i_micro_thruster, :)),...
            0.015...
        );
        micro_thruster_direction_array(i_micro_thruster, :) = [0, 0, 1] * micro_thruster_shape_models{i_micro_thruster}.rot';
    end     

    % Reaction Wheels: cylinder
    % num_reaction_wheel = 3;
    % reaction_wheel_position_array = [
    %     0.02, 0.02, 0.005;
    %     0.02, 0.02, 0.005;
    %     0.02, 0.02, 0.005;
    % ];
    % reaction_wheel_angle_array = [
    %     0, 0, 0;
    %     pi/2, 0, -pi;
    %     0, -pi/2, -pi/2;
    % ];
    num_reaction_wheel = 4;
    reaction_wheel_position_array = [
        0.02, 0.02, 0.005;
        0.02, 0.02, 0.005;
        0.02, 0.02, 0.005;
        0.02, 0.02, 0.005;
    ];
    reaction_wheel_angle_array = [
        -pi/2, -pi/4, 0;
        0, -pi/4, 0;
        pi/2, -pi/4, 0;
        pi, -pi/4, 0;
    ];
    reaction_wheel_shape_models = cell(num_reaction_wheel, 1);
    for i_reaction_wheel = 1:1:num_reaction_wheel
        reaction_wheel_shape_models{i_reaction_wheel} = create_shape_cylinder(...
            0.02,...
            0.01,...
            reaction_wheel_position_array(i_reaction_wheel, :)',...
            eul2rotm(reaction_wheel_angle_array(i_reaction_wheel, :)),...
            0.02...
        );
    end

    % Store propulsion and reaction wheel shape models and data
    propulsion_rwa_shape_models = [...
        {sep_thruster_shape};...
        micro_thruster_shape_models;...
        reaction_wheel_shape_models;...
    ];
    propulsion_rwa_shape_data = [...
        {"SEP Thruster", [210, 178, 178], 0.6};...
        mat2cell(["Micro Thruster"; repmat("", num_micro_thruster-1, 1)], ones(num_micro_thruster, 1)),...
        mat2cell(repmat([210, 178, 178], num_micro_thruster, 1), ones(num_micro_thruster, 1)),...
        mat2cell(repmat(0.6, num_micro_thruster, 1), ones(num_micro_thruster, 1));...
        mat2cell(["Reaction Wheel"; repmat("", num_reaction_wheel-1, 1)], ones(num_reaction_wheel, 1)),...
        mat2cell(repmat([210, 178, 178], num_reaction_wheel, 1), ones(num_reaction_wheel, 1)),...
        mat2cell(repmat(0, num_reaction_wheel, 1), ones(num_reaction_wheel, 1));...
    ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Sensor & Instrument Shape Models

    % Wide-angle cameras: cuboid
    num_wac_camera = 5;
    wac_camera_location_array = [
        length/2, width/2, 0.01/2;              % Nadir pointed
        0.01/2, width/2-0.03, height/2;         % Post-track pointed
        0.01/2, width/2+0.03, height/2;         % Post-track pointed
        length-0.01/2, width/2-0.03, height/2;  % Along-track pointed
        length-0.01/2, width/2+0.03, height/2;  % Along-track pointed
    ];
    wac_camera_angle_array = [
        0, 0, 0;...      % Nadir pointed
        0, pi/2, 0;...   % Post-track pointed
        0, pi/2, 0;...   % Post-track pointed
        0, -pi/2, 0;...  % Along-track pointed
        0, -pi/2, 0;...  % Along-track pointed
    ];
    wac_camera_shape_models = cell(num_wac_camera, 1);
    for i_wac_camera = 1:1:num_wac_camera
        wac_camera_shape_models{i_wac_camera} = create_shape_cuboid(...
            0.01,...
            0.01,...
            0.01,...
            wac_camera_location_array(i_wac_camera, :)',...
            eul2rotm(wac_camera_angle_array(i_wac_camera, :))...
        );
    end

    % Narrow-angle camera: cuboid
    num_nac_camera = 1;
    nac_camera_location_array = [length/2, width-0.02/2, 0.02/2];  % Nadir pointed
    nac_camera_angle_array = [0, 0, 0];                            % Nadir pointed
    nac_camera_shape_models = cell(num_nac_camera, 1);
    for i_nac_camera = 1:1:num_nac_camera
        nac_camera_shape_models{i_nac_camera} = create_shape_cuboid(...
            0.02,...
            0.02,...
            0.02,...
            nac_camera_location_array(i_nac_camera, :)',...
            eul2rotm(nac_camera_angle_array(i_nac_camera, :))...
        );
    end

    % X-ray spectrometer: cuboid
    xray_spectrometer_shape = create_shape_cuboid(...  % Nadir pointed
        0.05,...
        0.05,...
        0.05,...
        [length/4, 3*width/4, 0.05/2]',...
        eul2rotm([0, 0, 0])...
    );

    % Mid-/long-wave infrared point spectrometer: cuboid
    mid_long_wave_infrared_spectrometer_shape = create_shape_cuboid(...  % Nadir pointed
        0.03,...
        0.03,...
        0.03,...
        [length-0.03/2, width-0.03/2, 0.03/2]',...
        eul2rotm([0, 0, 0])...
    );

    % Store sensor and instrument shape models and data
    sensor_instrument_shape_models = [...
        wac_camera_shape_models;...
        nac_camera_shape_models;...
        xray_spectrometer_shape;...
        mid_long_wave_infrared_spectrometer_shape;...
    ];
    sensor_instrument_shape_data = [...
        mat2cell(["Wide-Angle Camera"; repmat("", num_wac_camera-1, 1)], ones(num_wac_camera, 1)),...
        mat2cell(repmat([120, 192, 100], num_wac_camera, 1), ones(num_wac_camera, 1)),...
        mat2cell(repmat(0, num_wac_camera, 1), ones(num_wac_camera, 1));...
        mat2cell(["Narrow-Angle Camera"; repmat("", num_nac_camera-1, 1)], ones(num_nac_camera, 1)),...
        mat2cell(repmat([120, 192, 100], num_nac_camera, 1), ones(num_nac_camera, 1)),...
        mat2cell(repmat(0, num_nac_camera, 1), ones(num_nac_camera, 1));...
        {"X-Ray Spectrometer", [100, 192, 174], 0};...
        {"Mid-/Long-wave Infrared Spectrometer", [100, 192, 174], 0};...
    ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solar Array Shape Models
    
    % Body mounted solar arrays: panel 
    array_margin = 0.02;
    num_body_mounted_solar_array = 3;
    body_mounted_solar_array_size_array = [
        width/2-2*array_margin, height-2*array_margin;  % Post-track (back) array
        length-2*array_margin, height-2*array_margin;   % Cross-track (left) array
        length-2*array_margin, height-2*array_margin;   % Cross-track (right) array
    ];
    body_mounted_solar_array_position_array = [
        0, width/2, height/4;       % Post-track (back) array
        length/2, 0, height/2;      % Cross-track (left) array
        length/2, width, height/2;  % Cross-track (right) array
    ];
    body_mounted_solar_array_orientation_array = [
        0, -pi/2, 0;  % Post-track (back) array
        0, 0, pi/2;   % Cross-track (left) array
        0, 0, -pi/2;  % Cross-track (right) array
    ];
    body_mounted_solar_array_shape_models = cell(num_body_mounted_solar_array, 1);
    for i_body_mounted_solar_array = 1:1:num_body_mounted_solar_array
        body_mounted_solar_array_shape_models{i_body_mounted_solar_array} = create_shape_panel(... 
            body_mounted_solar_array_size_array(i_body_mounted_solar_array, 1),...
            body_mounted_solar_array_size_array(i_body_mounted_solar_array, 2),...
            body_mounted_solar_array_position_array(i_body_mounted_solar_array, :)',...
            eul2rotm(body_mounted_solar_array_orientation_array(i_body_mounted_solar_array, :))...
        );
    end

    % Panel mounted (SEP) solar arrays: panel
    num_panel_mounted_solar_array = 2;
    panel_mounted_solar_array_position_array = [
        length/2, -4*width/2, sep_connector_max_z+(sep_struct_max_z-sep_connector_max_z)/2;      % Cross-track (left) array
        length/2, width+4*width/2, sep_connector_max_z+(sep_struct_max_z-sep_connector_max_z)/2  % Cross-track (right) array  
    ];
    panel_mounted_solar_array_orientation_array = [
        0, -pi/2, 0;  % Cross-track (left) array
        0, -pi/2, 0;  % Cross-track (right) array
    ];
    panel_mounted_solar_array_shape_models = cell(num_panel_mounted_solar_array, 1);
    solar_array_panel_mount_models = cell(num_panel_mounted_solar_array, 1);
    for i_panel_mounted_solar_array = 1:1:num_panel_mounted_solar_array
        panel_mounted_solar_array_shape_models{i_panel_mounted_solar_array} = create_shape_panel(... 
            length,...   % Visual estimate: length of the primary structure
            4*width,...  % Visual estimate: four times the width of the primary structure
            panel_mounted_solar_array_position_array(i_panel_mounted_solar_array, :)',...
            eul2rotm(panel_mounted_solar_array_orientation_array(i_panel_mounted_solar_array, :))...
        );
        solar_array_panel_mount_models{i_panel_mounted_solar_array} = create_shape_cuboid(...
            length,...   % Visual estimate: length of the primary structure
            4*width,...  % Visual estimate: four times the width of the primary structure
            0.05,...     % TODO: look at panel mount spec.
            panel_mounted_solar_array_position_array(i_panel_mounted_solar_array, :)' + [0.05/2, 0, 0]',...
            eul2rotm(panel_mounted_solar_array_orientation_array(i_panel_mounted_solar_array, :))...
        );
    end
    
    % Store solar array shape models and data
    solar_array_shape_models = [...
        body_mounted_solar_array_shape_models;...
        panel_mounted_solar_array_shape_models;...
    ];
    solar_array_panel_mount_shape_data = [...
        mat2cell(["Panel Mounted Solar Array Mount"; repmat("", num_panel_mounted_solar_array-1, 1)], ones(num_panel_mounted_solar_array, 1)),...
        mat2cell(repmat([200, 200, 200], num_panel_mounted_solar_array, 1), ones(num_panel_mounted_solar_array, 1)),...
        mat2cell(repmat(0.6, num_panel_mounted_solar_array, 1), ones(num_panel_mounted_solar_array, 1));...
    ];
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Full Spacecraft Models

    % Add color and relfectance factor to shape models
    shape_models = [
        structural_shape_models;
        propulsion_rwa_shape_models;
        sensor_instrument_shape_models;
        solar_array_panel_mount_models;
    ];
    shape_data = [
        structural_shape_data;
        propulsion_rwa_shape_data;
        sensor_instrument_shape_data;
        solar_array_panel_mount_shape_data;
    ];
    assert(size(shape_models, 1) == size(shape_data, 1));
    
    for i_shape = 1:1:size(shape_models, 1)
        n_faces_i = size(shape_models{i_shape}.Faces, 1);
        shape_models{i_shape}.name = shape_data{i_shape, 1};
        shape_models{i_shape}.reflectance_factor = shape_data{i_shape, 3} * ones(n_faces_i, 1);
        shape_models{i_shape}.color = shape_data{i_shape, 2} / 255;
    end

    % Assign shape models to spacecraft init data
    sc_body_init_data.shape_model = shape_models;
    
    % Inertia properties
    sc_body_init_data.flag_given_inertia_matrix = 0;  % [boolean] : flag to use given inertia matrix instead of internally computed one
    sc_body_init_data.given_inertia_matrix = [];      % Inertia of body shape in body center of mass
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solar panels
    sc_body_init_data.number_solar_panel = num_body_mounted_solar_array + num_panel_mounted_solar_array;
    for i_solar_panel = 1:1:num_body_mounted_solar_array+num_panel_mounted_solar_array
        sc_body_init_data.solar_panel(i_solar_panel).shape_model = solar_array_shape_models{i_solar_panel};
        sc_body_init_data.solar_panel(i_solar_panel).shape_model.reflectance_factor_solar_cell_side = 0.01 * [1, 1]';
        sc_body_init_data.solar_panel(i_solar_panel).shape_model.reflectance_factor_opposite_side = 0 * [1, 1]';     
        sc_body_init_data.solar_panel(i_solar_panel).shape_model.orientation_solar_cell_side = [0, 0, 1] * solar_array_shape_models{i_solar_panel}.rot';
        sc_body_init_data.solar_panel(i_solar_panel).solar_panel_instantaneous_power_consumption = 0.01;  % [W]
        sc_body_init_data.solar_panel(i_solar_panel).solar_panel_mass = 2 * solar_array_shape_models{i_solar_panel}.volume / 0.05;  % [kg] ~ 2 kg/m^2
        sc_body_init_data.solar_panel(i_solar_panel).packing_fraction = 1.0;
        sc_body_init_data.solar_panel(i_solar_panel).solar_cell_efficiency = 0.3;
        sc_body_init_data.solar_panel(i_solar_panel).type = 2;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % HW exists
    sc_body_init_data.flag_hardware_exists = [];
    sc_body_init_data.flag_hardware_exists.adc_sun_sensor = 1;                % [boolean]
    sc_body_init_data.flag_hardware_exists.adc_star_tracker = 1;              % [boolean]
    sc_body_init_data.flag_hardware_exists.adc_imu = 1;                       % [boolean]
    sc_body_init_data.flag_hardware_exists.adc_micro_thruster = 1;            % [boolean]
    sc_body_init_data.flag_hardware_exists.adc_reaction_wheel_assembly = 1;   % [boolean]
    sc_body_init_data.flag_hardware_exists.navigation_camera = 1;             % [boolean]
    sc_body_init_data.flag_hardware_exists.navigation_chemical_thruster = 1;  % [boolean]
    sc_body_init_data.flag_hardware_exists.navigation_sep_thruster = 0;       % [boolean]
    sc_body_init_data.flag_hardware_exists.power_solar_panel = 1;             % [boolean]
    sc_body_init_data.flag_hardware_exists.power_battery = 1;                 % [boolean]
    sc_body_init_data.flag_hardware_exists.power_pdc = 1;                     % [boolean]
    sc_body_init_data.flag_hardware_exists.dte_communication = 1;             % [boolean]
    sc_body_init_data.flag_hardware_exists.intersat_communication = 0;        % [boolean]
    sc_body_init_data.flag_hardware_exists.science_altimeter = 0;             % [boolean]
    sc_body_init_data.flag_hardware_exists.science_radar = 0;                 % [boolean]
    sc_body_init_data.flag_hardware_exists.science_mlps = 1;                  % [boolean]
    sc_body_init_data.flag_hardware_exists.science_xrs = 1;                   % [boolean]
    
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
    SC_pos_vel_t0 = cspice_spkezr(sc_body_init_data.spice_name, mission_true_time.date, 'J2000', 'NONE', 'SUN');
    sc_body_init_data.position = SC_pos_vel_t0(1:3);  % [km]
    sc_body_init_data.velocity = SC_pos_vel_t0(4:6);  % [km/sec]
    
    % SB centered - J2000 frame (inertial)
    SC_pos_vel_t0_SBcentered = cspice_spkezr(sc_body_init_data.spice_name,mission_true_time.date, 'J2000', 'NONE', mission_true_small_body.spice_name);
    sc_body_init_data.position_relative_SB = SC_pos_vel_t0_SBcentered(1:3);  % [km]
    sc_body_init_data.velocity_relative_SB = SC_pos_vel_t0_SBcentered(4:6);  % [km/sec]
    
    sc_body_init_data.flag_use_precomputed_spice_trajectory = 0;  % [boolean] : 1 = Use SPICE trajectory, else compute ODE dynamics
    sc_body_init_data.spice_filename = '../MuSCAT_Supporting_Files/SC_data/traj_daresim_simple.bsp';  % [string] : SC's SPICE FileName
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initial Attitude
    sc_body_init_data.SC_MRP_init = [0.1, 0.2, 0.3]';  % MRP
    sc_body_init_data.SC_omega_init = [0, 0, 0]';      % [rad/sec]
    
    sc_body_init_data.SC_e_init = sc_body_init_data.SC_MRP_init / norm(sc_body_init_data.SC_MRP_init);
    sc_body_init_data.SC_Phi_init = 4 * atand(sc_body_init_data.SC_MRP_init(1)/sc_body_init_data.SC_e_init(1));  % [deg]
    sc_body_init_data.SC_beta_v_init = sc_body_init_data.SC_e_init * sind(sc_body_init_data.SC_Phi_init/2);
    sc_body_init_data.SC_beta_4_init = cosd(sc_body_init_data.SC_Phi_init/2);
    
    sc_body_init_data.attitude = [sc_body_init_data.SC_beta_v_init; sc_body_init_data.SC_beta_4_init];
    sc_body_init_data.attitude = sc_body_init_data.attitude / norm(sc_body_init_data.attitude);
    sc_body_init_data.angular_velocity = sc_body_init_data.SC_omega_init;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Attitude controller tuning
    sc_body_init_data.control_gain_MT = [1, 0.2];   % [Kr; Lambda_r] for MT control
    sc_body_init_data.control_gain_RWA = [0.1, 1];  % [Kr; Lambda_r] for RWA control
    sc_body_init_data.actuator_selection_threshold_torque = 0.05;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % External disturbance
    sc_body_init_data.enable_SRP = 1;  % enable Solar Radiation Pressure disturbance torque and force of SC
    sc_body_init_data.enable_G2  = 1;  % enable Gravity Gradient disturbance torque
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Battery
    sc_body_init_data.num_battery = 2;
    sc_body_init_data.battery_maximum_capacity = [40, 40];  % [W * hrs]
    sc_body_init_data.charging_efficiency = [0.96, 0.96];
    sc_body_init_data.discharging_efficiency = [0.96, 0.96];
    sc_body_init_data.battery_instantaneous_power_consumption = [1e-4, 1e-4];
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Onboard clock
    sc_body_init_data.num_onboard_clock = 1;
    sc_body_init_data.onboard_clock_orientation = [1, 0, 0];
    sc_body_init_data.onboard_clock_location = [0, 0, 0];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Onboard memory
    sc_body_init_data.num_memory = 1;
    sc_body_init_data.memory_instantaneous_power_consumption = [0.1];  % [W]
    sc_body_init_data.memory_maximum_data_storage = (1e3) * [1];       % [Gb]
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Antennas
    sc_body_init_data.number_antenna = 1;
    sc_body_init_data.antenna_type = ["dipole"];       % antenna type
    sc_body_init_data.antenna_gain = [28.1];           % [dB] Gain of DTE antenna SC
    sc_body_init_data.antenna_axis = [[0, 1, 0]];      % [unit vector] antenna physical axis in Body frame
    sc_body_init_data.antenna_pointing = [[0, 0, 1]];  % [unit vector] antenna pointing direction for nominal gain in Body frame
    sc_body_init_data.antenna_frequency = [8450];      % [MHz]
    sc_body_init_data.tx_power = [50];                 % [W]
    sc_body_init_data.tx_line_loss = [1];              % [dB]
    sc_body_init_data.SC_noise_temperature = [100];    % [K]
    sc_body_init_data.SC_energy_bit_required = [4.2];  % [dB]
    sc_body_init_data.SC_coding_gain = [7.3];          % [dB]
    sc_body_init_data.SC_beamwidth = [0.1];            % [MHz]
    sc_body_init_data.SC_maximum_data_rate = [1000];   % [kbps]
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Communication (general)
    sc_body_init_data.communication_interlocutor = [-1];            % -1 : DTE / n>0 : index of spacecraft to communicate with
    sc_body_init_data.communication_direction = [-1];               % -1 : send / 1 : receive / 0 : bidirectionnal
    sc_body_init_data.communication_needs_attitude_pointing = [1];  % [boolean] define if SC needs to orient itself before communication to maximize antenna gain
    sc_body_init_data.communication_use_antenna_index = [1];        % index of antenna to use for each communication
    sc_body_init_data.minimum_required_link_margin = 3;             % [dB] required minimum link margin
    sc_body_init_data.compute_data_rate = 1;                        % [boolean] 1= compute data rate / otherwise assume 'given_data_rate'
    sc_body_init_data.given_data_rate = 0;                          % [kbps] data rate assumed if 'compute_data_rate' = 0
    
    %% Sensors
    % Sun Sensor
    if sc_body_init_data.flag_hardware_exists.adc_sun_sensor == 1
        sc_body_init_data.num_sun_sensor = 1;
        sc_body_init_data.sun_sensor_location = [[0, 0, 0]];
        sc_body_init_data.sun_sensor_orientation = [[1, 0, 0]];
        sc_body_init_data.sun_sunsor_instantaneous_power_consumption = 36e-3;      % [W] https://www.cubesatshop.com/wp-content/uploads/2016/06/SSOCA60-Technical-Specifications.pdf
        sc_body_init_data.sun_sunsor_instantaneous_data_volume = (4 + 1) * 16e-3;  % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        sc_body_init_data.sun_sunsor_measurement_noise = deg2rad(0.5);             % [rad] 0.5 degrees
        sc_body_init_data.sun_sunsor_measurement_frequency = 10;                   % [Hz]
        mission_true_SC{i_SC}.true_SC_sun_sensor = True_SC_Sun_Sensor(mission_true_time, sc_body_init_data);
    end
    
    % Star Tracker
    if sc_body_init_data.flag_hardware_exists.adc_star_tracker == 1
        sc_body_init_data.num_star_tracker = 1;
        sc_body_init_data.star_tracker_location = [[0, 0, 0]];
        sc_body_init_data.star_tracker_orientation = [[1, 0, 0]];
        sc_body_init_data.star_tracker_instantaneous_power_consumption = 1.5;        % [W] https://www.bluecanyontech.com/static/datasheet/BCT_DataSheet_Components_StarTrackers.pdf
        sc_body_init_data.star_tracker_instantaneous_data_volume = (4 + 1) * 16e-3;  % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        sc_body_init_data.star_tracker_measurement_noise = 2e-4;                     % [rad] 0.5 degrees
        sc_body_init_data.star_tracker_measurement_frequency = 10;                   % [Hz]sc_body_init_data.sun_sunsor_instantaneous_power_consumption = 36e-3; % [W] https://www.cubesatshop.com/wp-content/uploads/2016/06/SSOCA60-Technical-Specifications.pdf
        mission_true_SC{i_SC}.true_SC_star_tracker_sensor = True_SC_Star_Tracker_Sensor(mission_true_time, sc_body_init_data);
    end
    
    % IMU Sensor
    if sc_body_init_data.flag_hardware_exists.adc_imu == 1
        sc_body_init_data.num_imu_sensor = 1;
        sc_body_init_data.imu_sensor_location = [[0, 0, 0]];
        sc_body_init_data.imu_sensor_orientation = eye(3);
        sc_body_init_data.imu_sensor_instantaneous_power_consumption = 0.6;        % [W] https://www.micro-a.net/imu-tmpl.html
        sc_body_init_data.imu_sensor_instantaneous_data_volume = (4 + 1) * 16e-3;  % [kb] : 4 quaternion + 1 time vector, each of 16-bit depth
        sc_body_init_data.imu_sensor_measurement_noise = 9.7e-5;                   % [rad/sec]
        sc_body_init_data.imu_sensor_measurement_frequency = 10;                   % [Hz] between to measurement
        mission_true_SC{i_SC}.true_SC_imu_sensor = True_SC_IMU_Sensor(mission_true_time, sc_body_init_data);
    end
    
    % Camera
    if sc_body_init_data.flag_hardware_exists.navigation_camera == 1
        % Note: True_SC_Camera_Sensor only supports a single camera
        % NAC spec.: https://schneiderkreuznach.com/application/files/8016/4725/5615/PYRITE_56_120_10x_V38_1097277_datasheet.pdf
        % Note: While optics data sheet was provided, detector datasheet was not. Only specified APXS_M2020_CAM
        sc_body_init_data.num_camera = 1;
        sc_body_init_data.camera_instantaneous_power_consumption = [10];  % [W]
        sc_body_init_data.camera_measurement_frequency = 1/10;            % [Hz]
        sc_body_init_data.camera_angle_array = nac_camera_angle_array;
        sc_body_init_data.camera_location = nac_camera_location_array;
        sc_body_init_data.camera_measurement_noise = [0];
        sc_body_init_data.camera_resolution = [5120, 3840];               % [x y] pixel
        sc_body_init_data.camera_FOV = [21];                              % [deg]
        sc_body_init_data.camera_flag_show_stars = [1];                   % [boolean] : show star. slow for WFOV
        sc_body_init_data.flag_show_camera_plot=[0];                      % [boolean] : show plot during loop
        mission_true_SC{i_SC}.true_SC_camera_sensor = True_SC_Camera_Sensor(sc_body_init_data, mission_true_time);
    end
    
    %%%%%%% True component
    mission_true_SC{i_SC}.true_SC_body = True_SC_Body(mission_true_time, i_SC, sc_body_init_data);
    mission_true_SC{i_SC}.true_SC_navigation = True_SC_Navigation(mission_true_time, sc_body_init_data);
    mission_true_SC{i_SC}.true_SC_adc = True_SC_ADC(mission_true_time, sc_body_init_data, mission_true_SC{i_SC}.true_SC_body);
    mission_true_SC{i_SC}.true_SC_onboard_clock_sensor = True_SC_Onboard_Clock_Sensor(sc_body_init_data);
    mission_true_SC{i_SC}.true_SC_data_handling = True_SC_Data_handling(mission_true_time);
    mission_true_SC{i_SC}.true_SC_solar_panel = True_SC_Solar_Panel(sc_body_init_data);
    mission_true_SC{i_SC}.true_SC_power = True_SC_Power(mission_true_time);
    mission_true_SC{i_SC}.true_SC_gravity_gradient = True_SC_Gravity_Gradient(sc_body_init_data);
    mission_true_SC{i_SC}.true_SC_srp = True_SC_SRP(sc_body_init_data, mission_true_SC{i_SC}.true_SC_body);
    mission_true_SC{i_SC}.true_SC_battery = True_SC_Battery(sc_body_init_data, mission_true_time);
    mission_true_SC{i_SC}.true_SC_onboard_memory = True_SC_Onboard_Memory(mission_true_time, sc_body_init_data);
    mission_true_SC{i_SC}.true_SC_radio = True_SC_Radio(sc_body_init_data, mission_true_solar_system);
    mission_true_SC{i_SC}.true_SC_communication = True_SC_Communication(sc_body_init_data, mission_true_time);
    
    % DTE Communication
    if sc_body_init_data.flag_hardware_exists.dte_communication == 1
        sc_body_init_data.ground_station_antenna_gain = 90;          % [dB] gain of Earth receiver
        sc_body_init_data.ground_station_temperature_noise = 100;    % [K] temperature noise
        sc_body_init_data.ground_station_beamwidth = 0.1;            % [MHz] receiver beamwwidth
        sc_body_init_data.ground_station_Energy_bit_required = 4.2;  % [dB] Minimum energy bit required
        sc_body_init_data.ground_station_loss_pointing = 0;          % [dB] Loss due to pointing
        sc_body_init_data.ground_station_coding_gain = 7.3;          % [dB] Coding gain
        mission_true_SC{i_SC}.true_earth_dte_communication = True_Earth_DTE_communication(sc_body_init_data);
    end
    
    % Science altimeter instrument
    if sc_body_init_data.flag_hardware_exists.science_altimeter == 1
        sc_body_init_data.altimeter_power_consumption = 2;      % [W]
        sc_body_init_data.altimeter_data_volume = 0.1;          % [kb]
        sc_body_init_data.altimeter_measurement_frequency = 1;  % [Hz]
        sc_body_init_data.altimeter_orientation = [1, 0, 0]';   % [Unit vector]
        sc_body_init_data.altimeter_mesh_selection = 9;         % 1 = 500 points, 2 = 1000 points, 3 = 2500 points, 4 = 5000 points, 5 = 10000 points, 6 = 50000 points, 7 = 100000 points, 8 = 200000 points, 9 = 400000 points, 10 = 500000 points, 11 = 1000000 points, 12 = 50 points, 13 = 100 points, 14 = 200 points, 15 = 300 points, 16 = 400 points
        mission_true_SC{i_SC}.true_SC_altimeter = True_SC_Altimeter(sc_body_init_data, mission_true_time);
    end

    % Science radar instrument
    if sc_body_init_data.flag_hardware_exists.science_radar == 1
        sc_body_init_data.radar_power_consumption = 20;     % [W]
        sc_body_init_data.radar_data_volume = 20;           % [kb]
        sc_body_init_data.radar_measurement_frequency = 1;  % [Hz]
        sc_body_init_data.radar_orientation = [1, 0, 0]';   % [Unit vector]
        sc_body_init_data.radar_mesh_selection = 9;         % 1 = 500 points, 2 = 1000 points, 3 = 2500 points, 4 = 5000 points, 5 = 10000 points, 6 = 50000 points, 7 = 100000 points, 8 = 200000 points, 9 = 400000 points, 10 = 500000 points, 11 = 1000000 points, 12 = 50 points, 13 = 100 points, 14 = 200 points, 15 = 300 points, 16 = 400 points
        mission_true_SC{i_SC}.true_SC_radar = True_SC_Radar(...
            sc_body_init_data,...
            mission_true_time,...
            mission_true_SC{i_SC}.true_SC_navigation...
        );
    end

    % Science mid-/long-wave infrared point spectrometer (MLPS) instrument
    if sc_body_init_data.flag_hardware_exists.science_mlps == 1
        sc_body_init_data.mlps_power_consumption = 10;     % [W]
        sc_body_init_data.mlps_data_volume = 20;           % [kb]
        sc_body_init_data.mlps_measurement_frequency = 1;  % [Hz] TODO
        sc_body_init_data.mlps_location = [0, 0, 0];
        sc_body_init_data.mlps_orientation = [0, 0, 0];
        mission_true_SC{i_SC}.true_SC_mlps = True_SC_MLPS(mission_true_time, sc_body_init_data);
    end

    % Science x-ray spectrometer (XRS) instrument
    if sc_body_init_data.flag_hardware_exists.science_xrs == 1
        % XRS spec.: https://link.springer.com/content/pdf/10.1007/s11214-018-0483-8.pdf?pdf=button
        sc_body_init_data.xrs_power_consumption = 12.4;   % [W]
        sc_body_init_data.xrs_data_volume = 40;           % [kb]
        sc_body_init_data.xrs_measurement_frequency = 1;  % [Hz]
        sc_body_init_data.xrs_location = [0, 0, 0];
        sc_body_init_data.xrs_orientation = [0, 0, 0];
        mission_true_SC{i_SC}.true_SC_xrs = True_SC_XRS(mission_true_time, sc_body_init_data);
    end
    
    
   
    
    
    %% Actuators
    % Micro thruster actuator
    if sc_body_init_data.flag_hardware_exists.adc_micro_thruster == 1
        % HPGP MT spec.: https://www.ecaps.space/products-100mn.php
        sc_body_init_data.num_micro_thruster = num_micro_thruster;
        sc_body_init_data.micro_thruster_position_array = micro_thruster_position_array;
        sc_body_init_data.micro_thruster_direction_array = micro_thruster_direction_array;
        sc_body_init_data.micro_thruster_maximum_thrust = 100 * (1e-3) * ones(num_micro_thruster, 1);        % [N]
        sc_body_init_data.micro_thruster_thruster_noise = 100 * (1e-6) * ones(num_micro_thruster, 1);        % [N]
        sc_body_init_data.micro_thruster_instantaneous_power_consumption = 8 * ones(num_micro_thruster, 1);  % [W]
        sc_body_init_data.micro_thruster_instantaneous_data_volume = (1e-3) * 16;  % [kb]
        sc_body_init_data.micro_thruster_ISP = 209 * ones(num_micro_thruster, 1);  % [s]
        mission_true_SC{i_SC}.true_SC_micro_thruster_actuator = True_SC_Micro_Thruster_Actuator(...
            sc_body_init_data,...
            mission_true_time,...
            mission_true_SC{i_SC}.true_SC_body...
        );
    end
    
    % Reaction wheel assembly actuator
    if sc_body_init_data.flag_hardware_exists.adc_reaction_wheel_assembly == 1
        % % RW RW-0.2 spec.: https://www.rocketlabusa.com/assets/Uploads/RL-RW-0.4-Data-Sheet.pdf
        % % Additional spec.: https://www.rocketlabusa.com/assets/Uploads/400-MNMS-product-sheet.pdf
        rw_mass = 0.6;             % [kg]
        rw_radius = (103e-3) / 2;  % [m]
        sc_body_init_data.num_reaction_wheel = num_reaction_wheel;
        sc_body_init_data.rw_position = reaction_wheel_position_array;
        sc_body_init_data.rw_angle_array = reaction_wheel_angle_array;
        sc_body_init_data.rw_radius = rw_radius;
        sc_body_init_data.rw_mass = rw_mass;
        sc_body_init_data.rw_maximum_angular_velocity = 0.2 / (0.5 * rw_mass * rw_radius^2);  % [rad/s] 0.2 [N * m * s]
        sc_body_init_data.rw_angular_velocity_noise = 0;                % [rad/s] velocity noise
        sc_body_init_data.rw_instantaneous_data_volume = (3) * 16e-3;   % [kb] : velocity + health + temperature, each of 16-bit depth
        sc_body_init_data.rw_power_consumed_angular_velocity_array = [
            1e-3 * [180, 600, 6000]; 
            [0, 250*2*pi/60, sc_body_init_data.rw_maximum_angular_velocity]
        ]; % [power_array; velocity_array]
        mission_true_SC{i_SC}.true_SC_rwa_actuator = True_SC_RWA_Actuator(...
            sc_body_init_data,...
            mission_true_time,...
            mission_true_SC{i_SC}...
        );
    end
    
    % Chemical thruster actuator
    if sc_body_init_data.flag_hardware_exists.navigation_chemical_thruster == 1
        % SEP thruster spec.: https://www.space-propulsion.com/spacecraft-propulsion/propulsion-systems/electric-propulsion/index.html
        sc_body_init_data.num_chemical_thruster = 1;
        sc_body_init_data.chemical_thruster_position_array = sep_thruster_position;     % [m]
        sc_body_init_data.chemical_thruster_direction_array = sep_thruster_direction;  % [unit vector]
        sc_body_init_data.chemical_thruster_warmup_time = 0 * 30 * 60;                  % [s] Warmup for 30min before the DeltaV
        sc_body_init_data.chemical_thruster_warmup_power = 50;                          % [W] to heat the propellant
        sc_body_init_data.chemical_thruster_maximum_thrust = 25e-3;                     % [N]
        sc_body_init_data.chemical_thruster_minimum_thrust = 5e-3;                      % [N]
        sc_body_init_data.chemical_thruster_noise = 100e-6;                             % [N]
        sc_body_init_data.chemical_thruster_ISP = 200;                                  % [s]
        sc_body_init_data.chemical_thruster_instantaneous_data_volume = (1e-3) * 16;    % [Kb]
        sc_body_init_data.chemical_thruster_instantaneous_power_consumption = 145;      % [W]
        mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator = True_SC_Chemical_Thruster_Actuator(...
            mission_true_time,...
            sc_body_init_data...
        );
    end


    %% Software component
    mission_true_SC{i_SC}.software_SC_communication = Software_SC_Communication(...
        sc_body_init_data,...
        mission_true_SC{i_SC},...
        mission_true_time...
    );
    mission_true_SC{i_SC}.software_SC_executive = Software_SC_Executive(...
        mission_true_SC{i_SC}.true_SC_body,...
        mission_true_time,...
        mission_true_SC{i_SC}.software_SC_communication...
    );
    mission_true_SC{i_SC}.software_SC_estimate_attitude = Software_SC_Estimate_Attitude(...
        mission_true_SC{i_SC},...
        mission_true_time...
    );
    mission_true_SC{i_SC}.software_SC_estimate_SC_SB_orbit = Software_SC_Estimate_SC_SB_Orbit(...
        mission_true_SC{i_SC}.true_SC_body,...
        mission_true_SC{i_SC}.true_SC_navigation,...
        mission_true_time, mission_true_small_body,...
        mission_true_solar_system...
    );
    % mission_true_SC{i_SC}.software_SC_control_attitude = Software_SC_Control_Attitude(sc_body_init_data);
    mission_true_SC{i_SC}.software_SC_control_attitude = Software_SC_Control_Attitude(sc_body_init_data, mission_true_time);
    mission_true_SC{i_SC}.software_SC_control_orbit = Software_SC_Control_Orbit(...
        mission_init_data,...
        mission_true_time,...
        sc_body_init_data,...
        mission_true_SC{i_SC}.software_SC_executive,...
        mission_true_small_body...
    );
    
    clear sc_body_init_data
    
end

%% Visualize Spacecrafts
flag_visualize_SC = 1;
flag_save_plot = 1;

if flag_visualize_SC
    func_visualize_spacecraft_DARE(mission_true_SC, mission_init_data, flag_save_plot)
end

%% Setup Storage
mission_storage = Storage();
mission_storage.true_time.time_step = 1;  % [sec] Use 0 to use the True_time.time_step value
mission_storage.plot_parameters.storage_save_plots = 0;
mission_storage.func_create_storage_data_structures( ...
    mission_true_time, ...
    mission_init_data, ...
    mission_true_SC);
mission_storage.func_update_storage_data_structures( ...
    mission_true_time, ...
    mission_true_small_body, ...
    mission_true_solar_system, ...
    mission_init_data, ...
    mission_true_SC);

% mission_storage = func_create_storage_data_structures(...
%     mission_storage,... 
%     mission_true_time,... 
%     mission_init_data,...
%     mission_true_SC...
% );
% mission_storage = func_update_storage_data_structures(...
%     mission_storage,...
%     mission_true_time,...
%     mission_true_small_body,...
%     mission_true_solar_system,...
%     mission_init_data,...
%     mission_true_SC...
% );

%% Execute Main File
save();
run main_v2.m

%% Save data
save([mission_init_data.output_folder 'mission_storage_', char(datetime('today')),'.mat'], 'mission_storage')
save([mission_init_data.output_folder 'all_data.mat'])

return

%% Plot Storage
mission_storage.plot_parameters.plot_SC_number = 1;
func_plot_storage_data_structures(mission_storage,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC);

%% Plot video
% mission_storage.plot_parameters.plot_type = 'camera'; % "DROID" / "coverage" / "camera" / "DART"
% func_make_video(mission_storage,mission_true_time,mission_init_data,mission_true_small_body,mission_true_stars,mission_true_solar_system,mission_true_SC);
