classdef True_Small_Body < handle
    %TRUE_SMALL_BODY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name % [string] Name of SB
        rotation_period % [sec] Period of one rotation
        rotation_rate % [rad/sec]
        gravity_filename % [string] Filename of the SB gravity field in a particular format
        gravity_field % SB's gravity field, computed from gravity_filename
        gravity_degree_harmonics % [integer] Degree harmonics of the gravity field
        shape_model % Shape model of SB
        shape_model_type % [string] Type of shape model
        %         – shape_model.Vertices [m] : Position of vertices
        %         – shape_model.Faces : Triplet of vertex indices define a face
        radius % [km] Radius of SB
        pole_RA % [deg] Right Ascension (RA) of SB s pole
        pole_Dec % [deg] Declination (DEC) of SB s pole
        prime_meridian % [deg] : Initial Prime Meridian angle of SB
        rotation_matrix_pole_RA_Dec % Rotation matrix of SB s pole due to RA, Dec only (dosent' change with time)
        rotation_matrix % Rotation matrix of SB's pole and prime meridian (changes with time)
        spice_filename % [string] SB s SPICE FileName
        spice_name % [string] SB's SPICE Name
        mu_SB % [km^3 sec^-2] SB's standard gravitational parameter μ = GM
        position_SB % [km] Current position of SB in inertial frame I
        velocity_SB % [km/sec] Current velocity of SB in inertial frame I
    end
    
    methods
        function obj = True_Small_Body(mission_init_data,mission_true_time)
            %TRUE_SMALL_BODY Construct an instance of this class
            
            path_body_data = '../MuSCAT_Supporting_Files/SB_data/';
            
            switch mission_init_data.small_body_type
                
                case 1 % Bennu
                    obj.name = 'Bennu';
                    obj.rotation_period = 4.296057*3600; % [sec] From https://en.wikipedia.org/wiki/101955_Bennu
                    
                    % Gravity Model
                    obj.gravity_filename = [path_body_data 'Bennu/bennu_harmonics_jpl5.txt'];
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    obj.gravity_degree_harmonics = 8;
                    
                    % Shape Model
                    % readObj.m from https://www.mathworks.com/matlabcentral/fileexchange/18957-readobj
                    obj_shape = readObj([path_body_data 'Bennu/bennu_g_06290mm_spc_obj_0000n00000_v008.obj']);
                    % other options are Bennu_OSIRIS-REx_2018.obj (2.6MB) and Bennu_OSIRIS-REx_2019.obj (7.2MB)
                    
                    obj.shape_model_type = 'trisurf';
                    obj.shape_model = [];
                    obj.shape_model.Vertices = obj_shape.v; % [m]
                    obj.shape_model.Faces = obj_shape.f.v;
                    
                    % SPICE
                    obj.spice_filename = [path_body_data 'Bennu/sb-101955-118.bsp'];
                    obj.spice_name = '2101955';
                    
                    % Pole Data
                    obj.pole_RA = 85.65; % [deg]
                    obj.pole_Dec = -60.17; % [deg]
                    obj.prime_meridian = 0; % [deg] (at t_init)
                    
                    
                case 2 % Apophis
                    obj.name = 'Apophis';
                    
                    obj.rotation_period = (360/3.155588020452885e+02)*86400; % [sec] From SPICE
                    
                    % Gravity Model
                    obj.gravity_filename = [path_body_data 'Apophis/Apophis_CMoffset.txt'];
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    obj.gravity_degree_harmonics = 8;
                    
                    % Shape Model
                    obj_shape = readObj([path_body_data 'Apophis/ApophisModel1.obj']);
                    obj.shape_model = [];
                    obj.shape_model_type = 'trisurf';
                    obj.shape_model.Vertices = obj.gravity_field.R * 1e3 * obj_shape.v; % [m]
                    obj.shape_model.Faces = obj_shape.f.v;
                    
                    % SPICE
                    obj.spice_filename = [path_body_data 'Apophis/apophis.bsp'];
                    obj.spice_name = '2099942';
                    
                    % Pole Data
                    obj.pole_RA = 250; % [deg]
                    obj.pole_Dec = -75; % [deg]
                    obj.prime_meridian = 0; % [deg] (at t_init)
                    
             
                case 3 % Toutatis
                    % Initialize Toutatis
                    obj.name = 'Toutatis';

                    % Rotation Rate
                    obj.rotation_period = 176*3600; % [sec] https://en.wikipedia.org/wiki/4179_Toutatis
                    obj.rotation_rate = 2*pi/obj.rotation_period; % [rad /sec]
                    warning('Toutatis is tumbling!') % http://abyss.uoregon.edu/~js/ast121/lectures/toutatis.html

                    % Gravity Model
                    obj.gravity_filename = 'Toutatis_CMoffset.txt';
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    obj.gravity_degree_harmonics = 8;
                    
                   

                    obj_shape = readObj([path_body_data 'Toutatis/4179toutatis.tab.obj']);
                    obj.shape_model = [];
                    obj.shape_model_type = 'trisurf';
                    obj.shape_model.Vertices = obj.gravity_field.R * 1e3 * obj_shape.v; % [m]
                    obj.shape_model.Faces = obj_shape.f.v;
                    


                    obj.spice_filename = [path_body_data 'Toutatis/2004179.bsp'];
                    obj.spice_name = '2004179';
                    
                    % Pole Data
                    obj.pole_RA = 0; % [deg]
                    obj.pole_Dec = 90*pi/180; % [deg]
                    obj.prime_meridian = 0; % [deg] (at t_init)

                    obj.mass_SB =  1.9e13;
                    obj.ode_options =  odeset( 'RelTol',1e-14,'AbsTol',1e-14);

                 
                case 4 % Itokawa
                    % Initialize Itokawa
                    obj.name = 'Itokawa';

                    % Rotation Rate
                    obj.rotation_period = 12.132*3600; % [sec] https://en.wikipedia.org/wiki/25143_Itokawa
                    obj.rotation_rate   = 2*pi/obj.rotation_period; % [rad /sec]

                    % Gravity Model
                    obj.gravity_filename = 'Itokawa_CMoffset.txt';
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    obj.gravity_degree_harmonics = 8;

                    % Shape Model
                    % Shape model from https://sbn.psi.edu/pds/shape-models/
                    % readObj.m from https://www.mathworks.com/matlabcentral/fileexchange/18957-readobj
                    obj_shape = readObj([path_body_data 'Itokawa/Itokawa_ver64q.tab.obj']);
                    obj.shape_model = [];
                    obj.shape_model_type = 'trisurf';
                    obj.shape_model.Vertices = obj.gravity_field.R * 1e3 * obj_shape.v; % [m]
                    obj.shape_model.Faces = obj_shape.f.v;

                    obj.spice_filename = [path_body_data 'Itokawa/2025143.bsp'];
                    obj.spice_name = '2025143';

                    % Pole Data
                    obj.pole_RA =90.53*pi/180; % [rad]
                    obj.pole_Dec =-66.30*pi/180; % [rad]
                    obj.prime_meridian = 0; % [deg] (at t_init)

                    obj.mass_SB =  3.51e10;
                    obj.ode_options =  odeset( 'RelTol',1e-14,'AbsTol',1e-14);

                case 5 % 1996HW1
                    % Initialize 1996 HW1 (8567)
                    obj.name = '1996HW1';

                    % Rotation Rate
                    obj.rotation_period = 8.762*3600; % [sec] https://echo.jpl.nasa.gov/asteroids/1996HW1/1996hw1.html
                    obj.rotation_rate = 2*pi/obj.rotation_period; % [rad /sec]
                 
                     % Gravity Model
                    obj.gravity_filename = '1996HW1_CMoffset.txt';
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    obj.gravity_degree_harmonics = 8;


                    
                    % Shape Model
                    % Shape model from https://sbn.psi.edu/pds/shape-models/
                    % readObj.m from https://www.mathworks.com/matlabcentral/fileexchange/18957-readobj
                     obj_shape = readObj([path_body_data '1996HW1/1996HW1_a8567.tab.obj']);
                    obj.shape_model = [];
                    obj.shape_model_type = 'trisurf';
                    obj.shape_model.Vertices = obj.gravity_field.R * 1e3 * obj_shape.v; % [m]
                    obj.shape_model.Faces = obj_shape.f.v;

                    
                     obj.spice_filename = [path_body_data 'Itokawa/2008567.bsp'];
                    obj.spice_name = '2008567';

                    % Pole Data
                    obj.pole_RA =281*pi/180; % [rad]
                    obj.pole_Dec =-30*pi/180; % [rad]
                    obj.prime_meridian = 0; % [deg] (at t_init)

                    obj.mass_SB =  3.51e10; % I CANT FIND IT 
                    obj.ode_options =  odeset( 'RelTol',1e-14,'AbsTol',1e-14);

                case 6 % Earth

                    obj.name = 'Earth';

                    obj.rotation_period = 23.9345 * 3600; % [sec] From https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html

                    % Gravity Model (https://www2.csr.utexas.edu/grace/gravity/ggm02/)
                    obj.gravity_filename = [path_body_data 'Earth/ggm02c.txt'];
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    if isfield(mission_init_data,'gravity_degree_harmonics')
                        obj.gravity_degree_harmonics = mission_init_data.gravity_degree_harmonics;
                    else
                        obj.gravity_degree_harmonics = 8;
                    end
                    
                    % Shape Model
                    obj.shape_model = [];
                    obj.shape_model_type = 'sphere';
                    obj.shape_model.img = [path_body_data 'Earth/earth_surface.jpg'];
                    
                    % SPICE
                    obj.spice_name = '399';
                    obj.spice_filename = [path_body_data 'Earth/earth_200101_990825_predict.bpc'];
                    
                    % Pole Data
                    % From https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
                    ref_date_time = cal2sec('01-JAN-2000 00:12:00');
                    T = (mission_true_time.t_initial_date - ref_date_time) / (365.525*86400*100); % Julian centuries from reference date
                    obj.pole_RA = 0.00 - 0.641 * T; % [deg]
                    obj.pole_Dec = 90.00 - 0.557 * T; % [deg]
                    
                    cspice_furnsh([path_body_data 'Earth/naif0012.tls']) % Leapseconds kernel file
                    JD_UTC = cspice_et2utc(mission_true_time.t_initial_date, 'J', 6); % Format 'JD 2446533.18834276'
                    JD_UTC = strsplit(JD_UTC, ' ');
                    JD_UTC = str2double(JD_UTC{2});
                    JD_UT1 = JD_UTC; % Approximation, UTC is design to follow UT1 within +/- 0.9s
                    % From NASA TP 20220014814, Sec 4.3.2 Sidereal Motion
                    obj.prime_meridian = rad2deg(wrapTo2Pi(2*pi*(0.7790572732640 + 1.00273781191135448 * (JD_UT1 - 2451545.0)))); % [deg] (at t_init)
                case 7 
                    
                    obj.name = 'IBD_Asteroid';
                    
                    obj.rotation_period = (360/3.155588020452885e+02)*86400; % [sec] From SPICE
                    
                    % Gravity Model
                    obj.gravity_filename = [path_body_data 'IBDAst/Apophis_CMoffset.txt'];
                    obj.gravity_field = GravityField(obj.gravity_filename); % Load gravity field
                    obj.gravity_degree_harmonics = 8;
                    
                    % Shape Model
                    obj_shape = readObj([path_body_data 'IBDAst/Bennu-Radar.obj']);
                    obj.shape_model = [];
                    obj.shape_model_type = 'trisurf';
                    obj.shape_model.Vertices = obj.gravity_field.R * 1e3 * obj_shape.v; % [m]
                    obj.shape_model.Faces = obj_shape.f.v;
                    
                    % SPICE
                    obj.spice_filename = [path_body_data 'IBDAst/apophis.bsp'];
                    obj.spice_name = '2099942';
                    
                    % Pole Data
                    obj.pole_RA = 250; % [deg]
                    obj.pole_Dec = -75; % [deg]
                    obj.prime_meridian = 0; % [deg] (at t_init)

                
                    obj.mass_SB =  6.1e10;
                    obj.ode_options =  odeset( 'RelTol',1e-14,'AbsTol',1e-14);
             
                otherwise
                    error('Invalid small body type')
            end
            cspice_furnsh('../MuSCAT_Supporting_Files/SPICE/de440s.bsp');
            cspice_furnsh(obj.spice_filename);
            
            obj.rotation_rate = 360/obj.rotation_period; % [deg /sec]
            
            % Rotation Matrix to SB's Body-fixed Intertial Frame
            Rot_Z_pole_RA = [
                cosd(90 + obj.pole_RA) -sind(90 + obj.pole_RA) 0;
                sind(90 + obj.pole_RA)  cosd(90 + obj.pole_RA) 0;
                0 0 1];
            Rot_X_pole_Dec = [
                1 0 0;
                0 cosd(90 - obj.pole_Dec) -sind(90 - obj.pole_Dec);
                0 sind(90 - obj.pole_Dec)  cosd(90 - obj.pole_Dec)];
            
            obj.rotation_matrix_pole_RA_Dec = Rot_Z_pole_RA * Rot_X_pole_Dec;
            
            % Other Parameters
            obj.mu_SB = obj.gravity_field.GM; % [km^3 sec^-2]
            obj.radius = obj.gravity_field.R ; % [km]
            
            % Initialize position, velocity, and rotation matrix
            obj = func_update_SB_position_velocity_rot_matrix(obj, mission_true_time);
        end
        
        function obj = func_update_SB_position_velocity_rot_matrix(obj,mission_true_time)
            % Function to update position_SB and velocity_SB given current time
            
            SB_pos_vel_this_time = cspice_spkezr(obj.spice_name,mission_true_time.date,'J2000','NONE','SUN');
            obj.position_SB = SB_pos_vel_this_time(1:3);
            obj.velocity_SB = SB_pos_vel_this_time(4:6);
            
            % Small bodies
            obj.rotation_matrix = func_compute_rotation_matrix(obj, mission_true_time, mission_true_time.time);
            
            % Planets
            % obj.rotation_matrix = cspice_pxform('J2000', ['IAU_',obj.name], mission_true_time.date);
        end
        
        function rot = func_compute_rotation_matrix(obj, mission_true_time, t)
            theta_PM = obj.prime_meridian + obj.rotation_rate * (t - mission_true_time.t_initial);
            Rot_Z_PM = [
                cosd(theta_PM) sind(theta_PM) 0;
                -sind(theta_PM) cosd(theta_PM) 0;
                0 0 1];
            rot = Rot_Z_PM*obj.rotation_matrix_pole_RA_Dec;
        end
        
        function [rv, rot] = func_get_position_velocity_rot(obj, true_time, tspan)
            % Input:
            %   true_time: True time object
            %   tspan: Time span [s] (N x 1)
            % Output:
            %   rv: Position and velocity of SB in inertial frame [km, km/sec] (N x 6)
            %   rot: Rotation matrix of from inertial frame to the body-fixed frame (N x 3 x 3)
            
            rv = cspice_spkezr(obj.spice_name, true_time.t_initial_date + tspan, 'J2000', 'NONE', 'SUN')';
            if nargout > 1
                rot = zeros(length(tspan),3,3);
                for i = 1:length(tspan)
                    rot(i,:,:) = func_compute_rotation_matrix(obj, true_time, tspan(i));
                end
            end
        end
    end
end

