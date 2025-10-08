%% Class: True_Target_Computed
% Tracks the main target body (uses integration based on dynamic equations of motions for updating position, velocity)

classdef True_Target_Computed < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        name % [string] Name of Target

        %% [ ] Properties: Variables Computed Internally

        rotation_period % [sec] Period of one rotation
        rotation_rate % [rad/sec]

        gravity_filename % [string] Filename of the Target gravity field in a particular format
        gravity_field % Target's gravity field, computed from gravity_filename
        gravity_degree_harmonics % [integer] Degree harmonics of the gravity field

        shape_model % Shape model of Target
        % – shape_model.Vertices [m] : Position of vertices
        % – shape_model.Faces : Triplet of vertex indices define a face
        shape_model_type % [string] Type of shape model
        radius % [km] Radius of Target

        pole_RA % [deg] Right Ascension (RA) of Target's pole
        pole_Dec % [deg] Declination (DEC) of Target's pole
        prime_meridian % [deg] : Initial Prime Meridian angle of Target
        rotation_matrix_pole_RA_Dec % Rotation matrix of Target's pole due to RA, Dec only (dosent' change with time)
        rotation_matrix % Rotation matrix of Target from Body Frame to J2000 (changes with time)

        spice_filename % [string] Target's SPICE FileName
        spice_name % [string] Target's SPICE Name

        mu % [km^3 sec^-2] Target's standard gravitational parameter μ = GM
        mass % [kg] Mass of Target

        position % [km] Current position of Target wrt Solar System Barycenter centered J2000
        velocity % [km/sec] Current velocity of Target wrt Solar System Barycenter centered J2000
        position_array % [km] Position array of Target wrt Solar System Barycenter centered J2000, corresponding to time array in mission.true_time.time_position_array
        prev_velocity % [km/sec] Previous velocity of Target wrt Solar System Barycenter centered J2000 at the previous time step
        acceleration % [km/sec^2] Current acceleration of Target wrt Solar System Barycenter centered J2000

        acceleration_SS_body % [km/sec^2] Current acceleration of Target wrt Solar System Barycenter centered J2000 due to Solar System bodies

        position_SPICE % [km] Current position of Target wrt Solar System Barycenter centered J2000
        velocity_SPICE % [km/sec] Current velocity of Target wrt Solar System Barycenter centered J2000
        prev_velocity_SPICE % [km/sec] Previous velocity of Target wrt Solar System Barycenter centered J2000 at the previous time step
        position_array_SPICE % [km] Position array of Target wrt Solar System Barycenter centered J2000, corresponding to time array in mission.true_time.time_position_array
        acceleration_SPICE % [km/sec^2] Current acceleration of Target wrt Solar System Barycenter centered J2000

        control_force % [N] : Control force vector that passes through Center of Mass of SB
        disturbance_force % [N] : Disturbance force vector that passes through Center of Mass of SB
        SRP_disturbance_force % [N] : SRP-only disturbance force vector that passes through Center of Mass of SB
        Yarkovsky_disturbance_force % [N] : Yarkovsky-only disturbance force vector that passes through Center of Mass of SB

        Bond_Albedo % Used for Yarkovsky and SRP calculations
        SRP_reflectance_factor % Used for SRP calculations
        Gamma % [J m^-2 K^-1 s^-0.5] Thermal inertia, Used for Yarkovsky calculations
        eps % Emissivity, Used for Yarkovsky calculations

        ode_options % : Options for Matlab%s ODE function odeset( RelTol',1e-14,'AbsTol',1e-14)

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_Target_Computed(init_data, mission)

            path_body_data = '../../MuSCAT_Supporting_Files/SB_data/';

            obj.name = init_data.target_name;

            switch obj.name

                case 'Apophis'
                    % obj.name = 'Apophis';

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
                    % obj.spice_filename = [path_body_data 'Apophis/apophis.bsp'];
                    % obj.spice_name = '2099942';
                    % Summary for: ../../../SB_data/Apophis/apophis.bsp
                    %
                    % Body: 2099942
                    %       Start of Interval (ET)              End of Interval (ET)
                    %       -----------------------------       -----------------------------
                    %       2027 JAN 01 00:00:00.000            2030 JAN 01 00:00:00.000


                    obj.spice_filename = [path_body_data 'Apophis/apophis_v2.bsp'];
                    obj.spice_name = '20099942';
                    % Summary for: ../../../SB_data/Apophis/apophis_v2.bsp
                    %
                    % Body: 20099942
                    %       Start of Interval (ET)              End of Interval (ET)
                    %       -----------------------------       -----------------------------
                    %       2025 AUG 07 00:00:00.000            2032 SEP 06 00:00:00.000


                    % Pole Data
                    obj.pole_RA = 250; % [deg]
                    obj.pole_Dec = -75; % [deg]
                    obj.prime_meridian = 0; % [deg] (at t_init)


                    obj.Bond_Albedo = 0.14;
                    obj.SRP_reflectance_factor = 1 + ((4/9)*obj.Bond_Albedo);
                    obj.Gamma = 600; % [J m^-2 K^-1 s^-0.5] Thermal inertia
                    obj.eps = 0.9; % Emissivity

                otherwise
                    error('Invalid Target type')
            end

            cspice_furnsh('../../MuSCAT_Supporting_Files/SPICE/de440s.bsp');
            cspice_furnsh(obj.spice_filename);

            obj.rotation_rate = 360/obj.rotation_period; % [deg /sec]

            % Rotation Matrix to Target's Body-fixed Intertial Frame.
            % See details here: https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/MATLAB/req/pck.html#Orientation%20Models%20used%20by%20PCK%20Software
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
            obj.mu = obj.gravity_field.GM; % [km^3 sec^-2]
            obj.radius = obj.gravity_field.R ; % [km]
            obj.mass = obj.mu/mission.true_solar_system.gravitational_constant; % [km]


            % Initialize position, velocity, and rotation matrix

            target_pos_vel_this_time = cspice_spkezr(obj.spice_name,mission.true_time.date,'J2000','NONE','SOLAR SYSTEM BARYCENTER');
            obj.position_SPICE = target_pos_vel_this_time(1:3)'; % [km]
            obj.velocity_SPICE = target_pos_vel_this_time(4:6)'; % [km/sec]
            obj.acceleration_SPICE = [0 0 0]; % [km/sec^2]
            obj.prev_velocity_SPICE = obj.velocity_SPICE; % [km/sec]

            target_pos_vel_array = (cspice_spkezr(obj.spice_name, mission.true_time.prev_date + mission.true_time.time_position_array' ,'J2000','NONE','SOLAR SYSTEM BARYCENTER'))';
            obj.position_array_SPICE = target_pos_vel_array(:,1:3); % [km]

            obj.position = obj.position_SPICE; % [km]
            obj.velocity = obj.velocity_SPICE; % [km/sec]
            obj.position_array = obj.position_array_SPICE; % [km]
            obj.acceleration = [0 0 0]; % [km/sec^2]
            obj.prev_velocity = obj.velocity; % [km/sec]

            obj.acceleration_SS_body = zeros(1,3*mission.true_solar_system.num_SS_body); % [km/sec^2]

            % Rotation Matrix
            obj.rotation_matrix = func_update_target_rotation_matrix(obj, mission);

            % Variables for Integration
            obj.ode_options = odeset('RelTol',1e-20,'AbsTol',1e-20);
            obj.control_force = [0 0 0]; % [N]
            obj.disturbance_force = [0 0 0]; % [N]
            obj.SRP_disturbance_force = [0 0 0]; % [N]
            obj.Yarkovsky_disturbance_force = [0 0 0]; % [N]


            % Initialize Variables to store position and velocity of target
            obj.store = [];

            obj.store.name = obj.name;
            obj.store.position = zeros(mission.storage.num_storage_steps, length(obj.position));
            obj.store.velocity = zeros(mission.storage.num_storage_steps, length(obj.velocity));
            obj.store.acceleration = zeros(mission.storage.num_storage_steps, length(obj.acceleration));
            obj.store.acceleration_SS_body = zeros(mission.storage.num_storage_steps, length(obj.acceleration_SS_body));

            obj.store.position_SPICE = zeros(mission.storage.num_storage_steps, length(obj.position_SPICE));
            obj.store.velocity_SPICE = zeros(mission.storage.num_storage_steps, length(obj.velocity_SPICE));
            obj.store.acceleration_SPICE = zeros(mission.storage.num_storage_steps, length(obj.acceleration_SPICE));

            obj.store.control_force = zeros(mission.storage.num_storage_steps, length(obj.control_force));
            obj.store.disturbance_force = zeros(mission.storage.num_storage_steps, length(obj.disturbance_force));
            obj.store.SRP_disturbance_force = zeros(mission.storage.num_storage_steps, length(obj.SRP_disturbance_force));
            obj.store.Yarkovsky_disturbance_force = zeros(mission.storage.num_storage_steps, length(obj.Yarkovsky_disturbance_force));

            obj = func_update_target_store(obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_target_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.position(mission.storage.k_storage,:) = obj.position; % [km]
                obj.store.velocity(mission.storage.k_storage,:) = obj.velocity; % [km/sec]
                obj.store.acceleration(mission.storage.k_storage,:) = obj.acceleration; % [km/sec^2]
                obj.store.acceleration_SS_body(mission.storage.k_storage,:) = obj.acceleration_SS_body; % [km/sec^2]

                obj.store.position_SPICE(mission.storage.k_storage,:) = obj.position_SPICE; % [km]
                obj.store.velocity_SPICE(mission.storage.k_storage,:) = obj.velocity_SPICE; % [km/sec]
                obj.store.acceleration_SPICE(mission.storage.k_storage,:) = obj.acceleration_SPICE; % [km/sec^2]

                obj.store.control_force(mission.storage.k_storage,:) = obj.control_force; % [N]
                obj.store.disturbance_force(mission.storage.k_storage,:) = obj.disturbance_force; % [N]
                obj.store.SRP_disturbance_force(mission.storage.k_storage,:) = obj.SRP_disturbance_force; % [N]
                obj.store.Yarkovsky_disturbance_force(mission.storage.k_storage,:) = obj.Yarkovsky_disturbance_force; % [N]
            end

        end


        %% [ ] Methods: Main
        % Update target's position, velocity, rotation matrix for current time

        function obj = func_main_true_target(obj,mission)

            % Pos and Velocity from SPICE
            target_pos_vel_this_time = cspice_spkezr(obj.spice_name,mission.true_time.date,'J2000','NONE','SOLAR SYSTEM BARYCENTER');
            obj.position_SPICE = target_pos_vel_this_time(1:3)'; % [km]
            obj.velocity_SPICE = target_pos_vel_this_time(4:6)'; % [km/sec]
            obj.acceleration_SPICE = (obj.velocity_SPICE - obj.prev_velocity_SPICE)/mission.true_time.time_step; % [km/sec^2]

            target_pos_vel_array = (cspice_spkezr(obj.spice_name, mission.true_time.prev_date + mission.true_time.time_position_array' ,'J2000','NONE','SOLAR SYSTEM BARYCENTER'))';
            obj.position_array_SPICE = target_pos_vel_array(:,1:3);

            % Rotation Matrix
            obj.rotation_matrix = func_update_target_rotation_matrix(obj, mission);

            % Update SRP Disturbance Force
            obj = func_update_SRP_disturbance_force(obj, mission);
            obj.disturbance_force = obj.disturbance_force + obj.SRP_disturbance_force; % [N]

            % Update Accleration due to Solar System Bodies
            obj= func_update_SS_Body_disturbance_force(obj, mission);
            
            % Update Yarkovsky Disturbance Force
            obj = func_update_Yarkovsky_disturbance_force(obj, mission);
            obj.disturbance_force = obj.disturbance_force + obj.Yarkovsky_disturbance_force; % [N]


            % Compute Pos and Velocity using Integration
            this_SB_pos_vel_t0 = [obj.position, obj.velocity];

            odefun =   @(t,X) func_orbit_inertial_static_SB_v3( t, X, obj, mission);

            [T,X]=ode15s(odefun, mission.true_time.time_position_array, this_SB_pos_vel_t0', obj.ode_options);

            obj.position = X(end,1:3); % [km]
            obj.velocity = X(end,4:6); % [km/sec]
            obj.acceleration = (obj.velocity - obj.prev_velocity)/mission.true_time.time_step; % [km/sec^2]

            if length(mission.true_time.time_position_array) == 2
                obj.position_array(1,1:3) = X(1,1:3); % [km]
                obj.position_array(2,1:3) = X(end,1:3); % [km]
            else
                obj.position_array = X(:,1:3); % [km]
            end

            %             obj.position = obj.position_SPICE; % [km]
            %             obj.velocity = obj.velocity_SPICE; % [km/sec]
            %             obj.position_array = obj.position_array_SPICE; % [km]

            % Store
            obj = func_update_target_store(obj, mission);

            % Reset Forces and Previous Velocity
            obj.control_force = [0 0 0]; % [N]
            obj.disturbance_force = [0 0 0]; % [N]
            
            obj.prev_velocity = obj.velocity; % [km/sec]
            obj.prev_velocity_SPICE = obj.velocity_SPICE; % [km/sec]

        end

        %% [ ] Methods: Update Rotation Matrix
        % Update target's rotation matrix for current time

        function rot = func_update_target_rotation_matrix(obj, mission)

            switch obj.name

                case 'Enceladus'
                    fromFrame = 'IAU_ENCELADUS'; % Body-fixed frame of Enceladus
                    toFrame = 'J2000';           % Inertial frame (e.g., J2000)

                    rot = cspice_pxform(fromFrame, toFrame, mission.true_time.date);

                otherwise
                    theta_PM = obj.prime_meridian + obj.rotation_rate * (mission.true_time.time - mission.true_time.t_initial);
                    Rot_Z_PM = [
                        cosd(theta_PM) sind(theta_PM) 0;
                        -sind(theta_PM) cosd(theta_PM) 0;
                        0 0 1];
                    rot = Rot_Z_PM*obj.rotation_matrix_pole_RA_Dec; % From J2000 to Body frame

                    rot = rot'; % From Body frame to J2000
            end
        end


        %% [ ] Methods: Update SRP Disturbance Force

        function obj = func_update_SRP_disturbance_force(obj, mission)

            R_Sun_to_SB = obj.position - mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position; % [km]
            distance_SB_Sun = norm(R_Sun_to_SB); % [km]
            SB_projected_area = pi*((obj.radius * 1e3)^2); % [m^2]
            
            % F_SRP_magnitude = (mission.true_solar_system.solar_constant_AU / mission.true_solar_system.light_speed) * ...
            % (mission.true_solar_system.AU_distance / distance_SB_Sun)^2 * SB_projected_area * (1 + obj.SRP_reflectance_factor); % [N]

            F_SRP_magnitude = obj.SRP_reflectance_factor * (mission.true_solar_system.solar_constant_AU / mission.true_solar_system.light_speed) * (mission.true_solar_system.AU_distance / distance_SB_Sun)^2 * SB_projected_area; % [N]

            obj.SRP_disturbance_force = F_SRP_magnitude * func_normalize_vec(R_Sun_to_SB); % [N]

        end


        %% [ ] Methods: Update Accleration due to Solar System Bodies

        function obj= func_update_SS_Body_disturbance_force(obj, mission)

            for i_SS_body = 1:1:mission.true_solar_system.num_SS_body

                R_SS_Body_to_Ast = (obj.position - mission.true_solar_system.SS_body{i_SS_body}.position); % [km]
                dist_SS_Body_to_Ast = norm(R_SS_Body_to_Ast); % [km]
                mu_SS_body = mission.true_solar_system.SS_body{i_SS_body}.mu; % [km^3 sec^-2]

                acc = (mu_SS_body * R_SS_Body_to_Ast / dist_SS_Body_to_Ast^3); % [km/sec^2]
                obj.acceleration_SS_body(1,(i_SS_body-1)*3+1:(i_SS_body-1)*3+3) = acc; % [km/sec^2]
            end

        end


        %% [ ] Methods: Update Yarkovsky Disturbance Force

        function obj = func_update_Yarkovsky_disturbance_force(obj, mission)

            % aY = yarkovsky_accel(r_vec, v_vec, R, rho, A_bond, eps, Gamma, omega, obliq_rad, muSun)

            % r_vec, v_vec: heliocentric state in SI (m, m/s)
            % R (m), rho (kg/m^3), A_bond (-), eps (-), Gamma (J m^-2 K^-1 s^-0.5)
            % omega (rad/s), obliq_rad (rad), muSun (m^3/s^2)
            % Returns aY (3x1) in m/s^2

            %==== Constants (SI where appropriate) ==================================

            S0 = mission.true_solar_system.solar_constant_AU; % [W/m^2] at 1 AU
            AU = mission.true_solar_system.AU_distance; % [km]
            c  = mission.true_solar_system.light_speed; % [m/s]
            sigma = mission.true_solar_system.sigma; % [W m^-2 K^-4] Stefan–Boltzmann constant
            muSun = mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu; % [km^3 sec^-2]

            %==== Obliquity ==================================            
            % Obliquity: angle between spin axis and orbit normal                      

            rv_SB = [obj.position, obj.velocity]';
            coe_SB = Rv2Coe(rv_SB, muSun);
            inc = coe_SB(3); % [rad]
            Omega = coe_SB(4); % [rad]

            % Orbit normal (equatorial)
            si = sin(inc); ci = cos(inc);
            sO = sin(Omega); cO = cos(Omega);
            n_hat = func_normalize_vec([si*sO; -si*cO; ci]);   

            % SB Spin unit vector (equatorial)
            alpha_deg = obj.pole_RA; % [deg]
            delta_deg = obj.pole_Dec; % [deg]

            ca = cosd(alpha_deg);  sa = sind(alpha_deg);
            cd = cosd(delta_deg);  sd = sind(delta_deg);
            s_hat = func_normalize_vec([cd*ca; cd*sa; sd]);

            % Obliquity
            cang = max(-1,min(1, dot(s_hat, n_hat) ));
            obliq_rad = acos(cang); % [rad] 
            
            omega = 2*pi/obj.rotation_period; % [rad/sec]

            %==== Geometry / kinematics (km units) =================================

            r_vec = obj.position - mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position; % [km]
            v_vec = obj.velocity - mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.velocity; % [km/sec]

            r = norm(r_vec); % [km]
            v = norm(v_vec); % [km/sec]
            that = v_vec / v; % along-track unit vector

            %==== Solar flux at current distance (unit-safe via AU ratio) ===========
            % Φ uses W/m^2; ratio (AU/r) is dimensionless even with km.

            Phi = S0 * (AU/r)^2; % [W/m^2] solar flux at r

            %==== Temperature scale and thermal parameters (SI) =====================
            % T* sets the scale; Θ_d uses ω, Θ_s uses n.
            Tstar = ((1 - obj.Bond_Albedo) * Phi / (obj.eps * sigma))^(1/4); % [K]
            R_m  = obj.radius * 1000; % [m] 
            rho = obj.mass/((4/3)*pi*(R_m^3)); % [kg/m^3] Bulk density 

            % Semi-major axis from vis-viva in km-consistent units:
            a = 1 / (2/r - (v^2)/muSun); % [km]
            n = sqrt(muSun / a^3); % [rad/sec]

            % Thermal parameters (dimensionless)
            Theta_d = (obj.Gamma * sqrt(omega)) / (obj.eps * sigma * Tstar^3);
            Theta_s = (obj.Gamma * sqrt(n))     / (obj.eps * sigma * Tstar^3);

            % Efficiency function W(Θ) with stable algebra
            W = @(Th) (0.5*Th) ./ (1 + Th + 0.5*Th.^2);

            %==== Along-track acceleration magnitude (SI -> then convert) ==========
            % Base coefficient yields acceleration in m/s^2 when R in m, rho in kg/m^3.
            coef_mps2 = (4/9) * ((1 - obj.Bond_Albedo) * Phi) / (c * rho * R_m);     % [m/s^2]

            a_t_diurnal_mps2  =  coef_mps2 * W(Theta_d) * cos(obliq_rad);
            a_t_seasonal_mps2 = -coef_mps2 * W(Theta_s) * sin(obliq_rad)^2;
            a_t_mps2          =  a_t_diurnal_mps2 + a_t_seasonal_mps2;

            % Direction and unit conversion to km/s^2
            aY_kmps2 = (a_t_mps2 / 1000) * that; % [km/sec^2]

            obj.Yarkovsky_disturbance_force = (1e3) * aY_kmps2 * obj.mass; % [N]

        end


    end
end

