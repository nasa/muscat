classdef Software_SC_Control_Orbit < handle
    %CONTROL_ORBIT Summary of this class goes here
    %   Detailed explanation goes here

    properties
        % dynamic parameters
        time
        time_units
        date

        desired_control_DeltaV
        desired_control_DeltaV_units
        total_DeltaV_executed

        desired_control_thrust
        desired_control_thrust_units

        time_horizon
        time_intercept
        time_horizon_DeltaV
        time_DeltaV
        time_horizon_data_cutoff
        time_data_cutoff
        intercept_SB_position
        intercept_SB_velocity
        intercept_distance
        desired_intercept_distance
        options

        % boolean parameters
        desired_DeltaV_needs_to_be_executed
        desired_DeltaV_computed

        % desired trajectory
        desired_time_array
        desired_SC_pos_vel_current_SBcentered
        flag_position_velocity_burn
        desired_control_DeltaV_position_burn
        time_DeltaV_position_burn
        desired_control_DeltaV_velocity_burn
        time_DeltaV_velocity_burn

    end

    properties (SetAccess = private)
        % boolean parameters
        desired_attitude_for_DeltaV_achieved
    end

    methods

        function obj = Software_SC_Control_Orbit(mission_init_data, mission_true_time, sc_body_init_data, SC_executive, mission_true_small_body)
            %CONTROL_ORBIT Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = -inf;
            obj.time_units = 'sec';
            obj.desired_control_DeltaV_units = 'm/sec';
            obj.desired_control_thrust_units = 'N';

            obj.desired_control_thrust = 0; % [N]
            obj.options = odeset('RelTol',1e-14,'AbsTol',1e-14);

            obj.time_DeltaV = obj.time; % [sec]
            obj.time_horizon = 10*24*60*60; % [sec]
            obj.desired_DeltaV_needs_to_be_executed = 0;
            obj.desired_DeltaV_computed = 0;

            obj.intercept_distance = 0; % [km]
            obj.desired_intercept_distance = 0; % [km]
            obj.desired_control_DeltaV = zeros(3,1); % [m/sec]

            obj.time_horizon_DeltaV = 30*60; % [sec]
            obj.time_horizon_data_cutoff = 0*60*60; % [sec]

            if (mission_init_data.mission_type == 3) && (sc_body_init_data.flag_use_precomputed_spice_trajectory == 0)
                disp('Compute desired trajectory')

                SC_pos_vel_current_SBcentered = [sc_body_init_data.position_relative_SB; sc_body_init_data.velocity_relative_SB]; % [km, km/sec]

                control_force = [0 0 0]'; % [N]
                disturbance_force = [0 0 0]'; % [N]

                obj.desired_time_array = [mission_true_time.t_initial: (10*mission_true_time.time_step) :(mission_true_time.t_final + (2*obj.time_horizon_DeltaV) + obj.time_horizon_data_cutoff)];
                mission_true_small_body.gravity_degree_harmonics = 0;

                [T,X]=ode113(@(t,X) func_orbit_inertial_static_SB_centered(t,X, control_force, disturbance_force, SC_executive.mass.total_mass, mission_true_small_body), obj.desired_time_array, SC_pos_vel_current_SBcentered, obj.options);

                obj.desired_SC_pos_vel_current_SBcentered = X;
                obj.desired_time_array = obj.desired_time_array';

            end


        end

        function obj = func_estimate_SB_intercept_location_time(obj,SC_estimate_orbits)

            % Estimate SB orbit

            this_SB_pos_vel_current = [SC_estimate_orbits.SB_position; SC_estimate_orbits.SB_velocity]; % [km, km/sec]

            this_time_array = [0 : 60 : obj.time_horizon]; % [sec]

            Sun_pos_t0_tf = cspice_spkezr('10',obj.date+this_time_array,'J2000','NONE','10');

            [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,this_SB_pos_vel_current,obj.options);

            SB_pos_t0_tf = X';

            % Estimated SC orbit

            this_SC_pos_vel_current = [SC_estimate_orbits.SC_position; SC_estimate_orbits.SC_velocity]; % [km, km/sec]

            %             [T,X]=ode113(@(t,X) func_orbit_3rd_body(t,X, SC_estimate_orbits.mu_SB, SC_estimate_orbits.mu_Sun, SB_pos_t0_tf, Sun_pos_t0_tf, this_time_array),this_time_array,this_SC_pos_vel_current,obj.options);
            [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,this_SC_pos_vel_current,obj.options);

            SC_pos_t0_tf = X';

            Distance_array = vecnorm(SC_pos_t0_tf(1:3,:)' - SB_pos_t0_tf(1:3,:)',2,2);

            [M,I] = min(Distance_array);

            if I == 1
                % the orbits are diverging
                obj.time_intercept = obj.time_horizon/5 + obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff; % [sec]

            elseif (I > 1) && (I < length(Distance_array))
                % minima within 2 days
                obj.time_intercept = this_time_array(I); % [sec]

                if obj.time_intercept < (obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff)
                    obj.time_intercept = obj.time_horizon/5 + obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff; % [sec]
                end

            else
                % orbits are converging very slowly
                obj.time_intercept = obj.time_horizon; % [sec]

            end

            [minValue,closestIndex] = min(abs(this_time_array - obj.time_intercept));

            % Final Values
            obj.time_intercept = obj.time + this_time_array(closestIndex); % [sec] since t0
            obj.intercept_SB_position = SB_pos_t0_tf(1:3,closestIndex); % [km]
            obj.intercept_SB_velocity = SB_pos_t0_tf(4:6,closestIndex); % [km/sec]
            obj.intercept_distance = Distance_array(closestIndex); % [km]

        end

        %         function obj = func_compute_TCM_Lambert_Battin(obj,SC_estimate_orbits)
        %
        %             r1 = 1e3*SC_estimate_orbits.SC_position;  % [m]
        %             r2 = 1e3*obj.intercept_SB_position; % [m]
        %             tof = obj.time_intercept - obj.time;
        %             [V1, V2] = LAMBERTBATTIN(r1, r2, 'pro', tof);
        %
        %             obj.desired_control_DeltaV = V1' - 1e3*SC_estimate_orbits.SC_velocity; % [m/sec]
        %
        %             % Estimate SB orbit
        %             this_SB_pos_vel_current = [SC_estimate_orbits.SB_position; SC_estimate_orbits.SB_velocity]; % [km, km/sec]
        %             this_time_array = [0 : 10 : obj.time_intercept - obj.time]; % [sec]
        %             Sun_pos_t0_tf = cspice_spkezr('10',obj.date+this_time_array,'J2000','NONE','10');
        %             [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,this_SB_pos_vel_current,obj.options);
        %             SB_pos_t0_tf = X';
        %
        %             % Estimated SC orbit
        %             new_this_SC_pos_vel_current = [SC_estimate_orbits.SC_position; SC_estimate_orbits.SC_velocity + (1e-3*obj.desired_control_DeltaV)]; % [km, km/sec]
        %             %             [T,X]=ode113(@(t,X) func_orbit_3rd_body(t,X, SC_estimate_orbits.mu_SB, SC_estimate_orbits.mu_Sun, SB_pos_t0_tf, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.options);
        %             [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.options);
        %             SC_pos_t0_tf = X';
        %
        %             new_intercept_distance = norm(SC_pos_t0_tf(1:3,end) - SB_pos_t0_tf(1:3,end))
        %             obj.desired_intercept_distance = new_intercept_distance; % [km]
        %
        %             obj.time_DeltaV = obj.time; % [sec] since t0
        %             obj.time_data_cutoff = obj.time + obj.time_horizon_data_cutoff; % [sec] since t0
        %             obj.desired_DeltaV_needs_to_be_executed = 1;
        %             obj.desired_DeltaV_computed = 1;
        %             obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
        %             obj.desired_attitude_for_DeltaV_achieved = 0;
        %
        %         end

        function obj = func_compute_TCM_Lambert_Battin_v2(obj,SC_estimate_orbits)

            % obj.time_horizon_DeltaV = 300; % [sec]

            % Estimate SB orbit
            this_SB_pos_vel_current = [SC_estimate_orbits.SB_position; SC_estimate_orbits.SB_velocity]; % [km, km/sec]
            this_time_array = [0 : 10 : obj.time_horizon_DeltaV+obj.time_horizon_data_cutoff]; % [sec]
            Sun_pos_t0_tf = cspice_spkezr('10',obj.date+this_time_array,'J2000','NONE','10');
            %             [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,this_SB_pos_vel_current,obj.options);
            %             SB_pos_t0_tf = X';

            % Estimated SC orbit
            new_this_SC_pos_vel_current = [SC_estimate_orbits.SC_position; SC_estimate_orbits.SC_velocity]; % [km, km/sec]
            %             [T,X]=ode113(@(t,X) func_orbit_3rd_body(t,X, SC_estimate_orbits.mu_SB, SC_estimate_orbits.mu_Sun, SB_pos_t0_tf, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.options);
            [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.options);
            SC_pos_t_DeltaV = X(end,:)';


            % r1 = 1e3*SC_estimate_orbits.SC_position;  % [m]
            r1 = 1e3*SC_pos_t_DeltaV(1:3);  % [m]
            r2 = 1e3*obj.intercept_SB_position; % [m]
            tof = obj.time_intercept - obj.time - obj.time_horizon_DeltaV - obj.time_horizon_data_cutoff;
            [V1, V2] = LAMBERTBATTIN(r1, r2, 'pro', tof); 

            obj.desired_control_DeltaV = V1' - 1e3*SC_pos_t_DeltaV(4:6); % [m/sec]
            
            % Estimate SB orbit
            this_SB_pos_vel_current = [SC_estimate_orbits.SB_position; SC_estimate_orbits.SB_velocity]; % [km, km/sec]
            this_time_array = [0 : 10 : obj.time_intercept - obj.time]; % [sec]
            Sun_pos_t0_tf = cspice_spkezr('10',obj.date+this_time_array,'J2000','NONE','10');
            [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),this_time_array,this_SB_pos_vel_current,obj.options);
            SB_pos_t0_tf = X';

            % Estimated SC orbit
            new_this_time_array = [obj.time_horizon_DeltaV+obj.time_horizon_data_cutoff : 10 : obj.time_intercept - obj.time]; % [sec]
            new_this_SC_pos_vel_current = [SC_pos_t_DeltaV(1:3); SC_pos_t_DeltaV(4:6) + (1e-3*obj.desired_control_DeltaV)]; % [km, km/sec]
            %             [T,X]=ode113(@(t,X) func_orbit_3rd_body(t,X, SC_estimate_orbits.mu_SB, SC_estimate_orbits.mu_Sun, SB_pos_t0_tf, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.options);
            [T,X]=ode113(@(t,X) func_orbit_SB_body(t,X, SC_estimate_orbits.mu_Sun, Sun_pos_t0_tf, this_time_array),new_this_time_array,new_this_SC_pos_vel_current,obj.options);
            SC_pos_tDeltaV_tf = X';

            new_intercept_distance = norm(SC_pos_tDeltaV_tf(1:3,end) - SB_pos_t0_tf(1:3,end))
            obj.desired_intercept_distance = new_intercept_distance; % [km]

            obj.time_DeltaV = obj.time + obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff; % [sec] since t0
            obj.time_data_cutoff = obj.time + obj.time_horizon_data_cutoff; % [sec] since t0
            obj.desired_DeltaV_needs_to_be_executed = 1;
            obj.desired_DeltaV_computed = 1;
            obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
            obj.desired_attitude_for_DeltaV_achieved = 0;

            obj.flag_position_velocity_burn = 2;

        end

        function obj = func_compute_circular_orbit(obj)

            obj.desired_DeltaV_needs_to_be_executed = 0;
            obj.desired_control_DeltaV = zeros(3,1);

        end

        function obj = func_compute_distance_desired_circular_trajectory(obj,SC_estimate_orbits)

            this_SB_pos_vel_current = [SC_estimate_orbits.SC_position_relative_SB; SC_estimate_orbits.SC_velocity_relative_SB]; % [km, km/sec]

            this_desired_SC_pos_vel_current_SBcentered = (interp1(obj.desired_time_array,obj.desired_SC_pos_vel_current_SBcentered,obj.time))';

            obj.intercept_distance = norm(this_SB_pos_vel_current(1:3) - this_desired_SC_pos_vel_current_SBcentered(1:3)); % [km]
        end

        function obj = func_compute_Lambert_intercept_desired_circular_trajectory(obj, SC_estimate_orbits, mission_true_small_body, SC_executive)

            % Estimated SC orbit at obj.time_horizon_DeltaV+obj.time_horizon_data_cutoff

            this_SB_pos_vel_current_SBcentered = [SC_estimate_orbits.SC_position_relative_SB; SC_estimate_orbits.SC_velocity_relative_SB]; % [km, km/sec]

            this_time_array = [0 : SC_executive.time_step : obj.time_horizon_DeltaV+obj.time_horizon_data_cutoff]; % [sec]

            control_force = [0 0 0]'; % [N]
            disturbance_force = [0 0 0]'; % [N]

            mission_true_small_body.gravity_degree_harmonics = 0;

            [T,X]=ode113(@(t,X) func_orbit_inertial_static_SB_centered(t,X, control_force, disturbance_force, SC_executive.mass.total_mass, mission_true_small_body), this_time_array, this_SB_pos_vel_current_SBcentered, obj.options);

            SC_pos_t_DeltaV = X(end,:)';

            this_desired_SC_pos_vel_current_SBcentered = (interp1(obj.desired_time_array,obj.desired_SC_pos_vel_current_SBcentered, (obj.time + (2*obj.time_horizon_DeltaV) + obj.time_horizon_data_cutoff) ))';

            % Lambert maneuver
            r1 = 1e3*SC_pos_t_DeltaV(1:3);  % [m]
            r2 = 1e3*this_desired_SC_pos_vel_current_SBcentered(1:3); % [m]
            tof = obj.time_horizon_DeltaV;
            % [V1, V2] = LAMBERTBATTIN(r1, r2, 'pro', tof);
            [V1, V2] = LAMBERTBATTIN_v2(r1, r2, 'pro', tof, (1e9*mission_true_small_body.mu_SB));

            obj.desired_control_DeltaV_position_burn = V1' - 1e3*SC_pos_t_DeltaV(4:6); % [m/sec]
            obj.time_DeltaV_position_burn = obj.time + obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff; % [sec] since t0

            obj.desired_control_DeltaV_velocity_burn = 1e3*this_desired_SC_pos_vel_current_SBcentered(4:6) - V2'; % [m/sec]
            obj.time_DeltaV_velocity_burn = obj.time + (2*obj.time_horizon_DeltaV) + obj.time_horizon_data_cutoff; % [sec] since t0
            
            % Values to use in code
            obj.flag_position_velocity_burn = 1;
            obj.desired_intercept_distance = 0; % [km]
            obj.desired_control_DeltaV = obj.desired_control_DeltaV_position_burn;
            obj.time_DeltaV = obj.time_DeltaV_position_burn; % [sec] since t0
            obj.time_data_cutoff = obj.time + obj.time_horizon_data_cutoff; % [sec] since t0
            obj.desired_DeltaV_needs_to_be_executed = 1;
            obj.desired_DeltaV_computed = 1;
            obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
            obj.desired_attitude_for_DeltaV_achieved = 0;

        end

        function obj = func_check_attitude_for_DeltaV(obj,SC_control_attitude,SC_estimate_attitude)

            obj.desired_attitude_for_DeltaV_achieved = 0;

            % Check desired attitude for DeltaV is achieved
            if (SC_control_attitude.desired_SC_attitude_mode == 3) && (norm(SC_control_attitude.desired_attitude - SC_estimate_attitude.attitude) <= 0.01) && (norm(SC_estimate_attitude.angular_velocity - SC_control_attitude.desired_angular_velocity) < 1e-3)
                obj.desired_attitude_for_DeltaV_achieved = 1;
            end

        end

        function obj = func_command_DeltaV(obj,SC_chem_thruster,SC_estimate_orbits,SC_executive)

            obj.desired_control_thrust = 0;

            if (obj.desired_attitude_for_DeltaV_achieved == 1) && (sum([SC_chem_thruster.chemical_thruster_data.health]) > 0) && (obj.time >= obj.time_DeltaV)

                if norm(obj.desired_control_DeltaV - obj.total_DeltaV_executed) > (sum([SC_chem_thruster.chemical_thruster_data.maximum_thrust])*SC_executive.time_step/SC_estimate_orbits.mass.total_mass)
                    obj.desired_control_thrust = sum([SC_chem_thruster.chemical_thruster_data.maximum_thrust]);
                else
                    obj.desired_control_thrust = norm(obj.desired_control_DeltaV - obj.total_DeltaV_executed)*SC_estimate_orbits.mass.total_mass/SC_executive.time_step;
                end

                obj.total_DeltaV_executed = obj.total_DeltaV_executed + (obj.desired_control_thrust*SC_executive.time_step/SC_estimate_orbits.mass.total_mass)*(obj.desired_control_DeltaV/norm(obj.desired_control_DeltaV));

            end

            if norm(obj.desired_control_DeltaV - obj.total_DeltaV_executed) < 1e-6
                obj.desired_DeltaV_needs_to_be_executed = 0;
            end

            % For position and velocity burn
            if (obj.desired_DeltaV_needs_to_be_executed == 0) && (obj.flag_position_velocity_burn < 2)
                obj.flag_position_velocity_burn = 2;
                obj.desired_control_DeltaV = obj.desired_control_DeltaV_velocity_burn;
                obj.time_DeltaV = obj.time_DeltaV_velocity_burn; % [sec] since t0
                obj.time_data_cutoff = obj.time + obj.time_horizon_data_cutoff; % [sec] since t0
                obj.desired_DeltaV_needs_to_be_executed = 1;
                obj.desired_DeltaV_computed = 1;
                obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
                obj.desired_attitude_for_DeltaV_achieved = 0;
            end

        end




    end
end

