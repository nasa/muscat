%% Class: Software_SC_Control_Orbit
% Control the Orbit of the Spacecraft

classdef Software_SC_Control_Orbit < handle
    % Software_SC_Control_Orbit: Manages spacecraft control parameters for orbiting small bodies.
    % Includes functionalities for trajectory calculations, intercept predictions,
    % and delta-V computations tailored for spacecraft mission scenarios.

    %% Properties
    properties

        data % Other useful data

        mode_software_SC_control_orbit_selector % [string] Different orbit control modes
        % - 'DART Oracle' : Directly change True_SC_ADC.attitude values for DART mission
        % - 'Nightingale Oracle' : Directly change True_SC_ADC.attitude values for Nightingale mission
        % - 'Oracle with Control' : Use True_SC_ADC values + Noise

        last_time_control % [s]
        flag_executive    % [bool]
        max_time_before_control % [s]

        % Maneuver tracking
        maneuver_start_time % [s] Time when current maneuver began execution
        thruster_fired_successfully % [bool] Flag to track if thruster actually fired

        % Delta-V control parameters
        desired_control_DeltaV % [m/s]: Desired Delta-V vector
        desired_control_DeltaV_units % [string]: Units for Delta-V
        total_DeltaV_executed % [m/s]: Accumulated executed Delta-V

        burn_duration % [s]

        % Thrust control parameters
        desired_control_thrust % [N]: Desired thrust for trajectory control
        desired_control_thrust_units % [string]: Units for thrust (e.g., N, kN)

        % Time horizons and intercept data
        time_horizon % [sec]: Time horizon for trajectory planning
        time_intercept % [sec]: Time of intercept with target
        time_horizon_DeltaV % [sec]: Time horizon for Delta-V execution
        time_DeltaV % [sec]: Time of Delta-V execution
        time_horizon_data_cutoff % [sec]: Time cutoff for data integration
        time_data_cutoff % [sec]: Data cutoff time

        % Intercept and trajectory details
        intercept_SB_position % [km]: Intercept position relative to small body
        intercept_SB_velocity % [km/s]: Intercept velocity relative to small body
        intercept_distance % [km]: Distance at intercept
        desired_intercept_distance % [km]: Desired distance at intercept

        % Options for numerical integrations
        options % [struct]: Options for ODE solvers

        % Flags and execution controls
        desired_DeltaV_needs_to_be_executed % [boolean]: Whether Delta-V needs execution
        desired_DeltaV_computed % [boolean]: Whether Delta-V is computed
        desired_attitude_for_DeltaV_achieved % [boolean]: Attitude flag for Delta-V execution
        desired_DeltaV_achieved % [boolean]: Whether Delta-V has been achieved

        % Desired trajectory details
        desired_time_array % [sec]: Array of planned trajectory times
        desired_SC_pos_vel_current_SBcentered % [km, km/s]: Desired spacecraft state in small body frame
        flag_position_velocity_burn % [int]: Indicator for position or velocity-based burns
        desired_control_DeltaV_position_burn % [m/s]: Position-based Delta-V
        time_DeltaV_position_burn % [sec]: Time for position-based burn
        desired_control_DeltaV_velocity_burn % [m/s]: Velocity-based Delta-V
        time_DeltaV_velocity_burn % [sec]: Time for velocity-based burn
        threshold_minimum_deltaV % [m/s] - the minimum deltaV that is worth executing, usually 0.001 m/s

        % Fuel management properties
        min_fuel_threshold % [kg]: Minimum fuel threshold to allow maneuvers
        estimated_fuel_required % [kg]: Estimated fuel required for maneuver
        flag_insufficient_fuel % [boolean]: Flag indicating insufficient fuel

        store % Structure to store historical data
    end

    %% Constructor
    methods
        function obj = Software_SC_Control_Orbit(init_data, mission, i_SC)
            % Initialize the spacecraft control orbit class

            obj.mode_software_SC_control_orbit_selector = init_data.mode_software_SC_control_orbit_selector;
            obj.max_time_before_control = init_data.max_time_before_control;

            obj.flag_executive = 0;
            obj.last_time_control = 0;

            obj.desired_control_thrust = 0; % Initialize thrust to zero
            obj.options = odeset('RelTol', 1e-14, 'AbsTol', 1e-14);

            obj.time_DeltaV = 0; % Initialize Delta-V time
            obj.time_horizon = 10 * 24 * 60 * 60; % Default planning horizon (10 days)

            obj.desired_DeltaV_needs_to_be_executed = false;
            obj.desired_DeltaV_computed = false;
            obj.desired_attitude_for_DeltaV_achieved = 0;


            obj.intercept_distance = 0; % Initialize intercept distance
            obj.desired_intercept_distance = 0; % Initialize desired intercept distance

            obj.desired_control_DeltaV = zeros(3, 1); % Initialize Delta-V vector
            obj.desired_DeltaV_achieved = false;

            obj.total_DeltaV_executed =  zeros(3, 1); % Initialize Delta-V vector

            obj.time_horizon_DeltaV = 30 * 60; % Default Delta-V planning horizon (30 minutes)
            obj.time_horizon_data_cutoff = 0; % No data cutoff initially

            obj.threshold_minimum_deltaV = 0.01;

            % Initialize fuel management properties
            obj.min_fuel_threshold = 0.1; % Minimum fuel threshold in kg
            obj.estimated_fuel_required = 0; % Estimated fuel required for maneuver
            obj.flag_insufficient_fuel = false; % Flag indicating insufficient fuel

            % Initialize Storage Variables
            obj.store = [];
            obj.store.intercept_distance = zeros(mission.storage.num_storage_steps_attitude,1); % Initialize as empty numeric array
            obj.store.desired_DeltaV_computed = zeros(mission.storage.num_storage_steps_attitude,1);
            obj.store.desired_attitude_for_DeltaV_achieved = zeros(mission.storage.num_storage_steps_attitude,1);

            obj.store.desired_control_DeltaV = zeros(mission.storage.num_storage_steps_attitude, 3);
            obj.store.total_DeltaV_executed = zeros(mission.storage.num_storage_steps_attitude, 3);
            obj.store.attitude_error = zeros(mission.storage.num_storage_steps_attitude, 3); % Euler angles error
            obj.store.deltaV_magnitude_desired = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.deltaV_magnitude_executed = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.flag_insufficient_fuel = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.estimated_fuel_required = zeros(mission.storage.num_storage_steps_attitude, 1);

            obj.desired_DeltaV_achieved = 0;
            obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
            obj.desired_attitude_for_DeltaV_achieved = 0;

            % Initialize maneuver tracking properties
            obj.maneuver_start_time = 0;
            obj.thruster_fired_successfully = false;
        end
    end

    %% Methods
    methods

        %% Update Storage
        function obj = reset_variables(obj)
            % Reset time and control flags
            %obj.last_time_control = 0;
            obj.flag_executive = 0;
            %obj.desired_DeltaV_needs_to_be_executed = false;
            %obj.desired_DeltaV_computed = false;
            %obj.desired_attitude_for_DeltaV_achieved = 0;
            %obj.desired_DeltaV_achieved = false;

            % Reset Delta-V and thrust parameters
            % obj.desired_control_DeltaV = zeros(3, 1);
            %obj.total_DeltaV_executed = zeros(3, 1);
            obj.desired_control_thrust = 0;

            % Reset intercept and trajectory details
            %obj.intercept_SB_position = zeros(3, 1);
            %obj.intercept_SB_velocity = zeros(3, 1);
            %obj.intercept_distance = 0;
            %obj.desired_intercept_distance = 0;
            %obj.time_intercept = 0;
            %obj.time_DeltaV = 0;
            %obj.time_data_cutoff = 0;

            % Reset fuel management
            obj.flag_insufficient_fuel = false;
        end

        function obj = func_update_software_SC_Control_Orbit_store(obj, mission)
            obj.store.intercept_distance(mission.storage.k_storage,:) = obj.intercept_distance;
            obj.store.desired_DeltaV_computed(mission.storage.k_storage,:) = obj.desired_DeltaV_computed;
            obj.store.desired_attitude_for_DeltaV_achieved(mission.storage.k_storage,:) = obj.desired_attitude_for_DeltaV_achieved;

            % New variables
            obj.store.desired_control_DeltaV(mission.storage.k_storage, :) = obj.desired_control_DeltaV;
            obj.store.total_DeltaV_executed(mission.storage.k_storage, :) = obj.total_DeltaV_executed;

            % % Compute attitude error
            % actual_quat = mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude;
            % desired_quat = mission.true_SC{i_SC}.software_SC_control_attitude.desired_attitude;
            % error_quat = quatmultiply(quatconj(actual_quat), desired_quat);
            % euler_error = quat2eul(error_quat, 'ZYX'); % Convert to Euler angles (rad)
            % obj.store.attitude_error(mission.storage.k_storage, :) = euler_error;

            % DeltaV magnitudes
            obj.store.deltaV_magnitude_desired(mission.storage.k_storage) = norm(obj.desired_control_DeltaV);
            obj.store.deltaV_magnitude_executed(mission.storage.k_storage) = norm(obj.total_DeltaV_executed);

            % Fuel management data
            obj.store.flag_insufficient_fuel(mission.storage.k_storage) = obj.flag_insufficient_fuel;
            obj.store.estimated_fuel_required(mission.storage.k_storage) = obj.estimated_fuel_required;
        end

        % This function computes the intercept position, velocity, and time for a spacecraft
        % to intercept a target body. The computation involves simulating the orbital
        % trajectories of both the spacecraft and the target over a defined planning horizon
        % and finding the point of minimum distance between their positions.

        function func_estimate_target_intercept_location_time(obj, mission, i_SC)

            % Retrieve the current position and velocity of the target body as estimated
            % by the spacecraft's software.
            this_target_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target(:);
                mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target(:)];

            this_time_array = 0:60:obj.time_horizon;

            % Get the position of the Sun at each time step in the J2000 frame
            % for the defined time array.
            Sun_pos_t0_tf = cspice_spkezr('10', mission.true_time.date + this_time_array, 'J2000', 'NONE', '10');

            % Simulate the trajectory of the target body using an ODE solver.
            % The function `func_orbit_SB_body` models the orbital motion of the body.
            [T, X] = ode113(@(t, X) func_orbit_SB_body(t, X, ...
                mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, ...
                Sun_pos_t0_tf, this_time_array), this_time_array, this_target_pos_vel_current, obj.options);

            % Store the target body's position and velocity over the simulation period.
            target_pos_t0_tf = X';

            % Retrieve the current position and velocity of the spacecraft
            % as estimated by its onboard software.
            this_SC_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position(:);
                mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity(:)];

            % Simulate the spacecraft's trajectory over the same time horizon.
            [T, X] = ode113(@(t, X) func_orbit_SB_body(t, X, ...
                mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, ...
                Sun_pos_t0_tf, this_time_array), this_time_array, this_SC_pos_vel_current, obj.options);

            % Store the spacecraft's position and velocity over the simulation period.
            SC_pos_t0_tf = X';

            % Compute the Euclidean distance between the spacecraft and the target body
            % at each time step. The result is an array of distances.
            Distance_array = vecnorm(SC_pos_t0_tf(1:3, :) - target_pos_t0_tf(1:3, :), 2, 1);

            % Find the minimum distance and the corresponding index in the time array.
            % `min_index` tells us the time step where the closest approach occurs.
            [min_distance, min_index] = min(Distance_array);

            % Determine the time of interception based on the index of minimum distance.
            if min_index == 1
                % Case 1: If the minimum distance occurs at the first time step, it implies
                % that the orbits are diverging (the closest point is now, and they are moving apart).
                obj.time_intercept = mission.true_time.time + obj.time_horizon/5 + ...
                    obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff;
            elseif (min_index > 1) && (min_index < length(Distance_array))
                % Case 2: The minimum distance occurs somewhere within the planning horizon.
                obj.time_intercept = mission.true_time.time + this_time_array(min_index);
                % If the calculated intercept time is too soon (before certain operational
                % thresholds like `time_horizon_DeltaV`), adjust the intercept time.
                if (obj.time_intercept - mission.true_time.time) < ...
                        (obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff)
                    obj.time_intercept = mission.true_time.time + obj.time_horizon/5 + ...
                        obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff;
                end
            else
                % Case 3: If the minimum distance occurs at the last time step, it suggests
                % that the spacecraft and target body are converging slowly but don't intercept
                % within the planning horizon.
                obj.time_intercept = mission.true_time.time + obj.time_horizon;
            end

            % Record the target body's position and velocity at the time of interception.
            % These values correspond to the time step with the minimum distance.
            obj.intercept_SB_position = target_pos_t0_tf(1:3, min_index);
            obj.intercept_SB_velocity = target_pos_t0_tf(4:6, min_index);

            % Record the minimum distance at interception.
            obj.intercept_distance = min_distance;
        end

        function verify_fuel_availability(obj, mission, i_SC)

            % Get distance to target
            target_pos = mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target;
            sc_pos = mission.true_SC{i_SC}.software_SC_estimate_orbit.position;
            distance_to_target = norm(target_pos - sc_pos);

            % Check if we're already so close that we can consider this an intercept
            % For an impactor mission, if we're within 5km, we're essentially on an intercept course already
            % This prevents unnecessary maneuvers when we're extremely close
            if distance_to_target < 5.0
                % Get relative velocity to check if trajectory is favorable
                target_vel = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target;
                sc_vel = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity;
                rel_vel = sc_vel - target_vel;

                % Calculate dot product to see if we're approaching or moving away
                approach_direction = dot(rel_vel, (target_pos - sc_pos) / distance_to_target);

                if approach_direction < 0 % Negative means approaching
                    disp(['Already on intercept course at distance of ', num2str(distance_to_target), ...
                        ' km with approach velocity of ', num2str(-approach_direction), ' km/s. Skipping maneuver calculation.']);

                    % Skip the maneuver since we're already approaching the target
                    obj.flag_insufficient_fuel = false; % Not a fuel issue
                    obj.desired_DeltaV_needs_to_be_executed = false;
                    obj.desired_DeltaV_computed = false;
                    return;
                end
            end

        end


        % Compute Transfer Correction Maneuver (TCM) using Lambert-Battin method
        function func_compute_TCM_Lambert_Battin(obj, mission, i_SC)

            % Verify fuel availability
            obj.verify_fuel_availability(mission, i_SC);


            % We are solving the Lambert Battin problem using the position
            % of the spacecraft after 'time_horizon_DeltaV' has passed.
            this_time_array = 0:10:obj.time_horizon_DeltaV+obj.time_horizon_data_cutoff;
            Sun_pos_t0_tf = cspice_spkezr('10', mission.true_time.date + this_time_array, 'J2000', 'NONE', '10');
            new_this_SC_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position(:); mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity(:)]; % [km, km/sec]

            % Estimated SC orbit
            [T,X] = ode113(@(t,X) func_orbit_SB_body(t,X, mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.options);
            SC_pos_t_DeltaV = X(end,:)';
            r1 = 1e3*SC_pos_t_DeltaV(1:3);  % [m]

            % Extract the positions
            r2 = obj.intercept_SB_position * 1e3; % [m]

            % Time of flight [sec]
            tof = obj.time_intercept - mission.true_time.time - obj.time_horizon_DeltaV - obj.time_horizon_data_cutoff;

            % Safety check: Ensure time of flight is reasonable
            if tof <= 0
                warning('Time of flight is negative or zero. Adjusting intercept time.');
                tof = 3600; % Set a default 1 hour time of flight
                obj.time_intercept = mission.true_time.time + obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff + tof;
            end

            % Solve Lambert's problem
            [V1, ~] = LAMBERTBATTIN(r1, r2, 'pro', tof);

            % Compute required Delta-V
            obj.desired_control_DeltaV = V1' - 1e3*SC_pos_t_DeltaV(4:6); % [m/s]

            % Now that we have the new deltaV we can update the intercept distance.
            % Estimate SB orbit
            this_target_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target(:);
                mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target(:)];
            [~, X] = ode113(@(t, X) func_orbit_SB_body(t, X, ...
                mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, ...
                Sun_pos_t0_tf, this_time_array), this_time_array, this_target_pos_vel_current, obj.options);
            SB_pos_t0_tf = X';


            % Estimate SC orbit
            % We propagate using the time after the deltaV begins (after time_horizon_DeltaV)
            new_this_time_array = obj.time_horizon_DeltaV+obj.time_horizon_data_cutoff : 10 : obj.time_intercept - mission.true_time.time; % [sec]
            new_this_SC_pos_vel_current = [SC_pos_t_DeltaV(1:3); SC_pos_t_DeltaV(4:6) + (1e-3*obj.desired_control_DeltaV)]; % [km, km/sec]
            Sun_pos_t0_tf = cspice_spkezr('10', mission.true_time.date + new_this_time_array, 'J2000', 'NONE', '10');

            [~,X] = ode113(@(t,X) func_orbit_SB_body(t,X, mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, Sun_pos_t0_tf, new_this_time_array),new_this_time_array,new_this_SC_pos_vel_current,obj.options);

            SC_pos_tDeltaV_tf = X';

            new_intercept_distance = norm(SC_pos_tDeltaV_tf(1:3,end) - SB_pos_t0_tf(1:3,end));
            obj.desired_intercept_distance = new_intercept_distance; % [km]

            obj.time_DeltaV = mission.true_time.time + obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff; % [sec] since t0
            obj.time_data_cutoff = mission.true_time.time + obj.time_horizon_data_cutoff; % [sec] since t0

            % Calculate estimated fuel required for the maneuver
            % Using the rocket equation: m_fuel = m_sc * (1 - exp(-deltaV/g0/Isp))
            % But for small deltaV, we can approximate: m_fuel ≈ m_sc * deltaV/(g0*Isp)
            g0 = 9.80665; % m/s^2

            % Get average ISP from all available chemical thrusters
            total_isp = 0;
            num_thrusters = 0;
            for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                    total_isp = total_isp + mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).isp;
                    num_thrusters = num_thrusters + 1;
                end
            end

            % Calculate average ISP if any thruster is healthy
            if num_thrusters > 0
                avg_isp = total_isp / num_thrusters;
            else
                avg_isp = 200; % Default ISP if no healthy thrusters
                warning('No healthy thrusters available. Using default ISP of 200s.');
            end

            % Estimate fuel required
            sc_mass = mission.true_SC{i_SC}.true_SC_body.total_mass;
            deltaV_magnitude = norm(obj.desired_control_DeltaV);
            obj.estimated_fuel_required = sc_mass * deltaV_magnitude / (g0 * avg_isp);

            % Check if we have sufficient fuel
            total_available_fuel = 0;
            if isfield(mission.true_SC{i_SC}, 'true_SC_fuel_tank') && ...
                    mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
                for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
                    total_available_fuel = total_available_fuel + ...
                        mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.instantaneous_fuel_mass;
                end
            end

            % Set flags based on fuel availability
            if total_available_fuel < (obj.estimated_fuel_required + obj.min_fuel_threshold) && ~mission.true_SC{i_SC}.true_SC_navigation.flag_SC_crashed
                obj.flag_insufficient_fuel = true;
                warning(['Insufficient fuel for maneuver. Required: ', ...
                    num2str(obj.estimated_fuel_required), ' kg, Available: ', ...
                    num2str(total_available_fuel), ' kg']);

                % Don't execute the maneuver if we don't have enough fuel
                obj.desired_DeltaV_needs_to_be_executed = false;
                obj.desired_DeltaV_computed = false;
            else
                obj.flag_insufficient_fuel = false;
                obj.desired_DeltaV_needs_to_be_executed = true;
                obj.desired_DeltaV_computed = true;

            end

            obj.desired_DeltaV_achieved = false;
            obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
            obj.desired_attitude_for_DeltaV_achieved = false;
        end


        function func_command_DeltaV(obj, mission, i_SC)
            % Initialize desired control thrust to zero
            obj.desired_control_thrust = 0;

            % Get spacecraft parameters
            sc_mass = mission.true_SC{i_SC}.true_SC_body.total_mass;
            dt = mission.true_time.time_step;

            % Check for healthy thrusters
            healthy_thrusters = [];
            for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                    healthy_thrusters = [healthy_thrusters, i_thruster];
                end
            end

            if isempty(healthy_thrusters)
                warning('No healthy thrusters available for DeltaV execution.');
                return;
            end

            % Check execution conditions
            if (mission.true_time.time < obj.time_DeltaV) || obj.desired_DeltaV_achieved || obj.flag_insufficient_fuel
                return;
            end

            % Check if thruster is ready for firing
            all_thrusters_ready = true;
            for i = 1:length(healthy_thrusters)
                if ~mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).func_is_thruster_ready()
                    all_thrusters_ready = false;
                    break;
                end
            end

            if ~all_thrusters_ready
                % Thrusters still warming up, wait until next cycle
                return;
            end

            % Calculate remaining Delta-V
            remaining_DeltaV = obj.desired_control_DeltaV - obj.total_DeltaV_executed;
            remaining_magnitude = norm(remaining_DeltaV);

            if remaining_magnitude > 0
                % Record start time if this is the first thrust of a maneuver
                if obj.maneuver_start_time == 0
                    obj.maneuver_start_time = mission.true_time.time;
                    obj.thruster_fired_successfully = false;
                    disp(['Starting maneuver execution at time ', num2str(mission.true_time.time), ' s']);
                end

                % Get thruster parameters
                max_thrusts = zeros(1, length(healthy_thrusters));
                for i = 1:length(healthy_thrusters)
                    max_thrusts(i) = mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).maximum_thrust;
                end

                % Get current attitude matrix
                current_attitude = mission.true_SC{i_SC}.true_SC_adc.attitude;
                R = quaternionToRotationMatrix(current_attitude);

                % Rotate thruster directions to inertial frame
                thruster_body_directions = zeros(3, length(healthy_thrusters));
                for i = 1:length(healthy_thrusters)
                    thruster_body_directions(:, i) = mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).orientation';
                end
                orientations = R * thruster_body_directions;
                A = orientations;

                % Calculate maximum available thrust in desired direction
                max_thrust_dir = A * max_thrusts';
                max_DeltaV_per_step = (norm(max_thrust_dir) * dt) / sc_mass;

                % Calculate required thrust scaling factors
                if remaining_magnitude > max_DeltaV_per_step
                    % Use maximum thrust for each thruster
                    thrust_vector = max_thrusts;
                else
                    % Calculate required thrust to achieve the desired delta-V in this step
                    required_thrust_magnitude = (remaining_magnitude * sc_mass) / dt;

                    % Distribute thrust among thrusters (simplified approach - equal distribution)
                    thrust_vector = zeros(1, length(healthy_thrusters));
                    for i = 1:length(healthy_thrusters)
                        thrust_vector(i) = required_thrust_magnitude / length(healthy_thrusters);

                        % Clamp to thruster limits
                        thrust_vector(i) = min(max(thrust_vector(i),...
                            mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).minimum_thrust), ...
                            mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).maximum_thrust);
                    end
                end

                % Apply thrust commands to thrusters
                for i = 1:length(healthy_thrusters)
                    mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).commanded_thrust = thrust_vector(i);
                    mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).flag_executive = 1;

                    % Save that we commanded a thrust (needed for completion checks)
                    obj.desired_control_thrust = obj.desired_control_thrust + thrust_vector(i);
                end

                % Note: We no longer calculate applied DeltaV here - that's now done in the thruster itself
                % which will directly update obj.total_DeltaV_executed
            end
        end


        function reset_after_completion(obj, mission, i_SC)

            % IMPORTANT: Clear all maneuver flags and reset values to zero
            obj.desired_DeltaV_achieved = true;
            obj.desired_DeltaV_needs_to_be_executed = false;
            obj.desired_control_DeltaV = [0 0 0]'; 
            obj.desired_DeltaV_computed = false;
            obj.last_time_control = mission.true_time.time;
            obj.total_DeltaV_executed = [0 0 0]'; 
            
            % Reset maneuver tracking
            obj.maneuver_start_time = 0;
            obj.thruster_fired_successfully = false;
            obj.flag_executive = 0;
            
            % Add: Clear pending thruster commands
            for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                    mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).pending_fire = false;
                    mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).pending_thrust = 0;
                end
            end
        end



        function func_main_software_SC_control_orbit(obj, mission, i_SC)


            % Main control loop for spacecraft orbit
            if obj.flag_executive

                first_pass = false; 

                % Always verify the capacity of the fuel tanks.
                total_fuel = 0;
                if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
                    for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
                        total_fuel = total_fuel + mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.instantaneous_fuel_mass;
                    end

                    % Log fuel level if it falls below certain thresholds
                    if total_fuel < 0.5
                        warning('CRITICAL: Spacecraft fuel level below 0.5 kg. Remaining: %.3f kg', total_fuel);
                    elseif total_fuel < 1.0
                        warning('WARNING: Spacecraft fuel level below 1.0 kg. Remaining: %.3f kg', total_fuel);
                    end
                end

                if ~obj.desired_DeltaV_computed


                    % First, assess whether a maneuver calculation is actually needed
                    [is_maneuver_needed, reason] = obj.func_assess_maneuver_necessity(mission, i_SC);

                    if ~is_maneuver_needed
                        % Log the reason we're skipping the maneuver calculation
                        disp(['Skipping maneuver calculation: ', reason]);

                        % If we're skipping, make sure to reset the flags
                        obj.reset_after_completion(mission, i_SC);
                        return;
                    end

                    % Maneuver seems necessary, proceed with calculations
                    obj.func_estimate_target_intercept_location_time(mission, i_SC);
                    obj.func_compute_TCM_Lambert_Battin(mission, i_SC);

                    % Log the computed maneuver for mission awareness
                    if obj.desired_DeltaV_computed
                        disp(['Computed maneuver: DeltaV magnitude = ', num2str(norm(obj.desired_control_DeltaV)), ' m/s']);
                        disp(['Execution time: ', datestr(datetime('now') + seconds(obj.time_DeltaV - mission.true_time.time))]);
                        disp(['Estimated fuel required: ', num2str(obj.estimated_fuel_required), ' kg']);
                    end

                    first_pass = true; 
                end



                % Validate attitude and execute DeltaV if conditions are met
                if obj.desired_DeltaV_needs_to_be_executed && ...
                        (mission.true_time.time >= obj.time_DeltaV)  && ...
                        ~first_pass && ...
                        (norm(mission.true_SC{i_SC}.software_SC_control_attitude.get_attitude_error(mission,i_SC)) < 0.03) && ... % rad
                        ~obj.flag_insufficient_fuel
                    obj.func_command_DeltaV(mission, i_SC);
                end

                % Handle thruster warm-up logic if maneuver is planned
                if obj.desired_DeltaV_needs_to_be_executed && obj.desired_DeltaV_computed
                    % Check if remaining DeltaV is above threshold
                    remaining_DeltaV = obj.desired_control_DeltaV - obj.total_DeltaV_executed;
                    if norm(remaining_DeltaV) < obj.threshold_minimum_deltaV
                        % If DeltaV is below threshold, consider it complete
                        disp(['Skipping maneuver - remaining DeltaV (', num2str(norm(remaining_DeltaV)), ...
                            ' m/s) below threshold (', num2str(obj.threshold_minimum_deltaV), ' m/s)']);

                        % Stop warming up thruster if needed
                        for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                            if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = false;
                                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_executive = false; 
                            end
                        end

                        obj.reset_after_completion(mission, i_SC);
                        return;
                    end

                    % Check for sufficient fuel before proceeding
                    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
                        total_fuel = 0;
                        for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
                            total_fuel = total_fuel + mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.instantaneous_fuel_mass;
                        end

                        % Add safety margin
                        fuel_with_margin = obj.estimated_fuel_required * 1.1; % 10% margin

                        if total_fuel < fuel_with_margin
                            warning('Insufficient fuel for DeltaV execution. Required: %.3f kg (with margin), Available: %.3f kg', ...
                                fuel_with_margin, total_fuel);

                            % Cancel the maneuver
                            obj.flag_insufficient_fuel = true;
                            obj.desired_DeltaV_needs_to_be_executed = false;
                            obj.desired_DeltaV_computed = false;

                            % Stop warming up thruster
                            for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                                if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                                    mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = false;
                                end
                            end
                            return;
                        end
                    end

                    % Get time until DeltaV execution
                    time_to_DeltaV = obj.time_DeltaV - mission.true_time.time;

                    % Get thruster warm-up time
                    thruster_warm_up_time = 30; % Default value
                    for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                        if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                            thruster_warm_up_time = mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).thruster_warm_up_time;
                            break;
                        end
                    end

                    % Start thruster warm-up with a safety margin (45 sec before needed)
                    thruster_warming = false;
                    for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                        if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up
                            thruster_warming = true;
                            break;
                        end
                    end

                    if time_to_DeltaV > 0 && time_to_DeltaV <= (thruster_warm_up_time + 45) && ~thruster_warming
                        % Time to start warming up the thruster
                        for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                            if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = true;
                                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).func_start_warm_up(mission);
                            end
                        end
                        disp(['Starting thruster warm-up at T-', num2str(time_to_DeltaV), ' seconds before maneuver']);
                    end
                end

                % Check if maneuver is complete - simplified approach
                if obj.desired_DeltaV_needs_to_be_executed && obj.desired_DeltaV_computed
                    % Check remaining DeltaV - this is updated directly by the thruster
                    remaining_DeltaV = obj.desired_control_DeltaV - obj.total_DeltaV_executed;

                    % Check timeout conditions
                    maneuver_timeout = false;
                    time_since_planned = mission.true_time.time - obj.time_DeltaV;

                    % Timeout if running too long since start
                    if obj.maneuver_start_time > 0 && (mission.true_time.time - obj.maneuver_start_time) > 120
                        maneuver_timeout = true;
                        warning('Maneuver timeout reached after %d seconds from maneuver start', ...
                            mission.true_time.time - obj.maneuver_start_time);
                    end

                    % Timeout if waiting too long after planned execution
                    if time_since_planned > 120 && ~maneuver_timeout
                        maneuver_timeout = true;
                        warning('Maneuver timeout: %d seconds since planned execution time', time_since_planned);
                    end

                    % Check if we should complete the maneuver
                    is_deltaV_complete = norm(remaining_DeltaV) < obj.threshold_minimum_deltaV;
                    if is_deltaV_complete || maneuver_timeout || time_since_planned > 60
                        % Report reason for completion
                        if is_deltaV_complete
                            disp(['Maneuver completed: Target DeltaV achieved within ', ...
                                num2str(obj.threshold_minimum_deltaV), ' m/s threshold']);
                        elseif maneuver_timeout
                            disp(['Maneuver timeout after ', num2str(mission.true_time.time - obj.maneuver_start_time), ' seconds']);
                        else
                            disp(['Maneuver forced to complete after ', num2str(time_since_planned), ' seconds from planned time']);
                        end

                        % Reset everything
                        obj.reset_after_completion(mission, i_SC);

                        % Turn off thruster warm-up
                        for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                            if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = false;
                                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).commanded_thrust = 0;
                            end
                        end

                        disp('Maneuver completed and all flags reset');
                    end
                end
            end

            % Store and reset
            obj.func_update_software_SC_Control_Orbit_store(mission);

            % Reset the executive flag
            obj.flag_executive = 0;
        end

        function [is_needed, reason] = func_assess_maneuver_necessity(obj, mission, i_SC)
            % This function evaluates whether a maneuver calculation is actually needed
            % based on current trajectory and intercept parameters

            % Default to needing a maneuver
            is_needed = true;
            reason = '';

            % Get orbit estimation data
            position_sc = mission.true_SC{i_SC}.software_SC_estimate_orbit.position;
            velocity_sc = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity;

            position_target = mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target;
            velocity_target = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target;

            % Calculate distance to target
            rel_position = position_target - position_sc;
            distance_to_target = norm(rel_position);

            % Calculate relative velocity
            rel_velocity = velocity_target - velocity_sc;
            relative_speed = norm(rel_velocity);

            approach_projection = dot(rel_position, rel_velocity) / (distance_to_target * relative_speed);


            % Check if we're already very close to target (less than 3 km)
            if distance_to_target < 3
                % Check if we're on a reasonable approach (approaching, not departing)

                % If projection is negative, we're approaching
                if approach_projection < 0
                    % Calculate miss distance approximation using simple projection
                    is_needed = false;
                    reason = ['Already on successful intercept trajectory. Distance: ', num2str(distance_to_target), ' km'];
                end
            end

            % Check available fuel vs expected requirements
            if is_needed && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
                % Estimate using very rough heuristic - actual calculation will happen in Lambert-Battin
                % This is just a pre-check to avoid unnecessary calculation
                available_fuel = mission.true_SC{i_SC}.true_SC_fuel_tank{1}.instantaneous_fuel_mass;

                % Very rough estimate of fuel needed (not accurate, just for pre-screening)
                approximate_fuel_required = relative_speed * 0.005 * mission.true_SC{i_SC}.true_SC_body.total_mass / 3000;

                % If clearly insufficient fuel (with large margin to be safe)
                if approximate_fuel_required > 10 * available_fuel
                    is_needed = false;
                    reason = ['Insufficient fuel for likely maneuver. Est. required: ', num2str(approximate_fuel_required), ' kg, Available: ', num2str(available_fuel), ' kg'];
                end
            end

            % Check if time-to-intercept is very short
            if is_needed && approach_projection < 0
                % Estimate time to closest approach
                time_to_closest_approach = distance_to_target / relative_speed;

                % If very close to intercept (less than 1 minute)
                if time_to_closest_approach < 60
                    is_needed = false;
                    reason = ['Intercept imminent (', num2str(time_to_closest_approach), ' seconds). Maneuver computation skipped.'];
                end
            end
        end
    end
end
