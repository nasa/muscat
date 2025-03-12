%% Class: Software_SC_Control_Attitude
% Control the Attitude of the Spacecraft
classdef Software_SC_Control_Attitude < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_software_SC_control_attitude_selector % [string] Different attitude control modes
        % - 'DART Oracle' : Directly change True_SC_ADC.attitude values for DART mission
        % - 'Nightingale Oracle' : Directly change True_SC_ADC.attitude values for Nightingale mission
        % - 'Oracle with Control' : Use True_SC_ADC values + Noise

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Control Attitude

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        desired_attitude % [quanternion] : Orientation of inertial frame I with respect to the body frame B

        desired_angular_velocity % [rad/sec] : Angular velocity of inertial frame I with respect to the body frame B

        desired_control_torque % [Nm] : Desired control torque

        data % Other useful data

        desaturation_procedure % [Bool] - Is the procedure started ?
        thruster_contribution_matix        % thruster contribution
        pinv_reaction_wheel_contribution_matrix % reaction wheel contribution matrix
        reaction_wheel_attitude_control_threshold % [rad] Angle from which wheels can take over the correction
        optim_data

        max_thrust   % [N] : Maximum thrust of the thruster
        min_thrust   % [N] : Minimum thrust of the thruster

        % Cached torque capabilities
        max_rw_torque % [Nm] : Maximum torque capability of reaction wheels
        max_mt_torque % [Nm] : Maximum torque capability of micro thrusters

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Control_Attitude(init_data, mission, i_SC)

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Control Attitude']; % [string]
            obj.flag_executive = 1;

            obj.instantaneous_data_generated_per_sample = 0; % [kb]
            obj.mode_software_SC_control_attitude_selector = init_data.mode_software_SC_control_attitude_selector; % [string]


            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end


            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            %% Initialization of thruster and RW optimization data
            func_initialize_optimization_data(obj, mission, i_SC);

            % Initialize matrices and hardware data
            obj = obj.initialize_thruster_contribution(mission, i_SC);
            obj = obj.initialize_reaction_wheel_contribution(mission, i_SC);

            % Calculate and cache max torque capabilities
            obj.max_rw_torque = obj.calculate_max_reaction_wheel_torque(mission, i_SC);
            obj.max_mt_torque = obj.calculate_max_thruster_torque(mission, i_SC);

            % All the zeros
            obj.desaturation_procedure = 0;
            obj.desired_attitude = zeros(1,4); % [quaternion]
            obj.desired_control_torque = zeros(1,3); % [Toraue Vector Nm]
            obj.desired_angular_velocity = zeros(1,3); % [rad/sec]
            obj.data.integral_error = zeros(3,1); % Initialize as 3x1 vector for X/Y/Z axes

            % Initialize control gains
            obj.data.control_gain = init_data.control_gain;

            % Initialize Variables to store
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps_attitude, length(obj.flag_executive));
            obj.store.desaturation_procedure = zeros(mission.storage.num_storage_steps_attitude, length(obj.desaturation_procedure));
            obj.store.desired_attitude = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_attitude));
            obj.store.desired_angular_velocity = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_angular_velocity));
            obj.store.desired_control_torque = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_control_torque));

            % Update Storage
            obj = func_update_software_SC_control_attitude_store(obj, mission);


            if isfield(init_data, 'reaction_wheel_attitude_control_threshold')
                obj.reaction_wheel_attitude_control_threshold = init_data.reaction_wheel_attitude_control_threshold;
            else
                obj.reaction_wheel_attitude_control_threshold = 0.1; % [Rad]- Nominal is 0.05
            end

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable
        function obj = func_update_software_SC_control_attitude_store(obj, mission)
            if mission.storage.flag_store_this_time_step_attitude == 1
                obj.store.flag_executive(mission.storage.k_storage_attitude,:) = obj.flag_executive;
                obj.store.desaturation_procedure(mission.storage.k_storage_attitude,:) = obj.desaturation_procedure; % [quaternion]

                obj.store.desired_attitude(mission.storage.k_storage_attitude,:) = obj.desired_attitude; % [quaternion]
                obj.store.desired_angular_velocity(mission.storage.k_storage_attitude,:) = obj.desired_angular_velocity; % [rad/sec]
                obj.store.desired_control_torque(mission.storage.k_storage_attitude,:) = obj.desired_control_torque; % [Nm]
            end
        end

        function obj = initialize_from_init_data(obj, mission, i_SC, init_data)

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            if isfield(init_data, 'control_gain')
                obj.data.control_gain = init_data.control_gain;
            else
                obj.data.control_gain = [0.1 1];   % [Kr ; Lambda_r] for RWA/MT control
            end
        end


        %% Initialize Thruster Contribution Matrix
        function obj = initialize_thruster_contribution(obj, mission, i_SC)
            % Build thruster torque contribution matrix
            num_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster;
            obj.thruster_contribution_matix = zeros(3, num_thrusters);
            obj.max_thrust = zeros(1, num_thrusters);
            obj.min_thrust = zeros(1, num_thrusters);

            for i = 1:num_thrusters
                thruster = mission.true_SC{i_SC}.true_SC_micro_thruster{i};
                r = thruster.location - mission.true_SC{i_SC}.true_SC_body.location_COM;
                obj.thruster_contribution_matix(:,i) = cross(r, thruster.orientation);
                obj.max_thrust(i) = thruster.maximum_thrust;
                obj.min_thrust(i) = thruster.minimum_thrust;
            end
        end


        %% Initialize Reaction Wheel Contribution Matrix for Pinverse Optimization
        function obj = initialize_reaction_wheel_contribution(obj, mission, i_SC)
            % Build RWA momentum contribution matrix
            reaction_wheel_contribution_matrix = zeros(3,  mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel);

            for i = 1: mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                wheel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i};
                if wheel.health
                    reaction_wheel_contribution_matrix(:,i) = wheel.orientation' * wheel.moment_of_inertia;
                end
            end

            obj.pinv_reaction_wheel_contribution_matrix = pinv(reaction_wheel_contribution_matrix);
        end

        %% [ ] Methods: Main
        % Main Function
        function obj = func_main_software_SC_control_attitude(obj, mission, i_SC)

            if (obj.flag_executive == 1)


                switch obj.mode_software_SC_control_attitude_selector

                    case {'DART Oracle', 'DART Control Asymptotically Stable send to ADC directly', 'DART Control PD', 'DART Control Asymptotically Stable send to actuators', 'DART Control Asymptotically Stable send to thrusters'}
                        obj = func_update_software_SC_control_attitude_DART(obj, mission, i_SC);

                    case {'Nightingale Oracle', 'Nightingale Control Asymptotically Stable send to actuators', 'Nightingale Control Asymptotically Stable send to rwa', 'Nightingale Control Asymptotically Stable send to thrusters'}
                        obj = func_update_software_SC_control_attitude_Nightingale(obj, mission, i_SC);

                    otherwise
                        error('Attitude Control mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
            end

            % Update Storage
            obj = func_update_software_SC_control_attitude_store(obj, mission);

            % DO NOT SWITCH OFF FUNCTIONS USING flag_executive INSIDE Attitude Dynamics Loop (ADL)
        end

        %% [ ] Methods: Control Attitude DART Oracle
        % Use Truth Data

        function obj = set_pointing_vectors(obj, mission, i_SC)

            % Navigation and Guidance
            switch mission.true_SC{i_SC}.software_SC_executive.this_sc_mode

                case 'Point camera to Target'
                    % Point camera to target
                    obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_camera{1}.orientation); % In body frame
                    obj.data.desired_primary_vector = func_normalize_vec(mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame
                    % Optimize for solar pannels orientation to sun
                    obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
                    obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

                case 'DTE Comm'
                    % Point antenna to earth
                    % DART has one antenna
                    obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation); % In body frame
                    obj.data.desired_primary_vector = func_normalize_vec( ...
                        mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - ...
                        mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

                    % Optimize for solar pannels orientation to sun
                    obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
                    obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame


                case 'Point Thruster along DeltaV direction'
                    % Point thruster in direction of desired deltaV
                    obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_chemical_thruster.orientation); % In body frame

                    % Use the deltaV vector direction instead of target direction
                    deltaV = mission.true_SC{i_SC}.software_SC_control_orbit.desired_control_DeltaV;
                    obj.data.desired_primary_vector = func_normalize_vec(deltaV)'; % In Inertial frame

                    % Secondary vector remains optimized for solar panels
                    obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
                    obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame


                case 'Maximize SP Power'
                    % Primary vector is the solar pannels to sun
                    obj.data.primary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_solar_panel{1}.shape_model.Face_orientation_solar_cell_side); % In body frame
                    obj.data.desired_primary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

                    % [Optional] Optimize for DTE to earth
                    obj.data.secondary_vector = func_normalize_vec(mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation); % In body frame
                    obj.data.desired_secondary_vector = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % In Inertial frame

                otherwise
                    error('Attitude Control mode for this SC mode not defined!')
            end


        end

        %% [ ] Methods: Control Attitude DART Oracle
        % Use Truth Data

        function obj = func_update_software_SC_control_attitude_DART(obj, mission, i_SC)

            obj.instantaneous_data_generated_per_sample = (1e-3)*8*7; % [kb] i.e. 7 Bytes per sample

            % Navigation and Guidance
            % Set pointing vectors based on SC mode
            obj = obj.set_pointing_vectors(mission, i_SC);

            % Compute Desired Attitude using Array Search
            obj = func_compute_desired_attitude_Array_Search(obj);
            obj.desired_angular_velocity = zeros(1,3); % [rad/sec]

            % Control
            switch obj.mode_software_SC_control_attitude_selector

                case 'DART Oracle'
                    % Use Oracle
                    obj = func_update_software_SC_control_attitude_Oracle(obj, mission, i_SC);

                case 'DART Control Asymptotically Stable send to ADC directly'
                    obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC);
                    mission.true_SC{i_SC}.true_SC_adc.control_torque = obj.desired_control_torque;

                case 'DART Control Asymptotically Stable send to actuators'
                    obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC);
                    obj = func_actuator_selection(obj, mission, i_SC);

                case 'DART Control PD'
                    obj = function_update_desired_control_torque_PD(obj, mission, i_SC);
                    obj = func_actuator_selection(obj, mission, i_SC);

                case 'DART Control Asymptotically Stable send to thrusters'
                    obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC);
                    func_decompose_control_torque_into_thrusters_optimization_kkt(obj,mission, i_SC);

                otherwise
                    error('DART Attitude Control mode not defined!')
            end
        end

        %% [ ] Methods: Get Attitude Error

        function error = get_attitude_error_euler(obj, mission, i_SC)
            % This method is used by the orbit control
            % Decide which actuator to use
            actual_euler = ConvertAttitude(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude'/norm(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude)', 'quaternion','321');
            desired_euler = ConvertAttitude(obj.desired_attitude'/norm(obj.desired_attitude)', 'quaternion','321');
            error = abs(actual_euler - desired_euler);
        end

        function error = get_attitude_error(obj, mission, i_SC)
            % This method calculates attitude error more efficiently
            % by working directly in quaternion space

            % Get quaternions and ensure proper format/normalization
            q_actual = mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude;
            q_desired = obj.desired_attitude;

            % Ensure column vectors and normalize once
            if size(q_actual,1) == 1
                q_actual = q_actual';
            end
            if size(q_desired,1) == 1
                q_desired = q_desired';
            end

            q_actual = q_actual / norm(q_actual);
            q_desired = q_desired / norm(q_desired);

            % Compute quaternion error: q_error = q_desired^(-1) * q_actual
            % For quaternion conjugate, negate the vector part (first 3 elements)
            q_desired_conj = q_desired;
            q_desired_conj(1:3) = -q_desired_conj(1:3);

            % Quaternion multiplication (using standard quaternion multiplication formula)
            % This is q_error = q_desired_conj ⊗ q_actual
            q_error = zeros(4,1);
            q_error(1) = q_desired_conj(4)*q_actual(1) + q_desired_conj(1)*q_actual(4) + q_desired_conj(2)*q_actual(3) - q_desired_conj(3)*q_actual(2);
            q_error(2) = q_desired_conj(4)*q_actual(2) + q_desired_conj(2)*q_actual(4) + q_desired_conj(3)*q_actual(1) - q_desired_conj(1)*q_actual(3);
            q_error(3) = q_desired_conj(4)*q_actual(3) + q_desired_conj(3)*q_actual(4) + q_desired_conj(1)*q_actual(2) - q_desired_conj(2)*q_actual(1);
            q_error(4) = q_desired_conj(4)*q_actual(4) - q_desired_conj(1)*q_actual(1) - q_desired_conj(2)*q_actual(2) - q_desired_conj(3)*q_actual(3);

            % Extract error angle from quaternion (in radians)
            % The scalar part of the quaternion (q_error(4)) is cos(θ/2)
            % So θ = 2*acos(q_error(4)) is the rotation angle
            error_angle = 2 * acos(min(1, max(-1, q_error(4))));

            % Return error as a 3x1 vector (for compatibility with existing code)
            % Scale by rotation axis to get component errors
            if error_angle > 1e-10  % Avoid division by zero
                axis = q_error(1:3) / sin(error_angle/2);
                error = error_angle * axis;
            else
                % For very small errors, just return the vector part (scaled)
                error = 2 * q_error(1:3);
            end
        end


        function obj = func_actuator_selection(obj, mission, i_SC)

            % Get current attitude error
            attitude_error = obj.get_attitude_error(mission, i_SC);

            % STEP 0: Check if wheels need desaturation
            if obj.is_desaturation_needed(mission, i_SC)
                % If wheels are already saturated, handle desaturation
                disp('Performing wheel desaturation');
                obj.handle_desaturation(mission, i_SC);
                return;
            end
            
            % STEP 1: Check if attitude error is too large for wheels
            if norm(attitude_error) > obj.reaction_wheel_attitude_control_threshold
                % For large angle corrections, always use thrusters - simple rule
                func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC);
                return;
            end

            % STEP 2: Check if torque request exceeds wheel capability
            if norm(obj.desired_control_torque) > obj.max_rw_torque
                % If torque is too high for wheels, use thrusters
                func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC);
                return;
            end
            
            % STEP 3: Check for predicted wheel saturation (simplified)
            [will_saturate_quickly, ~] = obj.predict_wheel_saturation(mission, i_SC);
            if will_saturate_quickly
                % If wheels will saturate soon, use thrusters
                disp('Using thrusters due to predicted wheel saturation');
                func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC);
                return;
            end
            
            
            % STEP 4: If we reached here, use reaction wheels (all conditions satisfied)
            reset_thrusters(obj, mission, i_SC);
            func_compute_rw_command_pinv(obj, mission, i_SC);
        end

        %% [ ] Methods: Predict wheel saturation
        function [will_saturate_quickly, time_steps_to_saturation] = predict_wheel_saturation(obj, mission, i_SC)
            % Simplified prediction of reaction wheel saturation
            
            % Initialize output
            will_saturate_quickly = false;
            time_steps_to_saturation = Inf;
            
            % Minimum acceptable time steps before saturation
            min_acceptable_time_steps = 5;
            
            % Skip calculation if desired torque is negligible
            if norm(obj.desired_control_torque) < 1e-6
                return;
            end
            
            % Calculate required wheel accelerations using pseudo-inverse
            wheel_accelerations = obj.pinv_reaction_wheel_contribution_matrix * obj.desired_control_torque';
            
            % Check each wheel
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                wheel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i};
                
                % Skip unhealthy wheels
                if ~wheel.health
                    continue;
                end
                
                % Skip if the commanded acceleration is negligible
                if abs(wheel_accelerations(i)) < 1e-6
                    continue;
                end
                
                % Limit command to maximum acceleration
                cmd_accel = wheel_accelerations(i);
                if abs(cmd_accel) > wheel.maximum_acceleration
                    cmd_accel = sign(cmd_accel) * wheel.maximum_acceleration;
                end
                
                % Current angular velocity
                current_velocity = wheel.angular_velocity;
                
                % Saturation threshold (80% of max, matching the wheel's internal check)
                saturation_threshold = wheel.max_angular_velocity * 0.8;
                
                % Two main checks:
                
                % 1. Direction reversal check - these are problematic
                if abs(current_velocity) > 0.3 * wheel.max_angular_velocity && ...
                        sign(current_velocity) ~= sign(current_velocity + cmd_accel * mission.true_time.time_step_attitude)
                    % Large direction reversal detected
                    will_saturate_quickly = true;
                    time_steps_to_saturation = 1;
                    return;
                end
                
                % 2. Acceleration toward saturation
                if sign(cmd_accel) == sign(current_velocity)
                    % Wheel accelerating in same direction (toward saturation)
                    remaining_velocity = saturation_threshold - abs(current_velocity);
                    
                    if remaining_velocity <= 0
                        % Already at or beyond saturation threshold
                        will_saturate_quickly = true;
                        time_steps_to_saturation = 0;
                        return;
                    end
                    
                    % Time steps until saturation at current acceleration
                    steps = remaining_velocity / (abs(cmd_accel) * mission.true_time.time_step_attitude);
                    
                    if steps < min_acceptable_time_steps
                        will_saturate_quickly = true;
                        time_steps_to_saturation = steps;
                        return;
                    end
                    
                    % Keep track of minimum time to saturation across all wheels
                    if steps < time_steps_to_saturation
                        time_steps_to_saturation = steps;
                    end
                end
            end
            
            % Check for significant disturbance torque (simplified version)
            if isfield(mission.true_SC{i_SC}.true_SC_adc, 'disturbance_torque')
                disturbance_torque = mission.true_SC{i_SC}.true_SC_adc.disturbance_torque;
                desired_torque_mag = norm(obj.desired_control_torque);
                
                % If significant disturbance present relative to command
                if desired_torque_mag > 1e-6 && norm(disturbance_torque) > 0.5 * desired_torque_mag
                    will_saturate_quickly = true;
                    time_steps_to_saturation = min_acceptable_time_steps; % Conservative estimate
                end
            end
        end

        %% [ ] Methods: Reset Micro Thrusters
        function reset_thrusters(~, mission, i_SC)
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 0;
                mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = 0;
            end
        end


        %% [ ] Methods: Reset Reaction Wheels
        function reset_wheels(~, mission, i_SC)
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.flag_executive = 0;
                mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = 0;
            end
        end

        %% [ ] Methods: Compute Reaction Wheels Command using pseudo inverse
        function func_compute_rw_command_pinv(obj, mission, i_SC)
            % Compute reaction wheel commands to achieve desired control torque

            % Use scaled torque for wheel commands
            wheel_accelerations = obj.pinv_reaction_wheel_contribution_matrix * obj.desired_control_torque';

            % Apply momentum management to avoid direction reversals
            wheel_accelerations = obj.manage_wheel_momentum(mission, i_SC, wheel_accelerations);

            % Apply commands to reaction wheels
            obj.apply_reaction_wheel_commands(mission, i_SC, wheel_accelerations);
        end

        %% [ ] Methods: Manage Wheel Momentum to Avoid Direction Reversals
        function wheel_accelerations = manage_wheel_momentum(obj, mission, i_SC, wheel_accelerations)
            % Apply momentum management to avoid abrupt direction changes
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel

                % Skip unhealthy wheels
                if ~mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.health
                    wheel_accelerations(i) = 0;
                    continue;
                end

                % Get current wheel velocity
                current_velocity = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.angular_velocity;

                % Calculate absolute velocity as percentage of max
                velocity_percentage = abs(current_velocity) / mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity;

                % ADDED: Special handling for mode transitions
                if isfield(obj.data, 'mode_transition_detected') && obj.data.mode_transition_detected
                    % During mode transitions, be extremely conservative with commands

                    % Calculate expected velocity after one time step
                    predicted_vel = current_velocity + wheel_accelerations(i) * mission.true_time.time_step_attitude;

                    % Check for problematic direction reversal 
                    if sign(predicted_vel) ~= sign(current_velocity) && abs(current_velocity) > 0.3 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity
                        % Direction would reverse and current speed is significant

                        % Instead of allowing the direction change, apply gradual deceleration
                        % Set deceleration to a fraction of what would be needed to stop in 5 time steps
                        safe_decel = -sign(current_velocity) * min(abs(current_velocity) / (5 * mission.true_time.time_step_attitude), 0.05 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration);

                        % Limit the change and log
                        wheel_accelerations(i) = safe_decel;

                        % Log this limiting with detailed diagnostics
                        disp(['MODE TRANSITION SAFETY: Limiting wheel ', num2str(i), ' acceleration during mode change.']);
                        disp(['Current velocity: ', num2str(current_velocity), ' rad/s (', num2str(velocity_percentage*100), '% of max)']);
                        disp(['Original command: ', num2str(wheel_accelerations(i)), ' rad/s^2']);
                        disp(['Safe deceleration: ', num2str(safe_decel), ' rad/s^2']);

                        continue; % Skip the rest of the checks for this wheel
                    end

                    % Even when not reversing direction, limit acceleration during transitions
                    max_safe_accel = 0.1 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration;
                    if abs(wheel_accelerations(i)) > max_safe_accel
                        wheel_accelerations(i) = sign(wheel_accelerations(i)) * max_safe_accel;
                        disp(['Limiting wheel ', num2str(i), ' acceleration to ', num2str(max_safe_accel), ' rad/s^2 during mode transition']);
                    end

                    continue; % Skip further checks during mode transitions
                end

                % Check for direction reversal (wheel going at high speed in one direction
                % and acceleration trying to go in the opposite)
                if abs(current_velocity) > 0.5 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity && sign(current_velocity) ~= sign(wheel_accelerations(i))
                    % Calculate expected velocity after one time step
                    predicted_vel = current_velocity + wheel_accelerations(i) * mission.true_time.time_step_attitude;

                    % If direction would reverse, limit the acceleration to begin deceleration
                    % rather than attempting a full reversal
                    if sign(predicted_vel) ~= sign(current_velocity)
                        % Set acceleration to a safe deceleration value (~ 5% of max velocity per second)
                        safe_decel = -sign(current_velocity) * min(abs(wheel_accelerations(i)), 0.05 * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.max_angular_velocity / mission.true_time.time_step_attitude);

                        % Limit the change
                        wheel_accelerations(i) = safe_decel;

                        % Log this limiting
                        if abs(wheel_accelerations(i)) > 0.01
                            disp(['Limiting wheel ', num2str(i), ' acceleration to prevent direction reversal. Current vel: ', ...
                                num2str(current_velocity), ', cmd accel: ', num2str(wheel_accelerations(i))]);
                        end
                    end
                end

                % ADDED: Additional safety for high-speed wheels
                if velocity_percentage > 0.7
                    % For wheels already at high speeds, be more conservative with acceleration
                    max_safe_accel = (1 - velocity_percentage) * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration;

                    if abs(wheel_accelerations(i)) > max_safe_accel && sign(wheel_accelerations(i)) == sign(current_velocity)
                        % Only limit acceleration in the same direction as current velocity
                        wheel_accelerations(i) = sign(wheel_accelerations(i)) * max_safe_accel;
                    end
                end
            end
        end

        %% [ ] Methods: Safely Apply Reaction Wheels Command
        function apply_reaction_wheel_commands(~, mission, i_SC, wheel_accelerations)
            % Safely apply calculated commands to the reaction wheels

            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                if mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.health && abs(wheel_accelerations(i)) > 0
                    % Additional safety check before applying command
                    if abs(wheel_accelerations(i)) > mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration
                        % Limit command to maximum acceleration while preserving direction
                        wheel_accelerations(i) = sign(wheel_accelerations(i)) * mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.maximum_acceleration;
                    end

                    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.flag_executive = 1;
                    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = wheel_accelerations(i);
                else
                    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.flag_executive = 0;
                    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = 0;
                end
            end
        end

        %% [ ] Methods: Handle wheel desaturation
        function handle_desaturation(obj, mission, i_SC)
            % Perform wheel desaturation if required

            % Define desaturation threshold (in radians per second)
            % Change from 10 RPM to 100 RPM (~10.47 rad/s)
            desaturation_threshold = 100 * 2 * pi / 60; % 100 RPM

            wheels = mission.true_SC{i_SC}.true_SC_reaction_wheel;

            % Check if desaturation is complete for all wheels
            all_wheels_safe = true;
            was_in_desaturation = obj.desaturation_procedure;

            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                wheel = wheels{i};
                if abs(wheel.angular_velocity) > desaturation_threshold
                    all_wheels_safe = false;
                    break;
                end
            end

            if all_wheels_safe
                if was_in_desaturation
                    % Just exited desaturation mode - update capabilities
                    obj.desaturation_procedure = false;
                    obj = update_torque_capabilities(obj, mission, i_SC);
                end
                return;
            else
                if ~was_in_desaturation
                    % Just entered desaturation mode - update capabilities
                    obj.desaturation_procedure = true;
                    obj = update_torque_capabilities(obj, mission, i_SC);
                else
                    obj.desaturation_procedure = true;
                end
            end

            wheel_torque = zeros(1,3);

            % Desaturate individual wheels and compute thruster torques
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                wheel = wheels{i};

                % Calculate command to reduce wheel velocity - typically 10% per time step
                cmd = -wheel.angular_velocity * 0.1; % 

                % Apply safety limit to desaturation command
                if abs(cmd) > wheel.maximum_acceleration
                    % Limit the commanded acceleration to maximum safe value
                    cmd = sign(cmd) * wheel.maximum_acceleration;
                end

                wheel.commanded_angular_acceleration = cmd;
                wheel.flag_executive = true;

                % Compute the resulting torque from the reaction wheel
                wheel_torque = wheel_torque + (wheel.moment_of_inertia * cmd * wheel.orientation);
            end

            % Compute the required thruster torque to compensate
            residual_torque = obj.desired_control_torque - wheel_torque;
            % Optimize thruster operation
            obj.func_decompose_control_torque_into_thrusters_optimization_kkt(mission, i_SC, residual_torque);
        end


        %% [ ] Methods: Checks if desaturation is needed
        function needed = is_desaturation_needed(obj, mission, i_SC)
            % Simplified function to check if wheel desaturation is needed
            % Returns true if any wheel is saturated or desaturation is in progress
            
            needed = false;
            
            % If desaturation is already in progress, continue it
            if obj.desaturation_procedure
                needed = true;
                return;
            end
            
            % Check each reaction wheel for saturation
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                
                % Only consider healthy wheels
                if mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.health && mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.saturated
                    needed = true;
                    return;
                end
            end
        end

        %% [ ] Methods: Optimize Thrusters using Pseudo Inverse
        function optimize_thruster_pinv(obj, mission, i_SC, desired_torque)

            % Optimize the use of micro-thrusters for the desired control torque
            if nargin < 4
                % Allows to pass another torque as parameter
                desired_torque = obj.desired_control_torque';
            end

            % Solve for thrusts using linear least squares
            thrusts = pinv(obj.thruster_contribution_matix) * desired_torque;

            % Apply thrust min limit
            for i=1:length(thrusts)
                if(thrusts(i) < mission.true_SC{i_SC}.true_SC_micro_thruster{i}.minimum_thrust)
                    if (rem(i, 2) == 0)
                        % Add on the next one
                        thrusts(i-1) = thrusts(i-1)+abs(thrusts(i));
                    else
                        thrusts(i+1) = thrusts(i+1)+abs(thrusts(i));
                    end
                    thrusts(i) = 0;
                end
            end

            % Scale all thrusts proportionally if any exceed the maximum limit
            if max(thrusts) > max(obj.max_thrust)
                scaling_factor = max(thrusts) / max(obj.max_thrust);
                thrusts = thrusts / scaling_factor;
            end


            % Assign the thrust individually
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                if thrusts(i) > mission.true_SC{i_SC}.true_SC_micro_thruster{i}.minimum_thrust
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 1;
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = thrusts(i);
                else
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 0;
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = 0;
                end
            end
        end

        function obj = func_initialize_optimization_data(obj, mission, i_SC)
            % Assumption: The thrust of each microthruster can be compensated by the
            % thrust of another microthruster with an equal and opposite resultant torque.

            %% Step 1: Identify microthrusters with the same resultant torques
            % Extract the thruster input matrix, where each row represents a microthruster's contribution.
            % Columns correspond to force/torque contributions in each axis.

            % Initialize the thruster input array
            thruster_input_array = zeros(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster, 3);

            % Get the center of mass
            center_of_mass = mission.true_SC{1, 1}.true_SC_body.location_COM;

            % Loop through each thruster
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                % Calculate the relative position vector
                relative_position = mission.true_SC{i_SC}.true_SC_micro_thruster{i}.location - center_of_mass;

                % Compute the torque using the cross product
                torque = cross(relative_position, mission.true_SC{i_SC}.true_SC_micro_thruster{i}.orientation);

                % Round the result to three decimal places
                torque_rounded = round(torque, 3);

                % Store the result in the thruster input array
                thruster_input_array(i, :) = torque_rounded;
            end

            % Number of microthrusters
            N_mt = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster;

            % Flags to track checked microthrusters
            flag_mt_checked = zeros(N_mt, 1);

            % Preallocate for storing indices and matrix of same torque directions
            idxs_bool_same_torque = []; % Logical array for MTs with the same torque
            matrix_same_torque = [];   % Reduced input matrix for MTs with same torque

            % Iterate through each microthruster
            for i_mt = 1:N_mt
                if flag_mt_checked(i_mt)
                    continue; % Skip already processed microthrusters
                end

                % Extract the torque direction vector for the current microthruster
                torque_dir_i = thruster_input_array(i_mt, :);

                % Identify microthrusters with the same torque direction
                idxs_bool = ismember(thruster_input_array, torque_dir_i, 'rows');

                % Append the identified torque direction and indices to the results
                matrix_same_torque = [matrix_same_torque; torque_dir_i];
                idxs_bool_same_torque = [idxs_bool_same_torque; idxs_bool'];

                % Mark these microthrusters as processed
                flag_mt_checked(idxs_bool) = 1;
            end

            % Convert the logical indices into a boolean array
            idxs_bool_same_torque = boolean(idxs_bool_same_torque);

            %% Step 2: Identify microthrusters with opposite resultant torques
            % Extract the size of the reduced matrix containing same torques
            N_mt_same = size(matrix_same_torque, 1);

            % Reset flags to track processed microthrusters
            flag_mt_checked = zeros(N_mt, 1);

            % Preallocate for storing indices and matrix of opposite torque directions
            idxs_num_opposite_torque = []; % Numerical indices for MT pairs with opposite torque
            matrix_opposite_torque = [];  % Reduced input matrix for opposite torques

            % Iterate through the reduced set of same torque microthrusters
            for i_mt = 1:N_mt_same
                if flag_mt_checked(i_mt)
                    continue; % Skip already processed microthrusters
                end

                % Extract the torque direction vector for the current microthruster
                torque_dir_i = matrix_same_torque(i_mt, :);

                % Identify microthrusters with opposite torque direction
                idxs_bool = ismember(matrix_same_torque, -torque_dir_i, 'rows');

                % Ensure there is exactly one matching opposite torque
                assert(sum(idxs_bool) == 1, 'There must be a unique opposite torque match.');

                % Find the index of the matching opposite torque
                j_mt = find(idxs_bool);

                % Append the torque direction and indices to the results
                matrix_opposite_torque = [matrix_opposite_torque; matrix_same_torque(i_mt, :)];
                idxs_num_opposite_torque = [idxs_num_opposite_torque; [i_mt, j_mt]];

                % Mark these microthrusters as processed
                flag_mt_checked([i_mt, j_mt]) = 1;
            end

            %% Step 3: Formulate the optimization problem

            % The goal is to distribute thrusts across the
            % microthrusters such that the net torque matches the desired control torque
            % This is a least-squares problem

            % The optimization minimizes ||Ax - b||_2^2 subject to Cx = d.

            % Number of microthrusters with opposite torques
            N_mt_opposite = size(matrix_opposite_torque, 1);

            % Problem dimensions
            n = N_mt_opposite; % Number of variables (thruster pairs)
            p = 3;             % Number of constraints (force/torque balance in 3D)

            % Constraint matrix: Each column corresponds to the torque contribution of a thruster pair
            C = matrix_opposite_torque';

            % Quadratic cost function: Ax approximates b, initialized as identity for simplicity
            A = eye(n);
            b = zeros(n, 1);

            % Form the Karush-Kuhn-Tucker (KKT) system
            % E = [A'A  C';
            %      C    O]
            O = zeros(p, p); % Zero block matrix for constraints
            E = [A' * A, C';
                C,      O];

            % Perform LU decomposition of the KKT matrix for efficient solution
            [L, U, P] = lu(E);

            %% Step 4: Store optimization data in obj
            obj.optim_data.A = A;
            obj.optim_data.b = b;
            obj.optim_data.U = U;
            obj.optim_data.L = L;
            obj.optim_data.P = P;
            obj.optim_data.n = n;
            obj.optim_data.N_mt = N_mt;
            obj.optim_data.N_mt_opposite = N_mt_opposite;
            obj.optim_data.N_mt_same = N_mt_same;
            obj.optim_data.idxs_num_opposite_torque = idxs_num_opposite_torque;
            obj.optim_data.idxs_bool_same_torque = idxs_bool_same_torque;
        end

        %% [ ] Methods: Optimize Thrusters using KKT Condition
        function obj = func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC, desired_torque)


            % Optimize the use of micro-thrusters for the desired control torque
            if nargin < 4
                % Allows to pass another torque as parameter
                desired_torque = obj.desired_control_torque;
            end

            %% Step 1: Reset thruster commands
            % Reset the commanded thrust and torque for each microthruster to ensure a clean start
            reset_thrusters(obj,mission, i_SC);

            %% Step 2: Retrieve pre-computed optimization data
            % Extract optimization data that was initialized during the preparation phase.
            A = obj.optim_data.A;                                   % Quadratic cost matrix
            b = obj.optim_data.b;                                   % Right-hand side of the cost function
            U = obj.optim_data.U;                                   % Upper triangular matrix from LU decomposition
            L = obj.optim_data.L;                                   % Lower triangular matrix from LU decomposition
            P = obj.optim_data.P;                                   % Permutation matrix from LU decomposition
            n = obj.optim_data.n;                                   % Number of optimization variables
            N_mt = obj.optim_data.N_mt;                             % Total number of microthrusters
            N_mt_opposite = obj.optim_data.N_mt_opposite;           % Number of thruster pairs with opposite torque
            N_mt_same = obj.optim_data.N_mt_same;                   % Number of unique torque directions
            idxs_num_opposite_torque = obj.optim_data.idxs_num_opposite_torque; % Indices of opposite torque pairs
            idxs_bool_same_torque = obj.optim_data.idxs_bool_same_torque;       % Logical array of same torque indices

            %% Step 3: Solve the optimization problem using the KKT system
            % Formulate the KKT system and solve for the optimal thrust values.
            d = desired_torque';                         % Desired control torque for the spacecraft
            f = [A'*b; d];                                        % Combine cost and constraint terms into a single vector
            xz = U\(L\(P*f));                                 % Solve the KKT system using LU decomposition
            x_opposite = xz(1:n);                                   % Extract the solution for thruster pairs

            %% Step 4: Undo the torque pair reduction
            % Recover the thrust values for all microthrusters from the reduced solution.
            x_same = zeros(N_mt_same, 1);                           % Preallocate thrust values for unique torque directions
            for i_mt = 1:N_mt_opposite
                idxs_num_i = idxs_num_opposite_torque(i_mt, :);     % Get the indices of the current pair
                x_same(idxs_num_i(1)) = max(0, +x_opposite(i_mt));  % Assign positive thrust to the first thruster
                x_same(idxs_num_i(2)) = max(0, -x_opposite(i_mt));  % Assign negative thrust to the second thruster
            end

            % Distribute the thrust values among all microthrusters with the same torque
            x = zeros(N_mt, 1);                                     % Preallocate full thrust vector
            for i_mt = 1:N_mt_same
                idxs_bool_i = idxs_bool_same_torque(i_mt, :);       % Logical indices for thrusters with same torque
                x(idxs_bool_i) = x_same(i_mt) / sum(idxs_bool_i);   % Evenly distribute thrust among the group
            end

            % Scale all thrusts proportionally if any exceed the maximum limit
            if max(x) > max(obj.max_thrust)
                scaling_factor = max(x) / max(obj.max_thrust);
                x = x / scaling_factor;
            end

            %% Step 5: Assign computed thrust and torque to each microthruster
            % Assign the thrust individually and safely
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                if x(i) >  mission.true_SC{i_SC}.true_SC_micro_thruster{i}.minimum_thrust

                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 1;
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = x(i);
                else
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation   = 0;
                    mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = 0;
                end
            end
        end

        function func_compute_rw_command_kkt(obj, mission, i_SC)
            % Compute reaction wheel commands to achieve desired control torque

            if norm(obj.desired_control_torque) > 0

                % Get the number of reaction wheels
                N = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel;

                % Compute required wheel accelerations using optimization
                n = 4 * N; % number of variables
                p = 3 * (N + 1); % number of constraints

                % Constraint vector initialization
                d = zeros(p, 1);
                for i = 1:N
                    idx1 = (i-1)*3 + 1;
                    idx2 = (i-1)*3 + 3;
                    % Assuming dot_angular_velocity is correctly accessed
                    d(idx1:idx2, 1) = mission.true_SC{i_SC}.software_SC_estimate_attitude.dot_angular_velocity';
                end

                % Scale desired torque vector
                torque_desired = obj.desired_control_torque;
                max_torque = mission.true_SC{i_SC}.true_SC_reaction_wheel{1}.max_torque; % Assuming all RWs have the same max torque
                if max_torque < Inf
                    factor = min(1, (max_torque * 1) / norm(torque_desired)); % envelope_ratio assumed as 1
                else
                    factor = 1;
                end
                d(3*N + 1:p, 1) = factor * torque_desired;

                % Objective matrix setup
                A = diag([zeros(3*N, 1); ones(n - 3*N, 1)]);
                b = zeros(n, 1);

                % Constraint matrix setup
                C = zeros(p, n);
                for i = 1:N
                    idx1 = (i-1)*3 + 1;
                    idx2 = (i-1)*3 + 3;
                    % Assuming orientation gives rotation matrix R (transposed for usage here)
                    R = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.orientation';

                    % Moment of inertia handling
                    moment_of_inertia = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.moment_of_inertia;
                    if any(diag(moment_of_inertia) == 0)
                        % Regularization for singular inertia matrices
                        moment_of_inertia = moment_of_inertia + eye(size(moment_of_inertia)) * 1e-6;
                    end

                    C(idx1:idx2, idx1:idx2) = inv(moment_of_inertia);
                    C(idx1:idx2, 3*N + i) = -R(:, 1);
                    C(3*N + 1:p, idx1:idx2) = eye(3, 3);
                end

                % Solve optimization problem (constrained least squares)
                % Form Karush-Kuhn-Tucker (KKT) system
                O = zeros(p, p);
                E = [A'*A, C'; C, O];
                E = E + eye(size(E)) * 1e-6; % Regularization for numerical stability
                [L, U, P] = lu(E);

                f = [A'*b; d];
                xz = U\(L\(P*f));
                x = xz(1:n);

                % Results storing
                for i = 1:N
                    idx1 = (i-1)*3 + 1;
                    idx2 = (i-1)*3 + 3;
                    % Assign computed torque and angular acceleration
                    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_torque = x(idx1:idx2);
                    % disp("RWA Commanded Torque")
                    % disp(x(idx1:idx2))
                    mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = x(3*N + i, 1);
                    % disp("RWA Commanded Angular Acceleration")
                    % disp(x(3*N + i, 1))

                end

            end
        end

        %% [ ] Methods: Control Attitude Oracle
        % Oracle directly moves the SC's attitude and rotation matrix, without the control actuators
        function obj = func_update_software_SC_control_attitude_Oracle(obj, mission, i_SC)
            % Oracle Move!
            mission.true_SC{i_SC}.true_SC_adc.attitude = obj.desired_attitude; % [quaternion]
            mission.true_SC{i_SC}.true_SC_adc.angular_velocity = obj.desired_angular_velocity; % [rad/sec]

            % Compute Rotation Matrix
            func_update_true_SC_ADC_rotation_matrix(mission.true_SC{i_SC}.true_SC_adc);
        end

        %% [ ] Methods: Desired Control Torque Asymptotically Stable

        function obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC)

            omega_est = mission.true_SC{i_SC}.software_SC_estimate_attitude.angular_velocity; % [rad/sec]
            omega_desired = obj.desired_angular_velocity; % [rad/sec]
            beta_est = func_quaternion_properize(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude); % [quaternion]
            beta_desired = func_quaternion_properize(obj.desired_attitude); % [quaternion]

            Zbeta = zeros(4,3);
            Zbeta(1:3,1:3) =  beta_est(4) * eye(3) + skew(beta_est(1:3));
            Zbeta(4,  1:3) = -beta_est(1:3)';

            % output desired control torque
            omega_r = omega_desired + ( pinv(0.5 * Zbeta) * obj.data.control_gain(2) * (beta_desired - beta_est)' )';
            obj.desired_control_torque = - obj.data.control_gain(1)*(omega_est - omega_r);

        end

        %% [ ] Methods: Desired Control Torque PD

        function obj = function_update_desired_control_torque_PD(obj, mission, i_SC)

            % Parameters
            J = mission.true_SC{i_SC}.true_SC_body.total_MI; % [kg m^2] Spacecraft inertia matrix (excluding wheels)

            Kp = eye(3) * obj.data.control_gain(1);
            Kd = eye(3) * obj.data.control_gain(2);

            % Estimated and final values
            omega_est = mission.true_SC{i_SC}.software_SC_estimate_attitude.angular_velocity; % [rad/sec]
            omega_desired = obj.desired_angular_velocity; % [rad/sec]
            delta_omega = omega_desired - omega_est;


            beta_est = func_quaternion_properize(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude); % [quaternion]
            beta_desired = func_quaternion_properize(obj.desired_attitude); % [quaternion]

            % Calculate Delta q and Delta theta
            delta_beta = func_quaternion_multiply(beta_desired, func_quaternion_conjugate(beta_est));
            delta_theta = 2 * delta_beta(1:3); % Assuming small angle approximations

            % Calculate control torques
            uc = (J * (Kd * delta_omega' + Kp * delta_theta') )';

            % Gyroscopic terms
            vc = cross(omega_est, (J * omega_est')' ); % meas_rw_momentum

            % Total control torque
            obj.desired_control_torque = uc + vc;

        end

        %% [ ] Methods: Desired Attitude CVX
        % Compute Desired Attitude using CVX

        function obj = func_compute_desired_attitude_CVX(obj)
            disp('Compute Desired Attitude using CVX')

            cvx_begin
            variable r(3,3)
            minimize ( norm(r * obj.data.secondary_vector' - obj.data.desired_secondary_vector') + norm(r' * obj.data.desired_secondary_vector' - obj.data.secondary_vector'))
            subject to

            r * obj.data.primary_vector' == obj.data.desired_primary_vector';
            r' * obj.data.desired_primary_vector' == obj.data.primary_vector';

            cvx_end

            [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);

            SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
            SC_beta_4_desired = cos(SC_Phi_desired/2);
            obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];

        end

        %% [ ] Methods: Desired Attitude Array Search
        % Compute Desired Attitude using Array Search
        % The array search component iteratively searches for an optimized secondary alignment by testing different rotation angles around the primary vector.

        function obj = func_compute_desired_attitude_Array_Search(obj)

            % 1) Aligning the Primary Vector
            primary_vector = obj.data.primary_vector;
            desired_primary_vector = obj.data.desired_primary_vector;

            % Compute rotation to rotate pointing aligned with target (in J2000)
            v = cross(primary_vector, desired_primary_vector);
            Rot_primary = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(primary_vector, desired_primary_vector)));

            % 2) Optimizing the Secondary Vector
            theta = 0:2*pi/60:2*pi;
            optimized_error_SP_pointing = zeros(1,length(theta));

            desired_secondary_vector = obj.data.desired_secondary_vector;
            secondary_vector = obj.data.secondary_vector;

            for i=1:length(theta)
                U = desired_primary_vector;
                R = func_axis_angle_rot(U, theta(i));
                optimized_error_SP_pointing(i) = func_angle_between_vectors(desired_secondary_vector, R*secondary_vector');
            end

            [~,index_optimized_theta] = min(optimized_error_SP_pointing);
            Rot_secondary = func_axis_angle_rot(U, theta(index_optimized_theta));

            % 3) Combine Rotations
            r = Rot_secondary * Rot_primary;

            % 4) Convert Rotation Matrix to Quaternion
            [SC_e_desired, SC_Phi_desired] = func_decompose_rotation_matrix(r);

            SC_beta_v_desired = SC_e_desired' * sin(SC_Phi_desired/2);
            SC_beta_4_desired = cos(SC_Phi_desired/2);

            obj.desired_attitude = func_quaternion_properize([SC_beta_v_desired, SC_beta_4_desired]);

        end

        %% [ ] Methods: Update torque capabilities
        % Should be called if hardware configuration/health changes
        function obj = update_torque_capabilities(obj, mission, i_SC)
            % Recalculate maximum torque capabilities after hardware changes
            obj.max_rw_torque = obj.calculate_max_reaction_wheel_torque(mission, i_SC);
            obj.max_mt_torque = obj.calculate_max_thruster_torque(mission, i_SC);
        end

        %% [ ] Methods: Calculate maximum reaction wheel torque capability
        function max_torque = calculate_max_reaction_wheel_torque(obj, mission, i_SC)
            % Calculate the maximum torque that can be provided by the reaction wheel array

            % Initialize maximum torque
            max_torque = 0;

            % Count active (healthy) wheels
            active_wheels = 0;

            % Check each reaction wheel
            for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                wheel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i};

                if wheel.health
                    % For each healthy wheel, add its maximum torque capability
                    wheel_max_torque = wheel.maximum_torque;

                    % Calculate projection of wheel torque onto each axis
                    wheel_torque_projection = wheel.orientation * wheel_max_torque;

                    % Add to overall torque capability
                    max_torque = max_torque + norm(wheel_torque_projection);

                    active_wheels = active_wheels + 1;
                end
            end

            % Apply a configuration factor based on wheel geometry
            % This is a conservative estimate of what the wheel array can actually provide
            if active_wheels > 0
                % The factor accounts for the fact that not all wheels can contribute equally
                % to torque in all directions
                config_factor = 1/sqrt(active_wheels);
                max_torque = max_torque * config_factor;
            end

            return;
        end

        %% [ ] Methods: Calculate maximum thruster torque capability
        function max_torque = calculate_max_thruster_torque(obj, mission, i_SC)
            % Calculate the maximum torque that can be provided by micro thrusters

            % Use the thruster contribution matrix and maximum thrust values
            % to calculate maximum possible torque

            % Initialize maximum torque
            max_torque = 0;

            % Calculate maximum torque for each principal axis
            for axis = 1:3
                % Calculate maximum positive torque
                pos_torque = 0;
                neg_torque = 0;

                for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                    thruster = mission.true_SC{i_SC}.true_SC_micro_thruster{i};

                    % Skip unhealthy thrusters
                    if ~thruster.health
                        continue;
                    end

                    % Get torque contribution of this thruster for this axis
                    torque_contrib = obj.thruster_contribution_matix(axis, i);

                    % Add to positive or negative torque as appropriate
                    if torque_contrib > 0
                        pos_torque = pos_torque + torque_contrib * thruster.maximum_thrust;
                    elseif torque_contrib < 0
                        neg_torque = neg_torque + abs(torque_contrib) * thruster.maximum_thrust;
                    end
                end

                % The maximum torque for this axis is the minimum of positive and negative capability
                % (since we need to be able to control in both directions)
                axis_max_torque = min(pos_torque, neg_torque);

                % The overall maximum torque is the maximum across all axes
                max_torque = max(max_torque, axis_max_torque);
            end

            return;
        end

        %% Helper function: Calculate angle between quaternions
        function angle = func_angle_between_quaternions(obj, q1, q2)
            % Calculates the angle between two quaternions in radians
            % This is the minimum rotation angle to get from one orientation to the other

            % Ensure quaternions are normalized
            q1 = q1 / norm(q1);
            q2 = q2 / norm(q2);

            % Calculate the quaternion product q1^-1 * q2
            % For a quaternion, the inverse is the conjugate if it's normalized
            q1_conj = q1;
            q1_conj(1:3) = -q1_conj(1:3); % Conjugate: negate vector part

            % Quaternion multiplication
            q_diff = zeros(4,1);
            if length(q1) == 4 && length(q2) == 4
                q_diff(1) = q1_conj(4)*q2(1) + q1_conj(1)*q2(4) + q1_conj(2)*q2(3) - q1_conj(3)*q2(2);
                q_diff(2) = q1_conj(4)*q2(2) + q1_conj(2)*q2(4) + q1_conj(3)*q2(1) - q1_conj(1)*q2(3);
                q_diff(3) = q1_conj(4)*q2(3) + q1_conj(3)*q2(4) + q1_conj(1)*q2(2) - q1_conj(2)*q2(1);
                q_diff(4) = q1_conj(4)*q2(4) - q1_conj(1)*q2(1) - q1_conj(2)*q2(2) - q1_conj(3)*q2(3);

                % The rotation angle is 2*acos(q_diff(4))
                % Clamp scalar part to [-1,1] to avoid numerical errors
                angle = 2 * acos(min(1, max(-1, q_diff(4))));
            else
                % Handle invalid quaternions by returning a large angle
                angle = pi;
                warning('Invalid quaternion dimensions in func_angle_between_quaternions');
            end
        end

    end

end