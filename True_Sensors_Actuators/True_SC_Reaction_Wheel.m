%% Class: True_SC_Reaction_Wheel
% Represents a single reaction wheel for spacecraft attitude control
classdef True_SC_Reaction_Wheel < handle

    %% Properties
    properties
        %% [EXTERNAL] Parameters set by systems engineers
        name                         % Name of the reaction wheel
        health                       % Health status (0: Not working, 1: Working)
        location                     %[m] : Location of the reaction wheel in the body frame
        orientation                  %[Unit vector] : Axis of rotation in the body frame
        max_torque                   %[Nm] : Maximum torque the wheel can apply
        max_angular_velocity         %[rad/s] : Maximum angular velocity
        moment_of_inertia            %[kg·m^2] : Moment of inertia of the wheel
        radius                       %[m] radius of 1 RW
        mass                         %[kg] : Mass of the reaction wheel
        power_consumed_angular_velocity_array % [power_array ; velocity_array]
        angular_velocity_noise       %[rad/s] : Random noise added to commanded torque
        rpm_values                   % RPM values for interpolation
        torque_values                % Torque values for interpolation
        power_matrix                 % Power consumption matrix for interpolation
        
        %% [INTERNAL] Computed internally
        temperature                  %[deg C]
        total_momentum               %[kg·m^2/s] : Total momentum of the wheel
        inertia_matrix               %[kg·m^2] : Inertia matrix of the wheel
        commanded_angular_acceleration % [rad/s] : Desired angular velocity of the wheel
        actual_angular_acceleration  % [rad/s] : Actual realised angular velocity of the wheel
        min_acceleration             % [rad/s^-2]
        angular_velocity             %[rad/s] : Current angular velocity of the wheel
        commanded_torque             %[Nm] : Torque commanded by the control system
        actual_torque                %[Nm] : Actual torque considering saturation and noise
        saturated                    %[Bool] Is the wheel currently saturated ?
        instantaneous_power_consumption %[Watts] : Power consumed during operation
        instantaneous_power_consumed    %[Watts] : Alias for power tracking system
        command_actuation_power_consumed %[Watts] : Power consumed during actuation
        instantaneous_data_generated %[Kb] : Data generated during the current time step
        store                        %[Struct] : Store of the reaction wheel
        flag_executive               %[Bool] : Is the wheel currently executing a command ?
        envelope_ratio               %[Ratio] : Ratio of the minimum distance between wheels to the radius of the wheel
        maximum_torque               %[Nm] : Maximum torque the wheel can apply
        momentum_capacity            %[kg·m^2/s] : Momentum capacity of the wheel
        maximum_acceleration         %[rad/s^2] : Maximum acceleration the wheel can apply
    end

    %% Methods
    methods
        %% Constructor
        function obj = True_SC_Reaction_Wheel(init_data, mission, i_SC, i_RW)

            obj.name = ['Reaction Wheel ', num2str(i_RW)];

            obj.health = true;
            obj.location = init_data.location;
            obj.orientation = init_data.orientation;
            obj.max_angular_velocity = init_data.max_angular_velocity;
            obj.radius = init_data.radius;
            obj.mass = init_data.mass;

            % Calculate moment of inertia
            % For a disk rotating around its center axis : 1/2*m*r^2
            obj.moment_of_inertia = 0.5 * obj.mass * obj.radius^2;

            obj.power_consumed_angular_velocity_array = init_data.power_consumed_angular_velocity_array;

            % Torque envelope
            if isfield(init_data, 'max_torque')
                % Maximum Torque and Momentum Envelopes for Reaction Wheel Arrays
                % https://ntrs.nasa.gov/api/citations/20110015369/downloads/20110015369.pdf
                assert(ismember(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel, [3 4 5 6]), 'Number of reaction wheel must be 3, 4, 5 or 6')
                switch  mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                    case 3
                        d_min = 1; % d_12
                    case 4
                        d_min = 1.633; % d_12 = d_13
                    case 5
                        d_min = 5/8 * 2.667; % d_24 = d_42
                    case 6
                        d_min = 2.667; % d_23
                end
                obj.envelope_ratio = d_min;
                obj.maximum_torque = init_data.max_torque;
                
                % Get maximum acceleration from init_data if provided, otherwise calculate it
                if isfield(init_data, 'maximum_acceleration')
                    obj.maximum_acceleration = init_data.maximum_acceleration;
                else
                    obj.maximum_acceleration = obj.maximum_torque / obj.moment_of_inertia;  % rad/s^2
                end
            else
                obj.envelope_ratio = 1;
                obj.maximum_torque = Inf;
                obj.momentum_capacity = Inf;
                obj.maximum_acceleration = Inf;
            end

            obj.min_acceleration = 1; % rad/s^2

            obj.angular_velocity_noise = init_data.angular_velocity_noise;

            % Initialize dynamic state
            obj.angular_velocity = 0;
            obj.commanded_angular_acceleration = 0;
            obj.saturated = false;
            obj.flag_executive = false;
            obj.instantaneous_power_consumption = 0;
            obj.instantaneous_power_consumed = 0;

            obj.actual_torque = [0,0,0];

            % Initialize storage
            obj.total_momentum = 0;

            obj.store = [];
            obj.store.angular_velocity = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.torque = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.saturated = zeros(mission.storage.num_storage_steps_attitude, 1);;

            obj.store.commanded_angular_acceleration = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.actual_angular_acceleration = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.actual_torque = zeros(mission.storage.num_storage_steps_attitude, 3);

            obj.store.max_angular_velocity = obj.max_angular_velocity;

            % Register with power system
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

        end

        %% Update Storage
        function obj = func_update_reaction_wheel_store(obj, mission)
            obj.store.angular_velocity(mission.storage.k_storage_attitude, :) = obj.angular_velocity;
            obj.store.actual_torque(mission.storage.k_storage_attitude, :) = obj.actual_torque;
            obj.store.commanded_angular_acceleration(mission.storage.k_storage_attitude, :) = obj.commanded_angular_acceleration;
            obj.store.saturated(mission.storage.k_storage_attitude, :) = obj.saturated;
        end

        function func_main_true_reaction_wheel(obj, mission, i_SC, ~)
            % Main function that runs the reaction wheel simulation
            if ~obj.health
                obj.commanded_angular_acceleration = 0;
                obj.angular_velocity = 0;
                obj.instantaneous_power_consumption = 0;
                obj.instantaneous_power_consumed = 0;
                obj.instantaneous_data_generated = 0;
                obj.actual_torque = [0,0,0];
                obj.actual_angular_acceleration = 0;
                return;
            end

            if(obj.flag_executive && abs(obj.commanded_angular_acceleration) > 0)

                % Limit commanded acceleration
                if abs(obj.commanded_angular_acceleration) > obj.maximum_acceleration
                    obj.commanded_angular_acceleration = sign(obj.commanded_angular_acceleration) * obj.maximum_acceleration;
                end

                % Calculate torque directly from acceleration (τ = I*α)
                torque_magnitude = obj.moment_of_inertia * obj.commanded_angular_acceleration;
                obj.actual_torque = torque_magnitude * obj.orientation;

                % Update velocity with noise and limits
                if obj.commanded_angular_acceleration ~= 0
                    % Add noise only during active commands
                    current_vel_noise = obj.angular_velocity_noise * (2*rand() - 1);
                else
                    current_vel_noise = 0; % No noise when idle
                end

                % Calculate the new velocity without limits
                unconstrained_velocity = obj.angular_velocity + obj.commanded_angular_acceleration * mission.true_time.time_step_attitude + current_vel_noise;
                
                % Apply rate limiting to prevent dramatic velocity changes in a single step
                % Maximum allowed change in velocity per time step (10% of max velocity is a reasonable value)
                max_velocity_change = 0.1 * obj.max_angular_velocity;
                
                % ENHANCED SAFETY: Add more conservative limits for direction reversals
                % Detect potential reversal (wheel going one way, command pushing it the complete other way)
                if sign(unconstrained_velocity) ~= sign(obj.angular_velocity) && abs(obj.angular_velocity) > 0.3 * obj.max_angular_velocity
                    % This is a potential direction reversal and the wheel is at significant speed
                    % Reduce maximum allowed change dramatically for safer deceleration
                    max_velocity_change = 0.02 * obj.max_angular_velocity;
                    
                    % Log this event to help debugging
                    disp(['REACTION WHEEL SAFETY: Detected potential rapid direction reversal for ', obj.name]);
                    disp(['Current velocity: ', num2str(obj.angular_velocity), ' rad/s']);
                    disp(['Commanded acceleration: ', num2str(obj.commanded_angular_acceleration), ' rad/s^2']);
                    disp(['Limiting velocity change to ', num2str(max_velocity_change), ' rad/s per time step']);
                end
                
                % Limit the velocity change
                if abs(unconstrained_velocity - obj.angular_velocity) > max_velocity_change
                    limited_velocity = obj.angular_velocity + sign(unconstrained_velocity - obj.angular_velocity) * max_velocity_change;
                else
                    limited_velocity = unconstrained_velocity;
                end
                
                % Apply absolute velocity limits (don't exceed max angular velocity)
                new_velocity = max(min(limited_velocity, obj.max_angular_velocity), -obj.max_angular_velocity);
                
                % If the velocity didn't change much, leave it as is to avoid numerical issues
                velocity_tolerance = 1e-4;
                if abs(new_velocity - obj.angular_velocity) < velocity_tolerance
                    new_velocity = obj.angular_velocity; % Clamp near-zero velocities
                end

                % Calculate the actual acceleration that occurred (for telemetry)
                obj.actual_angular_acceleration = (new_velocity - obj.angular_velocity) / mission.true_time.time_step_attitude;
                
                % Update the angular velocity
                obj.angular_velocity = new_velocity;

                % Check for saturation
                % > If reaches 80% of max angular velocity
                obj.saturated = (abs(obj.angular_velocity) >= obj.max_angular_velocity * 0.80);

                % Update spacecraft torque directly
                mission.true_SC{i_SC}.true_SC_adc.control_torque = ...
                    mission.true_SC{i_SC}.true_SC_adc.control_torque + obj.actual_torque'; 
                                
                % Update power and data
                obj.instantaneous_power_consumption = abs(obj.actual_torque * obj.angular_velocity);


                % Calculate individual wheel momentum: h = I * ω * direction 
                % The wheel momentum is a vector along the wheel's spin axis (orientation)
                momentum_magnitude = obj.moment_of_inertia * obj.angular_velocity;
                obj.total_momentum = momentum_magnitude * obj.orientation;

                            
                % Update power and data
                power_consumptions = obj.power_consumed_angular_velocity_array(1, :);
                angular_velocities = obj.power_consumed_angular_velocity_array(2, :);
                
                % Ensure the array is sorted by angular velocity
                [angular_velocities, sortIdx] = sort(angular_velocities);
                power_consumptions = power_consumptions(sortIdx);
                
                % Interpolate to find the power consumption for the current angular velocity
                if obj.angular_velocity < min(angular_velocities)
                    obj.instantaneous_power_consumption = power_consumptions(1);
                elseif obj.angular_velocity > max(angular_velocities)
                    obj.instantaneous_power_consumption = power_consumptions(end);
                else
                    obj.instantaneous_power_consumption = interp1(angular_velocities, power_consumptions, obj.angular_velocity, 'linear');
                end
                % Set property for power tracking system
                obj.instantaneous_power_consumed = obj.instantaneous_power_consumption;
                            
                % Update the power system with this consumption
                func_update_instantaneous_power_consumed_attitude(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            

                obj.instantaneous_data_generated = obj.instantaneous_power_consumption / 10;

            else
                obj.commanded_angular_acceleration = 0;
                obj.instantaneous_power_consumption = 0;
                obj.instantaneous_power_consumed = 0;
                obj.instantaneous_data_generated = 0;
            end

            % Update storage
            func_update_reaction_wheel_store(obj, mission);
            
            % CRITICAL FIX: Reset command flags at the end of each cycle
            % This prevents commands from persisting indefinitely between cycles
            obj.flag_executive = false;
            obj.commanded_angular_acceleration = 0;
        end
    end
end
