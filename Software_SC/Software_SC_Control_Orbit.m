%% Class: Software_SC_Control_Orbit
% Control the Orbit of the Spacecraft

classdef Software_SC_Control_Orbit < handle
    % Software_SC_Control_Orbit: Manages spacecraft control parameters for orbiting small bodies.
    % Includes functionalities for trajectory calculations, intercept predictions,
    % and delta-V computations tailored for spacecraft mission scenarios.

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        mode_software_SC_control_orbit_selector % [string] Different orbit control modes
        % - 'DART'

        max_time_before_control % [s]

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Control Orbit

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        data % Other useful data

        last_time_control % [s]
        flag_executive    % [bool]


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

        desried_attitude_for_hovering_achieved % [boolean]

        deflection_force_computed % Force exerted ON the target

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

        %% [ ] Properties: Storage Variables

        store % Structure to store historical data
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Control_Orbit(init_data, mission, i_SC)
            % Initialize the spacecraft control orbit class

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Control Orbit']; % [string]

            obj.mode_software_SC_control_orbit_selector = init_data.mode_software_SC_control_orbit_selector;
            obj.max_time_before_control = init_data.max_time_before_control;

            obj.flag_executive = 0;
            obj.last_time_control = 0;

            obj.instantaneous_data_generated_per_sample = 0; % [kb]

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

            obj.store.deflection_force_computed = zeros(mission.storage.num_storage_steps_attitude, 3);

            obj.desired_DeltaV_achieved = 0;
            obj.total_DeltaV_executed = [0 0 0]'; % [m/sec]
            obj.desired_attitude_for_DeltaV_achieved = 0;

            % Initialize maneuver tracking properties
            obj.maneuver_start_time = 0;
            obj.thruster_fired_successfully = false;

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
        end
            

        %% [ ] Methods: Store
        % Update the store variable

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
             
        

        %% [ ] Methods: Main
        % Main Function

        function func_main_software_SC_control_orbit(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_control_orbit_selector

                    case 'DART'
                        obj = func_update_software_SC_control_orbit_DART(obj, mission, i_SC);
                    
                    case 'IBEAM'
                        obj = func_update_software_SC_control_orbit_IBEAM(obj, mission, i_SC);

                    case 'Inactive'
                        % Do nothing!

                    otherwise
                        error('Orbit Control mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end

            % Update Storage
            obj.func_update_software_SC_Control_Orbit_store(mission);

            % Reset Variables
            obj.flag_executive = 0;


        end

        

        

    end
end
