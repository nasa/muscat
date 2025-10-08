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

        error_angle_desired_attitude % [radians] : Angle error between desired_attitude and true attitude

        desired_angular_velocity % [rad/sec] : Angular velocity of inertial frame I with respect to the body frame B

        desired_control_torque % [Nm] : Desired control torque

        flag_slew % [Boolean] Slew is on?

        desired_slew_angle_change % [deg] Angle between current attitude and desired attitude

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
            obj.flag_slew = 0;

            obj.instantaneous_data_generated_per_sample = 0; % [kb]
            obj.mode_software_SC_control_attitude_selector = init_data.mode_software_SC_control_attitude_selector; % [string]


            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            % Initialization of thruster and RW optimization data
            func_initialize_optimization_data(obj, mission, i_SC);

            % Initialize matrices and hardware data
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster > 0
                obj = func_initialize_micro_thruster_contribution(obj, mission, i_SC);
            end

            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel > 0
                obj = func_initialize_reaction_wheel_contribution(obj, mission, i_SC);
            end

            % Calculate and cache max torque capabilities
            obj.max_rw_torque = func_calculate_max_reaction_wheel_torque(obj, mission, i_SC);
            obj.max_mt_torque = func_calculate_max_thruster_torque(obj, mission, i_SC);

            % All the zeros
            obj.desaturation_procedure = 0;
            obj.desired_attitude = zeros(1,4); % [quaternion]
            obj.error_angle_desired_attitude = 0; % [rad]
            obj.desired_slew_angle_change = 0; % [deg]
            obj.desired_control_torque = zeros(1,3); % [Toraue Vector Nm]
            obj.desired_angular_velocity = zeros(1,3); % [rad/sec]
            obj.data.integral_error = zeros(3,1); % Initialize as 3x1 vector for X/Y/Z axes

            % Initialize control gains
            if isfield(init_data, 'control_gain')
                obj.data.control_gain = init_data.control_gain;
            else
                obj.data.control_gain = [0.1 1];   % [Kr ; Lambda_r] for RWA/MT control
            end
            obj.data.prev_desired_attitude = zeros(1,4); % [quaternion]
            obj.data.num_prev_desired_attitude = 60;

            % Initialize Variables to store
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps_attitude, length(obj.flag_executive));
            obj.store.flag_slew = zeros(mission.storage.num_storage_steps_attitude, length(obj.flag_slew));
            obj.store.desaturation_procedure = zeros(mission.storage.num_storage_steps_attitude, length(obj.desaturation_procedure));
            obj.store.desired_attitude = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_attitude));
            obj.store.desired_angular_velocity = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_angular_velocity));
            obj.store.desired_control_torque = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_control_torque));
            obj.store.error_angle_desired_attitude = zeros(mission.storage.num_storage_steps_attitude, length(obj.error_angle_desired_attitude));
            obj.store.desired_slew_angle_change = zeros(mission.storage.num_storage_steps_attitude, length(obj.desired_slew_angle_change));

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
                obj.store.flag_slew(mission.storage.k_storage_attitude,:) = obj.flag_slew;
                obj.store.desaturation_procedure(mission.storage.k_storage_attitude,:) = obj.desaturation_procedure; % [quaternion]

                obj.store.desired_attitude(mission.storage.k_storage_attitude,:) = obj.desired_attitude; % [quaternion]
                obj.store.desired_angular_velocity(mission.storage.k_storage_attitude,:) = obj.desired_angular_velocity; % [rad/sec]
                obj.store.desired_control_torque(mission.storage.k_storage_attitude,:) = obj.desired_control_torque; % [Nm]

                obj.store.error_angle_desired_attitude(mission.storage.k_storage_attitude,:) = obj.error_angle_desired_attitude; % [rad]
                obj.store.desired_slew_angle_change(mission.storage.k_storage_attitude,:) = obj.desired_slew_angle_change; % [deg]
            end
        end

        
        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_control_attitude(obj, mission, i_SC)

            if (obj.flag_executive == 1)


                switch obj.mode_software_SC_control_attitude_selector

                    case {'DART Oracle', 'DART Control Asymptotically Stable send to ADC directly', 'DART Control PD', 'DART Control Asymptotically Stable send to actuators', 'DART Control Asymptotically Stable send to thrusters'}
                        obj = func_update_software_SC_control_attitude_DART(obj, mission, i_SC);
                    
                    case {'IBEAM Oracle', 'IBEAM Control Asymptotically Stable send to ADC directly', 'IBEAM Control PD', 'IBEAM Control Asymptotically Stable send to actuators', 'IBEAM Control Asymptotically Stable send to thrusters'}
                        obj = func_update_software_SC_control_attitude_IBEAM(obj, mission, i_SC);

                    case {'Nightingale Oracle', 'Nightingale Slew Oracle', 'Nightingale Control Asymptotically Stable send to ADC directly', 'Nightingale Control PD send to ADC directly', 'Nightingale Control PD send to thrusters', 'Nightingale Control PD send to actuators', 'Nightingale Control Asymptotically Stable send to actuators', 'Nightingale Control Asymptotically Stable send to rwa', 'Nightingale Control Asymptotically Stable send to thrusters'}
                        obj = func_update_software_SC_control_attitude_Nightingale_v2(obj, mission, i_SC);

                    case {'NISAR Oracle'}
                        obj = func_update_software_SC_control_attitude_NISAR(obj, mission, i_SC);

                    otherwise
                        error('Attitude Control mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
            end

            % Compute Error Angle between desired_attitude and estimated attitude
            obj.error_angle_desired_attitude = min([func_error_angle_between_quaternions(obj.desired_attitude, mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude), ...
                                                    func_error_angle_between_quaternions(-obj.desired_attitude, mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude)]); % [rad]

            % Update Storage
            obj = func_update_software_SC_control_attitude_store(obj, mission);

            % DO NOT SWITCH OFF FUNCTIONS USING flag_executive INSIDE Attitude Dynamics Loop (ADL)
        end

        
        

    end

end