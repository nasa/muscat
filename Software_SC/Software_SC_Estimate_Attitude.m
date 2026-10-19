%% Class: Software_SC_Estimate_Attitude
% Estimates the Attitude of the Spacecraft

classdef Software_SC_Estimate_Attitude < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample 
                                    % used to avoid calling isprop() every time
                                    % step to determine if using instantaneous_data_rate_generated or
                                    % instantaneous_data_generated_per_sample, which is slow

        mode_software_SC_estimate_attitude_selector % [string] Different attitude estimation modes
        % - 'Truth' : Use True_SC_ADC values
        % - 'Truth with Noise' : Use True_SC_ADC values + Noise
        % - 'KF' : Use Kalman Filter

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Estimate Attitude

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        attitude % [quaternion] : Orientation of inertial frame I with respect to the body frame B

        angular_velocity % [rad/sec] : Angular velocity of inertial frame I with respect to the body frame B

        angular_momentum_BF % [kg m^2 /sec] : Angular momentum of Body Frame

        gyro_bias % [rad/sec] : Estimated gyro rate bias in body frame B

        dot_angular_velocity % [rad/sec^2] : Time derivative of angular_velocity (needed by RWA)

        attitude_uncertainty % [error in quaternion]

        angular_velocity_uncertainty % [rad/sec]

        gyro_bias_uncertainty % [rad/sec]

        dot_angular_velocity_uncertainty % [rad/sec^2]

        data % Other useful data

        attitude_filter % AttitudeFilter object class


        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Estimate_Attitude(init_data, mission, i_SC)

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Estimate Attitude']; % [string]
            obj.flag_executive = 1;

            obj.instantaneous_data_generated_per_sample = 0; % [kb]
            obj.mode_software_SC_estimate_attitude_selector = init_data.mode_software_SC_estimate_attitude_selector; % [string]

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            obj = func_update_software_SC_estimate_attitude_Truth(obj, mission, i_SC);

            

            % Initialize the Estimator

            if strcmp(init_data.mode_software_SC_estimate_attitude_selector,'KF')
                obj.attitude = init_data.initial_est_q_BF_IF;
                obj.gyro_bias = init_data.initial_est_gyro_bias_BF;
                obj.attitude_uncertainty = init_data.initial_sigma_theta_BF*ones(1,3);
                obj.gyro_bias_uncertainty = init_data.initial_sigma_gyro_bias_BF*ones(1,3);

                obj.attitude_filter = AttitudeKalmanFilter(...
                    mission.true_time.time_step_attitude,...
                    init_data.gyro_sigma,...
                    init_data.bias_sigma,...
                    init_data.startracker_sigma_SF,...
                    init_data.initial_est_q_BF_IF,...
                    init_data.initial_est_omega_BF,...
                    init_data.initial_est_gyro_bias_BF,...
                    init_data.initial_sigma_theta_BF,...
                    init_data.initial_sigma_gyro_bias_BF,...
                    init_data.startracker_C_BF_SF);
            else

                obj.attitude = zeros(1,4);
                obj.attitude_uncertainty = zeros(1,3);

                obj.angular_velocity = zeros(1,3);
                obj.angular_velocity_uncertainty = zeros(1,3); 

                obj.gyro_bias = zeros(1,3);
                obj.gyro_bias_uncertainty = zeros(1,3);

                obj.dot_angular_velocity = zeros(1,3);
                obj.dot_angular_velocity_uncertainty = zeros(1,3);

            end

            % Initialize momentum
            obj = func_update_angular_momentum(obj, mission, i_SC);

            % Initialize Variables to store: attitude angular_velocity dot_angular_velocity and uncertainties
            obj.store = [];

            obj.store.flag_executive = zeros(mission.storage.num_storage_steps_attitude, length(obj.flag_executive));

            obj.store.attitude = zeros(mission.storage.num_storage_steps_attitude, length(obj.attitude));
            obj.store.attitude_uncertainty = zeros(mission.storage.num_storage_steps_attitude, length(obj.attitude_uncertainty));

            obj.store.angular_velocity = zeros(mission.storage.num_storage_steps_attitude, length(obj.angular_velocity));
            obj.store.angular_velocity_uncertainty = zeros(mission.storage.num_storage_steps_attitude, length(obj.angular_velocity_uncertainty));

            obj.store.dot_angular_velocity = zeros(mission.storage.num_storage_steps_attitude, length(obj.dot_angular_velocity));
            obj.store.dot_angular_velocity_uncertainty = zeros(mission.storage.num_storage_steps_attitude, length(obj.dot_angular_velocity_uncertainty));

            obj.store.gyro_bias = zeros(mission.storage.num_storage_steps_attitude, 3);
            obj.store.gyro_bias_uncertainty = zeros(mission.storage.num_storage_steps_attitude, 3);

            % Update Storage
            obj = func_update_software_SC_estimate_attitude_store(obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_software_SC_estimate_attitude_store(obj, mission)

            if mission.storage.flag_store_this_time_step_attitude == 1
                obj.store.flag_executive(mission.storage.k_storage_attitude,:) = obj.flag_executive; 

                obj.store.attitude(mission.storage.k_storage_attitude,:) = obj.attitude; % [quaternion]
                obj.store.attitude_uncertainty(mission.storage.k_storage_attitude,:) = obj.attitude_uncertainty;

                obj.store.angular_velocity(mission.storage.k_storage_attitude,:) = obj.angular_velocity; % [rad/sec]
                obj.store.angular_velocity_uncertainty(mission.storage.k_storage_attitude,:) = obj.angular_velocity_uncertainty;

                obj.store.dot_angular_velocity(mission.storage.k_storage_attitude,:) = obj.dot_angular_velocity; % [rad/sec^2]
                obj.store.dot_angular_velocity_uncertainty(mission.storage.k_storage_attitude,:) = obj.dot_angular_velocity_uncertainty;

                obj.store.gyro_bias(mission.storage.k_storage_attitude,:) = obj.gyro_bias; % [rad/sec^2]
                obj.store.gyro_bias_uncertainty(mission.storage.k_storage_attitude,:) = obj.gyro_bias_uncertainty;
            end

        end

        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_estimate_attitude(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_estimate_attitude_selector

                    case 'Truth'
                        obj = func_update_software_SC_estimate_attitude_Truth(obj, mission, i_SC);
                        
                        % % Enable star tracker/IMU execution
                        % for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker
                        %     mission.true_SC{i_SC}.true_SC_star_tracker{i_HW}.flag_executive = 1;
                        % end
                        % for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_imu
                        %     mission.true_SC{i_SC}.true_SC_imu{i_HW}.flag_executive = 1;
                        % end

                    case 'KF'
                        % obj = func_update_software_SC_estimate_attitude_KF(obj, mission, i_SC);
                        obj = func_update_software_SC_estimate_attitude_KF_354GNC(obj, mission, i_SC);

                        % Enable star tracker/IMU execution
                        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker
                            mission.true_SC{i_SC}.true_SC_star_tracker{i_HW}.flag_executive = 1;
                        end
                        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_imu
                            mission.true_SC{i_SC}.true_SC_imu{i_HW}.flag_executive = 1;
                        end

                    otherwise
                        error('Attitude Estimation mode not defined!')
                end

                % Update momentum estimate
                obj = func_update_angular_momentum(obj, mission, i_SC);

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end

            % Update Storage
            obj = func_update_software_SC_estimate_attitude_store(obj, mission);
            
            % Reset Variables
            if abs(mission.true_time.time - mission.true_time.time_attitude) <= 1e-6
                obj.flag_executive = 0;
            else
                % DONOT SWITCH OFF FUNCTIONS USING flag_executive INSIDE Attitude Dynamics Loop (ADL)
            end
           
        end


        %% [ ] Methods: Update Angular Momentum
        % Compute the angular momentum of the spacecraft

        function obj = func_update_angular_momentum(obj, mission, i_SC)
            % TODO: Include solar array momentum when they are actively changing orientation 

            rwa_mom_BF = zeros(1,3);
            for ii = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                rwa_mom_BF = rwa_mom_BF + mission.true_SC{i_SC}.true_SC_reaction_wheel{ii}.total_momentum(:).';
            end

            obj.angular_momentum_BF = (rwa_mom_BF + (mission.true_SC{i_SC}.true_SC_body.total_MI*obj.angular_velocity(:))' );
        end

    end
end

