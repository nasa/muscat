%% Class: True_SC_IMU
% Tracks the IMU measurements

classdef True_SC_IMU < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_true_SC_imu_selector % [string]
        % - Truth
        % - Simple

        measurement_noise % [rad/sec] (1-sigma standard deviation)

        measurement_wait_time % [sec]

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Sun Sensor i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [quaternion]

        measurement_time % [sec] SC time when this measurement was taken

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_IMU(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['IMU ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_generated_per_sample = init_data.instantaneous_data_generated_per_sample; % [kb]

            obj.mode_true_SC_imu_selector = init_data.mode_true_SC_imu_selector; % [string]
            obj.measurement_wait_time = init_data.measurement_wait_time; % [sec]
            obj.measurement_noise = init_data.measurement_noise; % [rad]

            obj.measurement_vector = zeros(1,3);

            obj.flag_executive = 1;

            obj.measurement_time = -inf; % [sec]

            obj.location = init_data.location; % [m]
            obj.orientation = init_data.orientation; % [unit vector]

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize Variables to store: measurement_vector
            obj.store = [];
            obj.store.measurement_vector = zeros(mission.storage.num_storage_steps, length(obj.measurement_vector));

            % Update Storage
            obj = func_update_true_SC_imu_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_imu_store(obj, mission)

            if mission.storage.flag_store_this_time_step_attitude == 1
                obj.store.measurement_vector(mission.storage.k_storage_attitude,:) = obj.measurement_vector; % [quaternion]
            end

        end

        %% [ ] Methods: Main
        % Update Camera

        function obj = func_main_true_SC_imu(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time_attitude - obj.measurement_time) >= obj.measurement_wait_time

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time_attitude; % [sec]

                    switch obj.mode_true_SC_imu_selector

                        case 'Truth'
                            obj = func_true_SC_imu_Truth(obj, mission, i_SC);

                        case 'Simple'
                            obj = func_true_SC_imu_Simple(obj, mission, i_SC);

                        otherwise
                            error('IMU mode not defined!')
                    end
                    
                    % Update Data Generated
                    func_update_instantaneous_data_generated_attitude(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

                else
                    % Data not generated in this time step                    

                end

                % Update Power Consumed
                func_update_instantaneous_power_consumed_attitude(mission.true_SC{i_SC}.true_SC_power, obj, mission);


            else
                % Do nothing

            end

            % Update Storage
            obj = func_update_true_SC_imu_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

        %% [ ] Methods: Truth
        % IMU mode

        function obj = func_true_SC_imu_Truth(obj, mission, i_SC)
            obj.measurement_vector = mission.true_SC{i_SC}.true_SC_adc.angular_velocity;
        end

        %% [ ] Methods: Simple
        % IMU mode

        function obj = func_true_SC_imu_Simple(obj, mission, i_SC)
            obj.measurement_vector = mission.true_SC{i_SC}.true_SC_adc.angular_velocity + obj.measurement_noise*randn(1,3) ;
        end        

    end
end

