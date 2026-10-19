%% Class: True_SC_IMU
% Tracks the IMU measurements

classdef True_SC_IMU < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample 
                                    % used to avoid calling isprop() every time
                                    % step to determine if using instantaneous_data_rate_generated or
                                    % instantaneous_data_generated_per_sample, which is slow

        mode_true_SC_imu_selector % [string]
        % - Truth
        % - Simple

        measurement_noise % [rad/sec] (1-sigma standard deviation)

        measurement_wait_time % [sec]

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        % Parameters use for 'Rate bias' model
        rand_stream = RandStream('twister') % Random number stream for RNG control (Marsenne Twister)
        rng_seed % Random number generator seed

        initial_bias % [rad/s] Initial rate bias
        bias_long_term_stab = 0.035*pi/180/3600 % [rad/s] Bias long-term stability standard deviation

        sim_dt % [sec] Simulation delta-time to pre-compute ARW and RRW sigmas

        arw_psd = (0.01*pi/180/sqrt(3600))^2     % [rad^2/s = (rad^2/s^2)/Hz] Angle Random Walk (rate white noise) PSD height
        rrw_psd = (0.05*pi/180/3600)^2/(24*3600) % [(rad/s)^2/s = (rad/s^2)^2/Hz] Rate (bias) Random Walk PSD height

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Sun Sensor i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [rad/sec] Angular Velocity

        measurement_time % [sec] SC time when this measurement was taken

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        data % Other useful data

        % Parameters use for 'Rate bias' model
        rate_bias % [rad/sec] Gyro measurement bias relative to true rate
        rate_bias_prev % [rad/sec] Gyro measurement bias relative to true rate of previous sample

        arw_sigma % [rad/s] standard deviation of angle random walk (i.e. rate white noise)
        rrw_sigma % [rad/s] standard deviation of rate (bias) random walk

        ang_freq_bias_pole % [rad] Discrete angular frequency of the pole of the bias random walk (continuous pole * sim_dt)

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
            obj.measurement_noise = init_data.measurement_noise; % [rad/sec]

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

            % Set the RNG seed, if provided
            if isfield(init_data,'rng_seed') && ~isempty(init_data.rng_seed)
                obj.rng_seed = init_data.rng_seed;
                reset(obj.rand_stream, uint32(obj.rng_seed)); % Must be uint32
            end

            % Set the initial bias
            if isfield(init_data,'initial_bias') && ~isempty(init_data.initial_bias)
                obj.rate_bias = init_data.initial_bias(:).'; % Ensure row vector
            else
                obj.rate_bias = zeros(1,3);
            end
            obj.rate_bias_prev = obj.rate_bias;

            % Compute the ARW and RRW standard deviations from PSD heights
            % and sim dt
            obj.sim_dt = max([mission.true_time.time_step_attitude, obj.measurement_wait_time]);
            if isfield(init_data,'arw_psd') && ~isempty(init_data.arw_psd)
                obj.arw_psd = init_data.arw_psd;
            end
            if isfield(init_data,'rrw_psd') && ~isempty(init_data.rrw_psd)
                obj.rrw_psd = init_data.rrw_psd;
            end
            obj.arw_sigma = sqrt(obj.arw_psd/obj.sim_dt); % [rad/s]
		    obj.rrw_sigma = sqrt(obj.rrw_psd*obj.sim_dt); % [rad/s]

            % Compute the long-term bias stability frequency pole
            if isfield(init_data,'bias_long_term_stab') && ~isempty(init_data.bias_long_term_stab)
                obj.bias_long_term_stab = init_data.bias_long_term_stab;
            end
            if obj.bias_long_term_stab > 0
                obj.ang_freq_bias_pole = obj.rrw_psd/( 2*(obj.bias_long_term_stab)^2 )*obj.sim_dt;
            else
                obj.ang_freq_bias_pole = 0;
            end

            % Initialize Variables to store: measurement_vector
            obj.store = [];
            obj.store.measurement_vector = zeros(mission.storage.num_storage_steps, length(obj.measurement_vector));
            obj.store.rate_bias = zeros(mission.storage.num_storage_steps, length(obj.rate_bias));

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
                obj.store.measurement_vector(mission.storage.k_storage_attitude,:) = obj.measurement_vector; % [rad/sec]
                obj.store.rate_bias(mission.storage.k_storage_attitude,:) = obj.rate_bias; % [rad/sec]
            end

        end

        %% [ ] Methods: Main
        % Update Camera

        function obj = func_main_true_SC_imu(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time_attitude - obj.measurement_time) >= (obj.measurement_wait_time - 1e-6)

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time_attitude; % [sec]

                    switch obj.mode_true_SC_imu_selector

                        case 'Truth'
                            obj = func_true_SC_imu_Truth(obj, mission, i_SC);

                        case 'Simple'
                            obj = func_true_SC_imu_Simple(obj, mission, i_SC);

                        case 'Rate bias'
                            % TBD: include rate bias model
                            obj = func_true_SC_imu_Bias(obj, mission, i_SC);

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
        % Adds Gaussian white noise to the true rate

        function obj = func_true_SC_imu_Simple(obj, mission, i_SC)
            obj.measurement_vector = mission.true_SC{i_SC}.true_SC_adc.angular_velocity + obj.measurement_noise*randn(1,3) ;
        end   

        %% [ ] Methods: Rate Bias
        % IMU mode
        % Adds Gaussian white noise to the true rate, plus an initial
        % offset bias and rate (bias) random walk

        function obj = func_true_SC_imu_Bias(obj, mission, i_SC)
            % Propagate the true bias according to RRW. The first term
            % represents a first-order approximation to a decay that bounds
            % the long-term bias growth to the input long-term bias
            % stability. The second term is the random walk component.
            obj.rate_bias = (1 - obj.ang_freq_bias_pole)*obj.rate_bias_prev + obj.rrw_sigma*randn(obj.rand_stream,1,3);
            obj.rate_bias_prev = obj.rate_bias;

            % Apply errors to the true rate to get the measured rate
            obj.measurement_vector = mission.true_SC{i_SC}.true_SC_adc.angular_velocity + ...
                obj.arw_sigma*randn(obj.rand_stream,1,3) + obj.rate_bias;
        end        

    end
end

