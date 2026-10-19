%% Class: True_SC_Star_Tracker
% Tracks the Star Tracker measurements

classdef True_SC_Star_Tracker < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample 
                                    % used to avoid calling isprop() every time
                                    % step to determine if using instantaneous_data_rate_generated or
                                    % instantaneous_data_generated_per_sample, which is slow

        mode_true_SC_star_tracker_selector % [string]
        % - Truth
        % - Simple
        % - Simple with Sun outside FOV

        measurement_noise_cross_boresight = 4.2/3600*pi/180 % [rad] Cross-boresight error (1-sigma standard deviation) (Optional)
        measurement_noise_about_boresight = 34/3600*pi/180 % [rad] About-boresight error (1-sigma standard deviation) (Optional)
        angle_sigma_SF % [rad] 1x3 Measurement noise error in the sensor frame

        % Parameters for 'Bias + First Order Gauss Markov' model
        bias_sigma_cross      = 25/3600*pi/180     % [rad] 1-sigma cross-boresight constant bias (25 arcsec)
        bias_sigma_boresight  = 60/3600*pi/180     % [rad] 1-sigma about-boresight constant bias (60 arcsec)

        fogm_tau              = 1800               % [sec] FOGM correlation time constant
        fogm_sigma_cross      = 3/3600*pi/180      % [rad] 1-sigma cross-boresight FOGM steady-state (3 arcsec)
        fogm_sigma_boresight  = 15/3600*pi/180     % [rad] 1-sigma about-boresight FOGM steady-state (15 arcsec)

        measurement_wait_time % [sec]

        location % [m] : Location of sensor, in body frame B

        alignment_BF_SF % [DCM] : Sensor Frame to Body Frame alignment direction cosine matrix

        orientation % [unit vec] : unit vector of the sensor boresight in the body frame

        field_of_view = 35 % [deg] : Field of view (FOV) of the camera in deg (No measurement if Sun is within this FOV)



        rand_stream = RandStream('twister') % Random number stream for RNG control (Marsenne Twister)
        rng_seed % Random number generator seed

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Sun Sensor i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [quaternion]

        measurement_time % [sec] SC time when this measurement was taken

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        meas_update_occurred % [Boolean] Indicates if the measurement is stale (0) or fresh (1)

        % Internal state for 'Bias + First Order Gauss Markov' model
        bias_SF               % [rad] 1x3 constant bias in sensor frame (sampled at init)
        fogm_sigma_SF         % [rad] 3x1 FOGM steady-state std [cross;cross;boresight]
        fogm_SF               % [rad] 1x3 current FOGM error state in sensor frame
        fogm_phi              % [-]  discrete decay = exp(-sim_dt/fogm_tau)
        fogm_driving_sigma_SF % [rad] 3x1 per-step driving noise std
        sim_dt                % [sec] step used to propagate the FOGM

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Star_Tracker(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Star Tracker ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_generated_per_sample = init_data.instantaneous_data_generated_per_sample; % [kb]

            obj.mode_true_SC_star_tracker_selector = init_data.mode_true_SC_star_tracker_selector; % [string]
            obj.measurement_wait_time = init_data.measurement_wait_time; % [sec]

            if isfield(init_data,'measurement_noise_cross_boresight')
                obj.measurement_noise_cross_boresight = init_data.measurement_noise_cross_boresight; % [rad]
            else
                obj.measurement_noise_cross_boresight = 0; % [rad]
            end

            if isfield(init_data,'measurement_noise_about_boresight')
                obj.measurement_noise_about_boresight = init_data.measurement_noise_about_boresight; % [rad]
            else
                obj.measurement_noise_about_boresight = 0; % [rad]
            end

            if isfield(init_data,'alignment_BF_SF')
                obj.alignment_BF_SF = init_data.alignment_BF_SF;
            else
                obj.alignment_BF_SF = eye(3); 
            end
            
            
            obj.angle_sigma_SF = [obj.measurement_noise_cross_boresight*ones(2,1); obj.measurement_noise_about_boresight]; % [rad]

            obj.measurement_vector = zeros(1,4);

            obj.flag_executive = 1;

            obj.measurement_time = -inf; % [sec]

            obj.location = init_data.location; % [m]


            

            if isfield(init_data,'alignment_BF_SF') && isfield(init_data,'boresight_SF') && ...
                    ~isempty(init_data.alignment_BF_SF) && ~isempty(init_data.boresight_SF)
                obj.orientation = (init_data.alignment_BF_SF*init_data.boresight_SF(:)).';

            elseif isfield(init_data,'orientation') && ~isempty(init_data.orientation)
                obj.orientation = init_data.orientation;
            end
            obj.field_of_view = init_data.field_of_view; % [deg]

            obj.meas_update_occurred = 0;

            % Set the RNG seed, if provided
            if isfield(init_data,'rng_seed') && ~isempty(init_data.rng_seed)
                obj.rng_seed = init_data.rng_seed;
                reset(obj.rand_stream, uint32(obj.rng_seed)); % Must be uint32
            end

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % --- Bias + FOGM model setup ---
            % Sampled from obj.rand_stream (after any seed reset) for MC repeatability
            if isfield(init_data,'bias_sigma_cross') && ~isempty(init_data.bias_sigma_cross)
                obj.bias_sigma_cross = init_data.bias_sigma_cross;
            end
            if isfield(init_data,'bias_sigma_boresight') && ~isempty(init_data.bias_sigma_boresight)
                obj.bias_sigma_boresight = init_data.bias_sigma_boresight;
            end
            bias_sigma_SF = [obj.bias_sigma_cross; obj.bias_sigma_cross; obj.bias_sigma_boresight]; % [rad]
            if isfield(init_data,'bias_SF') && ~isempty(init_data.bias_SF)
                obj.bias_SF = init_data.bias_SF(:).';
            else
                obj.bias_SF = (bias_sigma_SF .* randn(obj.rand_stream,3,1)).'; % sample once
            end

            if isfield(init_data,'fogm_tau') && ~isempty(init_data.fogm_tau)
                obj.fogm_tau = init_data.fogm_tau;
            end
            if isfield(init_data,'fogm_sigma_cross') && ~isempty(init_data.fogm_sigma_cross)
                obj.fogm_sigma_cross = init_data.fogm_sigma_cross;
            end
            if isfield(init_data,'fogm_sigma_boresight') && ~isempty(init_data.fogm_sigma_boresight)
                obj.fogm_sigma_boresight = init_data.fogm_sigma_boresight;
            end
            obj.fogm_sigma_SF = [obj.fogm_sigma_cross; obj.fogm_sigma_cross; obj.fogm_sigma_boresight]; % [rad]

            % Discrete FOGM coefficients (steady-state-preserving driving noise)
            obj.sim_dt                = max([mission.true_time.time_step_attitude, obj.measurement_wait_time]); % [sec]
            obj.fogm_phi              = exp(-obj.sim_dt/obj.fogm_tau);
            obj.fogm_driving_sigma_SF = obj.fogm_sigma_SF*sqrt(1 - obj.fogm_phi^2); % [rad]

            % Initial FOGM state: use the provided value if given, otherwise
            % sample from the steady-state distribution
            if isfield(init_data,'initial_fogm_SF') && ~isempty(init_data.initial_fogm_SF)
                obj.fogm_SF = init_data.initial_fogm_SF(:).'; % [rad]
            else
                obj.fogm_SF = (obj.fogm_sigma_SF .* randn(obj.rand_stream,3,1)).'; % [rad]
            end

            % Initialize Variables to store: measurement_vector
            obj.store = [];
            obj.store.measurement_vector = zeros(mission.storage.num_storage_steps, length(obj.measurement_vector));
            obj.store.fogm_SF = zeros(mission.storage.num_storage_steps, length(obj.fogm_SF));

            % Update Storage
            obj = func_update_true_SC_star_tracker_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_star_tracker_store(obj, mission)

            if mission.storage.flag_store_this_time_step_attitude == 1
                obj.store.measurement_vector(mission.storage.k_storage_attitude,:) = obj.measurement_vector; % [quaternion]
                obj.store.fogm_SF(mission.storage.k_storage_attitude,:) = obj.fogm_SF; % [rad]
            end

        end

        %% [ ] Methods: Main
        % Update Camera

        function obj = func_main_true_SC_star_tracker(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time_attitude - obj.measurement_time) >= (obj.measurement_wait_time - 1e-6)
                    obj.meas_update_occurred = 1;

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time_attitude; % [sec]

                    switch obj.mode_true_SC_star_tracker_selector

                        case 'Truth'
                            obj = func_true_SC_star_tracker_Truth(obj, mission, i_SC);

                        case 'Simple'
                            obj = func_true_SC_star_tracker_Simple(obj, mission, i_SC);

                        case 'Simple with Sun outside FOV'

                            
                            this_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation')'; % [unit vector]

                            Sun_vector = mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.true_SC_orbit_propagation.position; % [km]
                            Sun_vector_normalized = func_normalize_vec(Sun_vector); % [unit vector]

                            if func_angle_between_vectors(this_orientation, Sun_vector_normalized) >= deg2rad(obj.field_of_view)
                                obj = func_true_SC_star_tracker_Simple(obj, mission, i_SC);
                            else
                                % Measurement doesn't exist
                                obj.measurement_vector = nan(1,4);
                            end

                        case 'Bias + First Order Gauss Markov'
                            obj = func_true_SC_star_tracker_bias_fogm(obj, mission, i_SC);

                        otherwise
                            error('Star Tracker mode not defined!')
                    end

                    % Update Data Generated
                    func_update_instantaneous_data_generated_attitude(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

                else
                    % Data not generated in this time step
                    obj.meas_update_occurred = 0;

                end

                % Update Power Consumed
                func_update_instantaneous_power_consumed_attitude(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            else
                % Do nothing
                obj.meas_update_occurred = 0;

            end

            % Update Storage
            obj = func_update_true_SC_star_tracker_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

        %% [ ] Methods: Truth
        % Star Tracker mode

        function obj = func_true_SC_star_tracker_Truth(obj, mission, i_SC)
            obj.measurement_vector = func_quaternion_properize( mission.true_SC{i_SC}.true_SC_adc.attitude);
        end

        %% [ ] Methods: Simple
        % Star Tracker mode

        function obj = func_true_SC_star_tracker_Simple(obj, mission, i_SC)
            % OLD Incorrect Method
            % obj.measurement_vector = func_quaternion_properize( mission.true_SC{i_SC}.true_SC_adc.attitude + obj.measurement_noise*randn(1,4) );

            % New Correct Method

            % Define the random angular error about each axis in the sensor frame
            % Assume the boresight is along +Z in the sensor frame
            angle_error_SF = obj.angle_sigma_SF(:).*randn(obj.rand_stream,3,1);
            
            % Transform the error into the body frame
            angle_error_BF = (obj.alignment_BF_SF*angle_error_SF).';
            angle_error_norm = norm(angle_error_BF);

            % Compute the error quaternion in the body frame
            error_quaternion_B = vec2q(angle_error_BF/angle_error_norm, angle_error_norm);

            % Compute the measured quaternion
            obj.measurement_vector = qmult(error_quaternion_B, mission.true_SC{i_SC}.true_SC_adc.attitude);
            obj.measurement_vector = qnorm(obj.measurement_vector);

            % Define the error quaternion (small rotation) in axis-angle format
            % error_angle = obj.measurement_noise; % [rad]
            % error_axis = randn(1,3);
            % error_axis = error_axis/norm(error_axis); % [unit vector]
            % 
            % % Convert axis-angle to quaternion
            % error_quaternion = [sin(error_angle/2) * error_axis, cos(error_angle/2)];
            % error_quaternion = func_quaternion_properize(error_quaternion);
            % 
            % % Apply the error by quaternion multiplication (Hamilton product)
            % obj.measurement_vector = func_quaternion_multiply(mission.true_SC{i_SC}.true_SC_adc.attitude, error_quaternion);
            % obj.measurement_vector = func_quaternion_properize(obj.measurement_vector);

        end

        % Apply
        %   - Boresight-dependent noise equivalent angle
        %   - Constant bias
        %   - First-order gauss markov error to represent slow thermal
        %     variations and low-frequency spatial errors (LFSE)
        function obj = func_true_SC_star_tracker_bias_fogm(obj, mission, i_SC)

            % Noise-equivalent angle (white noise, resampled each measurement)
            % Assume the boresight is along +Z in the sensor frame
            nea_SF = obj.angle_sigma_SF(:) .* randn(obj.rand_stream,3,1); % [rad]

            % Propagate first-order Gauss-Markov drift: x_k = phi*x_{k-1} + w_k
            obj.fogm_SF = ( obj.fogm_phi*obj.fogm_SF(:) ...
                            + obj.fogm_driving_sigma_SF .* randn(obj.rand_stream,3,1) ).'; % [rad]

            % Total small-angle error in sensor frame = NEA + constant bias + FOGM drift
            total_angle_SF = nea_SF + obj.bias_SF(:) + obj.fogm_SF(:); % [rad]

            % Transform to body frame and build the error quaternion
            total_angle_BF   = (obj.alignment_BF_SF*total_angle_SF).';
            total_angle_norm = norm(total_angle_BF);
            error_quat_B     = vec2q(total_angle_BF/total_angle_norm, total_angle_norm);

            % Apply error to the true attitude
            obj.measurement_vector = qmult(error_quat_B, mission.true_SC{i_SC}.true_SC_adc.attitude);
            obj.measurement_vector = qnorm(obj.measurement_vector);

        end

    end
end


