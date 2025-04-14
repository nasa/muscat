%% Class: True_SC_Star_Tracker
% Tracks the Star Tracker measurements

classdef True_SC_Star_Tracker < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_true_SC_star_tracker_selector % [string]
        % - Truth
        % - Simple
        % - Simple with Sun outside FOV

        measurement_noise % [rad] (1-sigma standard deviation) (Optional)

        measurement_wait_time % [sec]

        location % [m] : Location of sensor, in body frame B

        orientation % [unit vector] : Normal vector from location

        field_of_view % [deg] : Field of view (FOV) of the camera in deg (No measurement if Sun is within this FOV)

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
            obj.measurement_noise = init_data.measurement_noise; % [rad]

            obj.measurement_vector = zeros(1,4);

            obj.flag_executive = 1;

            obj.measurement_time = -inf; % [sec]

            obj.location = init_data.location; % [m]
            obj.orientation = init_data.orientation; % [unit vector]
            obj.field_of_view = init_data.field_of_view; % [deg]

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize Variables to store: measurement_vector
            obj.store = [];
            obj.store.measurement_vector = zeros(mission.storage.num_storage_steps, length(obj.measurement_vector));

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
            end

        end

        %% [ ] Methods: Main
        % Update Camera

        function obj = func_main_true_SC_star_tracker(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time_attitude - obj.measurement_time) >= obj.measurement_wait_time

                    % Sufficient time has elasped for a new measurement
                    obj.measurement_time = mission.true_time.time_attitude; % [sec]

                    switch obj.mode_true_SC_star_tracker_selector

                        case 'Truth'
                            obj = func_true_SC_star_tracker_Truth(obj, mission, i_SC);

                        case 'Simple'
                            obj = func_true_SC_star_tracker_Simple(obj, mission, i_SC);

                        case 'Simple with Sun outside FOV'

                            this_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation')'; % [unit vector]

                            Sun_vector = mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.true_SC_navigation.position; % [km]
                            Sun_vector_normalized = func_normalize_vec(Sun_vector); % [unit vector]

                            if func_angle_between_vectors(this_orientation, Sun_vector_normalized) >= deg2rad(obj.field_of_view)
                                obj = func_true_SC_star_tracker_Simple(obj, mission, i_SC);
                            else
                                % Measurement doesn't exist
                                obj.measurement_vector = nan(1,4);
                            end

                        otherwise
                            error('Star Tracker mode not defined!')
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

            % Define the error quaternion (small rotation) in axis-angle format
            error_angle = obj.measurement_noise; % [rad]
            error_axis = randn(1,3);
            error_axis = error_axis/norm(error_axis); % [unit vector]

            % Convert axis-angle to quaternion
            error_quaternion = [sin(error_angle/2) * error_axis, cos(error_angle/2)];
            error_quaternion = func_quaternion_properize(error_quaternion);

            % Apply the error by quaternion multiplication (Hamilton product)
            obj.measurement_vector = func_quaternion_multiply(mission.true_SC{i_SC}.true_SC_adc.attitude, error_quaternion);
            obj.measurement_vector = func_quaternion_properize(obj.measurement_vector);

        end



    end
end


