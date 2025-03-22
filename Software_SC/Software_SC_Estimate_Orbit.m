%% Class: Software_SC_Estimate_Orbit
% Estimates the Orbits of the Spacecraft and Target

classdef Software_SC_Estimate_Orbit < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_software_SC_estimate_orbit_selector % [string] Different attitude dynamics modes
        % - 'Truth' : Use True values
        % - 'Truth with Noise' : Use True values + Noise
        % - 'TruthWithErrorGrowth' : Use True values with error growth when target is not visible

        compute_wait_time % [sec]

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Estimate Orbit

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        position % [km] : Current position of SC in inertial frame I
        position_uncertainty % [km]

        velocity % [km/sec] : Current velocity of SC in inertial frame I
        velocity_uncertainty % [km/sec]

        position_relative_target % [km] : Current position of SC relative to SB-center J2000 inertial frame
        position_relative_target_uncertainty % [km]

        velocity_relative_target % [km/sec] : Current velocity of SC relative to SB-center J2000 inertial frame
        velocity_relative_target_uncertainty % [km/sec]

        name_relative_target % [string] : Name of the target, relative to which position and velocity are specified
        index_relative_target % [integer] : Index of the target, relative to which position and velocity are specified

        position_target % [km] Current position of Target wrt Sun-centered J2000
        position_target_uncertainty % [km]

        velocity_target % [km/sec] Current velocity of Target wrt Sun-centered J2000
        velocity_target_uncertainty % [km/sec]

        compute_time % [sec] SC time when this measurement was taken

        data % Other useful data


        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Estimate_Orbit(init_data, mission, i_SC)

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Estimate Orbit']; % [string]
            obj.flag_executive = 1;

            obj.instantaneous_data_generated_per_sample = 0; % [kb]
            obj.mode_software_SC_estimate_orbit_selector = init_data.mode_software_SC_estimate_orbit_selector; % [string]

            obj.name_relative_target = mission.true_SC{i_SC}.true_SC_navigation.name_relative_target; % [string]
            obj.index_relative_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target; % [integer]

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end
            
            % Initialize data fields for error growth mode
            if strcmp(obj.mode_software_SC_estimate_orbit_selector, 'TruthWithErrorGrowth')
                obj.data.last_target_visible_time = -inf; % [sec] Initialize to a value that ensures we start with uncertainty
                obj.data.position_error_growth_rate = 0.01; % [km/sec] Rate at which position uncertainty grows
                obj.data.velocity_error_growth_rate = 0.001; % [km/sec²] Rate at which velocity uncertainty grows
            end

            if isfield(init_data, 'compute_wait_time')
                obj.compute_wait_time = init_data.compute_wait_time; % [sec]
            else
                obj.compute_wait_time = 0; % [sec]
            end

            obj.compute_time = -inf; % [sec]

            obj = func_update_software_SC_estimate_orbit_Truth(obj, mission, i_SC);

            % Initialize Variables to store: position velocity of SC and Target
            obj.store = [];

            obj.store.position = zeros(mission.storage.num_storage_steps, length(obj.position));
            obj.store.position_uncertainty = zeros(mission.storage.num_storage_steps, length(obj.position_uncertainty));

            obj.store.velocity = zeros(mission.storage.num_storage_steps, length(obj.velocity));
            obj.store.velocity_uncertainty = zeros(mission.storage.num_storage_steps, length(obj.velocity_uncertainty));

            obj.store.position_relative_target = zeros(mission.storage.num_storage_steps, length(obj.position));
            obj.store.position_relative_target_uncertainty = zeros(mission.storage.num_storage_steps, length(obj.position_relative_target_uncertainty));

            obj.store.velocity_relative_target = zeros(mission.storage.num_storage_steps, length(obj.velocity));
            obj.store.velocity_relative_target_uncertainty = zeros(mission.storage.num_storage_steps, length(obj.velocity_relative_target_uncertainty));

            obj.store.position_target = zeros(mission.storage.num_storage_steps, length(obj.position_target));
            obj.store.position_target_uncertainty = zeros(mission.storage.num_storage_steps, length(obj.position_target_uncertainty));

            obj.store.velocity_target = zeros(mission.storage.num_storage_steps, length(obj.velocity_target));
            obj.store.velocity_target_uncertainty = zeros(mission.storage.num_storage_steps, length(obj.velocity_target_uncertainty));

            % Update Storage
            obj = func_update_software_SC_estimate_orbit_store(obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_software_SC_estimate_orbit_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.position(mission.storage.k_storage,:) = obj.position; % [km]
                obj.store.position_uncertainty(mission.storage.k_storage,:) = obj.position_uncertainty;

                obj.store.velocity(mission.storage.k_storage,:) = obj.velocity; % [km/sec]
                obj.store.velocity_uncertainty(mission.storage.k_storage,:) = obj.velocity_uncertainty;

                obj.store.position_relative_target(mission.storage.k_storage,:) = obj.position_relative_target; % [km]
                obj.store.position_relative_target_uncertainty(mission.storage.k_storage,:) = obj.position_relative_target_uncertainty;

                obj.store.velocity_relative_target(mission.storage.k_storage,:) = obj.velocity_relative_target; % [km/sec]
                obj.store.velocity_relative_target_uncertainty(mission.storage.k_storage,:) = obj.velocity_relative_target_uncertainty;

                obj.store.position_target(mission.storage.k_storage,:) = obj.position_target; % [km]
                obj.store.position_target_uncertainty(mission.storage.k_storage,:) = obj.position_target_uncertainty;

                obj.store.velocity_target(mission.storage.k_storage,:) = obj.velocity_target; % [km/sec]
                obj.store.velocity_target_uncertainty(mission.storage.k_storage,:) = obj.velocity_target_uncertainty;

            end

        end

        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_estimate_orbit(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_estimate_orbit_selector

                    case 'Truth'
                        obj = func_update_software_SC_estimate_orbit_Truth(obj, mission, i_SC);
                        
                    case 'TruthWithErrorGrowth'
                        obj = func_update_software_SC_estimate_orbit_TruthWithErrorGrowth(obj, mission, i_SC);

                    otherwise
                        error('Orbit Estimation mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end

            % Update Storage
            obj = func_update_software_SC_estimate_orbit_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

        
        
    end
end

