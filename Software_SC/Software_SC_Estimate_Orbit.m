%% Class: Software_SC_Estimate_Orbit
% Estimates the Orbits of the Spacecraft and Target

classdef Software_SC_Estimate_Orbit < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample
        % used to avoid calling isprop() every time
        % step to determine if using instantaneous_data_rate_generated or
        % instantaneous_data_generated_per_sample, which is slow

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

        position_target % [km] Current position of Target wrt SUN centered J2000
        position_target_uncertainty % [km]

        velocity_target % [km/sec] Current velocity of Target wrt SUN centered J2000
        velocity_target_uncertainty % [km/sec]

        compute_time % [sec] SC time when this measurement was taken

        data % Other useful data

        mode_selector_name % [string] Alternative name of mode

        flag_mission_specific_storage_variables_exists % [Boolean] Do additional mission-specific storage variables exisit? 


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

            obj.name_relative_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.name_relative_target; % [string]
            obj.index_relative_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target; % [integer]

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            if isfield(init_data, 'mode_selector_name')
                obj.mode_selector_name = init_data.mode_selector_name;
            else
                obj.mode_selector_name = mission.name;
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

            obj = func_update_sw_SC_est_orbit_Truth(obj, mission, i_SC);
            

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


            % Additional Mission-Specific Control Orbit Variables
            switch obj.mode_software_SC_estimate_orbit_selector

                case 'mission specific'
                    func_name = ['func_sw_SC_est_orbit_constructor_', obj.mode_selector_name];

                    if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                        disp('Additional Mission-Specific Estimate Orbit constructor function exists!');
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, init_data, mission, i_SC);

                    else
                        disp('Additional Mission-Specific Estimate Orbit constructor function does NOT exist.');
                        % Do nothing!
                    end

                otherwise
                    disp('Additional Estimate Orbit constructor mode not defined!')
            end

            % Additional Mission-Specific Storage Variables
            func_name = ['func_update_sw_SC_est_orbit_store_', obj.mode_selector_name];

            if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                disp('Additional Mission-Specific Estimate Orbit storage function exists!');
                obj.flag_mission_specific_storage_variables_exists = true;                
            else
                disp('Additional Mission-Specific Estimate Orbit storage function does NOT exist.');
                obj.flag_mission_specific_storage_variables_exists = false;
            end

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
                
                % Additional Mission-Specific Storage Variables
                if obj.flag_mission_specific_storage_variables_exists
                    func_name = ['func_update_sw_SC_est_orbit_store_', obj.mode_selector_name];
                    this_func_handle = str2func(func_name);
                    obj = this_func_handle(obj, mission);
                end

            end

        end

        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_estimate_orbit(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_estimate_orbit_selector

                    case 'mission specific'
                        func_name = ['func_update_sw_SC_est_orbit_', obj.mode_selector_name];
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, mission, i_SC);

                    case 'Truth'
                        obj = func_update_sw_SC_est_orbit_Truth(obj, mission, i_SC);

                    case 'TruthWithErrorGrowth'
                        obj = func_update_sw_SC_est_orbit_TruthWithErrorGrowth(obj, mission, i_SC);

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


        %% [ ] Methods: Estimate Orbit Truth        

        function obj = func_update_sw_SC_est_orbit_Truth(obj, mission, i_SC)


            % Update compute time
            obj.compute_time = mission.true_SC{i_SC}.software_SC_executive.time;

            obj.instantaneous_data_generated_per_sample = (1e-3)*8*36; % [kb] i.e. 36 Bytes per sample

            obj.position = mission.true_SC{i_SC}.true_SC_orbit_propagation.position; % [km]
            obj.position_uncertainty = zeros(1,3);

            obj.velocity = mission.true_SC{i_SC}.true_SC_orbit_propagation.velocity; % [km/sec]
            obj.velocity_uncertainty = zeros(1,3);

            obj.position_relative_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.position_relative_target; % [km]
            obj.position_relative_target_uncertainty = zeros(1,3);

            obj.velocity_relative_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.velocity_relative_target; % [km/sec]
            obj.velocity_relative_target_uncertainty = zeros(1,3);

            obj.position_target = mission.true_target{obj.index_relative_target}.position; % [km]
            obj.position_target_uncertainty = zeros(1,3);

            obj.velocity_target = mission.true_target{obj.index_relative_target}.velocity; % [km/sec]
            obj.velocity_target_uncertainty = zeros(1,3);

        end



    end
end

