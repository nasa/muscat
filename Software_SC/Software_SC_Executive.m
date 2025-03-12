%% Class: Software_SC_Executive
% Tracks the tasks performed by Executive

classdef Software_SC_Executive < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        sc_modes % [Cells of strings] All Spacecraft Modes

        mode_software_SC_executive_selector % [string] Select which Executive to run

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        compute_wait_time % [sec]

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Executive

        time % [sec] : Current SC time
        date % [sec from J2000] : Current SC date

        this_sc_mode % [string] : Current SC mode
        this_sc_mode_value % [integer] : Current SC mode

        compute_time % [sec] SC time when this measurement was taken
        time_SB_visible % [sec]

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Executive(init_data, mission, i_SC)

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Executive']; % [string]
            obj.time = mission.true_time.time; % [sec]
            obj.date = mission.true_time.date; % [sec from J2000]

            obj.sc_modes = init_data.sc_modes;
            obj.mode_software_SC_executive_selector = init_data.mode_software_SC_executive_selector;
            obj.instantaneous_data_generated_per_sample = 0; % [kb]

            obj.this_sc_mode = obj.sc_modes{1};
            obj.this_sc_mode_value = func_find_this_sc_mode_value(obj, obj.this_sc_mode);

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            if isfield(init_data, 'compute_wait_time')
                obj.compute_wait_time = init_data.compute_wait_time; % [sec]
            else
                obj.compute_wait_time = 0; % [sec]
            end

            obj.compute_time = -inf; % [sec]

            % Initialize Variables to store: this_sc_mode_value time date data
            obj.store = [];

            obj.store.this_sc_mode_value = zeros(mission.storage.num_storage_steps, length(obj.this_sc_mode_value));
            obj.store.time = zeros(mission.storage.num_storage_steps, length(obj.time));
            obj.store.date = zeros(mission.storage.num_storage_steps, length(obj.date));
            obj.store.sc_modes = obj.sc_modes; % [sec]

            % Additional Executive Variables
            switch obj.mode_software_SC_executive_selector

                case 'DART'
                    obj.func_software_SC_executive_Dart_constructor(mission, i_SC);

                case 'Nightingale'
                    obj = func_software_SC_executive_Nightingale_constructor(obj, mission, i_SC);

                otherwise
                    % Do nothing!
                    disp('Using only DART Executive variables!')
            end

            % Update Storage
            obj = func_update_software_SC_executive_store(obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end


        %% [ ] Methods: Store
        % Update the store variable
        function obj = func_update_software_SC_executive_store(obj, mission)
            if mission.storage.flag_store_this_time_step == 1
                obj.store.this_sc_mode_value(mission.storage.k_storage,:) = obj.this_sc_mode_value; % [integer]
                obj.store.time(mission.storage.k_storage,:) = obj.time; % [sec]
                obj.store.date(mission.storage.k_storage,:) = obj.date; % [sec]

                % Additional Storage Variables
                switch obj.mode_software_SC_executive_selector
                    case 'Nightingale'
                        obj = func_update_software_SC_executive_store_Nightingale(obj, mission);

                    otherwise
                        % Do nothing!
                end
            end
        end

        %% [ ] Methods: Main
        % Main Function
        function obj = func_main_software_SC_executive(obj, mission, i_SC)

            % Update Time
            for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_clock
                mission.true_SC{i_SC}.true_SC_onboard_clock{i_HW}.flag_executive = 1; % Make sure time is measured
                obj.time = mission.true_SC{i_SC}.true_SC_onboard_clock{i_HW}.measurement_vector(1); % [sec]
                obj.date = mission.true_SC{i_SC}.true_SC_onboard_clock{i_HW}.measurement_vector(2); % [sec from J2000]
            end

            switch obj.mode_software_SC_executive_selector

                case 'DART'
                    obj = func_software_SC_executive_DART(obj, mission, i_SC);
                    obj.this_sc_mode_value = func_find_this_sc_mode_value(obj, obj.this_sc_mode);


                case 'Nightingale'
                    obj = func_software_SC_executive_Nightingale(obj, mission, i_SC);

                otherwise
                    error('Executive mode not defined!')
            end


            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            % Update Storage
            obj = func_update_software_SC_executive_store(obj, mission);

        end

        %% [ ] Methods: Find SC Mode Value
        % Finds the value of a given SC mode
        function val = func_find_this_sc_mode_value(obj, this_sc_mode)
            % Find the index of the given SC mode
            IndexC = strcmp(obj.sc_modes, this_sc_mode);
            val = find(IndexC);

            % Check if the result is empty
            if isempty(val)
                % Raise an error if nothing is found
                error('SC mode "%s" not found in obj.sc_modes.', this_sc_mode);
            end
        end


        %% [ ] Methods: DART Executive
        % Main Executive Function for DART mission
        function obj = func_software_SC_executive_DART(obj, mission, i_SC)
            % Executive logic for DART mission

            mission.true_SC{i_SC}.software_SC_estimate_attitude.flag_executive = 1;
            mission.true_SC{i_SC}.software_SC_control_attitude.flag_executive = 1;

            % %% 1. Check for Power Emergency
            % if mission.true_SC{i_SC}.true_SC_power.power_emergency
            %     warning('POWER EMERGENCY: Power deficit of %.2f W-hr detected. Entering safe mode.', ...
            %         mission.true_SC{i_SC}.true_SC_power.power_deficit);
            %
            %     % Enter power-saving mode by maximizing solar panel exposure
            %     obj.this_sc_mode = 'Maximize SP Power';
            %
            %     % Disable non-critical subsystems
            %     % - Keep only essential attitude determination and control active
            %     % - Disable science instruments
            %     for i_HW = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
            %         mission.true_SC{i_SC}.true_SC_camera{i_HW}.flag_executive = 0;
            %     end
            %
            %     % Disable orbit control and planned maneuvers
            %     mission.true_SC{i_SC}.software_SC_control_orbit.flag_executive = 0;
            %     mission.true_SC{i_SC}.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed = false;
            %     return;
            % end


            % Check if this is a new mode transition
            if ~strcmp(obj.this_sc_mode, obj.data.previous_mode)
                obj.data.last_mode_change_time = mission.true_time.time;
                disp(['Mode transition: ', obj.data.previous_mode, ' -> ', obj.this_sc_mode, ' at t=', num2str(mission.true_time.time)]);

                % Store previous mode
                obj.data.previous_mode = obj.this_sc_mode;
            end

            % Add a small delay to prevent rapid mode switching
            % Only allow mode changes after minimum time threshold, to not
            % overload the attitude control system
            time_since_last_mode_change = mission.true_time.time - obj.data.last_mode_change_time;
            if time_since_last_mode_change < 100 && obj.data.last_mode_change_time > 0
                % Skip evaluation of mode changes to avoid rapid oscillation
                % Just maintain the current mode
                return;
            end

            %% 2. Power Check - Prioritize Survival if under 30%
            if mission.true_SC{i_SC}.software_SC_power.mean_state_of_charge < 30
                obj.this_sc_mode = 'Maximize SP Power';
                return; % Highest priority - exit early
            end

            %% 3. Periodic Data Transmission (Every 5 Hour)
            time_since_last_comm = mission.true_time.time - mission.true_SC{i_SC}.software_SC_communication.last_communication_time;

            % Check if we're already in communication mode with an active link
            already_communicating = strcmp(obj.this_sc_mode, 'DTE Comm') && mission.true_SC{i_SC}.true_SC_communication_link{1}.flag_executive == 1;

            % Check if transmission just completed (to exit comm mode)
            transmission_complete = isfield(mission.true_SC{i_SC}.software_SC_communication.data, 'transmission_complete') && ...
                mission.true_SC{i_SC}.software_SC_communication.data.transmission_complete;

            % Start comm if:
            % 1. Time threshold is met (3600 seconds since last communication) AND memory usage is significant OR
            % 2. We're already communicating AND haven't completed yet
            % 3. Earth is visible AND
            % 4. No orbit maneuver is in progress
            if ((time_since_last_comm >= 3600 * 5) || ...
                    (already_communicating && ~transmission_complete)) && ...
                    (mission.true_SC{i_SC}.true_SC_navigation.flag_visible_Earth) && ...
                    ~mission.true_SC{i_SC}.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed

                obj.this_sc_mode = 'DTE Comm';

                % Make sure data handling is activated to track data storage
                mission.true_SC{i_SC}.software_SC_data_handling.flag_executive = 1;

                % Activate communication software last to ensure proper sequencing
                mission.true_SC{i_SC}.software_SC_communication.flag_executive = 1;

                return;
            end

            %% 4. Collision Check & Orbit Control
            mission.true_SC{i_SC}.software_SC_control_orbit = mission.true_SC{i_SC}.software_SC_control_orbit;
            time_since_last_orbit_check = mission.true_time.time - mission.true_SC{i_SC}.software_SC_control_orbit.last_time_control;

            if (time_since_last_orbit_check >= mission.true_SC{i_SC}.software_SC_control_orbit.max_time_before_control) && ...
                    ~mission.true_SC{i_SC}.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed  && ...
                    ~mission.true_SC{i_SC}.true_SC_navigation.flag_SC_crashed

                % Activate orbit control system to compute necessary corrections
                mission.true_SC{i_SC}.software_SC_control_orbit.flag_executive = 1;
                mission.true_SC{i_SC}.software_SC_estimate_orbit.flag_executive = 1;
            end

            %% 5. Execute DeltaV to Ensure Collision
            if mission.true_SC{i_SC}.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed && ...
                    mission.true_SC{i_SC}.software_SC_control_orbit.desired_DeltaV_computed && ...
                    ~mission.true_SC{i_SC}.true_SC_navigation.flag_SC_crashed

                % Keep turning the spacecraft to the right orientation
                obj.this_sc_mode = 'Point Thruster along DeltaV direction';

                % Make sure orbit control stays active
                mission.true_SC{i_SC}.software_SC_control_orbit.flag_executive = 1;
                mission.true_SC{i_SC}.software_SC_estimate_orbit.flag_executive = 1;

                return;
            end

            %% 6. Default Mode: Camera Pointing
            obj.this_sc_mode = 'Point camera to Target';
            mission.true_SC{i_SC}.true_SC_camera{1}.flag_executive = 1; % Activate primary camera
            mission.true_SC{i_SC}.software_SC_estimate_orbit.flag_executive = 1; % Regular updates

            %% Update Mode Value
        end


        function func_software_SC_executive_Dart_constructor(obj, mission,i_SC)
            % Add mode transition management
            if ~isfield(obj.data, 'last_mode_change_time')
                obj.data.last_mode_change_time = 0;
                obj.data.previous_mode = obj.this_sc_mode;
            end
        end


    end
end

