%% Class: Software_SC_Communication
% Tracks the Communication Links of the Spacecraft

classdef Software_SC_Communication  < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        mode_software_SC_communication_selector % [string] Different communication modes
        % - 'DART'
        % - 'Nightingale'

        attitude_error_threshold % [rad]

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Communication

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        this_attitude_error % [rad] current attitude error

        data % Other useful data

        last_communication_time % [sec]
        wait_time_comm_dte  % [sec] time to wait for communicating info to earth

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Communication(init_data, mission, i_SC)
            
            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Communication']; % [string]
            obj.flag_executive = 0;

            obj.instantaneous_data_generated_per_sample = init_data.instantaneous_data_generated_per_sample; % [kb]
            obj.mode_software_SC_communication_selector = init_data.mode_software_SC_communication_selector; % [string]

            obj.attitude_error_threshold = deg2rad(init_data.attitude_error_threshold_deg); % [rad]

            obj.this_attitude_error = inf;

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize communication time tracker
            if isfield(init_data, 'last_communication_time')
                obj.last_communication_time = init_data.last_communication_time;
            else
                obj.last_communication_time = 0; % Start at 0 so first check will attempt communication
            end
            
            % Initialize wait time for DTE communications if provided
            if isfield(init_data, 'wait_time_comm_dte')
                obj.wait_time_comm_dte = init_data.wait_time_comm_dte;
            else
                obj.wait_time_comm_dte = 3600; % Default to 1 hour (3600 seconds)
            end

            % Initialize Variables to store
            obj.store = [];

            obj.data.transmission_complete = false;


            obj.store.flag_executive = zeros(mission.storage.num_storage_steps, length(obj.flag_executive));
            obj.store.this_attitude_error = zeros(mission.storage.num_storage_steps, length(obj.this_attitude_error));

            % Update Storage
            obj = func_update_software_SC_communication_store(obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variables

        function obj = func_update_software_SC_communication_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive; % [Boolean]
                obj.store.this_attitude_error(mission.storage.k_storage,:) = obj.this_attitude_error; % [rad]
            end

        end

        %% [ ] Methods: Main
        % Main Function

        function obj = func_main_software_SC_communication(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_communication_selector

                    case 'DART'
                        obj = func_update_software_SC_communication_Dart(obj, mission, i_SC);

                    case 'Nightingale'
                        obj = func_update_software_SC_communication_Nightingale(obj, mission, i_SC);

                    otherwise
                        disp('Communication mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end

            % Update Storage
            obj = func_update_software_SC_communication_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;
            obj.this_attitude_error = inf;

        end

        

    end
end

