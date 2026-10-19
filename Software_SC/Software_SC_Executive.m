%% Class: Software_SC_Executive
% Tracks the tasks performed by Executive

classdef Software_SC_Executive < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        sc_modes % [Cells of strings] All Spacecraft Modes

        mode_software_SC_executive_selector % [string] Select which Executive to run

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample 
                                    % used to avoid calling isprop() every time
                                    % step to determine if using instantaneous_data_rate_generated or
                                    % instantaneous_data_generated_per_sample, which is slow

        compute_wait_time % [sec]

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Executive

        time % [sec] : Current SC time
        date % [sec from J2000] : Current SC date

        this_sc_mode % [string] : Current SC mode
        this_sc_mode_value % [integer] : Current SC mode

        flag_slew_sc_mode_exists % [Boolean] : Does a mode called 'Slew' exist?

        data % [struct] Other useful data

        mode_selector_name % [string] Alternative name of mode

        flag_mission_specific_storage_variables_exists % [Boolean] Do additional mission-specific storage variables exisit? 

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

            if isfield(init_data, 'mode_selector_name')
                obj.mode_selector_name = init_data.mode_selector_name;
            else
                obj.mode_selector_name = mission.name;
            end

            if isfield(init_data, 'compute_wait_time')
                obj.compute_wait_time = init_data.compute_wait_time; % [sec]
            else
                obj.compute_wait_time = 0; % [sec]
            end

            if isfield(init_data, 'flag_slew_sc_mode_exists')
                obj.flag_slew_sc_mode_exists = init_data.flag_slew_sc_mode_exists;
            else
                obj.flag_slew_sc_mode_exists = 0; 
            end

            % Initialize Variables to store: this_sc_mode_value time date data
            obj.store = [];

            obj.store.this_sc_mode_value = zeros(mission.storage.num_storage_steps, length(obj.this_sc_mode_value));
            obj.store.time = zeros(mission.storage.num_storage_steps, length(obj.time));
            obj.store.date = zeros(mission.storage.num_storage_steps, length(obj.date));
            obj.store.sc_modes = obj.sc_modes; % [sec]

            % Additional Mission-Specific Executive Variables
            switch obj.mode_software_SC_executive_selector

                case 'mission specific'
                    func_name = ['func_software_SC_executive_constructor_', obj.mode_selector_name];     

                    if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                            disp('Additional Mission-Specific Executive constructor function exists!');
                            this_func_handle = str2func(func_name);
                            obj = this_func_handle(obj, mission, i_SC);

                        else
                            disp('Additional Mission-Specific Executive constructor function does NOT exist.');
                            % Do nothing! 
                    end                                                                  

                otherwise
                    disp('Additional Executive constructor mode not defined!')
            end

            % Additional Mission-Specific Storage Variables
            func_name = ['func_update_software_SC_executive_store_', obj.mode_selector_name];

            if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                disp('Additional Mission-Specific Executive storage function exists!');
                obj.flag_mission_specific_storage_variables_exists = true;                
            else
                disp('Additional Mission-Specific Executive storage function does NOT exist.');
                obj.flag_mission_specific_storage_variables_exists = false;
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
                
                % Additional Mission-Specific Storage Variables
                if obj.flag_mission_specific_storage_variables_exists
                    func_name = ['func_update_software_SC_executive_store_', obj.mode_selector_name];
                    this_func_handle = str2func(func_name);
                    obj = this_func_handle(obj, mission);
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

                case 'mission specific'
                    func_name = ['func_software_SC_executive_', obj.mode_selector_name];                     
                    this_func_handle = str2func(func_name);
                    obj = this_func_handle(obj, mission, i_SC);                                   
                                                                    
                otherwise
                    error('Executive mode not defined!')
            end

            % Update Mode Value
            obj.this_sc_mode_value = func_find_this_sc_mode_value(obj, obj.this_sc_mode);

            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            % Update Storage
            obj = func_update_software_SC_executive_store(obj, mission);

        end

        
        

    end
end

