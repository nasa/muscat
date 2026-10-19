%% Class: Software_SC_Control_Orbit
% Control the Orbit of the Spacecraft

classdef Software_SC_Control_Orbit < handle
    % Software_SC_Control_Orbit: Manages spacecraft control parameters for orbiting target bodies.
    % Includes functionalities for trajectory calculations, intercept predictions,
    % and delta-V computations tailored for spacecraft mission scenarios.

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        mode_software_SC_control_orbit_selector % [string] Different orbit control modes
        
        max_time_before_control % [s]

        %% [ ] Properties: Variables Computed Internally

        name % [string] =  SC j for jth SC + SW Control Orbit

        instantaneous_data_generated_per_sample % [kb] : Data generated per sample, in kilo bits (kb)

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample
        % used to avoid calling isprop() every time
        % step to determine if using instantaneous_data_rate_generated or
        % instantaneous_data_generated_per_sample, which is slow

        data % Other useful data

        mode_selector_name % [string] Alternative name of mode

        flag_mission_specific_storage_variables_exists % [Boolean] Do additional mission-specific storage variables exisit? 
        
        flag_executive    % [bool]

        

        %% [ ] Properties: Storage Variables

        store % Structure to store historical data
    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = Software_SC_Control_Orbit(init_data, mission, i_SC)
            % Initialize the spacecraft control orbit class

            obj.name = [mission.true_SC{i_SC}.true_SC_body.name, ' SW Control Orbit']; % [string]

            obj.mode_software_SC_control_orbit_selector = init_data.mode_software_SC_control_orbit_selector;

            if isfield(init_data, 'max_time_before_control')
                obj.max_time_before_control = init_data.max_time_before_control;
            else
                obj.max_time_before_control = 0; % [sec]
            end

            obj.flag_executive = 0;

            obj.instantaneous_data_generated_per_sample = 0; % [kb]
           
            % Initialize generic data
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

            % Initialize Storage Variables
            obj.store = [];
            obj.store.flag_executive = zeros(mission.storage.num_storage_steps,length(obj.flag_executive));
            

            % Additional Mission-Specific Control Orbit Variables
            switch obj.mode_software_SC_control_orbit_selector

                case 'mission specific'
                    func_name = ['func_sw_SC_ctrl_orbit_constructor_', obj.mode_selector_name];

                    if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                        disp('Additional Mission-Specific Control Orbit constructor function exists!');
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, init_data, mission, i_SC);

                    else
                        disp('Additional Mission-Specific Control Orbit constructor function does NOT exist.');
                        % Do nothing!
                    end

                otherwise
                    disp('Additional Control Orbit constructor mode not defined!')
            end

            % Additional Mission-Specific Storage Variables
            func_name = ['func_update_sw_SC_ctrl_orbit_store_', obj.mode_selector_name];

            if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                disp('Additional Mission-Specific Control Orbit storage function exists!');
                obj.flag_mission_specific_storage_variables_exists = true;                
            else
                disp('Additional Mission-Specific Control Orbit storage function does NOT exist.');
                obj.flag_mission_specific_storage_variables_exists = false;
            end
            

            % Update Storage
            obj.func_update_software_SC_control_orbit_store(mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);
        end


        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_software_SC_control_orbit_store(obj, mission)
            obj.store.flag_executive(mission.storage.k_storage,:) = obj.flag_executive;
                       
            % Additional Mission-Specific Storage Variables                        
            if obj.flag_mission_specific_storage_variables_exists
                func_name = ['func_update_sw_SC_ctrl_orbit_store_', obj.mode_selector_name];
                this_func_handle = str2func(func_name);
                obj = this_func_handle(obj, mission);
            end

        end



        %% [ ] Methods: Main
        % Main Function

        function func_main_software_SC_control_orbit(obj, mission, i_SC)

            if (obj.flag_executive == 1)

                switch obj.mode_software_SC_control_orbit_selector

                    case 'mission specific'
                        func_name = ['func_update_sw_SC_ctrl_orbit_', obj.mode_selector_name];
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, mission, i_SC);
                     
                    case 'Inactive'
                        % Do nothing!

                    otherwise
                        error('Orbit Control mode not defined!')
                end

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            end


            % Do this every time step, irrespective of flag_executive status
            switch obj.mode_software_SC_control_orbit_selector

                case 'mission specific'

                    func_name = ['func_update_sw_SC_ctrl_orbit_always_on_', obj.mode_selector_name];

                    if exist(func_name, 'file') == 2 || exist(func_name, 'builtin') == 5
                        % disp('Additional Mission-Specific Control Orbit function exists!');
                        this_func_handle = str2func(func_name);
                        obj = this_func_handle(obj, mission, i_SC);

                    else
                        % disp('Additional Mission-Specific Control Orbit function does NOT exist.');
                        % Do nothing!
                    end
                    
                case 'Inactive'
                    % Do nothing!

                otherwise
                    error('Orbit Control mode not defined!')
            end


            % Update Storage
            obj.func_update_software_SC_control_orbit_store(mission);

            % Reset Variables
            obj.flag_executive = 0;


        end






    end


end
