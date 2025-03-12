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

        function obj = func_update_software_SC_communication_Dart(obj, mission, i_SC)

            if strcmp(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode, 'DTE Comm')

                % When transmission is complete (as signaled by communication module), reset memory
                if obj.data.transmission_complete
                    % Empty memory after transmission is complete
                    mission.true_SC{i_SC}.software_SC_data_handling.mean_state_of_data_storage = 0;
                    mission.true_SC{i_SC}.software_SC_data_handling.total_data_storage = 0;
                    
                    % Log successful data transmission
                    if ~isfield(obj.data, 'successful_transmissions')
                        obj.data.successful_transmissions = 1;
                    else
                        obj.data.successful_transmissions = obj.data.successful_transmissions + 1;
                    end

                    % Update the timestamp after successful transmission completion
                    mission.true_SC{i_SC}.software_SC_communication.last_communication_time = mission.true_time.time;
                    
                    disp(['Executive: Memory cleared after successful transmission at time ', ...
                           num2str(mission.true_time.time), ' seconds']);
                           
                    % IMPORTANT: Reset the transmission_complete flag so we can start a new communication 
                    % cycle when conditions are met again
                    obj.data.transmission_complete = false;
                    
                    return;
                end
                
                % Block other operations during communication
                for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
                    mission.true_SC{i_SC}.true_SC_camera{i_HW}.flag_executive = 0;
                end
                
                for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar
                    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar >= i_HW
                        mission.true_SC{i_SC}.true_SC_science_radar{i_HW}.flag_executive = 0;
                    end
                end
                
                % Reset communication link flags to avoid "TX link already on" errors
                % This should be done regardless of previous state
                mission.true_SC{i_SC}.true_SC_communication_link{1}.flag_executive = 0;
                mission.true_SC{i_SC}.true_SC_communication_link{2}.flag_executive = 0;
                
                % Let the communication link manage these instead of setting them directly
                mission.true_SC{i_SC}.true_SC_radio_antenna{1}.flag_executive = 0;
                
                % Switch SC communication from Rx to Tx, and the contrary for GS
                mission.true_SC{i_SC}.true_SC_radio_antenna{1}.mode_true_SC_radio_antenna_selector = "TX"; % Switching SC Antenna to TX
                mission.true_GS_radio_antenna{1}.mode_true_GS_radio_antenna_selector = "RX"; % Switching GS Antenna to RX

                % Check Attitude
                this_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * mission.true_SC{i_SC}.true_SC_radio_antenna{1}.orientation')'; % [unit vector]
                vec_Earth_from_SC_normalized = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position); % [unit vector]
                obj.this_attitude_error = func_angle_between_vectors(this_orientation', vec_Earth_from_SC_normalized'); % [rad]
                
                if obj.this_attitude_error <= obj.attitude_error_threshold
                    % Attitude is good enough for communication
                    
                    % Switch on Communication Link for Dart (SC to Earth)
                    mission.true_SC{i_SC}.true_SC_communication_link{1}.flag_executive = 1;
                    
                    % Calculate data rate and transmission time
                    total_data = mission.true_SC{i_SC}.software_SC_data_handling.total_data_storage;
                    
                    % Get data rate directly - no need to use isfield since we know it's a property
                    data_rate_kbps = mission.true_SC{i_SC}.true_SC_communication_link{1}.this_data_rate;
                    
                    % Add safeguard for division by zero
                    if data_rate_kbps > 0 && total_data > 0
                        transmission_time = total_data / data_rate_kbps; % Time in seconds
                        
                        % Store communication start time and expected duration if not already tracking
                        if ~isfield(obj.data, 'current_transmission') || isempty(obj.data.current_transmission)
                            obj.data.current_transmission = struct('start_time', mission.true_time.time, ...
                                                                 'estimated_duration', transmission_time, ...
                                                                 'data_size', total_data, ...
                                                                 'data_rate', data_rate_kbps);
                            disp(['Started transmission of ', num2str(total_data), ' kb at ', ...
                                  num2str(data_rate_kbps), ' kbps (estimated time: ', ...
                                  num2str(transmission_time), ' seconds)']);
                        end
                        
                        % Check memory usage percentage against threshold
                        memory_percentage = mission.true_SC{i_SC}.software_SC_data_handling.mean_state_of_data_storage;
                        memory_threshold = 0.1; % 0.1% threshold
                        
                        % Check if memory is nearly empty (below 1%)
                        if memory_percentage <= memory_threshold
                            % Transmission complete - memory is sufficiently emptied
                                                        
                            % Log successful communication attempt
                            if ~isfield(obj.data, 'successful_communications')
                                obj.data.successful_communications = 1;
                            else
                                obj.data.successful_communications = obj.data.successful_communications + 1;
                            end
                            
                            % Reset transmission tracking
                            obj.data.current_transmission = [];
                            
                            % Signal completion to the executive to reset memory
                            obj.data.transmission_complete = true;
                            
                            disp(['Completed transmission at time ', num2str(mission.true_time.time), ...
                                  ' seconds. Memory usage now at ', num2str(memory_percentage), '%']);
                        else
                            % Still transmitting - memory not sufficiently emptied yet
                            obj.data.transmission_complete = false;
                            
                            % Optional: print status update occasionally
                            if mod(mission.true_time.time, 60) < mission.true_time.time_step
                                disp(['Transmission in progress. Memory usage: ', num2str(memory_percentage), ...
                                      '% (target: below ', num2str(memory_threshold), '%)']);
                            end
                        end
                    else
                        % Can't calculate transmission time, set flag to false
                        obj.data.transmission_complete = false;
                        
                        if total_data <= 0
                            % Memory is already empty, consider transmission complete
                            obj.data.transmission_complete = true;
                            obj.last_communication_time = mission.true_time.time;
                            disp('No data to transmit, considering transmission complete.');
                        end
                    end
                else
                    % Attitude error is too large, log failed communication attempt
                    if ~isfield(obj.data, 'failed_communications')
                        obj.data.failed_communications = 1;
                    else
                        obj.data.failed_communications = obj.data.failed_communications + 1;
                    end
                    
                    % Issue warning about poor pointing
                    warning('Communication attempt failed due to poor pointing. Attitude error: %.2f degrees, threshold: %.2f degrees', ...
                        rad2deg(obj.this_attitude_error), rad2deg(obj.attitude_error_threshold));
                        
                    % Ensure we don't have a completed flag set
                    obj.data.transmission_complete = false;
                end
            else
                % Not in communication mode, switch SC communication to Rx, and GS to Tx
                
                % Reset communication link flags
                mission.true_SC{i_SC}.true_SC_communication_link{1}.flag_executive = 0;
                mission.true_SC{i_SC}.true_SC_communication_link{2}.flag_executive = 0;
                
                % Reset radio antenna flags - let the communication link handle these
                mission.true_SC{i_SC}.true_SC_radio_antenna{1}.flag_executive = 0;
                
                mission.true_SC{i_SC}.true_SC_radio_antenna{1}.mode_true_SC_radio_antenna_selector = "RX"; % Switching SC Antenna to RX
                mission.true_GS_radio_antenna{1}.mode_true_GS_radio_antenna_selector = "TX"; % Switching GS Antenna to TX
                
                % Switch on Communication Link for GS to SC (for receiving commands)
                mission.true_SC{i_SC}.true_SC_communication_link{2}.flag_executive = 1;
                
                % Clear any ongoing transmission data
                if isfield(obj.data, 'current_transmission')
                    obj.data.current_transmission = [];
                end
                
                % Make sure transmission_complete is reset when not in communication mode
                obj.data.transmission_complete = false;
            end
        end

    end
end

