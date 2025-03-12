%% Class: True_SC_Onboard_Clock
% Tracks the time onboard the SC

classdef True_SC_Onboard_Clock < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed 

        instantaneous_data_rate_generated % [kbps] : Data rate generated during current time step, in kilo bits (kb) per sec

        mode_true_SC_onboard_clock_selector % [string]
        % - Simple 

        measurement_noise % [sec] (1-sigma standard deviation) (Optional)

        measurement_wait_time % [sec]
        
        %% [ ] Properties: Variables Computed Internally

        name % [string] 'Clock i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        measurement_vector % [sec] [Time, Date]
        
        measurement_time % [sec] SC time when this measurement was taken   

        flag_executive % [Boolean] Executive has told this sensor/actuator to do its job

        %% [ ] Properties: Storage Variables

        store

    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Onboard_Clock(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Clock ',num2str(i_HW)];
            end
            
            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]
            
            if isfield(init_data, 'measurement_noise')
                obj.measurement_noise = init_data.measurement_noise; % [sec]
            else
                obj.measurement_noise = 0; % [sec]
            end

            obj.mode_true_SC_onboard_clock_selector = init_data.mode_true_SC_onboard_clock_selector; % [string]
            obj.measurement_wait_time = init_data.measurement_wait_time; % [sec]

            obj.flag_executive = 1;

            this_measurement_noise = obj.measurement_noise*2*(rand-0.5); % [sec]
            obj.measurement_vector = [(mission.true_time.time + this_measurement_noise) (mission.true_time.date + this_measurement_noise)]; % [sec] [Time, Date]
            obj.measurement_time = obj.measurement_vector(1); % [sec]
           

            % Initialize Variables to store: measurement_vector measurement_time
            obj.store = [];

            obj.store.measurement_vector = zeros(mission.storage.num_storage_steps, length(obj.measurement_vector));
            obj.store.measurement_time = zeros(mission.storage.num_storage_steps, length(obj.measurement_time));
            obj.store.measurement_noise = obj.measurement_noise; % [sec]

            % Update Storage
            obj = func_update_true_SC_onboard_clock_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end


        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_onboard_clock_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.measurement_vector(mission.storage.k_storage,:) = obj.measurement_vector; % [sec]
                obj.store.measurement_time(mission.storage.k_storage,:) = obj.measurement_time; % [sec]
            end

        end

                
        %% [ ] Methods: Main
        % Update Clock Time

        function obj = func_main_true_SC_onboard_clock(obj, mission, i_SC)

            if (obj.flag_executive == 1) && (obj.health == 1)
                % Take measurement

                if (mission.true_time.time - obj.measurement_time) >= obj.measurement_wait_time
                    % Sufficient time has elasped for a new measurement

                    switch obj.mode_true_SC_onboard_clock_selector

                        case 'Simple'
                            this_measurement_noise = obj.measurement_noise*2*(rand-0.5); % [sec]  
                            obj.measurement_vector = [(mission.true_time.time + this_measurement_noise) (mission.true_time.date + this_measurement_noise)]; % [sec]
                            obj.measurement_time = mission.true_time.time; % [sec]

                        otherwise
                            error('Clock mode not defined!')
                    end

                end
        
                % Update Power Consumed
                func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
                

                % Update Data Generated
                func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            else
                % Do nothing

            end
            
            % Update Storage
            obj = func_update_true_SC_onboard_clock_store(obj, mission);

            % Reset Variables
            obj.flag_executive = 0;

        end

        

    end
end

