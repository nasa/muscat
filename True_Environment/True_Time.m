%% Class: True_Time 
% Keeps track of the all time variables in the simulation

classdef True_Time < handle


    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        t_initial % [sec] : Start time
        t_final % [sec] : Final time
        time_step % [sec] : Simulation time step
        t_initial_date_string % [string] : Start date of simulation. Format = [DD-MMM(words)-YYYY HH-MM-SS].
                        
        time_step_attitude % [sec] : Time step for attitude dynamics (Optional)

        print_progress_steps % [integer] : Number of steps to skip between printing progress (Optional)

        time_step_position_array % [sec] : Time step for position dynamics array (Optional)
        
        %% [ ] Properties: Variables Computed Internally

        t_initial_date % [sec from J2000] : Start date of simulation.

        num_time_steps % [integer] : Number of simulation time steps       #
        time % [sec] : Current true time                                   #
        date % [sec from J2000] : Current true date                        #

        num_time_steps_attitude % [integer] : Number of attitude dynamics loop time steps within one simulation time step #
        time_attitude % [sec] : Current true time within attitude dynamics loop #

        k % [integer] : Time loop variable                                 #
        k_attitude % [integer] : Attitude Dynamics Time loop variable      #

        num_time_steps_position_array % [integer]
        time_position_array % [sec]

        prev_time % [sec] : Previous true time                                   #
        prev_date % [sec from J2000] Previous true date

        data % to store other values

        %% [ ] Properties: Storage Variables

        store

    end
    

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_Time(init_data)
                       
            obj.t_initial = init_data.t_initial;
            obj.time = obj.t_initial;
            
            obj.t_final = init_data.t_final;
            obj.time_step = init_data.time_step;

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end
            
            obj.num_time_steps = ceil((obj.t_final - obj.t_initial)/obj.time_step);
            
            obj.t_initial_date_string = init_data.t_initial_date_string;
            obj.t_initial_date = cal2sec(obj.t_initial_date_string);
            obj.date = obj.t_initial_date + obj.time; % Seconds from '01-JAN-2000 00:00:00'

            if isfield(init_data, 'time_step_attitude')
                % time_step_attitude_dynamics has been specified
                obj.time_step_attitude = init_data.time_step_attitude; 
            else
                obj.time_step_attitude = obj.time_step;
            end

            obj.num_time_steps_attitude = (obj.time_step/obj.time_step_attitude); 

            if ~isinteger(int32(obj.num_time_steps_attitude))
                error('time_step must be divisble by time_step_attitude_dynamics!');
            end

            obj.time_attitude = obj.time;

            obj.k = 0;
            obj.k_attitude = 0;


            if isfield(init_data, 'print_progress_steps')
                % print_progress_steps has been specified
                obj.print_progress_steps = init_data.print_progress_steps;
            else
                obj.print_progress_steps = ceil((obj.num_time_steps)/1000);
            end

            
            % Create Time Array for Position Dynamics

            if isfield(init_data, 'time_step_position_array')
                obj.time_step_position_array = init_data.time_step_position_array;
            else
                obj.time_step_position_array = min([10, obj.time_step, obj.time_step_attitude]); % [sec]
            end

            obj.num_time_steps_position_array = (obj.time_step/obj.time_step_position_array) + 1; 

            if ~isinteger(int32(obj.num_time_steps_position_array))
                error('time_step must be divisble by time_step_position_array!');
            end

            obj.time_position_array = [0: obj.time_step_position_array : obj.time_step]';


            obj.prev_time = obj.time; % [sec] 
            obj.prev_date = obj.date; % [sec from J2000]           
                        
        end

        %% [ ] Methods: Initialize Store
        % Initialize the store variable

        function obj = func_initialize_time_store(obj, mission)

            % Variables to store: time, date, time_attitude
            obj.store.time = zeros(mission.storage.num_storage_steps, length(obj.time));
            obj.store.date = zeros(mission.storage.num_storage_steps, length(obj.date));
            obj.store.time_attitude = zeros(mission.storage.num_storage_steps_attitude, length(obj.time_attitude));            

            % Store first set of variables
            obj = func_update_time_store(obj, mission);
            obj = func_update_time_store_attitude(obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_time_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.time(mission.storage.k_storage,:) = obj.time; % [sec]
                obj.store.date(mission.storage.k_storage,:) = obj.date; % [sec]                
            end            

        end

        %% [ ] Methods: Store Attitude
        % Update the attitude store variable

        function obj = func_update_time_store_attitude(obj, mission)

            if mission.storage.flag_store_this_time_step_attitude == 1
                obj.store.time_attitude(mission.storage.k_storage_attitude,:) = obj.time_attitude; % [sec]
            end            

        end



        %% [ ] Methods: Main 
        % Function to update current time and date within main loop
        
        function obj = func_update_true_time_date(obj, k) 

            obj.k = k;

            obj.prev_time = obj.time; % [sec] 
            obj.prev_date = obj.date; % [sec from J2000]
            
            obj.time = obj.time + obj.time_step;
            obj.date = obj.t_initial_date + obj.time;
            
            % Print progress
            if mod(obj.k, obj.print_progress_steps) == 0

                % Expected time left
                time_elap = seconds(toc);
                time_per_loop = time_elap / obj.k;
                time_left = time_per_loop * (obj.num_time_steps - obj.k);

                time_sim_elapsed = seconds(obj.time - obj.t_initial);
                time_sim_total = seconds(obj.t_final - obj.t_initial);

                perc = round(time_sim_elapsed / time_sim_total * 100, 1);

                format = 'dd:hh:mm:ss';
                time_elap.Format = format;
                time_left.Format = format;
                time_sim_elapsed.Format = format;
                time_sim_total.Format = format;

                % Base progress message
                progress_msg = ['- Simulation: ', char(time_sim_elapsed), ' / ', char(time_sim_total), ' (', num2str(perc),'%),' ...
                    ' Elapsed: ', char(time_elap), ', Left: ', char(time_left), ', Total: ', char(time_elap + time_left)];

                % Add memory info every 100 iterations
                if mod(obj.k, obj.print_progress_steps * 100) == 0
                    memoryInfo = evalc('dispmemory()');
                    progress_msg = [progress_msg, '  |  Current Memory Usage By Matlab: ', strtrim(memoryInfo)];
                end

                disp(progress_msg)
            end
            
        end

        %% [ ] Methods: Main Attitude
        % Function to update current time within attitude dyanmics loop
        
        function obj = func_update_true_time_attitude(obj, k_attitude) 

            obj.k_attitude = k_attitude;            
            obj.time_attitude = obj.time_attitude + obj.time_step_attitude;
            
        end

        %% [ ] Methods: Set Time
        % Set time to a specific value

        function obj = func_set_time(obj, time)
            
            obj.time = time;
            obj.date = obj.t_initial_date + obj.time;

        end

        

    end
end
