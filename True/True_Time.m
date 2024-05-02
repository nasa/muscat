classdef True_Time < handle
    %TRUE_TIME Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        t_initial % [sec] : Start time
        t_final % [sec] : Final time
        time_step % [sec] : Simulation time step
        num_time_steps % Number of simulation time steps
        t_initial_date_string % [string] : Start date of simulation. Format = [DD-MMM(words)-YYYY HH-MM-SS].
        t_initial_date % [sec from J2000] : Start date of simulation.
        time % [sec] : Current true time
        date % [sec from J2000] : Current true date
        
        time_step_dynamics % [sec] : Time step for dynamics
    end
    
    methods
        
        function obj = True_Time(time_init_data)
            %TRUE_TIME Construct an instance of this class
            
            obj.t_initial = time_init_data.t_initial;
            obj.time = obj.t_initial;
            
            obj.t_final = time_init_data.t_final;
            obj.time_step = time_init_data.time_step;
            
            obj.num_time_steps = ceil((obj.t_final - obj.t_initial)/obj.time_step);
            
            obj.t_initial_date_string = time_init_data.t_initial_date_string;
            obj.t_initial_date = cal2sec(obj.t_initial_date_string);
            obj.date = obj.t_initial_date + obj.time; % Seconds from '01-JAN-2000 00:00:00'
            
            if isfield(time_init_data, 'time_step_dynamics')
                
                if rem(time_init_data.time_step_dynamics, time_init_data.time_step) ~= 0
                    error('Dynamics time step must be a multiple of the simulation time step')
                end
                obj.time_step_dynamics = time_init_data.time_step_dynamics;
            else
                obj.time_step_dynamics = obj.time_step;
            end
        end
        
        function obj = func_update_true_time_date(obj) % Function to update current time and date
            
            obj.time = obj.time + obj.time_step;
            obj.date = obj.t_initial_date + obj.time;
            
        end
        
        function func_set_time(obj, time)
            % Set time to a specific value
            obj.time = time;
            obj.date = obj.t_initial_date + obj.time;
        end
    end
end

