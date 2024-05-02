classdef Software_SC_Communication < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        time_interSC_comm
        time_dte_comm
        begin_comm_time
        
        %communication definition
        comm_interlocutor
        comm_direction
        number_communication
        commu_use_antenna_index
        comm_needs_attitude_pointing
        
        data_exchanged
        total_data_exchanged
        
        % dynamic parameters
        time_step
        
        waiting_for_interlocutor_flag
        flag_all_data_sent
        
        % Comm
        data_storage
        maximum_data_storage
        data_needs_to_be_exchanged
        
        max_data_rate
        
        desired_attitude_for_comm_achieved
        
    end
    
    methods
        
        function obj = Software_SC_Communication(sc_body_init_data,mission_true_SC,mission_true_time)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.time_interSC_comm = 0;
            obj.time_dte_comm =0;
            obj.begin_comm_time = 0;
            
            obj.comm_interlocutor = sc_body_init_data.communication_interlocutor;
            obj.comm_direction = sc_body_init_data.communication_direction;
            obj.comm_needs_attitude_pointing = sc_body_init_data.communication_needs_attitude_pointing;
            obj.commu_use_antenna_index = sc_body_init_data.communication_use_antenna_index;
            obj.number_communication = length(obj.comm_interlocutor);
            
            obj.time_step = mission_true_time.time_step;
            
            
            % Data
            obj.data_storage = sum([mission_true_SC.true_SC_onboard_memory.memory_data.instantaneous_data_storage]);
            obj.maximum_data_storage = sum([mission_true_SC.true_SC_onboard_memory.memory_data.maximum_data_storage]);
            
            % Comms
            obj.data_needs_to_be_exchanged = zeros(1,obj.number_communication);
            obj.desired_attitude_for_comm_achieved = zeros(1,obj.number_communication);
            
            obj.data_exchanged = zeros(1,obj.number_communication);
            obj.total_data_exchanged = zeros(1,obj.number_communication);
            obj.waiting_for_interlocutor_flag = zeros(1,obj.number_communication);
            obj.flag_all_data_sent = zeros(1,obj.number_communication);
            
            
            
        end
        
        function obj = update_communication_routine(obj, true_SC_onboard_memory, true_SC_communication)
            
            obj.data_storage = sum([true_SC_onboard_memory.memory_data.instantaneous_data_storage]); % [GB]
            
            obj.max_data_rate = true_SC_communication.max_data_rate; % [kbps]
            obj.data_exchanged = zeros(1,obj.number_communication);
            
        end
        
        function obj = func_check_attitude_for_comm(obj,SC_control_attitude,SC_estimate_attitude)
            
            obj.desired_attitude_for_comm_achieved = zeros(1,obj.number_communication);
            
            for i=1:obj.number_communication
                
                if obj.comm_needs_attitude_pointing(i) == 1 % this needs attitude pointing
                    
                    % Check desired attitude for DeltaV is achieved
                    if ((SC_control_attitude.desired_SC_attitude_mode == 4) ...
                            || (SC_control_attitude.desired_SC_attitude_mode == 5)) ...
                            && (norm(SC_control_attitude.desired_attitude - SC_estimate_attitude.attitude) <= 0.1)
                        obj.desired_attitude_for_comm_achieved(i) = 1;
                        % disp('DTE Desired attitude achieved!')
                    else
                        obj.desired_attitude_for_comm_achieved(i) = 0;
                    end
                else
                    obj.desired_attitude_for_comm_achieved(i) = 1;
                end
                
            end
            
        end
        
        
        function obj = func_exchange_data(obj, mission_true_SC, this_SC_index, mission_true_time, software_SC_executive)
            
            
            for i=1:obj.number_communication
                
                id_interlocutor = obj.comm_interlocutor(i);  % who are we talking to
                
                if (id_interlocutor~=-1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  INTER SC comm
                    
                    id_comm_for_interlocutor = find( ...
                        mission_true_SC{id_interlocutor}.software_SC_communication.comm_interlocutor == this_SC_index);
                    % id of this comm for interlocutor side
                    
                    if (obj.desired_attitude_for_comm_achieved(i) == 1) && (software_SC_executive.send_data_comm(i) == 1)
                        % We want to talk to this interlocutor and the attitude is ok to do it
                        if ( ...
                                mission_true_SC{id_interlocutor}.software_SC_executive.send_data_comm(id_comm_for_interlocutor) == 1) && ...
                                (mission_true_SC{id_interlocutor}.software_SC_communication.desired_attitude_for_comm_achieved(id_comm_for_interlocutor) == 1)
                            % Interlocutor wants to talk to us and attitude is ok to do it
                            obj.waiting_for_interlocutor_flag(i) = 0;
                            
                            if  (obj.comm_direction(i) == 1)
                                % receive data
                                obj.data_exchanged(i) = mission_true_SC{id_interlocutor}.software_SC_communication.data_exchanged(id_comm_for_interlocutor);    % [kb]
                                % total received data
                                obj.total_data_exchanged(i) = obj.total_data_exchanged(i) + ((1e-6/8)*obj.data_exchanged(i));                                   % [GB]
                            elseif (obj.comm_direction(i) == -1)
                                % send data
                                obj.data_exchanged(i) = obj.max_data_rate * obj.time_step;                                                                      % [kb]
                                % total sent data
                                obj.total_data_exchanged(i) = obj.total_data_exchanged(i) + ((1e-6/8)*obj.data_exchanged(i));                                   % [GB]
                            else
                                obj.data_exchanged(i) = 0;
                            end
                            
                        elseif (mission_true_SC{id_interlocutor}.software_SC_executive.send_data_comm(id_comm_for_interlocutor) == 0) && (mission_true_SC{id_interlocutor}.software_SC_communication.flag_all_data_sent(id_comm_for_interlocutor) == 0)
                            % interlocutor is not talking to you, but he
                            % just finished during this timestep
                            obj.waiting_for_interlocutor_flag(i) = 1;   % interlocutor is busy, wait for him to be free
                            obj.data_exchanged(i) = 0;
                        elseif (mission_true_SC{id_interlocutor}.software_SC_executive.send_data_comm(id_comm_for_interlocutor) == 1) && (mission_true_SC{id_interlocutor}.software_SC_communication.desired_attitude_for_comm_achieved(id_comm_for_interlocutor) == 0)
                            % interlocutor is not talking to you, because
                            % he's waiting for its attitude correction
                            obj.waiting_for_interlocutor_flag(i) = 1;   % interlocutor is busy, wait for him to be free
                            obj.data_exchanged(i) = 0;
                        else
                            obj.data_exchanged(i) = 0;
                        end
                        
                        break % only one comm at a time
                        
                    else
                        obj.data_exchanged(i) = 0;
                    end
                    
                    
                else %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  DTE comm
                    if (obj.desired_attitude_for_comm_achieved(i) == 1) && (obj.data_needs_to_be_exchanged(i) == 1) && (software_SC_executive.send_data_to_Earth == 1)
                        % send to earth
                        obj.data_exchanged(i) = obj.max_data_rate(i) * obj.time_step; % [kb]
                        % total sent data
                        obj.total_data_exchanged(i) = obj.total_data_exchanged(i) + ((1e-6/8)*obj.data_exchanged(i)); % [GB]
                        break  % only one comm at a time
                    else
                        obj.data_exchanged(i) = 0;
                    end
                    
                end
                
            end
            
            % Decided which comm to continue, start, end
            obj.data_needs_to_be_exchanged = zeros(1,obj.number_communication);
            for i=1:obj.number_communication
                
                id_interlocutor = obj.comm_interlocutor(i);  % who are we talking to
                
                if id_interlocutor ~= -1 % if comm inter SC
                    
                    id_comm_for_interlocutor = find(mission_true_SC{id_interlocutor}.software_SC_communication.comm_interlocutor == this_SC_index);         % id of this comm for interlocutor side
                    
                    if obj.waiting_for_interlocutor_flag(i) == 1
                        % keep waiting for this interlocutor to be free
                        obj.data_needs_to_be_exchanged(i) = 1;
                        break
                    elseif ((obj.comm_direction(i) == 1) && (mission_true_SC{id_interlocutor}.software_SC_communication.data_exchanged(id_comm_for_interlocutor) ~= 0) && (mission_true_SC{id_interlocutor}.software_SC_communication.flag_all_data_sent(id_comm_for_interlocutor) == 0))
                        % there is remaining data to be received from interlocuteur and he hasn't send it all yet
                        obj.data_needs_to_be_exchanged(i) = 1;
                        break
                    elseif (obj.comm_direction(i) == 1) && (mission_true_SC{id_interlocutor}.software_SC_communication.data_exchanged(id_comm_for_interlocutor) ~= 0) && (mission_true_SC{id_interlocutor}.software_SC_communication.flag_all_data_sent(id_comm_for_interlocutor) == 1)
                        % There is remaining data during last iteration but the comm is finished after this timestep for emitter
                        obj.data_needs_to_be_exchanged(i) = 0;
                        if i<obj.number_communication
                            % The reception is finished, go to the next comm on the list
                            obj.data_needs_to_be_exchanged(i+1) = 1;
                        elseif i == obj.number_communication
                            % Every reception is done
                            obj.data_needs_to_be_exchanged = zeros(1,obj.number_communication);
                            break
                        end
                        break
                    elseif (obj.comm_direction(i) == 1) && (mission_true_SC{id_interlocutor}.software_SC_communication.data_exchanged(id_comm_for_interlocutor) == 0) && (mission_true_SC{id_interlocutor}.software_SC_communication.waiting_for_interlocutor_flag(id_comm_for_interlocutor) == 1)
                        % no data emitted by interlocutor but he's waiting for us to focus our link
                        obj.data_needs_to_be_exchanged(i) = 1;
                        break
                    elseif (obj.comm_direction(i) == -1) && (obj.data_storage/(1e-6/8)-obj.data_exchanged(i) >= 1)
                        % there is remaining data to send in memory after this timestep
                        obj.data_needs_to_be_exchanged(i) = 1;
                        break
                    elseif (obj.comm_direction(i) == -1) && (obj.data_storage/(1e-6/8)-obj.data_exchanged(i) < 1)
                        % there is no remaining data to send in memory after this timestep
                        obj.data_needs_to_be_exchanged(i) = 0;
                        obj.flag_all_data_sent(i) = 1;
                        break
                    else
                        obj.data_needs_to_be_exchanged(i) = 0;
                        obj.begin_comm_time = 0;
                    end
                else % DTE
                    if (obj.data_storage/(1e-6/8) >= 1) && (software_SC_executive.send_data_to_Earth)
                        % there is remaining data to send in memory for DTE
                        obj.data_needs_to_be_exchanged(i) = 1;
                        break
                    else
                        obj.data_needs_to_be_exchanged(i) = 0;
                    end
                end
            end
        end
    end
end

