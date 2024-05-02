classdef True_SC_Onboard_Memory < handle
    %SC_DATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        time

        num_memory

        memory_data

%         – # name = ‘Memory i’
%         – health : Health of ith Memory unit
%         0. Switched off
%         1. Switched on, works nominally
%         – temperature [deg C] : Temperature of ith Memory unit
%         – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith Mem-
%         ory unit (irrespective of whether it is used or not)
%         – maximum_data_storage [GB] : Note Giga Bytes (GB) = 8 × 106 kilo bits (kb)
%         – instantaneous_data_storage [GB] : Instantaneous data stored
%         – # state_of_data_storage [percentage] : SoDS is defined by = 100× instantaneous_data_storage / maximum_data_storage


    end
    
    methods
        function obj = True_SC_Onboard_Memory(True_time,sc_body_init_data)
            %SC_DATA Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.time = True_time.time; % [sec] 

            obj.num_memory = sc_body_init_data.num_memory;

            for i=1:obj.num_memory

                obj.memory_data(i).name = ['Memory ',num2str(i)];
                obj.memory_data(i).health = 1;
                obj.memory_data(i).temperature = 20;
                obj.memory_data(i).instantaneous_power_consumption = sc_body_init_data.memory_instantaneous_power_consumption(i);
                obj.memory_data(i).instantaneous_power_consumed = 0;
                obj.memory_data(i).maximum_data_storage = sc_body_init_data.memory_maximum_data_storage(i);
                obj.memory_data(i).instantaneous_data_storage = 0;
                obj.memory_data(i).state_of_data_storage = 100*obj.memory_data(i).instantaneous_data_storage/obj.memory_data(i).maximum_data_storage;

            end

        end

        function obj = func_write_to_memory(obj,instantaneous_data)
            
            data_to_store = (1e-6/8)*instantaneous_data; % [GB]
            
            for i=1:obj.num_memory

                if obj.memory_data(i).instantaneous_data_storage + data_to_store > obj.memory_data(i).maximum_data_storage  % this memory will be full
                    data_to_store = data_to_store - (obj.memory_data(i).maximum_data_storage-obj.memory_data(i).instantaneous_data_storage);    % fill the remaining space available for this memory
                    obj.memory_data(i).instantaneous_data_storage = obj.memory_data(i).maximum_data_storage;                                    % keep the rest to store in the next memory
                else                                                                                                        % write data to this memory
                    obj.memory_data(i).instantaneous_data_storage = obj.memory_data(i).instantaneous_data_storage + data_to_store;
                    data_to_store = 0;
                end

            end

            % update state of data storage
            obj = func_update_state_of_data_storage(obj);
        end

        function obj = func_delete_from_memory(obj,instantaneous_data)
            
            data_to_remove = (1e-6/8)*instantaneous_data; % [GB] 
            
            for i=obj.num_memory:-1:1

                if obj.memory_data(i).instantaneous_data_storage - data_to_remove <= 0  % this memory will be totally emty
                    data_to_remove = data_to_remove - obj.memory_data(i).instantaneous_data_storage;      % remove all the remaining data until completely free
                    obj.memory_data(i).instantaneous_data_storage = 0;                                    % keep the rest to be removed in the next memory
                else                                                                                                        % remove data from this memory
                    obj.memory_data(i).instantaneous_data_storage = obj.memory_data(i).instantaneous_data_storage - data_to_remove;
                    data_to_remove = 0;
                end

            end

            % update state of data storage
            obj = func_update_state_of_data_storage(obj);
        end


        function obj = func_update_state_of_data_storage(obj)

            for i=1:obj.num_memory

                obj.memory_data(i).state_of_data_storage = 100*obj.memory_data(i).instantaneous_data_storage/obj.memory_data(i).maximum_data_storage;
            
            end
        end

        
    end
end

