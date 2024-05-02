classdef True_SC_Data_handling < handle
    %SC_DATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        time

        instantaneous_data_generated

    end
    
    methods
        function obj = True_SC_Data_handling(True_time)
            %SC_DATA Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.time = True_time.time; % [sec] 

            obj.instantaneous_data_generated = 0;

        end

        function obj = func_update_data_generated(obj,equipement)
            obj.instantaneous_data_generated = obj.instantaneous_data_generated + equipement.instantaneous_data_generated; % [kb]
        end

        
    end
end

