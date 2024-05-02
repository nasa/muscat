classdef True_Earth_DTE_communication < handle
    %SC_DATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        antenna_gain
        
        temperature_noise
        
        beamwidth
        
        Energy_bit_required
        
        coding_gain
        
        Loss_pointing
        
    end
    
    methods
        function obj = True_Earth_DTE_communication(sc_body_init_data)
            %True_Earth_DTE_communication Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.antenna_gain = sc_body_init_data.ground_station_antenna_gain;
            obj.temperature_noise = sc_body_init_data.ground_station_temperature_noise;
            obj.beamwidth = sc_body_init_data.ground_station_beamwidth;
            obj.Energy_bit_required  = sc_body_init_data.ground_station_Energy_bit_required;
            obj.coding_gain = sc_body_init_data.ground_station_coding_gain;
            obj.Loss_pointing = sc_body_init_data.ground_station_loss_pointing;
        end
        
    end
end

