classdef True_SC_Communication < handle
    %SC_DATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        number_communication
        
        time
        prev_time
        compute_time_threshold
        
        compute_data_rate
        given_data_rate
        link_margin
        
        max_data_rate
    end
    
    methods
        function obj = True_SC_Communication(sc_body_init_data,mission_true_time)
            %True_Earth_DTE_communication Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.number_communication = length(sc_body_init_data.communication_interlocutor);
            
            obj.time = mission_true_time.time;
            obj.prev_time = -inf;
            obj.compute_time_threshold = 1; % [sec]
            
            obj.compute_data_rate = sc_body_init_data.compute_data_rate;
            obj.given_data_rate = sc_body_init_data.given_data_rate;
            obj.link_margin = sc_body_init_data.minimum_required_link_margin;
            
            obj.max_data_rate = zeros(1,obj.number_communication);
            
        end
        
        function obj = func_link_budget(obj, mission_true_time, mission_true_SC, this_SC_index, mission_true_solar_system)
            
            obj.time = mission_true_time.time;
            
            if obj.compute_data_rate == 1 % Compute Data Rate
                
                if (obj.time - obj.prev_time) >= obj.compute_time_threshold
                    
                    for i=1:obj.number_communication
                        
                        id_antenna = mission_true_SC{this_SC_index}.software_SC_communication.commu_use_antenna_index(i); % index of antenna to use for this communication
                        
                        % Emitter : this SC
                        ERP = 10*log10(mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).TX_power) + mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).antenna_gain - mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).Loss_line - mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).Loss_pointing(i); % [dBW]
                        EIRP = (ERP + 2.15) + 30; % [dBm]
                        
                        SC_position = mission_true_SC{this_SC_index}.true_SC_navigation.position;
                        
                        % Receiver : interlocutor
                        id_interlocutor = mission_true_SC{this_SC_index}.software_SC_communication.comm_interlocutor(i); % index of interloctor for this communication
                        if (id_interlocutor == -1) % DTE communication
                            interlocutor_position = mission_true_solar_system.position_Earth;    % [km] Earth position in J2000
                            interlocutor_pointing_loss = mission_true_SC{this_SC_index}.true_earth_dte_communication.Loss_pointing;
                            interlocutor_antenna_gain = mission_true_SC{this_SC_index}.true_earth_dte_communication.antenna_gain;
                            interlocutor_noise_temperature = mission_true_SC{this_SC_index}.true_earth_dte_communication.temperature_noise;
                            interlocutor_Energy_bit_required = mission_true_SC{this_SC_index}.true_earth_dte_communication.Energy_bit_required;
                            interlocutor_coding_gain= mission_true_SC{this_SC_index}.true_earth_dte_communication.coding_gain;
                            interlocutor_beamwidth= mission_true_SC{this_SC_index}.true_earth_dte_communication.beamwidth;
                            
                        elseif (id_interlocutor > 0) % INTER SC communication
                            interlocutor_position = mission_true_SC{id_interlocutor}.true_SC_navigation.position;    % [km] interlocutor SC position in J2000
                            index_comm_for_interlocutor = find(mission_true_SC{id_interlocutor}.software_SC_communication.comm_interlocutor == this_SC_index);                  %find which index is this communication on interlocutor side
                            index_antenna_for_interlocutor = mission_true_SC{id_interlocutor}.software_SC_communication.commu_use_antenna_index(index_comm_for_interlocutor);   %find which antenna interlocutor is using for this comm
                            
                            antenna_data = mission_true_SC{id_interlocutor}.true_SC_radio.antenna_data(index_antenna_for_interlocutor);
                            interlocutor_pointing_loss = antenna_data.Loss_pointing(index_comm_for_interlocutor);
                            interlocutor_antenna_gain = antenna_data.antenna_gain;
                            interlocutor_noise_temperature = antenna_data.rx_noise_temperature;
                            interlocutor_Energy_bit_required = antenna_data.Energy_bit_required;
                            interlocutor_coding_gain = antenna_data.coding_gain;
                            interlocutor_beamwidth = antenna_data.beamwidth;
                            
                        else
                            % should not reach here
                        end
                        
                        
                        % Free space Loss
                        H = norm(SC_position - interlocutor_position)*1000;                 % [m]
                        Loss_freespace = 20*log10(4*pi*H/mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).wavelength);     % [dB]
                        
                        % Receiver : interlocutor
                        RIP = EIRP - Loss_freespace - interlocutor_pointing_loss;  % [dBm]
                        
                        P_rx = RIP + interlocutor_antenna_gain - 30; % [dB]
                        N0 = 10*log10(interlocutor_noise_temperature) - 228.6;% [dBW/Hz] 228.6dBW/K/Hz = Boltzman
                        
                        Eb_noise_ratio_net = interlocutor_Energy_bit_required + obj.link_margin;
                        Eb_noise_ratio = Eb_noise_ratio_net - interlocutor_coding_gain;
                        data_rate = (1e-3)*interlocutor_beamwidth*10^( ((P_rx-N0) - Eb_noise_ratio) / 10 ); % [kbps] data rate to get link margin
                        
                        %limit data rate
                        if data_rate > mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).maximum_data_rate
                            obj.max_data_rate(i) = mission_true_SC{this_SC_index}.true_SC_radio.antenna_data(id_antenna).maximum_data_rate;
                        end
                        
                    end
                    
                    obj.prev_time = obj.time;
                    
                else
                    % Dont change obj.max_data_rate
                    obj.max_data_rate = zeros(1,obj.number_communication) * obj.given_data_rate;
                end
                
                
            else
                obj.max_data_rate = obj.given_data_rate*ones(1,obj.number_communication); % [kbps]
            end
            
        end
        
    end
end

