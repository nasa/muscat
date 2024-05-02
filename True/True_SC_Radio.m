classdef True_SC_Radio < handle
    %SC_DATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        number_antenna
        
        antenna_data
        % antenna_type    % [] 'dipole' / 'high_gain'
        %
        % antenna_gain    % [dB] nominal gain
        %
        % antenna_axis        % [unit vector] physical axis of antenna implementation
        %
        % antenna_pointing    % [unit vector] axis for antenna pointing to get nominal gain
        %
        % Loss_pointing
        % Loss_line
        %
        % frequency
        % wavelength
        % TX_power
        %
        % k1_pattern
        % k2_pattern
        % radiation_pattern
        % elevation
        
    end
    
    methods
        function obj = True_SC_Radio(sc_body_init_data, true_solar_system)
            %True_SC_DTE_Radio Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.number_antenna = sc_body_init_data.number_antenna;
            
            for i = 1:obj.number_antenna
                
                obj.antenna_data(i).antenna_type = sc_body_init_data.antenna_type(i);
                obj.antenna_data(i).antenna_gain = sc_body_init_data.antenna_gain(i);
                obj.antenna_data(i).antenna_axis = sc_body_init_data.antenna_axis(i,:)';
                obj.antenna_data(i).antenna_pointing = sc_body_init_data.antenna_pointing(i,:)';
                obj.antenna_data(i).frequency = sc_body_init_data.antenna_frequency(i);
                obj.antenna_data(i).wavelength = true_solar_system.light_speed/(obj.antenna_data(i).frequency*1e6);
                obj.antenna_data(i).TX_power = sc_body_init_data.tx_power(i);
                obj.antenna_data(i).Loss_line = sc_body_init_data.tx_line_loss(i);
                
                
                % Antenna radiation pattern : dipole
                step = 0.05;
                theta=0:step:2*pi+step;
                % Evaluate radiation pattern
                switch obj.antenna_data(i).antenna_type
                    case 'dipole'
                        U1 = func_dipole_raw_pattern(obj,theta);
                    case 'high_gain'
                        U1 = func_dipole_raw_pattern(obj, theta);
                    otherwise
                        % do nothing
                end
                
                U1_1=10*log10(U1);      % converting to dB scale
                obj.antenna_data(i).k1_pattern = min(U1_1);
                U_2d=U1_1-obj.antenna_data(i).k1_pattern;       % normalizing in order to make U vector positive
                obj.antenna_data(i).k2_pattern = max(U_2d);
                obj.antenna_data(i).radiation_pattern(1,:) = theta;
                obj.antenna_data(i).radiation_pattern(2,:) = U_2d.*obj.antenna_data(i).antenna_gain/obj.antenna_data(i).k2_pattern; % rescale gain for antenna
                
                obj.antenna_data(i).elevation = zeros(1,length(sc_body_init_data.communication_interlocutor));      % [radian] comm elevation angle for the different communication
                obj.antenna_data(i).Loss_pointing = zeros(1,length(sc_body_init_data.communication_interlocutor));  % [dB] pointing loss for the different communication
                
                obj.antenna_data(i).rx_noise_temperature = sc_body_init_data.SC_noise_temperature(i);
                obj.antenna_data(i).Energy_bit_required = sc_body_init_data.SC_energy_bit_required(i);
                obj.antenna_data(i).coding_gain = sc_body_init_data.SC_coding_gain(i);
                obj.antenna_data(i).beamwidth = sc_body_init_data.SC_beamwidth(i);
                obj.antenna_data(i).maximum_data_rate = sc_body_init_data.SC_maximum_data_rate(i);
            end
            
        end
        
        
        function [U] = func_dipole_raw_pattern(obj, theta)
            
            % standard parameters to get the shape of the pattern
            l_lamda1=1; % length of antenna in terms of wavelengths
            I0=1;       % max current in antenna structure
            n=120*pi;   %eta
            
            % evaluating radiation intensity(U)
            U=( n*( I0^2 )*( ( cos(l_lamda1*cos(theta)/2) - cos(l_lamda1/2) )./ sin(theta) ).^2 )/(8*(pi)^2);
            
        end
        
        function obj = func_attitude_pointing_loss(obj, mission_true_SC, this_SC_index,mission_true_solar_system)
            
            for i = 1:obj.number_antenna
                % i = this antenna
                
                for j = 1:mission_true_SC{this_SC_index}.software_SC_communication.number_communication
                    % j = this communication
                    
                    if mission_true_SC{this_SC_index}.software_SC_communication.commu_use_antenna_index(j) == i
                        % This communication use this antenna
                        interlocutor_id = mission_true_SC{this_SC_index}.software_SC_communication.comm_interlocutor(j);
                        
                        if (interlocutor_id == -1) % DTE communication
                            interlocutor_position = mission_true_solar_system.position_Earth;    % [km] Earth position in J2000
                            
                        elseif (interlocutor_id > 0) % INTER SC communication
                            interlocutor_position = mission_true_SC{interlocutor_id}.true_SC_navigation.position;    % [km] interlocutor SC position in J2000
                            
                        else
                            % should not reach here
                        end
                        
                        SC_position = mission_true_SC{this_SC_index}.true_SC_navigation.position;        % [km] This spacecraft position in J2000
                        
                        % Once emitter and receiver indentified , compute
                        % pointing loss for this antenna (i) and this
                        % communication (j)
                        link_direction = (interlocutor_position-SC_position);     % [km] position in SC frame
                        link_direction_normalized = link_direction/norm(link_direction);
                        
                        obj.antenna_data(i).elevation(j) = func_angle_between_vectors(...
                            link_direction_normalized, ...
                            mission_true_SC{this_SC_index}.true_SC_adc.rotation_matrix_SC * obj.antenna_data(i).antenna_axis);
                        
                        switch obj.antenna_data(i).antenna_type
                            case 'dipole'
                                U_SC=func_dipole_raw_pattern(obj,obj.antenna_data(i).elevation(j));
                            case 'high_gain'
                                % U1 = func_dipole_raw_pattern(theta);
                                U_SC=func_dipole_raw_pattern(obj,obj.antenna_data(i).elevation(j));
                            otherwise
                                % do nothing
                        end      % evaluating radiation intensity
                        U_SC=10*log10(U_SC);
                        U_SC=U_SC-obj.antenna_data(i).k1_pattern;
                        U_SC=U_SC*obj.antenna_data(i).antenna_gain/obj.antenna_data(i).k2_pattern;   % [dB] rescaling in dB and for correct gain
                        obj.antenna_data(i).Loss_pointing(j) = obj.antenna_data(i).antenna_gain - U_SC; % [dB] Loss compared to nominal gain
                        
                    else
                        % this communication doesn't use this antenna,
                        % no need of computing the loss
                    end
                end
                
            end
            
        end
        
    end
    
end
