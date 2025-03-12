%% Class: True_Solar_System 
% Tracks the position of Sun, Earth, Moon, etc. and other useful panetary bodies

classdef True_Solar_System < handle    
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        num_SS_body        
        
        %% [ ] Properties: Variables Computed Internally

        SS_body % Data about selected SS body
        % - name 
        % - radius [km]
        % - mu [km^3 sec^-2]
        % - mass [kg]
        % - position [km] wrt Sun-centered J2000               
        % - velocity [km/sec] wrt Sun-centered J2000 
        % - position_array % [km] Position array wrt Sun-centered J2000, corresponding to time array in mission.true_time.time_position_array        
        % - rgb_color [string] Used for plotting

        solar_constant_AU % [W/m^2]
        AU_distance % [km]
        light_speed % [m/sec]
        gravitational_constant % [km^3 kg^{−1} s^{−2}]

        index_Sun % [integer] : Index of Sun
        index_Earth % [integer] : Index of Earth

        %% [ ] Properties: Storage Variables

        store

    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_Solar_System(init_data, mission)

            obj.solar_constant_AU = 1361; % [W/m^2]
            obj.AU_distance = 1.49598e8; % [km]
            obj.light_speed = 299792458; % [m/sec]
            obj.gravitational_constant = 6.67430e-20; % [km^3 kg^{−1} s^{−2}]            
            
            obj.num_SS_body = length(init_data.SS_body_names);
            obj.SS_body = [];

            all_SS_body_data = func_load_all_SS_body_data(obj);

            for i = 1:1:obj.num_SS_body

                this_SS_body_name = convertStringsToChars(init_data.SS_body_names(i));

                for j = 1:1:length(all_SS_body_data)

                    if strcmp(this_SS_body_name, all_SS_body_data{j}.name)
                        obj.SS_body{i} = all_SS_body_data{j};
                    end

                end

                obj.SS_body{i}.position = zeros(1,3); % [km]
                obj.SS_body{i}.velocity = zeros(1,3); % [km/sec]
                obj.SS_body{i}.position_array = zeros( mission.true_time.num_time_steps_position_array,3); % [km]


                if strcmp(obj.SS_body{i}.name, 'Sun')
                    obj.index_Sun = i;
                end

                if strcmp(obj.SS_body{i}.name, 'Earth')
                    obj.index_Earth = i;
                end

            end
                        
            % Update Position and Velocity
            cspice_furnsh('../../MuSCAT_Supporting_Files/SPICE/de440s.bsp')
            
            obj = func_main_true_solar_system(obj, mission);


            % Initialize Variables to store position and velocity of each body
            obj.store = [];

            for i = 1:1:obj.num_SS_body

                obj.store.SS_body{i}.name = obj.SS_body{i}.name;
                obj.store.SS_body{i}.position = zeros(mission.storage.num_storage_steps, length(obj.SS_body{i}.position));
                obj.store.SS_body{i}.velocity = zeros(mission.storage.num_storage_steps, length(obj.SS_body{i}.velocity));

            end

            obj = func_update_solar_system_store(obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_solar_system_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                for i = 1:1:obj.num_SS_body
                    obj.store.SS_body{i}.position(mission.storage.k_storage,:) = obj.SS_body{i}.position; % [km]
                    obj.store.SS_body{i}.velocity(mission.storage.k_storage,:) = obj.SS_body{i}.velocity; % [km/sec]
                end
            end            

        end

        
        %% [ ] Methods: Main
        % % Function to update position of Sun, Earth, Moon, etc. ... given current time

        function obj = func_main_true_solar_system(obj, mission)            

            for i = 1:1:obj.num_SS_body

                body_pos_vel_this_time = cspice_spkezr(obj.SS_body{i}.spice_name,mission.true_time.date,'J2000','NONE','SUN');

                obj.SS_body{i}.position = body_pos_vel_this_time(1:3,1)'; % [km]
                obj.SS_body{i}.velocity = body_pos_vel_this_time(4:6,1)'; % [km]

                body_pos_vel_array = (cspice_spkezr(obj.SS_body{i}.name, mission.true_time.prev_date + mission.true_time.time_position_array' ,'J2000','NONE','SUN'))';
                obj.SS_body{i}.position_array = body_pos_vel_array(:,1:3);

            end

            % Store 
            obj = func_update_solar_system_store(obj, mission);
            
        end

        
        %% [ ] Methods: Load Data
        % Store all useful data about all SS bodies

        function all_SS_body_data = func_load_all_SS_body_data(obj)            

            all_SS_body_data = [];
            k = 0;

            % Sun's Data
            this_data = [];
            this_data.name = 'Sun';
            this_data.spice_name = '10'; % [string] : Body's SPICE Name
            this_data.radius = 6.95700e5; % [km]
            this_data.mu = 1.32712440018e11; % [km^3 sec^-2]
            this_data.mass = 1.9885e30; % [kg]
            this_data.rgb_color = 'Gold'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Mercury's Data
            this_data = [];
            this_data.name = 'Mercury';
            this_data.spice_name = '199'; % [string] : Body's SPICE Name
            this_data.radius = 2.4397e3; % [km] https://en.wikipedia.org/wiki/Mercury_(planet)
            this_data.mu = 2.2032e4; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 3.3011e23; % [kg] 
            this_data.rgb_color = 'Silver'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Venus's Data
            this_data = [];
            this_data.name = 'Venus';
            this_data.spice_name = '299'; % [string] : Body's SPICE Name
            this_data.radius = 6.0518e3; % [km] https://en.wikipedia.org/wiki/Venus 
            this_data.mu = 3.24859e5; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 4.8675e24; % [kg] 
            this_data.rgb_color = 'Yellow'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Earth's Data
            this_data = [];
            this_data.name = 'Earth';
            this_data.spice_name = '399'; % [string] : Body's SPICE Name
            this_data.radius = 6.371e3; % [km]
            this_data.mu = 3.986004418e5; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 5.9722e24; % [kg]
            this_data.rgb_color = 'Navy'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Moon's Data
            this_data = [];
            this_data.name = 'Moon';
            this_data.spice_name = '301'; % [string] : Body's SPICE Name           
            this_data.radius = 1.7374e3; % [km]
            this_data.mu = 4.9048695e3; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 7.342e22; % [kg]
            this_data.rgb_color = 'Silver'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Mars's Data
            this_data = [];
            this_data.name = 'Mars';
            this_data.spice_name = '4'; % [string] : Body's SPICE Name           
            this_data.radius = 3.3895e3; % [km] https://en.wikipedia.org/wiki/Mars
            this_data.mu = 4.282837e4; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 6.4171e23; % [kg]
            this_data.rgb_color = 'DarkRed'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Jupiter's Data
            this_data = [];
            this_data.name = 'Jupiter';
            this_data.spice_name = '5'; % [string] : Body's SPICE Name           
            this_data.radius = 6.9911e4; % [km] https://en.wikipedia.org/wiki/Jupiter
            this_data.mu = 1.26686534e8; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 1.8982e27; % [kg]
            this_data.rgb_color = 'Orange'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Saturn's Data
            this_data = [];
            this_data.name = 'Saturn';
            this_data.spice_name = '699'; % [string] : Body's SPICE Name           
            this_data.radius = 5.8232e4; % [km] https://en.wikipedia.org/wiki/Saturn
            this_data.mu = 3.7931187e7; % [km^3 sec^-2] https://en.wikipedia.org/wiki/Standard_gravitational_parameter
            this_data.mass = 5.6834e26; % [kg]
            this_data.rgb_color = 'Goldenrod'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Mimas's Data
            this_data = [];
            this_data.name = 'Mimas';
            this_data.spice_name = '601'; % [string] : Body's SPICE Name           
            this_data.radius = 1.982e2; % [km] https://en.wikipedia.org/wiki/Mimas
            this_data.mass = 3.75094e19; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Enceladus's Data
            this_data = [];
            this_data.name = 'Enceladus';
            this_data.spice_name = '602'; % [string] : Body's SPICE Name           
            this_data.radius = 2.521e2; % [km] https://en.wikipedia.org/wiki/Enceladus
            this_data.mass = 1.080318e20; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Tethys's Data
            this_data = [];
            this_data.name = 'Tethys';
            this_data.spice_name = '603'; % [string] : Body's SPICE Name           
            this_data.radius = 5.614e2; % [km] https://en.wikipedia.org/wiki/Tethys_(moon)
            this_data.mass = 6.1749e20; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Dione's Data
            this_data = [];
            this_data.name = 'Dione';
            this_data.spice_name = '604'; % [string] : Body's SPICE Name           
            this_data.radius = 5.31e2; % [km] https://en.wikipedia.org/wiki/Dione_(moon)
            this_data.mass = 1.0954868e21; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Rhea's Data
            this_data = [];
            this_data.name = 'Rhea';
            this_data.spice_name = '605'; % [string] : Body's SPICE Name           
            this_data.radius = 7.635e2; % [km] https://en.wikipedia.org/wiki/Rhea_(moon)
            this_data.mass = 2.3064854e21; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Titan's Data
            this_data = [];
            this_data.name = 'Titan';
            this_data.spice_name = '606'; % [string] : Body's SPICE Name           
            this_data.radius = 2.57473e3; % [km] https://en.wikipedia.org/wiki/Titan_(moon)
            this_data.mass = 1.3452e23; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Hyperion's Data
            this_data = [];
            this_data.name = 'Hyperion';
            this_data.spice_name = '607'; % [string] : Body's SPICE Name           
            this_data.radius = 1.35e2; % [km] https://en.wikipedia.org/wiki/Hyperion_(moon)
            this_data.mass = 5.5510e18; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Iapetus's Data
            this_data = [];
            this_data.name = 'Iapetus';
            this_data.spice_name = '608'; % [string] : Body's SPICE Name           
            this_data.radius = 7.344e2; % [km] https://en.wikipedia.org/wiki/Iapetus_(moon)
            this_data.mass = 1.80565e21; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Phoebe's Data
            this_data = [];
            this_data.name = 'Phoebe';
            this_data.spice_name = '609'; % [string] : Body's SPICE Name           
            this_data.radius = 1.065e2; % [km] https://en.wikipedia.org/wiki/Iapetus_(moon)
            this_data.mass = 8.3123e18; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;

            % Helene's Data
            this_data = [];
            this_data.name = 'Helene';
            this_data.spice_name = '612'; % [string] : Body's SPICE Name           
            this_data.radius = 1.81e2; % [km] https://en.wikipedia.org/wiki/Iapetus_(moon)
            this_data.mass = 7.1e15; % [kg]
            this_data.mu = obj.gravitational_constant * this_data.mass; % [km^3 sec^-2]             
            this_data.rgb_color = 'Gray'; % [string]
            k = k + 1;
            all_SS_body_data{k} = this_data;


        end
        
    end
end
