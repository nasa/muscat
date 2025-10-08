%% main_v3
% This is the main time loop that runs everything!
tic

%% Time Loop
disp('Starting Main Time Loop')
for k = 1:1:mission.true_time.num_time_steps

    %% Update Time, Date
    func_update_true_time_date(mission.true_time, k);

    %% Update Storage
    func_update_storage_flag(mission.storage, mission);
    func_update_time_store(mission.true_time, mission);

    %% Update Solar System
    func_main_true_solar_system(mission.true_solar_system, mission);    

    %% Update Target Position Velocity
    for i_target = 1:1:mission.num_target
        func_main_true_target(mission.true_target{i_target}, mission);        
    end

    %% For Each Spacecraft
    for i_SC = 1:1:mission.num_SC

        %% [ ] Update Solar Radiation Pressure
        func_main_true_SC_SRP(mission.true_SC{i_SC}.true_SC_SRP, mission, i_SC); 

        %% [ ] Update Gravity Gradient
        func_main_true_SC_gravity_gradient(mission.true_SC{i_SC}.true_SC_gravity_gradient, mission, i_SC);     
    
        %% [ ] Update SC Body
        func_main_true_SC_body(mission.true_SC{i_SC}.true_SC_body, mission, i_SC);

        %% [ ] Update SC Position Velocity
        func_main_true_SC_navigation(mission.true_SC{i_SC}.true_SC_navigation, mission, i_SC);        
    
        %% [ ] Update SC Onbaord Clock
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_clock
            func_main_true_SC_onboard_clock(mission.true_SC{i_SC}.true_SC_onboard_clock{i_HW}, mission, i_SC);
        end

        %% [ ] Update SC Onboard Computer
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_computer
            func_main_true_SC_onboard_computer(mission.true_SC{i_SC}.true_SC_onboard_computer{i_HW}, mission, i_SC);
        end

        %% [ ] Update Software SC Executive
        func_main_software_SC_executive(mission.true_SC{i_SC}.software_SC_executive, mission, i_SC);
        

    end


    %% Attitude Dynamics Loop (ADL)
    for k_attitude = 1:1:mission.true_time.num_time_steps_attitude

        %% [ ] Update Time in ADL
        func_update_true_time_attitude(mission.true_time, k_attitude);
        
        %% [ ] Update Storage in ADL
        func_update_storage_flag_attitude(mission.storage, mission);
        func_update_time_store_attitude(mission.true_time, mission);

        %% [ ] For Each Spacecraft
        for i_SC = 1:1:mission.num_SC

            %% [ ] [ ] Update SC Attitude in ADL
            func_main_true_SC_attitude(mission.true_SC{i_SC}.true_SC_adc, mission, i_SC);

            %% [ ] [ ] Update SC Sun Sensor in ADL
            for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor
                func_main_true_SC_sun_sensor(mission.true_SC{i_SC}.true_SC_sun_sensor{i_HW}, mission, i_SC);
            end

            %% [ ] [ ] Update SC Star Tracker in ADL
            for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker
                func_main_true_SC_star_tracker(mission.true_SC{i_SC}.true_SC_star_tracker{i_HW}, mission, i_SC);
            end

            %% [ ] [ ] Update SC IMU in ADL
            for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_imu
                func_main_true_SC_imu(mission.true_SC{i_SC}.true_SC_imu{i_HW}, mission, i_SC);
            end

            %% [ ] [ ] Update Software SC Attitude Estimation in ADL
            func_main_software_SC_estimate_attitude(mission.true_SC{i_SC}.software_SC_estimate_attitude, mission, i_SC);

    
            %% [ ] [ ] Update Software SC Attitude Control 
            func_main_software_SC_control_attitude(mission.true_SC{i_SC}.software_SC_control_attitude, mission, i_SC); 


            %% [ ] Update Micro Thrusters
            for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                func_main_true_SC_micro_thruster(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}, mission, i_SC, i_HW);
            end

            %% [ ] Update Reaction wheels
            for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
                func_main_true_reaction_wheel(mission.true_SC{i_SC}.true_SC_reaction_wheel{i_HW}, mission, i_SC, i_HW);
            end


        end
    end

    %% For Each Spacecraft
    for i_SC = 1:1:mission.num_SC

        %% [ ] Update SC Camera
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
            func_main_true_SC_camera(mission.true_SC{i_SC}.true_SC_camera{i_HW}, mission, i_SC, i_HW);
        end

        %% [ ] Update SC Science Radar 
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar
            func_main_true_SC_science_radar(mission.true_SC{i_SC}.true_SC_science_radar{i_HW}, mission, i_SC, i_HW);
        end

        %% [ ] Update SC Science Processor
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_processor
            func_main_true_SC_science_processor(mission.true_SC{i_SC}.true_SC_science_processor{i_HW}, mission, i_SC);
        end

        %% [ ] Update SC Remote Sensing 
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing
            func_main_true_SC_remote_sensing(mission.true_SC{i_SC}.true_SC_remote_sensing{i_HW}, mission, i_SC, i_HW);
        end

        %% [ ] Update Software SC Communication 
        func_main_software_SC_communication(mission.true_SC{i_SC}.software_SC_communication, mission, i_SC);

        %% [ ] Update SC Communication Link
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_communication_link
            func_main_true_SC_communication_link(mission.true_SC{i_SC}.true_SC_communication_link{i_HW}, mission, i_SC);
        end

        %% [ ] Update SC Radio Antenna 
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna
            func_main_true_SC_radio_antenna(mission.true_SC{i_SC}.true_SC_radio_antenna{i_HW}, mission, i_SC);
        end

        %% [ ] Update SC Generic Sensor
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_generic_sensor
            func_main_true_SC_generic_sensor(mission.true_SC{i_SC}.true_SC_generic_sensor{i_HW}, mission, i_SC);
        end

        %% [ ] Update SC Solar Panels
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
            func_main_true_SC_solar_panel(mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}, mission, i_SC);
        end

        %% [ ] Update Software SC Orbit Estimation in ADL
        func_main_software_SC_estimate_orbit(mission.true_SC{i_SC}.software_SC_estimate_orbit, mission, i_SC);

        %% [ ] Update Software SC Orbit Control in ADL
        func_main_software_SC_control_orbit(mission.true_SC{i_SC}.software_SC_control_orbit, mission, i_SC);
        
        %% [ ] Update Chemical Thruster 
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
            func_main_true_chemical_thruster(mission.true_SC{i_SC}.true_SC_chemical_thruster{i_HW}, mission, i_SC, i_HW);
        end

        %% [ ] Update EP Thruster 
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_ep_thruster
            func_main_true_ep_thruster(mission.true_SC{i_SC}.true_SC_ep_thruster{i_HW}, mission, i_SC, i_HW);
        end

        %% [ ] Update Fuel Tanks
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
            func_main_true_SC_fuel_tank(mission.true_SC{i_SC}.true_SC_fuel_tank{i_HW}, mission, i_SC);
        end

        %% [ ] Update SC Battery
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery
            func_main_true_SC_battery(mission.true_SC{i_SC}.true_SC_battery{i_HW}, mission, i_SC);
        end

        %% [ ] Update Software SC Power
        func_main_software_SC_power(mission.true_SC{i_SC}.software_SC_power, mission, i_SC);
    
        %% [ ] Update SC Data Handling
        func_main_true_SC_data_handling(mission.true_SC{i_SC}.true_SC_data_handling, mission, i_SC);

        %% [ ] Update SC Onboard Memory
        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory
            func_main_true_SC_onboard_memory(mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW}, mission, i_SC);
        end

        %% [ ] Update Software SC Data Handling
        func_main_software_SC_data_handling(mission.true_SC{i_SC}.software_SC_data_handling, mission, i_SC);
        
        %% [ ] Update SC Power
        func_main_true_SC_power(mission.true_SC{i_SC}.true_SC_power, mission, i_SC);

    end

    %% Update Ground Stations's Radio Antenna
    for i_HW = 1:1:mission.true_ground_station.num_GS_radio_antenna
        func_main_true_GS_radio_antenna(mission.true_GS_radio_antenna{i_HW}, mission);
    end

    %% Update Ground Station
    func_main_true_ground_station(mission.true_ground_station, mission, i_SC);

    %% Fix Warnings
    w = warning('query','last');
    if  ~isempty(w)
        warning('off',w.identifier);
    end
    
    %% Update real-time visualization
    func_update_realtime_plot(mission.storage, mission);

    %% Stop Sim
    if mission.storage.flag_stop_sim == 1
        disp('Stopping Sim!')
        break
    end
    
end


%% Close all SPICE files
cspice_kclear
mission.flag_stop_sim = 1;
