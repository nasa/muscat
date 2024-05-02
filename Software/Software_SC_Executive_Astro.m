classdef Software_SC_Executive_Astro < handle
    % SC_EXECUTIVE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % dynamic parameters
        time
        t_initial_date
        date
        time_step
        prev_time
        geometry
        
        % boolean parameters
        get_time
        estimate_SC_attitude
        estimate_SC_SB_orbits
        estimate_SB_properties
        control_SC_attitude
        desired_SC_attitude_mode
        control_SC_orbit
        get_camera_image
        
        % objectives from MEXEC
        requested_estimate_SC_attitude
        requested_estimate_SC_SB_orbits
        requested_control_SC_attitude
        requested_control_SC_orbit
        requested_get_camera_image
        requested_send_data_to_Earth
        requested_point_solar_panels_to_Sun
        
        % status to MEXEC
        running_control_SC_orbit % 1 when running, 0 otherwise
        running_send_data_to_Earth
        running_point_solar_panels_to_Sun
        running_exchange_data
        
        desat_procedure
        RW_desat_threshold
        
        SP_needs_to_point_Sun % should move to SC_power
        point_solar_panels_to_Sun
        
        % wait times
        wait_time_estimate_SC_attitude
        wait_time_estimate_SC_SB_orbits
        wait_time_estimate_SB_properties
        wait_time_control_SC_attitude
        wait_time_control_SC_orbit
        wait_time_get_camera_image
        wait_time_comm_interSC
        wait_time_comm_dte
        wait_time_telescope
        
        
        last_camera_process_time
        
        % timelines
        trajectory_progress_timeline
        trajectory_max_distance_SB
        memory_timeline
        battery_soc_timeline
        sc_attitude_timeline
        tcm_atomic_timeline
        sbp_atomic_timeline
        shunt_atomic_timeline
        
        % Communication
        send_data_to_Earth
        Data_max_threshold
        send_data_comm
        
        % Battery Thresholds
        Battery_SOC_min_threshold
        Battery_SOC_max_threshold
        
        % science
        science_mode_enabled
        time_target_visible
        
        mass % [kg] : Mass of the SC
        %             – dry_mass % [kg] : Dry mass of the SC, that doesn’t change position/attitude
        %             – propellant_mass % [kg] : Propellant mass of the SC, that does change value
        %             – supplement_mass % [kg] : Positive/negative mass that is added/removed from the SC. (e.g. sample collection or projectile)
        %             – # solar_panel_mass % [kg] : Total solar panel mass of the SC, that doesn’t change value
        %             – # total_mass % [kg] : Total mass of the SC
        %             – location_propellant_mass % [m] : Location of the propellant mass of the SC, in body
        %             frame B
        %             – location_supplement_mass % [m] : Location of the positive/negative supplement mass of the SC, in body frame B
        
    end
    
    methods
        function obj = Software_SC_Executive_Astro(True_SC_body,True_time,software_SC_communication)
            % EXECUTIVE Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = True_time.time;
            obj.time_step = True_time.time_step;
            obj.t_initial_date = True_time.t_initial_date;
            obj.date = True_time.date;
            obj.prev_time = 0;
            obj.get_time = 1;
            
            obj.requested_estimate_SC_attitude = 1;
            obj.requested_estimate_SC_SB_orbits = 1;
            obj.requested_control_SC_attitude = 1;
            obj.requested_control_SC_orbit = 1;
            obj.requested_get_camera_image = 1;
            obj.requested_send_data_to_Earth = 1;
            obj.requested_point_solar_panels_to_Sun = 1;
            
            % approximate geometry from vertices into a rectangular shape (true if actually recatngular)
            all_vertices = cell2mat(cellfun(@(x) x.Vertices, True_SC_body.shape_model, 'UniformOutput', false));
            obj.geometry = max(all_vertices);
            
            obj.wait_time_estimate_SC_attitude = 0;
            obj.wait_time_estimate_SC_SB_orbits = 0;
            obj.wait_time_estimate_SB_properties = 10*60;
            obj.wait_time_control_SC_attitude = 0;
            obj.wait_time_control_SC_orbit = 1*60*60;
            obj.wait_time_get_camera_image = 60;
            obj.wait_time_comm_interSC = 0*60*60;
            obj.wait_time_comm_dte = 2*60*60;
            obj.wait_time_telescope = 1*60;
            
            obj.running_control_SC_orbit = 0;
            obj.running_send_data_to_Earth = 0;
            obj.running_point_solar_panels_to_Sun = 0;
            
            obj.SP_needs_to_point_Sun = 0;
            obj.point_solar_panels_to_Sun = 0;
            
            obj.RW_desat_threshold = 0.95; % 80% of RPM max
            obj.desat_procedure = 0;
            
            obj.Battery_SOC_min_threshold = 30; % [percentage]
            obj.Battery_SOC_max_threshold = 50; % [percentage]
            
            obj.time_target_visible = 0;
            
            obj.last_camera_process_time = 0;
            
            obj.desired_SC_attitude_mode = 1;
            
            obj.Data_max_threshold = 0.1;
            obj.send_data_to_Earth = 0;
            obj.send_data_comm = zeros(1,software_SC_communication.number_communication);
            
            % timelines
            obj.trajectory_progress_timeline = 0;
            obj.trajectory_max_distance_SB = 0;
            obj.memory_timeline = 0;
            obj.battery_soc_timeline = 0;
            obj.sc_attitude_timeline = 0;
            obj.tcm_atomic_timeline = 0;
            obj.sbp_atomic_timeline = 0;
            obj.shunt_atomic_timeline = 0;
            
            obj.science_mode_enabled = 0;
            
            obj.mass = True_SC_body.mass;
            
        end
        
        function obj = func_schedule_main_tasks(obj,mission_true_SC)
            
            % Estimate attitude
            if (obj.time - mission_true_SC.software_SC_estimate_attitude.time) >= obj.wait_time_estimate_SC_attitude
                obj.estimate_SC_attitude = obj.requested_estimate_SC_attitude; % Get the value from the MEXEC interface;
            else
                obj.estimate_SC_attitude = 0;
            end
            
            % Estimate orbit
            if (obj.time - mission_true_SC.software_SC_estimate_SC_SB_orbit.time) >= obj.wait_time_estimate_SC_SB_orbits
                obj.estimate_SC_SB_orbits = obj.requested_estimate_SC_SB_orbits; % Get the value from the MEXEC interface;
            else
                obj.estimate_SC_SB_orbits = 0;
            end
            
            % Control attitude
            if (obj.time - mission_true_SC.software_SC_control_attitude.time) >= obj.wait_time_control_SC_attitude
                obj.control_SC_attitude = obj.requested_control_SC_attitude; % Get the value from the MEXEC interface;
            else
                obj.control_SC_attitude = 0;
            end
            
            % Get Camera Images
            if isfield(mission_true_SC, 'true_SC_camera_sensor')
                if (obj.time - obj.last_camera_process_time) >= obj.wait_time_get_camera_image
                    obj.get_camera_image = obj.requested_get_camera_image; % Get the value from the MEXEC interface
                else
                    obj.get_camera_image = 0;
                end
                
                % Check if target is visible on any of the cameras (no occultation)
                flag_SB_visible_on_camera = 0;
                for c=1:mission_true_SC.true_SC_camera_sensor.num_camera
                    if mission_true_SC.true_SC_camera_sensor.camera_data(c).flag_SB_visible == 1
                        flag_SB_visible_on_camera = flag_SB_visible_on_camera + 1;
                    end
                end
                if flag_SB_visible_on_camera == 0
                    obj.time_target_visible = inf;
                else
                    if obj.time_target_visible == inf
                        obj.time_target_visible = obj.time;
                    end
                end
            end
            
            % Telescope update target
            if isfield(mission_true_SC, 'true_SC_telescope')
                if (obj.time - mission_true_SC.true_SC_telescope.time) >= obj.wait_time_telescope
                    mission_true_SC.true_SC_telescope = mission_true_SC.true_SC_telescope.func_update_target(mission_true_SC.true_SC_telescope);
                end
            end
            
            % Control orbit
            if ((obj.time - mission_true_SC.software_SC_control_orbit.time) >= obj.wait_time_control_SC_orbit)
                obj.estimate_SC_SB_orbits = 1; % Must estimate orbits first
                obj.control_SC_orbit = 1;
                obj.running_control_SC_orbit = 1;
            elseif mission_true_SC.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed == 1
                obj.control_SC_orbit = obj.requested_control_SC_orbit;  % Get the value from the MEXEC interface;
            else
                obj.control_SC_orbit = 0;
            end
            
            % Exchange data communication
            if (mission_true_SC.true_SC_body.flag_hardware_exists.intersat_communication == 1) ...
                    && ((obj.time - mission_true_SC.software_SC_communication.time_interSC_comm) >= obj.wait_time_comm_interSC)
                % Intersat communication
                % [ Condition of activation ] :
                %      * no orbit control running
                %      * last comm time is greater than wait_time_comm_interSC
                %      * This SC has inter SC comm
                
                % trigger all intersat comm, they will be organized in communication class
                if mission_true_SC.id == 1
                    % Chief SC, send data to all other SC
                    obj.send_data_comm(1) = 1; % DTE
                    obj.send_data_comm(2:end) = 1; % Deputy SC
                else
                    % Deputy SC, send data to chief SC
                    obj.send_data_comm(1) = 1; % Chief SC
                end
                
            elseif (mission_true_SC.true_SC_body.flag_hardware_exists.dte_communication == 1) ...
                    && (sum(mission_true_SC.software_SC_communication.data_needs_to_be_exchanged) == 0) ...
                    && (obj.control_SC_orbit == 0) ...
                    && ((obj.time - mission_true_SC.software_SC_control_orbit.time) < obj.wait_time_control_SC_orbit) ...
                    && ((obj.time - mission_true_SC.software_SC_communication.time_dte_comm) >= obj.wait_time_comm_dte ...
                    || ((mission_true_SC.software_SC_communication.data_storage / mission_true_SC.software_SC_communication.maximum_data_storage) > obj.Data_max_threshold))
                % DTE communication
                % [ Condition of activation ] :
                %      * no orbit control running
                %      * last comm time is greater than wait_time_comm_dte
                %      * This SC has inter DTE comm
                %      * no other comm running to overlap
                obj.send_data_to_Earth = 1;
            else
                % Check for unfinished comm
                for i=1:mission_true_SC.software_SC_communication.number_communication
                    if mission_true_SC.software_SC_communication.data_needs_to_be_exchanged(i) == 1
                        obj.send_data_comm(i) = 1;
                        if mission_true_SC.software_SC_communication.comm_interlocutor(i) == -1
                            obj.send_data_to_Earth = 1;
                        end
                        break
                    else
                        obj.send_data_comm(i) = 0;
                        if mission_true_SC.software_SC_communication.comm_interlocutor(i) == -1
                            obj.send_data_to_Earth = 0;
                        end
                    end
                end
            end
            if any(obj.send_data_comm == 1) || (obj.send_data_to_Earth == 1)
                obj.running_exchange_data = 1;
            else
                obj.running_exchange_data = 0;
            end
            
            
            % Check Power
            mean_battery_state_of_charge = 0;
            for i=1:1:mission_true_SC.true_SC_battery.num_battery
                mean_battery_state_of_charge = mean_battery_state_of_charge ...
                    + ((1/mission_true_SC.true_SC_battery.num_battery) * mission_true_SC.true_SC_battery.battery_data(i).state_of_charge);
            end
            
            if mean_battery_state_of_charge <= obj.Battery_SOC_min_threshold
                obj.point_solar_panels_to_Sun = obj.requested_point_solar_panels_to_Sun;
                % Switch off others
                obj.send_data_to_Earth = 0;
                obj.control_SC_orbit = 0;
            else
                obj.point_solar_panels_to_Sun = 0;
            end
            
            if (obj.point_solar_panels_to_Sun == 1)
                obj.running_point_solar_panels_to_Sun = 1;
            elseif (obj.running_point_solar_panels_to_Sun == 1)
                % Check if charging is finished
                for i=1:1:mission_true_SC.true_SC_battery.num_battery
                    if mission_true_SC.true_SC_battery.battery_data(i).state_of_charge <= 98
                        % While battery are not fully charged
                        obj.running_point_solar_panels_to_Sun = 1;
                        break
                    else
                        obj.running_point_solar_panels_to_Sun = 0;
                    end
                end
            else
                obj.running_point_solar_panels_to_Sun = 0;
            end
            
            % 1 = Point camera to SB
            % 2 = Maximize SP Power
            % 3 = Point Thurster along DeltaV direction
            % 4 = Point Antenna to Earth
            % 5 = Point Antenna for InterSat comm
            % 6 = Point to target (e.g., exoplanet) and maximize SP Power
            % 7 = Point to Earth, target (if possible) and maximize SP Power
            desired_SC_attitude_mode_prev = obj.desired_SC_attitude_mode;
            flag_simultaneous_dte_comm_and_science = true;
            if obj.running_point_solar_panels_to_Sun == 1
                obj.desired_SC_attitude_mode = 2;
                
            elseif (mission_true_SC.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed == 1) ...
                    && (obj.time >= mission_true_SC.software_SC_control_orbit.time_data_cutoff)
                obj.desired_SC_attitude_mode = 3;
                
            elseif (obj.send_data_to_Earth == 1) && ~flag_simultaneous_dte_comm_and_science
                id_dte_comm = find(mission_true_SC.software_SC_communication.comm_interlocutor == -1);
                if mission_true_SC.software_SC_communication.comm_needs_attitude_pointing(id_dte_comm) == 1
                    % if attitude pointing needed for this comm
                    obj.desired_SC_attitude_mode = 4;
                end
                
            elseif (obj.send_data_to_Earth == 0) && (sum(obj.send_data_comm) == 1) && ...
                    mission_true_SC.software_SC_communication.comm_needs_attitude_pointing(find(obj.send_data_comm == 1)) == 1
                % not a DTE and only one comm
                % if attitude pointing needed for this comm
                obj.desired_SC_attitude_mode = 5;
                
            else
                % Science can be performed
                vis_sun_earth = mission_true_SC.true_SC_telescope.visibility_sun_earth;
                vis_target_sun = mission_true_SC.true_SC_telescope.visibility_target_sun;
                
                if vis_sun_earth && ~vis_target_sun
                    % Maximize SP power if sun is visible and target is not
                    obj.desired_SC_attitude_mode = 2;
                else
                    if ~obj.send_data_to_Earth
                        obj.desired_SC_attitude_mode = 6;
                    else
                        obj.desired_SC_attitude_mode = 7;
                    end
                end
            end
            
            % Science
            if ismember(obj.desired_SC_attitude_mode, [6,7])
                obj.science_mode_enabled = 1;
            else
                obj.science_mode_enabled = 0;
            end
            
            % Check for RWA saturation
            obj.desat_procedure = 0;
            
            if mission_true_SC.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
                for i = 1:mission_true_SC.true_SC_rwa_actuator.num_reaction_wheel
                    RW_data_i = mission_true_SC.true_SC_rwa_actuator.RW_data(i);
                    if abs(RW_data_i.true_angular_velocity) >= obj.RW_desat_threshold * RW_data_i.maximum_angular_velocity
                        obj.desat_procedure = 1;
                        disp("RWA DESATURATION PROCEDURE")
                    end
                end
            end
            
            if obj.desired_SC_attitude_mode ~= desired_SC_attitude_mode_prev
                disp(['Attitude mode changed from ', num2str(desired_SC_attitude_mode_prev), ' to ', num2str(obj.desired_SC_attitude_mode)])
            end
            
        end
        
        
        function obj = func_update_timelines(obj,SC_estimate_orbits,True_SC_data_volume,True_SC_power,SC_control_orbit)
            
            % # 1. Trajectory progress %
            % # This is a cumulative rate timeline that represents, as a percentage, how far along the approach trajectory the s/c is: [0,100].
            % # 0 and 100 corresp
            % trajectory_progress_timeline = CumulativeRateTimeline(
            %                                 key = 'trajectory_progress',
            %                                 id = 1,
            %                                 init_value = 0.0,
            %                                 data_type=DataType.FLOAT_64,
            %                                 valid_range=MinMaxRange(0.0,100.0))
            % tasknet.add_timeline(trajectory_progress_timeline)
            
            this_distance_SB = norm(SC_estimate_orbits.SC_position - SC_estimate_orbits.SB_position); % [km]
            
            if this_distance_SB > obj.trajectory_max_distance_SB
                obj.trajectory_max_distance_SB = this_distance_SB;
            end
            
            obj.trajectory_progress_timeline = 100*(1 - (this_distance_SB/obj.trajectory_max_distance_SB)); % [percentage]
            
            % # 4. Memory (Data) %
            % # This is a cumulative rate timeline that represents, as a percentage, how full the memory is with data: [0,100].
            % # 0 and 100 correspond respectively to empty or completely full.
            % # MAYBE it should not be a "rate" timeline? Worry about this later.
            % memory_timeline = CumulativeRateTimeline(
            %                                 key = 'memory',
            %                                 id = 4,
            %                                 init_value = 0.0,
            %                                 data_type=DataType.FLOAT_64,
            %                                 valid_range=MinMaxRange(0.0,100.0))
            % tasknet.add_timeline(memory_timeline)
            
            obj.memory_timeline = 100*(True_SC_data_volume.data_storage/True_SC_data_volume.data_storage_maximum); % [percentage]
            
            
            % # 5. Battery SOC %
            % # This is a cumulative rate timeline that represents, as a percentage, how charged the battery is: [0,100].
            % # 0 and 100 correspond respectively to empty or fully charged.
            % battery_soc_timeline = CumulativeRateTimeline(
            %                                 key = 'battery_soc',
            %                                 id = 5,
            %                                 init_value = 0.0,
            %                                 data_type=DataType.FLOAT_64,
            %                                 valid_range=MinMaxRange(0.0,100.0))
            % tasknet.add_timeline(battery_soc_timeline)
            
            obj.battery_soc_timeline = True_SC_power.battery_data{1}.state_of_charge; % [percentage]
            % Warning: This is only for Battery 1. There are 2 batteries.
            
            
            % # 7. SC attitude control (control of the S/C attitude)
            % # This is an atomic (exclusive) timeline to make sure that various activities
            % # that require different S/C attitudes are mutually exclusive:
            %
            % sc_attitude_timeline = AtomicTimeline(
            %                                 key = 'SC_attitude_timeline', id = 7)
            % tasknet.add_timeline(sc_attitude_timeline)
            
            obj.sc_attitude_timeline = obj.desired_SC_attitude_mode;
            % 1 = Point camera to SB, 2 = Maximize SP Power, 3 = Point Thurster along DeltaV direction
            
            
            % # 2. TCM - Trajectory Correction Maneuver
            % tcm_atomic_timeline = AtomicTimeline(key = 'tcm_atomic', id = 9)
            % tasknet.add_timeline(tcm_atomic_timeline)
            
            obj.tcm_atomic_timeline = SC_control_orbit.desired_DeltaV_needs_to_be_executed;
            
            
            % # 3. SBP - Small Body measurement (imaging & computation) while Sun Pointed
            % sbp_atomic_timeline = AtomicTimeline(key = 'sbp_atomic', id = 10)
            % tasknet.add_timeline(sbp_atomic_timeline)
            
            if obj.desired_SC_attitude_mode == 1
                obj.sbp_atomic_timeline = 1;
            else
                obj.sbp_atomic_timeline = 0;
            end
            
            
            % # 4. Shunt - dump excess power into a radiator during SBP
            % shunt_atomic_timeline = AtomicTimeline(key = 'shunt_atomic', id = 11)
            % tasknet.add_timeline(shunt_atomic_timeline)
            
            if True_SC_power.energy_unused > 0
                obj.shunt_atomic_timeline = 1;
            else
                obj.shunt_atomic_timeline = 0;
            end
            
        end
        
    end
end

