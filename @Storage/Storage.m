classdef Storage < handle
    %STORAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        true_time
        % - counter
        % - time_step
        % - prev_time
        % - num_storage_steps
        % - time_array
        
        flag_must_store_data_for_this_time_step
        
        plot_parameters
        % - color_array
        % - marker_array
        % - standard_font_size
        % - title_font_size
        % - storage_save_plots
        
        true_small_body
        % - position_array
        % - velocity_array
        
        true_solar_system
        % - position_array_Sun
        % - velocity_array_Sun
        % - position_array_Earth
        % - velocity_array_Earth
        
        true_SC
        % - position_array X num_SC
        % - velocity_array X num_SC
        % - SC_Quaternion
        % - SC_Estimate_Omega_Quaternion
        % - SC_Angular_Velocity
        % - SC_Estimate_Omega_Quaternion_Uncertainty
        % - SC_Desired_Omega_Quaternion
        % - SC_Control_Torque
        % - SC_Desired_Control_Torque
        % - True_SC_RWA_Actuator
        % - SC_Data_Volume
        % - integrator_total_data_generated
        % - SC_Power_Generated_Consumed_Energy_Unused
        
        target_data
    end
    
    methods
        function obj = Storage()
            %STORAGE Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.true_time = [];
            obj.true_time.counter = 0;
            obj.true_time.counter_dynamics = 0;
            obj.true_time.time_step = 0.1; % [sec]
            obj.true_time.time_step_dynamics = 10; % [sec]
            obj.true_time.prev_time = -inf;
            obj.true_time.prev_time_dynamics = -inf;
            obj.true_time.num_storage_steps = 0;
            obj.true_time.num_storage_steps_dynamics = 0;
            
            obj.flag_must_store_data_for_this_time_step = 0;
            
            obj.plot_parameters = [];
            obj.plot_parameters.color_array = ['c' 'r' 'b' 'g' 'y' 'm' 'k'];
            obj.plot_parameters.marker_array = ['o' 's' 'd' '^' 'v' '>' '<' 'p' 'h' '+'];
            obj.plot_parameters.standard_font_size = 13;
            obj.plot_parameters.standard_font_type = 'Times New Roman';
            obj.plot_parameters.title_font_size = 40;
            obj.plot_parameters.plot_SC_number = 1;
            
            obj.plot_parameters.storage_save_plots = 0; % 1: Save them (takes little time), 0: Doesnt save them
            obj.plot_parameters.storage_save_mission_video = 1;
        end
        
        function func_create_storage_data_structures(obj,mission_true_time,mission_init_data,mission_true_SC)
            
            if obj.true_time.time_step < mission_true_time.time_step
                obj.true_time.time_step = mission_true_time.time_step;
            end
            % if obj.true_time.time_step_dynamics < mission_true_time.time_step_dynamics
            %     obj.true_time.time_step_dynamics = mission_true_time.time_step_dynamics;
            % end
            obj.true_time.time_step_dynamics = obj.true_time.time_step; % Same dynamics time step
            
            
            obj.true_time.num_storage_steps = ceil((mission_true_time.t_final - mission_true_time.t_initial)/(obj.true_time.time_step));
            obj.true_time.num_storage_steps_dynamics = ceil((mission_true_time.t_final - mission_true_time.t_initial)/(obj.true_time.time_step_dynamics));
            
            % Number of storage steps
            N = obj.true_time.num_storage_steps;
            N_dyn = obj.true_time.num_storage_steps_dynamics;
            
            obj.true_time.time_array = zeros(N,1);
            obj.true_time.time_array_dynamics = zeros(N_dyn,1);
            
            obj.true_small_body = [];
            obj.true_small_body.position_array = zeros(N_dyn,3);
            obj.true_small_body.velocity_array = zeros(N_dyn,3);
            
            obj.true_solar_system = [];
            obj.true_solar_system.position_array_Sun = zeros(N_dyn,3);
            obj.true_solar_system.velocity_array_Sun = zeros(N_dyn,3);
            obj.true_solar_system.position_array_Earth = zeros(N_dyn,3);
            obj.true_solar_system.velocity_array_Earth = zeros(N_dyn,3);
            
            
            obj.true_SC = [];
            for i_SC = 1:1:mission_init_data.num_SC
                mission_true_SC{i_SC} = mission_true_SC{i_SC};
                
                obj.true_SC{i_SC} = [];
                
                % Dynamics
                obj.true_SC{i_SC}.position_array = zeros(N_dyn,3);
                obj.true_SC{i_SC}.velocity_array = zeros(N_dyn,3);
                obj.true_SC{i_SC}.rotation_matrix_SC = zeros(N_dyn,3,3);
                
                obj.true_SC{i_SC}.SC_Estimate_Position_Velocity = zeros(N_dyn,6);
                obj.true_SC{i_SC}.SC_Estimate_Position_Velocity_Uncertainty = zeros(N_dyn,6);
                obj.true_SC{i_SC}.SB_Estimate_Position_Velocity = zeros(N_dyn,6);
                obj.true_SC{i_SC}.SB_Estimate_Position_Velocity_Uncertainty = zeros(N_dyn,6);
                
                % Attitude
                obj.true_SC{i_SC}.SC_Quaternion = zeros(N,4);
                obj.true_SC{i_SC}.SC_Estimate_Omega_Quaternion = zeros(N,7);
                obj.true_SC{i_SC}.SC_Angular_Velocity = zeros(N,3);
                obj.true_SC{i_SC}.SC_Estimate_Omega_Quaternion_Uncertainty = zeros(N,7);
                obj.true_SC{i_SC}.SC_Desired_Omega_Quaternion = zeros(N,7);
                obj.true_SC{i_SC}.SC_Desired_Omega_Quaternion_Control = zeros(N,7);
                
                % SRP
                obj.true_SC{i_SC}.disturbance_torque_SRP = zeros(N,3);
                obj.true_SC{i_SC}.disturbance_force_SRP = zeros(N,3);
                
                % Dynamics
                obj.true_SC{i_SC}.SC_SB_Distance = zeros(N,2);
                obj.true_SC{i_SC}.SC_Estimated_Desired_Intercept_Distance = zeros(N,4);
                obj.true_SC{i_SC}.SC_Desired_Control_DeltaV = zeros(N,3);
                
                % Trajectory control
                obj.true_SC{i_SC}.SC_True_Command_Thrust = zeros(N,2);
                obj.true_SC{i_SC}.integrator_command_thrust = zeros(1,2);
                
                % Attitude control
                obj.true_SC{i_SC}.control_torque = zeros(N,3);
                obj.true_SC{i_SC}.control_torque_MT = zeros(N,3);
                
                obj.true_SC{i_SC}.desired_control_torque = zeros(N,3);
                obj.true_SC{i_SC}.desired_control_torque_MT = zeros(N,3);
                
                obj.true_SC{i_SC}.desired_desat_control_torque = zeros(N,3);
                
                obj.true_SC{i_SC}.actuator_to_use = zeros(N,1);
                obj.true_SC{i_SC}.desired_SC_attitude_mode = zeros(N,1);
                
                if  mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
                    N_RW =  mission_true_SC{i_SC}.true_SC_rwa_actuator.num_reaction_wheel;
                    obj.true_SC{i_SC}.control_torque_RWA = zeros(N,3);
                    obj.true_SC{i_SC}.desired_control_torque_RWA = zeros(N,3);
                    obj.true_SC{i_SC}.rwa_actuator_angular_velocity = zeros(N, N_RW);
                    obj.true_SC{i_SC}.rwa_actuator_angular_acceleration = zeros(N, N_RW);
                end
                
                if (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_altimeter == 1)
                    obj.true_SC{i_SC}.SC_altimeter = zeros(N,3);
                end
                if (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_radar == 1)
                    obj.true_SC{i_SC}.SC_radar_monostatic = zeros(N,1);
                    obj.true_SC{i_SC}.SC_radar_antipodal = zeros(N,length(mission_true_SC{i_SC}.true_SC_radar.antipodal_angle_array));
                    obj.true_SC{i_SC}.SC_radar_bistatic = zeros(N,1);
                    obj.true_SC{i_SC}.radar_interSC_angle = zeros(N,1);
                end
                obj.true_SC{i_SC}.SC_groundtrack = zeros(N,2);
                
                % data
                obj.true_SC{i_SC}.SC_Data_Volume = zeros(N,mission_true_SC{i_SC}.true_SC_onboard_memory.num_memory+1);
                obj.true_SC{i_SC}.integrator_total_data_generated = 0;
                obj.true_SC{i_SC}.integrator_data_sent_DTE = 0;
                obj.true_SC{i_SC}.SC_DTE = zeros(N,4);
                obj.true_SC{i_SC}.SC_communication_data = zeros(N,4*mission_true_SC{i_SC}.software_SC_communication.number_communication+1);
                
                % power
                obj.true_SC{i_SC}.SC_Power_Generated_Consumed_Energy_Unused = zeros(N,3);
                obj.true_SC{i_SC}.integrator_power_generated_consumed_energy_unused = zeros(1,3);
                obj.true_SC{i_SC}.SC_Solar_Panel = zeros(N,2*mission_true_SC{i_SC}.true_SC_solar_panel.num_solar_panels);
                obj.true_SC{i_SC}.SC_Battery = zeros(N,3*mission_true_SC{i_SC}.true_SC_battery.num_battery);
                
                % mass
                obj.true_SC{i_SC}.SC_Propelant_mass = zeros(N,2);
                
                % Science
                obj.true_SC{i_SC}.SC_target_index = zeros(N,1);
                obj.true_SC{i_SC}.SC_target_position = zeros(N,3);
                obj.true_SC{i_SC}.SC_target_obs_time = zeros(N,1);
                obj.true_SC{i_SC}.desired_SC_attitude_mode = zeros(N,1);
                
            end
            
        end
        
        function func_update_storage_data_structures(obj,mission_true_time,mission_true_small_body,mission_true_solar_system,mission_init_data,mission_true_SC)
            
            update = ((1+1e-5)*(mission_true_time.time - obj.true_time.prev_time) >= obj.true_time.time_step) || (obj.flag_must_store_data_for_this_time_step == 1);
            update_dynamics = ((1+1e-5)*(mission_true_time.time - obj.true_time.prev_time_dynamics) >= obj.true_time.time_step_dynamics) || (obj.flag_must_store_data_for_this_time_step == 1);
            
            if update
                % disp('Storing Data...')
                
                obj.true_time.prev_time = mission_true_time.time;
                obj.true_time.counter = obj.true_time.counter + 1;
                k = obj.true_time.counter;
                obj.true_time.time_array(k,1) = mission_true_time.time;
                
                if update_dynamics
                    obj.true_time.prev_time_dynamics = mission_true_time.time;
                    obj.true_time.counter_dynamics = obj.true_time.counter_dynamics + 1;
                    kd = obj.true_time.counter_dynamics;
                    obj.true_time.time_array_dynamics(kd,1) = mission_true_time.time;
                    
                    obj.true_small_body.position_array(kd,:) = mission_true_small_body.position_SB';
                    obj.true_small_body.velocity_array(kd,:) = mission_true_small_body.velocity_SB';
                    obj.true_solar_system.position_array_Sun(kd,:) = mission_true_solar_system.position_Sun';
                    obj.true_solar_system.velocity_array_Sun(kd,:) = mission_true_solar_system.velocity_Sun';
                    obj.true_solar_system.position_array_Earth(kd,:) = mission_true_solar_system.position_Earth';
                    obj.true_solar_system.velocity_array_Earth(kd,:) = mission_true_solar_system.velocity_Earth';
                    obj.true_solar_system.position_array_Moon(kd,:) = mission_true_solar_system.position_Moon';
                    obj.true_solar_system.velocity_array_Moon(kd,:) = mission_true_solar_system.velocity_Moon';
                end
                
                for i_SC = 1:1:mission_init_data.num_SC
                    
                    if ismember(mission_init_data.mission_type, [4,5])
                        obj.true_SC{i_SC}.SC_target_index(k,1) =  mission_true_SC{i_SC}.true_SC_telescope.current_target_idx;
                        obj.true_SC{i_SC}.SC_target_obs_time(k,1) =  mission_true_SC{i_SC}.true_SC_telescope.current_target_obs_time;
                        obj.true_SC{i_SC}.SC_target_position(k,:) =  mission_true_SC{i_SC}.true_SC_telescope.current_target_position;
                        obj.true_SC{i_SC}.desired_SC_attitude_mode(k,1) =  mission_true_SC{i_SC}.software_SC_control_attitude.desired_SC_attitude_mode;
                    end
                    
                    obj.true_SC{i_SC}.integrator_command_thrust = ...
                        obj.true_SC{i_SC}.integrator_command_thrust + mission_true_time.time_step * [
                        sum([mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data.true_commanded_thrust]), ...
                        sum([mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data.commanded_thrust])];
                    
                    obj.true_SC{i_SC}.integrator_total_data_generated = ...
                        obj.true_SC{i_SC}.integrator_total_data_generated ...
                        +  mission_true_SC{i_SC}.true_SC_data_handling.instantaneous_data_generated; % [kb]
                    
                    obj.true_SC{i_SC}.integrator_power_generated_consumed_energy_unused = ...
                        obj.true_SC{i_SC}.integrator_power_generated_consumed_energy_unused + obj.true_time.time_step*[
                        mission_true_SC{i_SC}.true_SC_power.instantaneous_power_generated, ...
                        mission_true_SC{i_SC}.true_SC_power.instantaneous_power_consumed, ...
                        mission_true_SC{i_SC}.true_SC_power.instantaneous_energy_unused];
                    % obj.true_SC{i_SC}.integrator_data_sent_DTE = obj.true_SC{i_SC}.integrator_data_sent_DTE +  mission_true_SC{i_SC}.software_SC_communication.data_sent_DTE; % [kb]
                    
                    % Navigation
                    if update_dynamics
                        obj.true_SC{i_SC}.position_array(kd,:) =  mission_true_SC{i_SC}.true_SC_navigation.position';
                        obj.true_SC{i_SC}.velocity_array(kd,:) =  mission_true_SC{i_SC}.true_SC_navigation.velocity';
                        
                        sw_est_orbit = mission_true_SC{i_SC}.software_SC_estimate_SC_SB_orbit;
                        switch mission_init_data.mission_type
                            case {0,1,2,3}
                                
                                obj.true_SC{i_SC}.SC_Estimate_Position_Velocity(kd,:) = [
                                    sw_est_orbit.SC_position;
                                    sw_est_orbit.SC_velocity]';
                                
                                obj.true_SC{i_SC}.SC_Estimate_Position_Velocity_Uncertainty(kd,:) = [
                                    sw_est_orbit.SC_position_uncertainty;
                                    sw_est_orbit.SC_velocity_uncertainty]';
                                
                                obj.true_SC{i_SC}.SB_Estimate_Position_Velocity_Uncertainty(kd,:) = [
                                    sw_est_orbit.SB_position_uncertainty;
                                    sw_est_orbit.SB_velocity_uncertainty]';
                                
                                obj.true_SC{i_SC}.SB_Estimate_Position_Velocity(kd,:) = [
                                    sw_est_orbit.SB_position;
                                    sw_est_orbit.SB_velocity]';
                                
                                % Interception
                                obj.true_SC{i_SC}.SC_SB_Distance(k,:) = [
                                    norm(mission_true_SC{i_SC}.true_SC_navigation.position - mission_true_small_body.position_SB), ...
                                    norm(sw_est_orbit.SC_position -  sw_est_orbit.SB_position)];
                                obj.true_SC{i_SC}.SC_Estimated_Desired_Intercept_Distance(k,:) = [
                                    mission_true_SC{i_SC}.software_SC_control_orbit.intercept_distance, ...
                                    mission_true_SC{i_SC}.software_SC_control_orbit.desired_intercept_distance, ...
                                    mission_true_SC{i_SC}.software_SC_control_orbit.desired_DeltaV_needs_to_be_executed, ...
                                    mission_true_SC{i_SC}.true_SC_navigation.intercept_distance];
                                
                            case {4,5,6}
                                % idx = 6*(i_SC-1)+1:6*i_SC;
                                % obj.true_SC{i_SC}.SC_Estimate_Position_Velocity(kd,:) = mission_true_SC{1}.software_SC_estimate_SC_SB_orbit.x_est(idx)';
                                % obj.true_SC{i_SC}.SC_Estimate_Position_Velocity_Uncertainty(kd,:) = diag(mission_true_SC{1}.software_SC_estimate_SC_SB_orbit.P_est(idx,idx));
                                obj.true_SC{i_SC}.SC_Estimate_Position_Velocity_relative_SB(kd,:) = [
                                    sw_est_orbit.SC_position_relative_SB;
                                    sw_est_orbit.SC_velocity_relative_SB]';
                                obj.true_SC{i_SC}.SC_Estimate_Position_Velocity_relative_SB_Uncertainty(kd,:) = [
                                    sw_est_orbit.SC_position_relative_SB_uncertainty;
                                    sw_est_orbit.SC_velocity_relative_SB_uncertainty]';
                                obj.true_SC{i_SC}.position_velocity_relative_SB(kd,:) = [
                                    mission_true_SC{i_SC}.true_SC_navigation.position_relative_SB;
                                    mission_true_SC{i_SC}.true_SC_navigation.velocity_relative_SB]';
                                obj.true_SC{i_SC}.rotation_matrix_SC(kd,:,:) =  mission_true_SC{i_SC}.true_SC_adc.rotation_matrix_SC;
                                
                                if i_SC == 1 && ...
                                        isfield(sw_est_orbit, 'relative_position_velocity') && ...
                                        ~isempty(sw_est_orbit.relative_position_velocity)
                                    for j_SC = 2:numel(obj.true_SC)
                                        idx = 6*j_SC-5:6*j_SC;
                                        idx = idx - 6;
                                        obj.true_SC{j_SC}.SC_Estimate_Relative_Position_Velocity(kd,:) = sw_est_orbit.relative_position_velocity(idx);
                                        obj.true_SC{j_SC}.SC_Estimate_Relative_Position_Velocity_Uncertainty(kd,:) = sw_est_orbit.relative_position_velocity_uncertainty(idx);
                                    end
                                end
                                
                            otherwise
                                error('Mission type not implemented')
                        end
                        
                        % Orbit Control
                        obj.true_SC{i_SC}.SC_Desired_Control_DeltaV(k,:) =  mission_true_SC{i_SC}.software_SC_control_orbit.desired_control_DeltaV';
                        obj.true_SC{i_SC}.SC_True_Command_Thrust(k,:) = obj.true_SC{i_SC}.integrator_command_thrust;
                        obj.true_SC{i_SC}.integrator_command_thrust = zeros(1,2);
                    end
                    
                    % Attitude
                    obj.true_SC{i_SC}.SC_Quaternion(k,:) =  mission_true_SC{i_SC}.true_SC_adc.attitude;
                    obj.true_SC{i_SC}.SC_Angular_Velocity(k,:) =  mission_true_SC{i_SC}.true_SC_adc.angular_velocity;
                    obj.true_SC{i_SC}.SC_Estimate_Omega_Quaternion(k,:) = [
                        mission_true_SC{i_SC}.software_SC_estimate_attitude.angular_velocity;
                        mission_true_SC{i_SC}.software_SC_estimate_attitude.attitude]';
                    obj.true_SC{i_SC}.SC_Estimate_Omega_Quaternion_Uncertainty(k,:) = [
                        mission_true_SC{i_SC}.software_SC_estimate_attitude.angular_velocity_uncertainty;
                        mission_true_SC{i_SC}.software_SC_estimate_attitude.attitude_uncertainty]';
                    obj.true_SC{i_SC}.SC_Desired_Omega_Quaternion(k,:) = [
                        mission_true_SC{i_SC}.software_SC_control_attitude.desired_angular_velocity;
                        mission_true_SC{i_SC}.software_SC_control_attitude.desired_attitude]';
                    obj.true_SC{i_SC}.SC_Desired_Omega_Quaternion_Control(k,:) = [
                        mission_true_SC{i_SC}.software_SC_control_attitude.desired_angular_velocity_control;
                        mission_true_SC{i_SC}.software_SC_control_attitude.desired_attitude_control]';
                    
                    
                    % SRP
                    obj.true_SC{i_SC}.disturbance_torque_SRP(k, 1:3) =  mission_true_SC{i_SC}.true_SC_srp.disturbance_torque_SRP';
                    obj.true_SC{i_SC}.disturbance_force_SRP(k, 1:3) =  mission_true_SC{i_SC}.true_SC_srp.disturbance_force_SRP';
                    
                    % Control torque
                    obj.true_SC{i_SC}.control_torque(k,:) =  mission_true_SC{i_SC}.true_SC_adc.control_torque';
                    obj.true_SC{i_SC}.desired_control_torque(k,:) =  mission_true_SC{i_SC}.software_SC_control_attitude.desired_control_torque';
                    
                    %RWA
                    if  mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
                        obj.true_SC{i_SC}.desired_control_torque_RWA(k,:) =  mission_true_SC{i_SC}.software_SC_control_attitude.desired_control_torque_RWA';
                        obj.true_SC{i_SC}.control_torque_RWA(k,:) = zeros(1,3);
                        obj.true_SC{i_SC}.desired_desat_control_torque(k,:) = ...
                            mission_true_SC{i_SC}.true_SC_rwa_actuator.desired_desat_control_torque';
                        
                        for i_rw=1:mission_true_SC{i_SC}.true_SC_rwa_actuator.num_reaction_wheel
                            obj.true_SC{i_SC}.control_torque_RWA(k,:) = obj.true_SC{i_SC}.control_torque_RWA(k,:) ...
                                +  mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(i_rw).true_control_torque_RW';
                            
                            obj.true_SC{i_SC}.rwa_actuator_angular_velocity(k,i_rw) = ...
                                [(mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(i_rw).true_angular_velocity)]';
                            obj.true_SC{i_SC}.rwa_actuator_angular_acceleration(k,i_rw) = ...
                                [(mission_true_SC{i_SC}.true_SC_rwa_actuator.RW_data(i_rw).true_angular_acceleration)]';
                        end
                    end
                    
                    %MT
                    obj.true_SC{i_SC}.desired_control_torque_MT(k,:) =  mission_true_SC{i_SC}.software_SC_control_attitude.desired_control_torque_MT';
                    if  mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
                        obj.true_SC{i_SC}.control_torque_MT(k,:) = zeros(1,3);
                        for i_mt=1:mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.num_micro_thruster
                            obj.true_SC{i_SC}.control_torque_MT(k,:) = obj.true_SC{i_SC}.control_torque_MT(k,:) ...
                                +  mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i_mt).control_torque_MT';
                        end
                        obj.true_SC{i_SC}.SC_Propelant_mass(k,1) = sum([mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data.mass_propelant_used]);
                    end
                    obj.true_SC{i_SC}.actuator_to_use(k) =  mission_true_SC{i_SC}.software_SC_control_attitude.actuator_to_use;
                    obj.true_SC{i_SC}.desired_SC_attitude_mode(k) =  mission_true_SC{i_SC}.software_SC_executive.desired_SC_attitude_mode;
                    
                    %CT
                    if  mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
                        obj.true_SC{i_SC}.SC_Propelant_mass(k,2) = sum([mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data.mass_propelant_used]);
                    end
                    
                    % Data
                    tmp = 4*mission_true_SC{i_SC}.software_SC_communication.number_communication;
                    switch mission_init_data.mission_type
                        case {1,2,3,4}
                            obj.true_SC{i_SC}.SC_communication_data(k,1:tmp) = [
                                [mission_true_SC{i_SC}.true_SC_radio.antenna_data.elevation]';
                                [mission_true_SC{i_SC}.true_SC_radio.antenna_data.Loss_pointing]';
                                mission_true_SC{i_SC}.true_SC_communication.max_data_rate';
                                mission_true_SC{i_SC}.software_SC_communication.total_data_exchanged']';
                        case 5
                    end
                    
                    for i=1:mission_true_SC{i_SC}.software_SC_communication.number_communication
                        if  mission_true_SC{i_SC}.software_SC_executive.send_data_comm(i) ~= 0
                            obj.true_SC{i_SC}.SC_communication_data(k,tmp+1) =  mission_true_SC{i_SC}.software_SC_communication.comm_interlocutor(i);
                            break
                        else
                            obj.true_SC{i_SC}.SC_communication_data(k,tmp+1) = 0;
                        end
                    end
                    
                    if k == 1
                        obj.true_SC{i_SC}.SC_Data_Volume(k,:) = [
                            [mission_true_SC{i_SC}.true_SC_onboard_memory.memory_data.instantaneous_data_storage], ...
                            ((1e-3/8)*obj.true_SC{i_SC}.integrator_total_data_generated)]; % [GB]
                    else
                        obj.true_SC{i_SC}.SC_Data_Volume(k,:) = [
                            [mission_true_SC{i_SC}.true_SC_onboard_memory.memory_data.instantaneous_data_storage], ...
                            ((1e-3/8)*obj.true_SC{i_SC}.integrator_total_data_generated)+obj.true_SC{i_SC}.SC_Data_Volume(k-1,2)]; % [GB]
                    end
                    obj.true_SC{i_SC}.integrator_total_data_generated = 0;
                    
                    % obj.true_SC{i_SC}.SC_DTE(k,:) = [mission_true_SC{i_SC}.software_SC_executive.send_data_to_Earth,  mission_true_SC{i_SC}.software_SC_communication.desired_attitude_for_comm_achieved, obj.true_SC{i_SC}.integrator_data_sent_DTE,  mission_true_SC{i_SC}.software_SC_communication.total_data_sent_DTE];
                    obj.true_SC{i_SC}.integrator_data_sent_DTE = 0;
                    
                    % Power
                    obj.true_SC{i_SC}.SC_Power_Generated_Consumed_Energy_Unused(k,:) = [
                        mission_true_SC{i_SC}.true_SC_power.instantaneous_power_generated, ...
                        mission_true_SC{i_SC}.true_SC_power.instantaneous_power_consumed, ...
                        mission_true_SC{i_SC}.true_SC_power.instantaneous_energy_unused];
                    obj.true_SC{i_SC}.integrator_power_generated_consumed_energy_unused = zeros(1,3);
                    
                    n_sp =  mission_true_SC{i_SC}.true_SC_solar_panel.num_solar_panels;
                    for i=1:1:n_sp
                        obj.true_SC{i_SC}.SC_Solar_Panel(k,i) =  mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(i).instantaneous_power_generated; % [Watts]
                        obj.true_SC{i_SC}.SC_Solar_Panel(k,n_sp+i) =  mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(i).maximum_power; % [Watts]
                    end
                    
                    n_bat =  mission_true_SC{i_SC}.true_SC_battery.num_battery;
                    for i=1:1:n_bat
                        obj.true_SC{i_SC}.SC_Battery(k,i) =  mission_true_SC{i_SC}.true_SC_battery.battery_data(i).instantaneous_capacity; % [Watts * hr]
                        obj.true_SC{i_SC}.SC_Battery(k,n_bat+i) =  mission_true_SC{i_SC}.true_SC_battery.battery_data(i).state_of_charge; % Percentage
                        obj.true_SC{i_SC}.SC_Battery(k,2*n_bat+i) =  mission_true_SC{i_SC}.true_SC_battery.battery_data(i).instantaneous_power_consumed; % [Watts]
                    end
                    
                    
                    % science
                    if (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_altimeter == 1)
                        obj.true_SC{i_SC}.SC_altimeter(k,:) = [mission_true_SC{i_SC}.true_SC_altimeter.measurement_vector(1) ,  mission_true_SC{i_SC}.true_SC_altimeter.num_point_observed,mission_true_SC{i_SC}.true_SC_altimeter.measurement_vector(2)];
                    end
                    if (mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.science_radar == 1)
                        obj.true_SC{i_SC}.SC_radar_monostatic(k,:) = [mission_true_SC{i_SC}.true_SC_radar.monostatic_num_point_observed];
                        obj.true_SC{i_SC}.SC_radar_antipodal(k,:) = [mission_true_SC{i_SC}.true_SC_radar.antipodal_num_point_observed];
                        obj.true_SC{i_SC}.SC_radar_bistatic(k,:) = [mission_true_SC{i_SC}.true_SC_radar.bistatic_num_point_observed];
                    end
                    % store antipodal inter spacecraft angle for 2 and 3
                    % only for droid mission
                    if mission_init_data.mission_type == 3
                        
                        if mission_init_data.num_SC == 3
                            SC2_SB = mission_true_SC{2}.true_SC_navigation.position_relative_SB/norm(mission_true_SC{2}.true_SC_navigation.position_relative_SB);
                            SC3_SB = mission_true_SC{3}.true_SC_navigation.position_relative_SB/norm(mission_true_SC{3}.true_SC_navigation.position_relative_SB);
                            obj.true_SC{i_SC}.radar_interSC_angle(k,:) = real(acosd(dot(SC2_SB,SC3_SB))); % [deg]
                            %                         mission_true_time.time=0;
                        end
                        
                        
                    end
                    
                    if update_dynamics
                        Rotation_matrix = mission_true_small_body.rotation_matrix;
                        %                         Rotation_matrix = func_rotation_matrix(mission_true_small_body.rotation_rate*obj.true_time.time_array(k));
                        r_I_time_k = obj.true_SC{i_SC}.position_array(kd,:)-obj.true_small_body.position_array(kd,:);
                        r_I_time_k = r_I_time_k/norm(r_I_time_k);
                        r_I_time_k = (Rotation_matrix' * r_I_time_k')';
                        [SC_azimuth,SC_elevation,SC_r] = cart2sph(r_I_time_k(1),r_I_time_k(2),r_I_time_k(3));
                        obj.true_SC{i_SC}.SC_groundtrack(kd,:) = [rad2deg(SC_azimuth),rad2deg(SC_elevation)];
                    end
                    
                end % end for i_SC
                
            end % end if update
            
        end % end function
        
    end % end methods
    
end % end classdef


