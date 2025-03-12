classdef True_Gravity_Gradient < handle
    % True_SC_Gravity_Gradient 
    properties
        disturbance_torque_G2 % [Nm]
        disturbance_force_G2 % [N] Force induced by gravity gradient
        enable_G2 % [boolean]
        main_body % [String] Main body around which we orbit. Has to fit with mission.true_solar_system.all_SS_body_data
        store 
    end
    
    methods
        function obj = True_Gravity_Gradient(init_data, mission, i_SC)

            % Optional parameters
            if isfield(init_data, 'main_body')
                obj.main_body = init_data.main_body;
            else
                obj.main_body = 'Earth';
            end

            obj.disturbance_torque_G2 = zeros(3,1);
            obj.disturbance_force_G2 = zeros(3,1);
            obj.enable_G2 = init_data.enable_G2;

            % Calculate GG before first iteration of ADL
            obj.func_update_disurbance_torque_G2(mission, i_SC);

            % Initialize storage
            obj.store.disturbance_torque_G2 = zeros(mission.storage.num_storage_steps, 3);
            obj.store.disturbance_force_G2 = zeros(mission.storage.num_storage_steps, 3);
            
        end
        
        %% [ ] Methods: Store
        function obj = func_update_true_gravity_gradient_store(obj, mission)
            if mission.storage.flag_store_this_time_step == 1
                obj.store.disturbance_torque_G2(mission.storage.k_storage,:) = obj.disturbance_torque_G2';
                obj.store.disturbance_force_G2(mission.storage.k_storage,:) = obj.disturbance_force_G2';
            end
        end
    

        %% [ ] Methods: Main Disturbance torque
        function obj = func_update_disurbance_torque_G2(obj, mission, i_SC) 
            
            % Reset disturbance torque and force
            obj.disturbance_torque_G2 = zeros(3,1);
            obj.disturbance_force_G2 = zeros(3,1);

            if obj.enable_G2 == 1
                % Gravity direction from SB
                Rc = 1e3*(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target);     %[m]
                Rc_sc = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix') * Rc';         %[m]
                
                % Gravity gradient torque [Nm]
                obj.disturbance_torque_G2 = (3*(mission.true_target{1, 1}.mu * 1e9)/(norm(Rc_sc)^5))*cross(Rc_sc, mission.true_SC{i_SC}.true_SC_body.total_MI *Rc_sc);
                
                mission.true_SC{i_SC}.true_SC_adc.disturbance_torque = mission.true_SC{i_SC}.true_SC_adc.disturbance_torque + obj.disturbance_torque_G2;
                
                % Gravity gradient force [N]
                % Using the standard gravity gradient force equation
                mu = mission.true_target{1, 1}.mu * 1e9; % Convert to m^3/s^2
                R = norm(Rc_sc);
                unit_Rc = Rc_sc/R;
                
                % Calculate gravity gradient force
                obj.disturbance_force_G2 = -mu*mission.true_SC{i_SC}.true_SC_body.total_mass/(R^2) * unit_Rc;
            end
            
            % Update storage after calculation
            obj.func_update_true_gravity_gradient_store(mission);
        end
    end
end