%% Class: True_Gravity_Gradient
% Computes Gravity Gradient disturbance torque

classdef True_Gravity_Gradient < handle
    % True_SC_Gravity_Gradient 
    properties
        disturbance_torque_G2 % [Nm]
        enable_G2 % [boolean]
        store 
    end
    
    methods
        function obj = True_Gravity_Gradient(init_data, mission, i_SC)

            % Optional parameters
            obj.disturbance_torque_G2 = zeros(3,1);
            obj.enable_G2 = init_data.enable_G2;

            % Calculate GG before first iteration of ADL
            obj.func_update_disurbance_torque_G2(mission, i_SC);

            % Initialize storage
            obj.store.disturbance_torque_G2 = zeros(mission.storage.num_storage_steps, 3);
            
        end
        
        %% [ ] Methods: Store
        function obj = func_update_true_gravity_gradient_store(obj, mission)
            if mission.storage.flag_store_this_time_step == 1
                obj.store.disturbance_torque_G2(mission.storage.k_storage,:) = obj.disturbance_torque_G2';                
            end
        end
    

        %% [ ] Methods: Main Disturbance torque
        function obj = func_update_disurbance_torque_G2(obj, mission, i_SC) 
            
            % Reset disturbance torque and force
            obj.disturbance_torque_G2 = zeros(3,1);

            if obj.enable_G2 == 1
                % Gravity direction from SB
                Rc = 1e3*(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target);     %[m]
                Rc_sc = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix') * Rc';         %[m]
                
                % Gravity gradient torque [Nm]
                obj.disturbance_torque_G2 = (3*(mission.true_target{mission.true_SC{i_SC}.true_SC_navigation.index_relative_target}.mu * 1e9)/(norm(Rc_sc)^5))*cross(Rc_sc, mission.true_SC{i_SC}.true_SC_body.total_MI *Rc_sc);
                
                mission.true_SC{i_SC}.true_SC_adc.disturbance_torque = mission.true_SC{i_SC}.true_SC_adc.disturbance_torque + obj.disturbance_torque_G2;
                
            end
            
            % Update storage after calculation
            obj.func_update_true_gravity_gradient_store(mission);
        end
    end
end