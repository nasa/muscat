%% Class: True_SC_Gravity_Gradient
% Computes Gravity Gradient disturbance torque

classdef True_SC_Gravity_Gradient < handle
    
    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        enable_G2 % [boolean]

        %% [ ] Properties: Variables Computed Internally
        
        disturbance_torque_G2 % [Nm]

        %% [ ] Properties: Storage Variables
        
        store 
    end
    
    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Gravity_Gradient(init_data, mission, i_SC)

            % Optional parameters
            obj.disturbance_torque_G2 = [0 0 0]; % [N]
            obj.enable_G2 = init_data.enable_G2;

            % Calculate GG before first iteration of ADL
            obj.func_main_true_SC_gravity_gradient(mission, i_SC);

            % Initialize storage
            obj.store.disturbance_torque_G2 = zeros(mission.storage.num_storage_steps, length(obj.disturbance_torque_G2));
            
        end
        
        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_gravity_gradient_store(obj, mission)
            if mission.storage.flag_store_this_time_step == 1
                obj.store.disturbance_torque_G2(mission.storage.k_storage,:) = obj.disturbance_torque_G2;                
            end
        end
    

        %% [ ] Methods: Main 
        % Compute GG Disturbance torque

        function obj = func_main_true_SC_gravity_gradient(obj, mission, i_SC) 
            
            % Reset disturbance torque and force
            obj.disturbance_torque_G2 = [0 0 0]; % [N]

            if obj.enable_G2 == 1
                % Gravity direction from SB
                Rc = 1e3*(mission.true_SC{i_SC}.true_SC_navigation.position_relative_target);     %[m]
                Rc_sc = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix') * Rc';         %[m]
                
                % Gravity gradient torque [Nm]
                obj.disturbance_torque_G2 = ( (3*(mission.true_target{mission.true_SC{i_SC}.true_SC_navigation.index_relative_target}.mu * 1e9)/(norm(Rc_sc)^5)) * cross(Rc_sc, mission.true_SC{i_SC}.true_SC_body.total_MI *Rc_sc) )';
                
                % Accumulate disturbances
                mission.true_SC{i_SC}.true_SC_adc.disturbance_torque = mission.true_SC{i_SC}.true_SC_adc.disturbance_torque + obj.disturbance_torque_G2;

            else
                % Don't do anything!
                
            end
            
            % Update storage after calculation
            obj.func_update_true_gravity_gradient_store(mission);

        end
    end
end