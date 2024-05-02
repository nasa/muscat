classdef True_SC_Gravity_Gradient < handle
    %True_SC_Gravity_Gradient Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        disturbance_torque_G2 % [Nm]

        enable_G2 % [boolean]
    end
    
    methods

        function obj = True_SC_Gravity_Gradient(sc_body_init_data)
            %TRUE_TIME Construct an instance of this class
            
            obj.disturbance_torque_G2 = zeros(3,1);

            obj.enable_G2 = sc_body_init_data.enable_G2;
        end             

        function obj = func_update_disurbance_torque_G2(obj, true_SC_adc, true_SC_navigation, mission_true_small_body) 

            obj.disturbance_torque_G2 = zeros(3,1);

            if obj.enable_G2 == 1
                % Gravity direction from SB
                Rc = 1e3*(true_SC_navigation.position_relative_SB);     %[m]
                Rc_sc = (true_SC_adc.rotation_matrix_SC') * Rc;         %[m]
                
                obj.disturbance_torque_G2 = (3*(mission_true_small_body.mu_SB * 1e9)/(norm(Rc_sc)^5))*cross( Rc_sc , true_SC_adc.moment_of_intertia.total_MI*Rc_sc );
            end
            
        end
    end
end

