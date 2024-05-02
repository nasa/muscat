classdef Software_SC_Control_Attitude < handle
    %SC_CONTROL_ATTITUDE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % dynamic parameters
        time
        control_torque
        last_desired_control_torque
        desired_control_torque
        desired_control_torque_MT
        desired_control_torque_RWA
        desired_attitude
        desired_attitude_control
        desired_angular_velocity
        desired_angular_velocity_control
        
        % Guidance
        flag_use_guidance
        i_guidance
        n_guidance
        desired_angular_velocity_guidance
        desired_attitude_guidance
        
        % boolean parameters
        compute_desired_attitude
        desired_SC_attitude_mode % 1 = Point camera to SB, 2 = Maximize SP Power
        select_micro_thruster_decomposition % 1 = Optimization (slow), 2 = Adhoc (Fast, but doesnt check resultant forces)
        
        % Controller gain
        control_gain
        control_gain_MT
        control_gain_RWA
        controller
        
        % wait times
        wait_time_compute_desired_attitude
        prev_time_compute_desired_attitude
        
        %Multiple actuator variable
        attitude_error_threshold
        angular_velocity_threshold
        desired_control_torque_magnitude_threshold
        actuator_to_use
        
        minimum_RW_operational
        minimum_MT_operational
        
        last_beta
        prev_time_compute_attitude_SB
        last_sc_position
        last_sb_position
        
        time_step
        max_angular_velocity
    end
    
    methods
        function obj = Software_SC_Control_Attitude(sc_body_init_data, mission_true_time)
            %SC_CONTROL_ATTITUDE Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = -inf;
            obj.prev_time_compute_desired_attitude = -inf;
            obj.desired_attitude = [0 0 0 0]';
            obj.desired_angular_velocity = [0 0 0]';
            obj.desired_SC_attitude_mode = 0;
            
            obj.select_micro_thruster_decomposition = 1;
            
            obj.control_gain_MT = sc_body_init_data.control_gain_MT;
            obj.control_gain_RWA = sc_body_init_data.control_gain_RWA;
            obj.control_gain = obj.control_gain_MT;
            
            obj.desired_control_torque = [0 0 0]'; % [N m]
            obj.desired_control_torque_MT = [0 0 0]'; % [N m]
            obj.desired_control_torque_RWA = [0 0 0]'; % [N m]
            obj.control_torque = [0 0 0]'; % [N m]
            obj.wait_time_compute_desired_attitude = 30; % [sec]
            
            obj.actuator_to_use = 0;
            if isfield(sc_body_init_data, 'attitude_error_threshold')
                obj.attitude_error_threshold = sc_body_init_data.attitude_error_threshold;
            else
                obj.attitude_error_threshold = 0.05;       %[rad] attitude above which we prefer MT instead of RWA
            end
            if isfield(sc_body_init_data, 'angular_velocity_threshold')
                obj.angular_velocity_threshold = sc_body_init_data.angular_velocity_threshold;
            else
                obj.angular_velocity_threshold = 0.003;   %[rad/sec] angular velocity threshold above which we prefere MTinstead of RWA
            end
            obj.desired_control_torque_magnitude_threshold = sc_body_init_data.actuator_selection_threshold_torque; % [N m]
            
            obj.minimum_RW_operational = 3;     % minimum number of RW operational (health ok) to suceed attitude control
            obj.minimum_MT_operational = 10;    % minimum number of MT operational (health ok) to suceed attitude control
            
            obj.last_beta = [0 0 0 0]';
            obj.prev_time_compute_attitude_SB = 0;
            
            obj.time_step = mission_true_time.time_step;
            if isfield(sc_body_init_data, 'max_angular_velocity')
                obj.max_angular_velocity = sc_body_init_data.max_angular_velocity;
            else
                obj.max_angular_velocity = Inf;  %[rad/sec] max spacecraft angular velocity for attitude guidance
            end
            
            if isfield(sc_body_init_data, 'controller')
                obj.controller = sc_body_init_data.controller;
            else
                obj.controller = 'asymptotically_stable';
            end
            
            if isfield(sc_body_init_data, 'flag_use_guidance')
                obj.flag_use_guidance = sc_body_init_data.flag_use_guidance;
            else
                obj.flag_use_guidance = 0;
            end
            
            obj.desired_attitude_guidance =  Inf(1,4);
            obj.desired_angular_velocity_guidance = Inf(1,3);
            
            obj.desired_angular_velocity_control = zeros(3,1);
            obj.desired_attitude_control = zeros(4,1);
        end
        
        function obj = func_start_selected_desired_attitude(obj,SC_executive)
            
            if obj.desired_SC_attitude_mode ~= SC_executive.desired_SC_attitude_mode
                
                obj.compute_desired_attitude = 1;
                obj.desired_SC_attitude_mode = SC_executive.desired_SC_attitude_mode;
                
            elseif (obj.time - obj.prev_time_compute_desired_attitude) >= obj.wait_time_compute_desired_attitude
                
                obj.compute_desired_attitude = 1;
                
            else
                obj.compute_desired_attitude = 0;
                
            end
            
        end
        
        
        function obj = func_compute_desired_attitude_for_SB(obj,SC_estimate_orbits, pointing)
            disp('func_compute_desired_attitude_for_SB')
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            target_vector = (SC_estimate_orbits.SB_position - SC_estimate_orbits.SC_position);    % [J2000]
            target_vector_hat = target_vector/norm(target_vector);
            
            Pointing_vector_hat = pointing'/norm(pointing);
            
            if abs(acos(dot( Pointing_vector_hat, target_vector_hat)) - pi) < 1e-6 

                % flip 
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot(Pointing_vector_hat, target_vector_hat))) < 1e-6
                
                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                
                % Compute rotation to rotate pointing aligned with target   (in J2000)
                v = cross(Pointing_vector_hat, target_vector_hat);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                
                % desired SC frame in J2000 frame to align Pointing with target
                x_desired_hat = Rotm*[1 0 0]';
                y_desired_hat = Rotm*[0 1 0]';
                z_desired_hat = Rotm*[0 0 1]';
                
                cvx_begin quiet
                variable r(3,3)
                minimize ( norm(r) )
                subject to
                
                r * [1 0 0]' == x_desired_hat
                r' * x_desired_hat == [1 0 0]'
                
                r * [0 1 0]' == y_desired_hat
                r' * y_desired_hat == [0 1 0]'
                
                r * [0 0 1]' == z_desired_hat
                r' * z_desired_hat == [0 0 1]'
                
                cvx_end
                
                [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                
                SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                SC_beta_4_desired = cos(SC_Phi_desired/2);
                obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            end

            obj.desired_angular_velocity = [0 0 0]';
            
        end
        
        function obj = func_compute_desired_attitude_for_SB_maximize_SP(obj,SC_estimate_orbits, pointing, SP_pointing)
            % disp('func_compute_desired_attitude_for_SB_maximize_SP')
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            target_vector = (SC_estimate_orbits.SB_position - SC_estimate_orbits.SC_position);    % [J2000]
            target_vector_hat = target_vector/norm(target_vector);
            
            Pointing_vector_hat = pointing'/norm(pointing);
            
            if abs(acos(dot( Pointing_vector_hat, target_vector_hat)) - pi) < 1e-6 

                % flip 
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot(Pointing_vector_hat, target_vector_hat))) < 1e-6
                
                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                
                % Compute rotation to rotate pointing aligned with target   (in J2000)
                v = cross(Pointing_vector_hat, target_vector_hat);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                
                % desired SC frame in J2000 frame to align Pointing with target
                x_desired_hat = Rotm*[1 0 0]';
                y_desired_hat = Rotm*[0 1 0]';
                z_desired_hat = Rotm*[0 0 1]';
                
                % optimize SP power
                theta = [0:2*pi/60:2*pi];
                optimized_error_SP_pointing = zeros(1,length(theta));
                sun_pointing_vector = (SC_estimate_orbits.Sun_position - SC_estimate_orbits.SC_position);
                sun_pointing_vector_hat = sun_pointing_vector/norm(sun_pointing_vector);
                for i=1:length(theta)
                    U = target_vector_hat;
                    R = func_axis_angle_rot(U, theta(i));
                    optimized_error_SP_pointing(i) = func_angle_between_vectors(sun_pointing_vector_hat, R*SP_pointing');
                end
                [m,index_optimized_theta] = min(optimized_error_SP_pointing);
                R_optim = func_axis_angle_rot(U, theta(index_optimized_theta));
                x_desired_hat = R_optim*x_desired_hat;
                y_desired_hat = R_optim*y_desired_hat;
                z_desired_hat = R_optim*z_desired_hat;
                
                %             cvx_begin quiet
                %             variable r(3,3)
                %             minimize ( norm(r) )
                %             subject to
                %
                %             r * [1 0 0]' == x_desired_hat
                %             r' * x_desired_hat == [1 0 0]'
                %
                %             r * [0 1 0]' == y_desired_hat
                %             r' * y_desired_hat == [0 1 0]'
                %
                %             r * [0 0 1]' == z_desired_hat
                %             r' * z_desired_hat == [0 0 1]'
                %
                %             cvx_end
                
                r=[x_desired_hat';y_desired_hat';z_desired_hat']';
                [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                
                SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                SC_beta_4_desired = cos(SC_Phi_desired/2);
                obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            end
            obj.desired_angular_velocity = [0 0 0]';
            
        end
        
        function obj = func_compute_desired_attitude_for_target_SP(obj, target_position, SC_estimate_orbits, ...
                pointing_target, pointing_sun)
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            % Directions (J2000 frame)
            target_vector = (target_position - SC_estimate_orbits.SC_position);
            target_vector = target_vector/norm(target_vector);
            
            sun_vector = (SC_estimate_orbits.Sun_position - SC_estimate_orbits.SC_position);
            sun_vector = sun_vector/norm(sun_vector);
            
            % Pointing (body frame)
            pointing_target = pointing_target/norm(pointing_target);
            pointing_sun = pointing_sun/norm(pointing_sun);
            
            if abs(acos(dot( Pointing_vector_hat, target_vector_hat)) - pi) < 1e-6 

                % flip 
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot(Pointing_vector_hat, target_vector_hat))) < 1e-6
                
                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                    
                % Compute rotation to rotate pointing aligned with target (in J2000)
                v = cross(pointing_target, target_vector);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(pointing_target, target_vector)));
                
                % Desired SC frame in J2000 frame to align Pointing with target
                x_des_target = Rotm*[1 0 0]';
                y_des_target = Rotm*[0 1 0]';
                z_des_target = Rotm*[0 0 1]';
                
                % Optimize SP power
                theta = 0 : 2*pi/60 : 2*pi;
                error_pointing_sun = zeros(1,length(theta));
                
                % Antenna angle
                for i=1:length(theta)
                    U = target_vector;
                    R = func_axis_angle_rot(U, theta(i));
                    error_sun = func_angle_between_vectors(sun_vector, R * Rotm * pointing_sun);
                    error_pointing_sun(i) = error_sun;
                end
                
                [~,idx_best] = min(error_pointing_sun);
                R_optim = func_axis_angle_rot(U, theta(idx_best));
                x_des_target = R_optim*x_des_target;
                y_des_target = R_optim*y_des_target;
                z_des_target = R_optim*z_des_target;
                
                r=[x_des_target'; y_des_target'; z_des_target']';
                obj.desired_attitude = rotationMatrixToQuaternion(r);
            end
            obj.desired_angular_velocity = [0 0 0]';
            
            % disp('Computed desired attitude for target maximize SP')
        end
        
        function obj = func_compute_desired_attitude_for_DTE_target_SP(obj, target_position, SC_estimate_orbits, ...
                pointing_target, pointing_sun, pointing_earth, antenna_gimbal_limit)
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            % Directions (J2000 frame)
            target_vector = (target_position - SC_estimate_orbits.SC_position);
            target_vector = target_vector/norm(target_vector);
            earth_vector = (SC_estimate_orbits.Earth_position - SC_estimate_orbits.SC_position);
            earth_vector = earth_vector/norm(earth_vector);
            sun_vector = (SC_estimate_orbits.Sun_position - SC_estimate_orbits.SC_position);
            sun_vector = sun_vector/norm(sun_vector);
            
            if func_angle_between_vectors(target_vector, earth_vector) < antenna_gimbal_limit
                % Simultaneous target and DTE
                
                % Pointing (body frame)
                pointing_target = pointing_target/norm(pointing_target);
                pointing_sun = pointing_sun/norm(pointing_sun);
                pointing_earth = pointing_earth/norm(pointing_earth);
                    
                     
                if abs(acos(dot( pointing_target, target_vector)) - pi) < 1e-6 
    
                    % flip 
                    obj.desired_attitude = [1; 0; 0; 0];
    
                elseif abs(acos(dot(pointing_target, target_vector))) < 1e-6
                    
                    % do nothing
                    obj.desired_attitude  = [0; 0; 0; 1];
    
                else
                    
    
                    % Compute rotation to rotate pointing aligned with target (in J2000)
                    v = cross(pointing_target, target_vector);
                    Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(pointing_target, target_vector)));
                    
                    % Desired SC frame in J2000 frame to align Pointing with target
                    x_des_target = Rotm*[1 0 0]';
                    y_des_target = Rotm*[0 1 0]';
                    z_des_target = Rotm*[0 0 1]';
                    
                    % Optimize SP power
                    theta = 0 : 2*pi/60 : 2*pi;
                    error_pointing_sun = zeros(1,length(theta));
                    
                    % Antenna angle
                    for i=1:length(theta)
                        U = target_vector;
                        R = func_axis_angle_rot(U, theta(i));
                        angle_sun = func_angle_between_vectors(sun_vector, R * Rotm * pointing_sun);
                        angle_antenna = func_angle_between_vectors(earth_vector, R * Rotm * pointing_earth);
                        if angle_antenna < antenna_gimbal_limit
                            error_pointing_sun(i) = angle_sun;
                        else
                            error_pointing_sun(i) = Inf;
                        end
                    end
                    
                    [angle_sun,idx_best] = min(error_pointing_sun);
                    assert(angle_sun < Inf, 'No solution found for simultaneous target and DTE')
                    
                    R_optim = func_axis_angle_rot(U, theta(idx_best));
                    x_des_target = R_optim*x_des_target;
                    y_des_target = R_optim*y_des_target;
                    z_des_target = R_optim*z_des_target;
                    
                    r=[x_des_target';y_des_target';z_des_target']';
                    obj.desired_attitude = rotationMatrixToQuaternion(r);
                end

                obj.desired_angular_velocity = [0 0 0]';
                
                % disp('Computed desired attitude for DTE, target, SP')
            else
                error('Not implemented yet');
            end
        end
        
        function obj = func_compute_desired_attitude_for_DeltaV(obj,SC_control_orbit, pointing)
            disp('Computing Desired Attitude for DeltaV')
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            target_vector = SC_control_orbit.desired_control_DeltaV;    % [J2000]
            target_vector_hat = target_vector/norm(target_vector);
            
            Pointing_vector_hat = pointing'/norm(pointing);


            if abs(acos(dot( Pointing_vector_hat, target_vector_hat )) - pi) < 1e-6

                % flip
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot( Pointing_vector_hat, target_vector_hat ))) < 1e-6

                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                        
                % Compute rotation to rotate pointing aligned with target   (in J2000)
                v = cross(Pointing_vector_hat, target_vector_hat);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                
                % desired SC frame in J2000 frame to align Pointing with target
                x_desired_hat = Rotm*[1 0 0]';
                y_desired_hat = Rotm*[0 1 0]';
                z_desired_hat = Rotm*[0 0 1]';
                
                cvx_begin quiet
                variable r(3,3)
                minimize ( norm(r) )
                subject to
                
                r * [1 0 0]' == x_desired_hat;
                r' * x_desired_hat == [1 0 0]'
                
                r * [0 1 0]' == y_desired_hat;
                r' * y_desired_hat == [0 1 0]'
                
                r * [0 0 1]' == z_desired_hat;
                r' * z_desired_hat == [0 0 1]'
                
                cvx_end
                
                [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                
                SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                SC_beta_4_desired = cos(SC_Phi_desired/2);
                obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            
            end
            
            obj.desired_angular_velocity = [0 0 0]';
            
        end
        
        function obj = func_compute_desired_attitude_for_DeltaV_maximize_SP(obj,SC_control_orbit, SC_estimate_orbits, pointing,SP_pointing)
            disp('Computing Desired Attitude for DeltaV')
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            target_vector = SC_control_orbit.desired_control_DeltaV;    % [J2000]
            target_vector_hat = target_vector/norm(target_vector);
            
            Pointing_vector_hat = pointing'/norm(pointing);
            
            if abs(acos(dot( Pointing_vector_hat, target_vector_hat )) - pi) < 1e-6

                % flip
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot( Pointing_vector_hat, target_vector_hat ))) < 1e-6

                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                % Compute rotation to rotate pointing aligned with target   (in J2000)
                v = cross(Pointing_vector_hat, target_vector_hat);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                
                % desired SC frame in J2000 frame to align Pointing with target
                x_desired_hat = Rotm*[1 0 0]';
                y_desired_hat = Rotm*[0 1 0]';
                z_desired_hat = Rotm*[0 0 1]';
                
                % optimize SP power
                theta = [0:2*pi/30:2*pi];
                optimized_error_SP_pointing = zeros(1,length(theta));
                sun_pointing_vector = (SC_estimate_orbits.Sun_position - SC_estimate_orbits.SC_position);
                sun_pointing_vector_hat = sun_pointing_vector/norm(sun_pointing_vector);
                for i=1:length(theta)
                    U = target_vector_hat;
                    R = func_axis_angle_rot(U, theta(i));
                    optimized_error_SP_pointing(i) = func_angle_between_vectors(sun_pointing_vector_hat, R*SP_pointing');
                end
                [m,index_optimized_theta] = min(optimized_error_SP_pointing);
                R_optim = func_axis_angle_rot(U, theta(index_optimized_theta));
                x_desired_hat = R_optim*x_desired_hat;
                y_desired_hat = R_optim*y_desired_hat;
                z_desired_hat = R_optim*z_desired_hat;
                
                
                %             cvx_begin quiet
                %             variable r(3,3)
                %             minimize ( norm(r) )
                %             subject to
                %
                %             r * [1 0 0]' == x_desired_hat
                %             r' * x_desired_hat == [1 0 0]'
                %
                %             r * [0 1 0]' == y_desired_hat
                %             r' * y_desired_hat == [0 1 0]'
                %
                %             r * [0 0 1]' == z_desired_hat
                %             r' * z_desired_hat == [0 0 1]'
                %
                %             cvx_end
                
                r = [x_desired_hat';y_desired_hat';z_desired_hat']';
                [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                
                SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                SC_beta_4_desired = cos(SC_Phi_desired/2);
                obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            
            end

            obj.desired_angular_velocity = [0 0 0]';
            
        end
        
        function obj = func_compute_desired_attitude_for_DTE(obj,SC_estimate_orbits, pointing)
            disp('Computing Desired Attitude for DTE')
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            target_vector = SC_estimate_orbits.Earth_position - SC_estimate_orbits.SC_position;    % [J2000]
            target_vector_hat = target_vector/norm(target_vector);
            
            Pointing_vector_hat = pointing'/norm(pointing);
            
            if abs(acos(dot( Pointing_vector_hat, target_vector_hat )) - pi) < 1e-6

                % flip
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot( Pointing_vector_hat, target_vector_hat ))) < 1e-6

                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                % Compute rotation to rotate pointing aligned with target   (in J2000)
                v = cross(Pointing_vector_hat, target_vector_hat);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                
                % desired SC frame in J2000 frame to align Pointing with target
                x_desired_hat = Rotm*[1 0 0]';
                y_desired_hat = Rotm*[0 1 0]';
                z_desired_hat = Rotm*[0 0 1]';
                
                %             cvx_begin quiet
                %             variable r(3,3)
                %             minimize ( norm(r) )
                %             subject to
                %
                %             r * [1 0 0]' == x_desired_hat
                %             r' * x_desired_hat == [1 0 0]'
                %
                %             r * [0 1 0]' == y_desired_hat
                %             r' * y_desired_hat == [0 1 0]'
                %
                %             r * [0 0 1]' == z_desired_hat
                %             r' * z_desired_hat == [0 0 1]'
                %
                %             cvx_end
                
                r = [x_desired_hat';y_desired_hat';z_desired_hat']';
                [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                
                SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                SC_beta_4_desired = cos(SC_Phi_desired/2);
                obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            end

            obj.desired_angular_velocity = [0 0 0]';
            
        end
        
        function obj = func_compute_desired_attitude_for_intersat_comm(obj,mission_true_SC,this_SC_index)
            
            % Find ID of interlocutor for the active comm
            if sum(mission_true_SC{this_SC_index}.software_SC_executive.send_data_comm) > 1
                % Multiple comm in the same time, not possible
                % Do nothing
            else
                id_interlocutor = mission_true_SC{this_SC_index}.software_SC_communication.comm_interlocutor(find(mission_true_SC{this_SC_index}.software_SC_executive.send_data_comm == 1));
                disp(['SC',num2str(this_SC_index),' point towards SC',num2str(id_interlocutor)])
                
                obj.prev_time_compute_desired_attitude = obj.time;
                
                % Position of talking SC
                position_interlocutor = mission_true_SC{id_interlocutor}.software_SC_estimate_SC_SB_orbit.SC_position;
                
                % pointing of our antenna to be used
                id_comm = find( mission_true_SC{this_SC_index}.software_SC_communication.comm_interlocutor == id_interlocutor );
                pointing = mission_true_SC{this_SC_index}.true_SC_radio.antenna_data( id_comm ).antenna_pointing;
                
                
                target_vector = position_interlocutor - mission_true_SC{this_SC_index}.software_SC_estimate_SC_SB_orbit.SC_position;    % [J2000]
                target_vector_hat = target_vector/norm(target_vector);
                
                Pointing_vector_hat = pointing'/norm(pointing);
                
                if abs(acos(dot( Pointing_vector_hat, target_vector_hat )) - pi) < 1e-6

                    % flip
                    obj.desired_attitude = [1; 0; 0; 0];

                elseif abs(acos(dot( Pointing_vector_hat, target_vector_hat ))) < 1e-6

                    % do nothing
                    obj.desired_attitude  = [0; 0; 0; 1];

                else
    
                    % Compute rotation to rotate pointing aligned with target   (in J2000)
                    v = cross(Pointing_vector_hat, target_vector_hat);
                    Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                    
                    % desired SC frame in J2000 frame to align Pointing with target
                    x_desired_hat = Rotm*[1 0 0]';
                    y_desired_hat = Rotm*[0 1 0]';
                    z_desired_hat = Rotm*[0 0 1]';
                    
                    
                    %                 cvx_begin quiet
                    %                 variable r(3,3)
                    %                 minimize ( norm(r) )
                    %                 subject to
                    %
                    %                 r * [1 0 0]' == x_desired_hat
                    %                 r' * x_desired_hat == [1 0 0]'
                    %
                    %                 r * [0 1 0]' == y_desired_hat
                    %                 r' * y_desired_hat == [0 1 0]'
                    %
                    %                 r * [0 0 1]' == z_desired_hat
                    %                 r' * z_desired_hat == [0 0 1]'
                    %
                    %                 cvx_end
                    
                    r = [x_desired_hat';y_desired_hat';z_desired_hat']';
                    [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                    
                    SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                    SC_beta_4_desired = cos(SC_Phi_desired/2);
                    obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
                    
                end
                
                obj.desired_angular_velocity = [0 0 0]';
            end
            
        end
        
        function obj = func_compute_desired_attitude_for_Solar_Panels_To_Sun(obj,SC_estimate_orbits,pointing)
            % disp('Computing Desired Attitude for SP to Sun')
            
            obj.prev_time_compute_desired_attitude = obj.time;
            
            target_vector = (SC_estimate_orbits.Sun_position - SC_estimate_orbits.SC_position);    % [J2000]
            target_vector_hat = target_vector/norm(target_vector);
            
            Pointing_vector_hat = pointing'/norm(pointing);
            
            if abs(acos(dot( Pointing_vector_hat, target_vector_hat )) - pi) < 1e-6

                % flip
                obj.desired_attitude = [1; 0; 0; 0];

            elseif abs(acos(dot( Pointing_vector_hat, target_vector_hat ))) < 1e-6

                % do nothing
                obj.desired_attitude  = [0; 0; 0; 1];

            else
                
                % Compute rotation to rotate pointing aligned with target   (in J2000)
                v = cross(Pointing_vector_hat, target_vector_hat);
                Rotm = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(Pointing_vector_hat', target_vector_hat)));
                
                % desired SC frame in J2000 frame to align Pointing with target
                x_desired_hat = Rotm*[1 0 0]';
                y_desired_hat = Rotm*[0 1 0]';
                z_desired_hat = Rotm*[0 0 1]';
                
                %             cvx_begin quiet
                %             variable r(3,3)
                %             minimize ( norm(r) )
                %             subject to
                %
                %             r * [1 0 0]' == x_desired_hat
                %             r' * x_desired_hat == [1 0 0]'
                %
                %             r * [0 1 0]' == y_desired_hat
                %             r' * y_desired_hat == [0 1 0]'
                %
                %             r * [0 0 1]' == z_desired_hat
                %             r' * z_desired_hat == [0 0 1]'
                %
                %             cvx_end
                
                r = [x_desired_hat';y_desired_hat';z_desired_hat']';
                [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
                
                SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
                SC_beta_4_desired = cos(SC_Phi_desired/2);
                obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            
            end    
            
            obj.desired_angular_velocity = [0 0 0]';
            
        end
        
        function obj = function_compute_guidance(obj, SC_estimate_attitude)
            % New guidance
            omega_est = SC_estimate_attitude.angular_velocity; % Not used
            omega_f = obj.desired_angular_velocity;
            beta_est = quatproperize(SC_estimate_attitude.attitude);
            beta_f = quatproperize(obj.desired_attitude);
            
            theta_max = obj.max_angular_velocity * obj.time_step;
            delta_beta = quatmultiply(beta_f, quatconj(beta_est));
            vec = delta_beta(1:3) / norm(delta_beta(1:3));
            theta = 2 * acos(delta_beta(4));
            n_steps = max(ceil(theta / theta_max), 1);
            
            % Rotation
            if n_steps > 1
                % theta_d = theta * 0.5 * (sin(linspace(-pi/2, pi/2, n_steps)) + 1);
                theta_d = theta * 0.5 * (2*sin(linspace(-pi/2,0, n_steps)).^3 + 2);
                
                
                beta_d = zeros(n_steps, 4);
                omega_d = zeros(n_steps, 3);
                beta_d(1, :) = beta_est';
                omega_d(1, :) = omega_est';
                for i = 2:n_steps-1
                    % Quaternion
                    delta_beta = [vec * sin(theta_d(i)/2); cos(theta_d(i)/2)];
                    beta_d(i, :) = quatmultiply(delta_beta, beta_est);
                    
                    % Angular velocity
                    beta = beta_d(i-1, :)';
                    Zbeta = zeros(4,3);
                    Zbeta(1:3,1:3) =  beta(4) * eye(3) + skew(beta(1:3));
                    Zbeta(4,  1:3) = -beta(1:3)';
                    omega_d(i, :) = (pinv(0.5 * Zbeta) * (beta_d(i, :)' - beta_d(i-1, :)') / obj.time_step)';
                end
                beta_d(end, :) = beta_f';
                omega_d(end, :) = omega_f';
            else
                beta_d = beta_f';
                omega_d = omega_f';
            end
            
            obj.i_guidance = 1;
            obj.n_guidance = n_steps;
            obj.desired_angular_velocity_guidance = omega_d;
            obj.desired_attitude_guidance = beta_d;
            
            disp('Computed attitude guidance');
        end
        
        function obj = function_compute_desired_control_torque(obj,SC_estimate_attitude,meas_rw_momentum,true_SC_ADC)
            % Guidance
            if ~obj.flag_use_guidance
                obj.desired_angular_velocity_guidance = obj.desired_angular_velocity';
                obj.desired_attitude_guidance = obj.desired_attitude';
                
                obj.desired_angular_velocity_control = obj.desired_angular_velocity;
                obj.desired_attitude_control = obj.desired_attitude;
            else
                if norm(obj.desired_angular_velocity - obj.desired_angular_velocity_guidance(end, :)') > 1e-6 || ...
                        norm(obj.desired_attitude - obj.desired_attitude_guidance(end, :)') > 1e-5
                    obj = function_compute_guidance(obj, SC_estimate_attitude);
                end
                obj.desired_angular_velocity_control = obj.desired_angular_velocity_guidance(obj.i_guidance, :)';
                obj.desired_attitude_control = obj.desired_attitude_guidance(obj.i_guidance, :)';
                if obj.i_guidance < obj.n_guidance
                    obj.i_guidance = obj.i_guidance + 1;
                end
            end
            
            % Control
            switch obj.controller
                case 'PD'
                    obj = function_compute_desired_control_torque_PD(obj,SC_estimate_attitude,meas_rw_momentum,true_SC_ADC);
                case 'asymptotically_stable'
                    obj = function_compute_desired_control_torque_asymptotically_stable(obj,SC_estimate_attitude);
                otherwise
                    error('Unknown controller')
            end
        end
        function obj = function_compute_desired_control_torque_PD(obj,SC_estimate_attitude,meas_rw_momentum,true_SC_ADC)
            % Parameters
            I = true_SC_ADC.moment_of_intertia.total_MI; % [kg m^2] Spacecraft inertia matrix (including wheels)
            J = I; % [kg m^2] Spacecraft inertia matrix (excluding wheels)
            
            Kp = eye(3) * obj.control_gain(1);
            Kd = eye(3) * obj.control_gain(2);
            
            % Estimated and final values
            omega_est = SC_estimate_attitude.angular_velocity;
            omega_d = obj.desired_angular_velocity_control;
            beta_est = quatproperize(SC_estimate_attitude.attitude);
            beta_d = quatproperize(obj.desired_attitude_control);
            
            % Calculate Delta q and Delta theta
            delta_beta = quatmultiply(beta_d, quatconj(beta_est));
            delta_theta = 2 * delta_beta(1:3); % Assuming small angle approximations
            
            % Calculate control torques
            delta_omega = omega_d - omega_est;
            uc = I * (Kd * delta_omega + Kp * delta_theta);
            
            % Gyroscopic terms
            vc = cross(omega_est, (J * omega_est + meas_rw_momentum));
            
            % Total control torque
            tau = uc + vc;
            
            obj.desired_control_torque = tau;
            obj.control_torque = [0 0 0]'; % [N m]
        end
        
        function obj = function_compute_desired_control_torque_asymptotically_stable(obj,SC_estimate_attitude)
            
            omega_est = SC_estimate_attitude.angular_velocity;
            omega_d = obj.desired_angular_velocity_control;
            beta_est = quatproperize(SC_estimate_attitude.attitude);
            beta_d = quatproperize(obj.desired_attitude_control);
            
            Zbeta = zeros(4,3);
            Zbeta(1:3,1:3) =  beta_est(4) * eye(3) + skew(beta_est(1:3));
            Zbeta(4,  1:3) = -beta_est(1:3)';
            
            % output torque command
            omega_r = omega_d + pinv(0.5 * Zbeta) * obj.control_gain(2)*(beta_d - beta_est);
            obj.desired_control_torque = -obj.control_gain(1)*(omega_est - omega_r);
            
            obj.control_torque = [0 0 0]'; % [N m]
        end
        
        function obj = func_actuator_selection(obj,mission_true_SC)
            
            if (mission_true_SC.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1) && (mission_true_SC.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1)
                
                % Decide which actuator to use
                actual_euler = ConvertAttitude(mission_true_SC.software_SC_estimate_attitude.attitude/norm(mission_true_SC.software_SC_estimate_attitude.attitude), 'quaternion','321');
                desired_euler = ConvertAttitude(obj.desired_attitude/norm(obj.desired_attitude), 'quaternion','321');
                error = abs(actual_euler - desired_euler);
                
                if (sum([mission_true_SC.true_SC_rwa_actuator.RW_data.health]) < obj.minimum_RW_operational) && (sum([mission_true_SC.true_SC_micro_thruster_actuator.MT_data.health]) >=obj.minimum_MT_operational)
                    obj.actuator_to_use = 1; % "MT" Forced to use MT
                    obj.desired_control_torque_MT = obj.desired_control_torque;
                    obj.desired_control_torque_RWA = [0 0 0]'; % [N m]
                    
                elseif (sum([mission_true_SC.true_SC_rwa_actuator.RW_data.health]) >= obj.minimum_RW_operational) && (sum([mission_true_SC.true_SC_micro_thruster_actuator.MT_data.health]) < obj.minimum_MT_operational)
                    obj.actuator_to_use = 2; % "RWA" Forced to use RWA
                    obj.desired_control_torque_MT = [0 0 0]'; % [N m]
                    obj.desired_control_torque_RWA = obj.desired_control_torque;
                    
                elseif mission_true_SC.true_SC_rwa_actuator.flag_desat_in_process == 1
                    obj.actuator_to_use = 3; % Desaturation of RWA, Use both
                    obj.desired_control_torque_MT =  obj.desired_control_torque - mission_true_SC.true_SC_rwa_actuator.desired_desat_control_torque;
                    obj.desired_control_torque_RWA = mission_true_SC.true_SC_rwa_actuator.desired_desat_control_torque;
                    
                    %                 elseif (norm(obj.desired_control_torque) >= obj.desired_control_torque_magnitude_threshold) ||(norm(error)>obj.attitude_error_threshold)% || (torque_dot>= 10*obj.desired_control_torque_magnitude_threshold) ||(norm(error)>obj.attitude_error_threshold) || (norm(mission_true_SC.software_SC_estimate_attitude.angular_velocity)>obj.angular_velocity_threshold)
                elseif (norm(error)>obj.attitude_error_threshold) || (norm(mission_true_SC.software_SC_estimate_attitude.angular_velocity)>obj.angular_velocity_threshold)
                    obj.actuator_to_use = 1; % "MT" Switch on MT if attitude correction is too large
                    obj.desired_control_torque_MT = obj.desired_control_torque;
                    obj.desired_control_torque_RWA = [0 0 0]'; % [N m]
                    
                else
                    obj.actuator_to_use = 2; % "RWA" Otherwise use RWA
                    obj.desired_control_torque_MT = [0 0 0]'; % [N m]
                    obj.desired_control_torque_RWA = obj.desired_control_torque;
                end
                
            elseif (mission_true_SC.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1) && (mission_true_SC.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly ~= 1)
                % if only MT available
                obj.actuator_to_use = 1; % "MT" Forced to use MT
                obj.desired_control_torque_MT = obj.desired_control_torque;
                
            elseif (mission_true_SC.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1) && (mission_true_SC.true_SC_body.flag_hardware_exists.adc_micro_thruster ~= 1)
                % if only RWA available
                obj.actuator_to_use = 2; % "RWA" Otherwise use RWA
                obj.desired_control_torque_RWA = obj.desired_control_torque;
            end
            
            % Select control gain tunning according to actuator
            if (obj.actuator_to_use == 1) || (obj.actuator_to_use == 3)
                obj.control_gain = obj.control_gain_MT;
            elseif obj.actuator_to_use == 2
                obj.control_gain = obj.control_gain_RWA;
            else
                % should not reach here
            end
            
        end
        
        function obj = func_compute_desired_attitude_for_SB_V2(obj,SC_estimate_orbits, SC_estimate_attitude)
            disp('func_compute_desired_attitude_for_SB_V2')
            
            % Compute desired attitude
            obj.prev_time_compute_desired_attitude = obj.time;
            
            x_desired = SC_estimate_orbits.SB_position - SC_estimate_orbits.SC_position;
            x_desired_hat = x_desired/norm(x_desired);
            
            y_angle = acosd(dot(x_desired_hat,[1 0 0]'));
            y_desired = [1 0 0]' - cosd(y_angle)*x_desired_hat;
            y_desired_hat = y_desired/norm(y_desired);
            
            z_desired_hat = cross(x_desired_hat,y_desired_hat);
            
            cvx_begin quiet
            variable r(3,3)
            minimize ( norm(r) )
            subject to
            
            r * [1 0 0]' == x_desired_hat
            r' * x_desired_hat == [1 0 0]'
            
            r * [0 1 0]' == y_desired_hat
            r' * y_desired_hat == [0 1 0]'
            
            r * [0 0 1]' == z_desired_hat
            r' * z_desired_hat == [0 0 1]'
            
            cvx_end
            
            [SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);
            
            SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
            SC_beta_4_desired = cos(SC_Phi_desired/2);
            obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];
            
            
            % Compute desired angular velocity
            beta = SC_estimate_attitude.attitude;
            % beta dot
            Sbeta_v = [0 -beta(3) beta(2);
                beta(3) 0 -beta(1);
                -beta(2) beta(1) 0];
            Zbeta = zeros(4,3);
            Zbeta(1:3,1:3) = 0.5*Sbeta_v;
            Zbeta(1,1) = 0.5*beta(4);
            Zbeta(2,2) = 0.5*beta(4);
            Zbeta(3,3) = 0.5*beta(4);
            Zbeta(4,1:3) = -0.5*beta(1:3)';
            
            % angular velocity command
            if obj.last_beta == zeros(1,4)
                % first iteration, just store the current desired attitude
                obj.last_beta = obj.desired_attitude;
                obj.prev_time_compute_attitude_SB = obj.time;
            else
                obj.desired_angular_velocity = pinv(Zbeta)*(obj.desired_attitude-obj.last_beta)/(obj.time-obj.prev_time_compute_attitude_SB);
                obj.last_beta = obj.desired_attitude;
                obj.prev_time_compute_attitude_SB = obj.time;
            end
            
            %             obj.desired_angular_velocity = (-1e-4)*[0.005 0.05 0.45]';
            
        end
        
    end
end
