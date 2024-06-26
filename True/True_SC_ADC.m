classdef True_SC_ADC < handle
    %TRUE_SC_ADC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        time % [sec] : Current true time
        time_step % [sec] : Time step
        prev_time % [sec] : Previous time when position/velocity was computed
        attitude % [quanternion] : Orientation of inertial frame I with respect to the body frame B
        rotation_matrix_SC % : Rotation matrix that converts vector in body frame B to vector in inertial frame I
        angular_velocity % [rad/sec] : Angular velocity of inertial frame I with respect to the body frame B
        dot_angular_velocity % [rad/sec^2] : Time derivative of angular_velocity (needed by RWA)
        
        moment_of_intertia % : Moment of Inertia in body frame B
        %         – dry_mass_MI [kg m^2] : Computed once during initialization, shouldn’t change with
        %         time
        %         – # solar_panel_MI [kg] : Total solar panel MI in body frame B, that does change with
        %         position and attitude of solar panel
        %         – # propellant_mass_MI [kg m^2] : MI of the propellant mass of the SC
        %         – # supplement_mass_MI [kg m^2] : MI of the supplement mass of the SC
        %         – # total_MI [kg m^2] : MI of the total SC
        
        control_torque % [Nm] : Control torque about Center of Mass of SC, generated by MT thrusters and RWA
        
        ode_options % : Options for Matlab%s ODE function odeset( RelTol’,1e-14,’AbsTol’,1e-14)
        
        plot_handle % : plot handle for attitude visualization
        
        % Disturbance
        disturbance_torque
        
    end
    
    methods
        function obj = True_SC_ADC(mission_true_time,sc_body_init_data,true_SC_body)
            %TRUE_SC_ADC Construct an instance of this class
            %   Detailed explanation goes here
            obj.time = mission_true_time.time;
            obj.time_step = mission_true_time.time_step;
            obj.attitude = quatproperize(sc_body_init_data.attitude);
            obj.angular_velocity = sc_body_init_data.angular_velocity;
            obj.dot_angular_velocity = [0 0 0]';
            obj.prev_time = obj.time;
            
            % Compute Rotation Matrix
            obj = func_update_rotation_matrix(obj);
            
            obj.control_torque = [0 0 0]';
            obj.disturbance_torque = zeros(3,1);
            
            
            obj.ode_options = odeset('RelTol',1e-14,'AbsTol',1e-14);
            
            obj.moment_of_intertia = [];
            
            % For backwards compatibility, not used (inertia matrices should always be given)
            if isfield(sc_body_init_data, 'flag_given_inertia_matrix')
                warning('Inertia matrix should always be given for the different shapes')
            end
            obj = func_compute_dry_mass_MI(obj, true_SC_body);
            
        end
        
        function obj = func_update_rotation_matrix(obj)
            
            % Compute Rotation Matrix
            SC_True_e_current = obj.attitude(1:3)/norm(obj.attitude(1:3));
            SC_True_Phi_current = 2*acos(obj.attitude(4)); % [rad]
            obj.rotation_matrix_SC = func_create_Rot_matrix(SC_True_e_current, SC_True_Phi_current);
            
        end
        
        function obj = func_compute_dry_mass_MI(obj, true_SC_body)
            
            % SC CM position
            r_CM_SC = true_SC_body.location_center_of_mass;
            
            % Dry Mass moment of inertia
            obj.moment_of_intertia.dry_mass_MI = zeros(3,3);
            
            % Iterate all the shapes
            for i_shape = 1:length(true_SC_body.shape_model)
                % Retrieve shape
                shape = true_SC_body.shape_model{i_shape};
                
                % Shape mass (proportional to the volume)
                mass_i = true_SC_body.mass.dry_mass * shape.volume / true_SC_body.volume_total;
                
                % Noise (5% of the inertia matrix norm)
                nois_magnitude = 0.00 * norm(shape.I_over_m);
                rand_matrix = rand(3,3);
                I_over_m_noise = nois_magnitude * (rand_matrix + rand_matrix')/2;
                
                % Displacement vector from the shape CM to the SC CM
                r_CM_i = r_CM_SC - shape.r_CM;
                
                % Parallel Axis Theorem: I_cm_sc = I_cm_shape + m * (r' * r * I_3 - r * r')
                I_i = (mass_i * (shape.I_over_m + I_over_m_noise)) + mass_i * (r_CM_i' * r_CM_i * eye(3) - r_CM_i * r_CM_i');
                
                % Add inertia matrix to the total inertia matrix
                obj.moment_of_intertia.dry_mass_MI = obj.moment_of_intertia.dry_mass_MI + I_i;
            end
            
            % Total moment of inertia
            obj.moment_of_intertia.total_MI = obj.moment_of_intertia.dry_mass_MI;
        end
        
        function obj = func_true_attitude_angular_velocity_dynamics(obj,mission_true_time, true_rw_momentum)
            
            obj.time = mission_true_time.time;
            
            X_Quaternion_Omega_current = [obj.attitude; obj.angular_velocity];
            
            obj.time_step = (obj.time - obj.prev_time); % [sec]
            this_time_array = [obj.prev_time, obj.time];
            
            [T,X]=ode45(@(t,X) func_attitude_Quaternion_spacecraft_v2(t,X, ...
                obj.moment_of_intertia.total_MI, obj.control_torque, obj.disturbance_torque, true_rw_momentum), ...
                this_time_array, X_Quaternion_Omega_current, obj.ode_options);
            
            new_X_Quaternion_Omega_current = X(end,:)';
            
            obj.attitude = quatproperize(new_X_Quaternion_Omega_current(1:4)); % [quaternion]
            obj.angular_velocity = new_X_Quaternion_Omega_current(5:7); % [rad/sec]
            
            %retrieve cache
            X_dot = func_attitude_Quaternion_spacecraft_v2(this_time_array(end), new_X_Quaternion_Omega_current, obj.moment_of_intertia.total_MI, obj.control_torque, obj.disturbance_torque, true_rw_momentum);
            obj.dot_angular_velocity = X_dot(1:3);
            
            obj.prev_time = obj.time;
            
            % Compute Rotation Matrix
            obj = func_update_rotation_matrix(obj);
            
        end
        
        function obj = func_update_control_torque(obj, mission_true_SC)
            
            obj.control_torque = [0 0 0]';
            control_torque_MT = [0 0 0]';
            control_torque_RWA = [0 0 0]';
            
            % RWA control torque
            if mission_true_SC.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
                
                for i=1:mission_true_SC.true_SC_rwa_actuator.num_reaction_wheel
                    control_torque_RWA = control_torque_RWA + mission_true_SC.true_SC_rwa_actuator.RW_data(i).true_control_torque_RW; %N.m
                end
                
            end
            
            % MT control torque
            if mission_true_SC.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
                
                for i=1:1:mission_true_SC.true_SC_micro_thruster_actuator.num_micro_thruster
                    
                    if mission_true_SC.true_SC_micro_thruster_actuator.MT_data(i).commanded_thrust <= mission_true_SC.true_SC_micro_thruster_actuator.MT_data(i).minimum_thrust
                        % Do nothing
                    else
                        control_torque_MT = control_torque_MT + mission_true_SC.true_SC_micro_thruster_actuator.MT_data(i).control_torque_MT; % [N m]
                    end
                end
                
            end
            
            obj.control_torque = control_torque_RWA + control_torque_MT;
            
        end
        
        
        
        
        function obj = func_update_disturbance_torque(obj, true_SC_srp, true_SC_gravity_gradient)
            
            obj.disturbance_torque = true_SC_srp.disturbance_torque_SRP + true_SC_gravity_gradient.disturbance_torque_G2;
            
        end
        
        function obj = func_visualize_attitude(obj,storage_data, true_SC_body, true_SC_solar_panel, software_SC_control_attitude,true_SC_micro_thruster_actuator, true_SC_chemical_thruster_actuator,mission_true_time)
            
            mArrow3([0 0 0]',[1 0 0]', 'facealpha', 0.1, 'color', 'r', 'stemWidth', 0.01);
            hold on
            mArrow3([0 0 0]',[0 1 0]', 'facealpha', 0.1, 'color', 'g', 'stemWidth', 0.01);
            mArrow3([0 0 0]',[0 0 1]', 'facealpha', 0.1, 'color', 'b', 'stemWidth', 0.01);
            
            mArrow3([0 0 0]',obj.rotation_matrix_SC*[0.5 0 0]', 'facealpha', 1, 'color', 'r', 'stemWidth', 0.005);
            mArrow3([0 0 0]',obj.rotation_matrix_SC*[0 0.5 0]', 'facealpha', 1, 'color', 'g', 'stemWidth', 0.005);
            mArrow3([0 0 0]',obj.rotation_matrix_SC*[0 0 0.5]', 'facealpha', 1, 'color', 'b', 'stemWidth', 0.005);
            
            % SC body shape
            SC_Shape_Model = [];
            SC_Shape_Model.Vertices = (obj.rotation_matrix_SC * true_SC_body.shape_model.Vertices')';
            SC_Shape_Model.Faces = true_SC_body.shape_model.Faces;
            patch(SC_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'none')
            
            % SP shape
            for i=1:true_SC_solar_panel.num_solar_panels
                SP_Shape_Model = [];
                SP_Shape_Model.Vertices = (obj.rotation_matrix_SC * true_SC_solar_panel.solar_panel_data(i).shape_model.Vertices')';
                SP_Shape_Model.Faces = true_SC_solar_panel.solar_panel_data(i).shape_model.Faces;
                patch(SP_Shape_Model, 'FaceColor','blue', 'EdgeColor', 'none')
            end
            
            % Microthruster
            if true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
                for i=1:true_SC_micro_thruster_actuator.num_micro_thruster
                    loc = obj.rotation_matrix_SC * true_SC_micro_thruster_actuator.MT_data(i).location;
                    dir = obj.rotation_matrix_SC * true_SC_micro_thruster_actuator.MT_data(i).orientation;
                    scale = true_SC_micro_thruster_actuator.MT_data(i).commanded_thrust*0.1/true_SC_micro_thruster_actuator.MT_data(i).maximum_thrust;
                    quiver3(loc(1),loc(2),loc(3) , dir(1),dir(2),dir(3),"LineWidth",3,"DisplayName",['MT ',num2str(i)],"AutoScaleFactor",scale)
                end
            end
            
            % Chemical Thruster
            if true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
                for i=1:true_SC_chemical_thruster_actuator.num_chemical_thruster
                    loc = obj.rotation_matrix_SC * true_SC_chemical_thruster_actuator.chemical_thruster_data(i).location;
                    dir = obj.rotation_matrix_SC * true_SC_chemical_thruster_actuator.chemical_thruster_data(i).orientation;
                    scale = true_SC_chemical_thruster_actuator.chemical_thruster_data(i).commanded_thrust*0.15/true_SC_micro_thruster_actuator.MT_data(i).maximum_thrust;
                    quiver3(loc(1),loc(2),loc(3) , dir(1),dir(2),dir(3),"LineWidth",10,"DisplayName",['CT ',num2str(i)],"AutoScaleFactor",scale)
                end
            end
            
            view([-25,30])
            axis equal
            xlim([-1 1])
            ylim([-1 1])
            zlim([-1 1])
            if software_SC_control_attitude.actuator_to_use == 1
                info_actuator ="Attitude Actuator in use : MT";
            elseif software_SC_control_attitude.actuator_to_use == 2
                info_actuator ="Attitude Actuator in use : RWA";
            elseif software_SC_control_attitude.actuator_to_use == 3
                info_actuator ="Attitude Actuator in use : both (DESAT)";
            else
                % nothing
            end
            if software_SC_control_attitude.desired_SC_attitude_mode == 1
                info_pointing = "Pointing for : SB";
            elseif software_SC_control_attitude.desired_SC_attitude_mode == 2
                info_pointing = "Pointing for : SUN";
            elseif software_SC_control_attitude.desired_SC_attitude_mode == 3
                info_pointing = "Pointing for: DELTA V";
            elseif software_SC_control_attitude.desired_SC_attitude_mode == 4
                info_pointing = "Pointing for: DTE";
            elseif software_SC_control_attitude.desired_SC_attitude_mode == 5
                info_pointing = "Pointing for: INTERSAT comm";
            else
                % nothing
            end
            if sum([true_SC_chemical_thruster_actuator.chemical_thruster_data.command_actuation])>0
                info_thrust = "Thruster firing !";
            else
                info_thrust = "Thruster OFF";
            end
            title ([['Mission Simulation, Time = ',num2str(round(mission_true_time.time)),' sec'],info_actuator,info_pointing,info_thrust])
            
            light
            camlight('headlight')
            grid on
            
            xlabel('X_{SUN}')
            ylabel('Y_{SUN}')
            zlabel('Z_{SUN}')
            
            set(gca, 'fontsize',storage_data.plot_parameters.standard_font_size,'FontName','Times New Roman')
            hold off
            
            drawnow limitrate
            
        end
        
    end
end

