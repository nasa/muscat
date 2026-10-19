%% Class: True_SC_SRP
% Tracks the Solar Radiation Pressure (SRP) effects on spacecraft

classdef True_SC_SRP < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        enable_SRP % [boolean] Enable/disable SRP calculations

        %% [ ] Properties: Variables Computed Internally

        disturbance_torque_SRP_body % [Nm] Torque induced by solar radiation pressure in Body frame
        disturbance_force_SRP_inertial % [N] Force induced by solar radiation pressure in J2000

        num_faces % Number of spacecraft body faces
        face_data % Data for spacecraft body faces
        % - reflectance_factor : ∈ [0, 1] for ith face
        % - area [m^2] : Area of face
        % - orientation [unit vector] : Normal vector in body frame B
        % - location_center_of_pressure [m] : Center of pressure

        num_solar_panel_faces % Number of solar panel faces
        solar_panels_face_data % Data for solar panel faces
        % - reflectance_factor : ∈ [0, 1] for ith face
        % - area [m^2] : Area of face
        % - orientation [unit vector] : Normal vector in body frame B
        % - location_center_of_pressure [m] : Center of pressure

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_SRP(init_data, mission, i_SC)

            % Initialize SRP enable flag
            obj.enable_SRP = init_data.enable_SRP;

            % Initialize disturbance vectors
            obj.disturbance_torque_SRP_body = zeros(1,3);
            obj.disturbance_force_SRP_inertial = zeros(1,3);

            % Process spacecraft body faces
            obj.num_faces = 0;
            for i_shape = 1:length(mission.true_SC{i_SC}.true_SC_body.shape_model)
                shape_i = mission.true_SC{i_SC}.true_SC_body.shape_model{i_shape};
                obj.num_faces = obj.num_faces + size(shape_i.Faces,1);

                for i_face = 1:size(shape_i.Faces,1)
                    i_face_cnt = i_face + (i_shape-1)*size(shape_i.Faces,1);
                    obj.face_data(i_face_cnt).reflectance_factor = shape_i.Face_reflectance_factor(i_face);
                    obj.face_data(i_face_cnt).area = shape_i.Face_area(i_face);
                    obj.face_data(i_face_cnt).orientation = shape_i.Face_normal(i_face,:);
                    obj.face_data(i_face_cnt).location_center_of_pressure = shape_i.Face_center(i_face,:);
                end
            end

            % Process solar panel faces
            obj.num_solar_panel_faces = 0;
            if isfield(mission.true_SC{i_SC}, 'true_SC_solar_panel')
                for i_SP = 1:length(mission.true_SC{i_SC}.true_SC_solar_panel)
                    SP = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP};
                    n_face = size(SP.shape_model.Faces,1);
                    obj.num_solar_panel_faces = obj.num_solar_panel_faces + 2*n_face; % Both sides

                    for j = 1:n_face
                        % Solar cell side
                        idx = (i_SP-1)*2*n_face + j;
                        obj.solar_panels_face_data(idx).reflectance_factor = SP.shape_model.Face_reflectance_factor_solar_cell_side(j);
                        obj.solar_panels_face_data(idx).area = SP.shape_model.Face_area(j);
                        obj.solar_panels_face_data(idx).orientation = SP.shape_model.Face_orientation_solar_cell_side;
                        obj.solar_panels_face_data(idx).location_center_of_pressure = SP.shape_model.Face_center(j,:);

                        % Opposite side
                        idx = (i_SP-1)*2*n_face + j + n_face;
                        obj.solar_panels_face_data(idx).reflectance_factor = SP.shape_model.Face_reflectance_factor_opposite_side(j);
                        obj.solar_panels_face_data(idx).area = SP.shape_model.Face_area(j);
                        obj.solar_panels_face_data(idx).orientation = -SP.shape_model.Face_orientation_solar_cell_side;
                        obj.solar_panels_face_data(idx).location_center_of_pressure = SP.shape_model.Face_center(j,:);
                    end
                end
            end

            % Calculate SRP before first iteration of ADL
            % [BWT] Commented this out because it causes the SRP to be
            % double-counted in the first time step
            % obj.func_main_true_SC_SRP(mission, i_SC);

            % Initialize storage
            obj.store.disturbance_torque_SRP_body = zeros(mission.storage.num_storage_steps, length(obj.disturbance_torque_SRP_body));
            obj.store.disturbance_force_SRP = zeros(mission.storage.num_storage_steps, length(obj.disturbance_force_SRP_inertial));

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_SRP_store(obj, mission)
            if mission.storage.flag_store_this_time_step == 1
                obj.store.disturbance_torque_SRP_body(mission.storage.k_storage,:) = obj.disturbance_torque_SRP_body;
                obj.store.disturbance_force_SRP(mission.storage.k_storage,:) = obj.disturbance_force_SRP_inertial;
            end
        end

        %% [ ] Methods: Main
        % Update SRP Disturbance Force and Torque

        function obj = func_main_true_SC_SRP(obj, mission, i_SC)

            % Reset disturbance terms
            obj.disturbance_torque_SRP_body = [0 0 0];
            obj.disturbance_force_SRP_inertial = [0 0 0];

            if obj.enable_SRP == 1

                % Process all faces (spacecraft body + solar panels)
                for i = 1:obj.num_faces + obj.num_solar_panel_faces
                    
                    % Get face data
                    if i <= obj.num_faces
                        face_i_body = obj.face_data(i).orientation';
                        faceCP_sc_body = obj.face_data(i).location_center_of_pressure';
                        face_area = obj.face_data(i).area;
                        face_reflectance_factor = obj.face_data(i).reflectance_factor;
                    else
                        face_i_body = obj.solar_panels_face_data(i-obj.num_faces).orientation';
                        faceCP_sc_body = obj.solar_panels_face_data(i-obj.num_faces).location_center_of_pressure';
                        face_area = obj.solar_panels_face_data(i-obj.num_faces).area;
                        face_reflectance_factor = obj.solar_panels_face_data(i-obj.num_faces).reflectance_factor;
                    end

                    % Calculate face-to-sun vector and incidence angle
                    faceCP_sun_inertial = -((mission.true_SC{i_SC}.true_SC_orbit_propagation.position - ...
                        mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position)' + ...
                        mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * faceCP_sc_body);

                    faceCP_sun_normalized_inertial = faceCP_sun_inertial/norm(faceCP_sun_inertial);

                    SC_face_normal_inertial = mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * face_i_body;
                    SC_face_normal_normalized_inertial = SC_face_normal_inertial/norm(SC_face_normal_inertial);

                    % Calculate incidence angle
                    incidence = real(acosd(dot(SC_face_normal_normalized_inertial, faceCP_sun_normalized_inertial)));


                    % Calculate force and torque if face is illuminated
                    if abs(incidence) < 90
                        % Force calculation
                        F_SRP_magnitude = (mission.true_solar_system.solar_constant_AU / mission.true_solar_system.light_speed) * ...
                            (mission.true_solar_system.AU_distance / norm(mission.true_SC{i_SC}.true_SC_orbit_propagation.position - ...
                            mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position))^2 * ...
                            face_area * (1 + face_reflectance_factor) * cosd(incidence);

                        % Force acts opposite the face-to-sun vector
                        F_SRP_vector_J2000 = F_SRP_magnitude * -faceCP_sun_normalized_inertial;

                        % Torque calculation
                        force_on_surface_at_cp_body = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix)' * F_SRP_vector_J2000;
                        this_face_lever_arm_body = faceCP_sc_body - mission.true_SC{i_SC}.true_SC_body.location_COM';
                        this_face_torque_body = cross(this_face_lever_arm_body, force_on_surface_at_cp_body);

                        % Accumulate disturbances
                        obj.disturbance_torque_SRP_body = obj.disturbance_torque_SRP_body + this_face_torque_body';

                        obj.disturbance_force_SRP_inertial = obj.disturbance_force_SRP_inertial + F_SRP_vector_J2000';
                    end
                end

                % Update disturbance Force and Torque
                mission.true_SC{i_SC}.true_SC_orbit_propagation.disturbance_force = mission.true_SC{i_SC}.true_SC_orbit_propagation.disturbance_force + obj.disturbance_force_SRP_inertial;
                mission.true_SC{i_SC}.true_SC_adc.disturbance_torque = mission.true_SC{i_SC}.true_SC_adc.disturbance_torque + obj.disturbance_torque_SRP_body;

            else
                % Don't do anything!
            end



            % Update storage
            obj = func_update_true_SC_SRP_store(obj, mission);
        end
    end
end