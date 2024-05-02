classdef True_SC_SRP < handle
    %True_SC_SRP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        disturbance_torque_SRP % [Nm]
        
        disturbance_force_SRP % [Nm]
        
        enable_SRP % [boolean]
        
        num_faces
        face_data
        %         – # reflectance_factor : ∈ [0, 1] for ith face
        %         – # area [m^2]
        %         – # orientation [unit vector] : Normal vector (going away from Center of Mass) in
        %           body frame B
        %         – # location_center_of_pressure [m] : Center of pressure of ith face
        num_solar_panel_faces
        solar_panels_face_data
        %         – # reflectance_factor : ∈ [0, 1] for ith face
        %         – # area [m^2]
        %         – # orientation [unit vector] : Normal vector (going away from Center of Mass) in
        %           body frame B
        %         – # location_center_of_pressure [m] : Center of pressure of ith face
    end
    
    methods
        
        function obj = True_SC_SRP(sc_body_init_data, true_SC_body)
            
            obj.disturbance_torque_SRP = zeros(3,1);
            obj.disturbance_force_SRP = zeros(3,1);
            obj.enable_SRP = sc_body_init_data.enable_SRP;
            
            obj.num_faces = sum(cell2mat(cellfun(@(x) size(x.Faces,1), true_SC_body.shape_model, 'UniformOutput', false)));
            
            i_face_cnt = 1;
            for i_shape = 1:length(true_SC_body.shape_model)
                shape_i = true_SC_body.shape_model{i_shape};
                for i_face = 1:size(shape_i.Faces,1)
                    obj.face_data(i_face_cnt).reflectance_factor = shape_i.reflectance_factor(i_face);
                    obj.face_data(i_face_cnt).area = shape_i.area(i_face);
                    obj.face_data(i_face_cnt).location_center_of_pressure = shape_i.Face_center(i_face,:);
                    obj.face_data(i_face_cnt).orientation = shape_i.Face_normal(i_face,:);
                    i_face_cnt = i_face_cnt + 1;
                end
            end
            
            obj.num_solar_panel_faces = 0;
            for i=1:sc_body_init_data.number_solar_panel
                
                Vertices = sc_body_init_data.solar_panel(i).shape_model.Vertices;
                Faces = sc_body_init_data.solar_panel(i).shape_model.Faces;
                
                n_face=size(Faces,1);
                obj.num_solar_panel_faces = obj.num_solar_panel_faces + 2*n_face;
                for j=1:n_face
                    %reflectance factor
                    obj.solar_panels_face_data((i-1)*2*n_face+j).reflectance_factor = ...
                        sc_body_init_data.solar_panel(i).shape_model.reflectance_factor_solar_cell_side(j); % solar cell side
                    obj.solar_panels_face_data((i-1)*2*n_face+j+n_face).reflectance_factor = ...
                        sc_body_init_data.solar_panel(i).shape_model.reflectance_factor_opposite_side(j);   % opposite side
                    
                    % area
                    vertex_index = Faces(j,:);  % index of vertices for this face
                    a = norm(Vertices(vertex_index(1),:) - Vertices(vertex_index(2),:));
                    b = norm(Vertices(vertex_index(2),:) - Vertices(vertex_index(3),:));
                    c = norm(Vertices(vertex_index(3),:) - Vertices(vertex_index(1),:));
                    s = (a+b+c)/2;  % semi perimeter
                    obj.solar_panels_face_data((i-1)*2*n_face+j).area = sqrt(s*(s-a)*(s-b)*(s-c));          % solar cell side
                    obj.solar_panels_face_data((i-1)*2*n_face+j+n_face).area = sqrt(s*(s-a)*(s-b)*(s-c));   % opposite side
                    
                    % center of pressure
                    obj.solar_panels_face_data((i-1)*2*n_face+j).location_center_of_pressure = [
                        (Vertices(Faces(j,1),1)+Vertices(Faces(j,2),1)+Vertices(Faces(j,3),1))/3;
                        (Vertices(Faces(j,1),2)+Vertices(Faces(j,2),2)+Vertices(Faces(j,3),2))/3;
                        (Vertices(Faces(j,1),3)+Vertices(Faces(j,2),3)+Vertices(Faces(j,3),3))/3]';
                    
                    obj.solar_panels_face_data((i-1)*2*n_face+j+n_face).location_center_of_pressure = ...
                        obj.solar_panels_face_data((i-1)*2*n_face+j).location_center_of_pressure;
                    obj.solar_panels_face_data((i-1)*2*n_face+j).orientation = ...
                        sc_body_init_data.solar_panel(i).shape_model.orientation_solar_cell_side;   % solar cell side
                    obj.solar_panels_face_data((i-1)*2*n_face+j+n_face).orientation = ...
                        -sc_body_init_data.solar_panel(i).shape_model.orientation_solar_cell_side;  % opposite side
                end
            end
        end
        
        function obj = func_update_disturbance_force_torque_SRP(obj,true_SC_body,true_SC_adc,true_SC_navigation,true_solar_system)
            
            % Re-initialize SRP terms
            obj.disturbance_torque_SRP = zeros(3, 1);
            obj.disturbance_force_SRP = zeros(3, 1);
            
            if obj.enable_SRP == 1
                
                for i=1:obj.num_faces+obj.num_solar_panel_faces
                    
                    % Face selection
                    if i <= obj.num_faces
                        % Spacecraft faces
                        face_i = obj.face_data(i).orientation';                          % [SC frame] : orientation
                        faceCP_sc = obj.face_data(i).location_center_of_pressure';       % [SC frame] : face center of pressure
                        face_area = obj.face_data(i).area;                              % area of this face
                        face_reflectance_factor = obj.face_data(i).reflectance_factor;  % reflectance_factor of this face
                    else
                        % Solar panels faces
                        face_i = obj.solar_panels_face_data(i-obj.num_faces).orientation';                          % [SC frame] : orientation
                        faceCP_sc = obj.solar_panels_face_data(i-obj.num_faces).location_center_of_pressure';       % [SC frame] : face center of pressure
                        face_area = obj.solar_panels_face_data(i-obj.num_faces).area;                              % area of this face
                        face_reflectance_factor = obj.solar_panels_face_data(i-obj.num_faces).reflectance_factor;  % reflectance_factor of this face
                    end
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%% Compute incidence
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % sun-Cp_i vector : SR vector from sun to the Cp of the i^th face
                    faceCP_sun = (true_SC_navigation.position - true_solar_system.position_Sun) + true_SC_adc.rotation_matrix_SC * faceCP_sc;    % [SUN frame] : face normal
                    faceCP_sun_normalized = faceCP_sun/norm(faceCP_sun);                            % normalize
                    
                    %face in inertial frame
                    SC_face_normal = true_SC_adc.rotation_matrix_SC * face_i;                               % [inertial frame] : face normal
                    SC_face_normal_normalized = SC_face_normal/norm(SC_face_normal);                % normalize
                    
                    %incidence of solar radiation
                    incidence = real(acosd(dot(SC_face_normal_normalized, faceCP_sun_normalized)));  %incidence angle of sun on face i
                    
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%% Compute torque
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    if ( abs(incidence) < 90 )
                        
                        % Disturbance force: F = (Phi / c) * (AU / r)^2 * A * (1 + R) * cos(incidence)
                        F_SRP_magnitude = ...
                            true_solar_system.solar_constant_AU / true_solar_system.light_speed ...
                            * (true_solar_system.AU_distance / norm(true_SC_navigation.position-true_solar_system.position_Sun))^2 ...
                            * face_area * (1+face_reflectance_factor)*cosd(incidence);   %force amplitude modulated by the incidence
                        
                        F_SRP_vector_J2000 = F_SRP_magnitude*faceCP_sun_normalized;
                        
                        % Disturbance Torque
                        force_on_surface_at_cp = (true_SC_adc.rotation_matrix_SC')* F_SRP_vector_J2000; % [SC frame] : direction of the sun for face i
                        this_face_lever_arm = faceCP_sc - true_SC_body.location_center_of_mass; % [SC frame] : lever arm = Cp of surface -> cm
                        this_face_torque = cross(this_face_lever_arm, force_on_surface_at_cp);  % [SC frame] : torque of face i
                        
                    else
                        F_SRP_vector_J2000 = zeros(3, 1);
                        this_face_torque = zeros(3, 1);
                    end
                    
                    obj.disturbance_torque_SRP = obj.disturbance_torque_SRP + this_face_torque;
                    obj.disturbance_force_SRP = obj.disturbance_force_SRP + F_SRP_vector_J2000;
                    
                end
                
            end
            
        end
    end
end

