classdef True_SC_Solar_Panel < handle
    %True_SC_Solar_Panel Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        time
        
        num_solar_panels
        
        solar_panel_data
        %             – # name = ‘Solar Panel i’
        %             – health : Health of ith solar panel
        %               0. Switched off
        %               1. Switched on, works nominally
        %             – temperature [deg C] : Temperature of ith solar panel
        %             – instantaneous_power_consumed [Watts] : Instantaneous power consumed by ith solar
        %               panel (irrespective of whether it is generating power or not)
        %             – mass [kg] : Mass of ith solar panel
        %             – shape_model : Shape model of Solar Panel
        %               ∗ Vertices [m] : Position of vertices in body frame B
        %               ∗ Faces : Triplet of vertex indices define a face
        %               ∗ orientation_solar_cell_side [unit vector] : Normal vector in body frame B
        %               ∗ reflectance_factor_solar_cell_side : ∈ [0, 1] for ith face (used for SRP)
        %               ∗ reflectance_factor_opposite_side : ∈ [0, 1] for ith face (used for SRP)
        %               ∗ # area
        %             – type : Solar panel type
        %               1. Stuck to SC side (only solar cell side is used for SRP)
        %               2. Passively deployed (orientation in body frame B does not change, i.e. it is static)
        %               3. Actively gimballed (orientation in body frame B changes)
        %             – packing_fraction : ∈ [0, 1] Packing fraction of solar cells in solar panel
        %             – solar_cell_efficiency : ∈ [0, 1] Efficiency of each solar cell
        %             – # instantaneous_power_generated [Watts] : Instantaneous power produced by ith
        %               solar panel
        %             – # maximum_power [Watts] : Maximum power that could have been produced by ith solar
        %               panel is the Sun was exactly along SP’s orientation
    end
    
    methods
        
        function obj = True_SC_Solar_Panel(sc_body_init_data)
            %TRUE_TIME Construct an instance of this class
            
            obj.num_solar_panels = sc_body_init_data.number_solar_panel;
            
            for i=1:obj.num_solar_panels
                obj.solar_panel_data(i).name = ['SP ',num2str(i)];
                obj.solar_panel_data(i).health = 1;
                obj.solar_panel_data(i).temperature = 10;
                obj.solar_panel_data(i).instantaneous_power_consumption = sc_body_init_data.solar_panel(i).solar_panel_instantaneous_power_consumption;
                obj.solar_panel_data(i).instantaneous_power_consumed = 0;
                obj.solar_panel_data(i).mass = sc_body_init_data.solar_panel(i).solar_panel_mass;
                obj.solar_panel_data(i).shape_model = sc_body_init_data.solar_panel(i).shape_model;
                obj.solar_panel_data(i).type = sc_body_init_data.solar_panel(i).type;
                
                obj.solar_panel_data(i).packing_fraction = sc_body_init_data.solar_panel(i).packing_fraction;
                obj.solar_panel_data(i).solar_cell_efficiency = sc_body_init_data.solar_panel(i).solar_cell_efficiency;
                obj.solar_panel_data(i).instantaneous_power_generated = 0;
                obj.solar_panel_data(i).maximum_power = 0;
                
                area = 0;
                
                Faces = obj.solar_panel_data(i).shape_model.Faces;
                Vertices = obj.solar_panel_data(i).shape_model.Vertices;
                
                % SP area from vertices :
                for f=1:size(Faces,1)
                    obj.solar_panel_data(i).shape_model.Face_center = [
                        (Vertices(Faces(1),1)+Vertices(Faces(2),1)+Vertices(Faces(3),1))/3;
                        (Vertices(Faces(1),2)+Vertices(Faces(2),2)+Vertices(Faces(3),2))/3;
                        (Vertices(Faces(1),3)+Vertices(Faces(2),3)+Vertices(Faces(3),3))/3];
                    
                    vertex_index = Faces(f,:);  % index of vertices for this face
                    a = norm(Vertices(vertex_index(1),:) - Vertices(vertex_index(2),:));
                    b = norm(Vertices(vertex_index(2),:) - Vertices(vertex_index(3),:));
                    c = norm(Vertices(vertex_index(3),:) - Vertices(vertex_index(1),:));
                    s = (a+b+c)/2;  % semi perimeter
                    area = area + sqrt(s*(s-a)*(s-b)*(s-c)); % Heron formula
                end
                
                obj.solar_panel_data(i).shape_model.area = area;
                
            end
        end
        
        function obj = func_update_SP_instantaneous_power(obj, true_SC_adc, true_SC_navigation, true_solar_system)
            
            for i=1:1:obj.num_solar_panels
                
                obj.solar_panel_data(i).instantaneous_power_generated = 0;
                obj.solar_panel_data(i).instantaneous_power_consumed = 0;
                
                if (obj.solar_panel_data(i).health == 1)
                    
                    sp_normal_vector = true_SC_adc.rotation_matrix_SC * obj.solar_panel_data(i).shape_model.orientation_solar_cell_side';
                    sp_normal_vector_normalized = sp_normal_vector/norm(sp_normal_vector);
                    
                    Sun_vector = true_solar_system.position_Sun - true_SC_navigation.position;
                    Sun_vector_normalized = Sun_vector/norm(Sun_vector);
                    
                    Sun_SP_incidence_angle = real(acosd(dot(sp_normal_vector_normalized,Sun_vector_normalized))); % [deg]
                    
                    if Sun_SP_incidence_angle <= 90 % [deg]
                        
                        obj.solar_panel_data(i).instantaneous_power_generated = true_solar_system.solar_constant_AU * (true_solar_system.AU_distance/norm(true_SC_navigation.position  - true_solar_system.position_Sun))^2 ...
                            * obj.solar_panel_data(i).shape_model.area * obj.solar_panel_data(i).packing_fraction * obj.solar_panel_data(i).solar_cell_efficiency * cosd(Sun_SP_incidence_angle); % [Watts]
                        
                    else
                        % No power generated
                        obj.solar_panel_data(i).instantaneous_power_generated = 0; % [Watts]
                        
                    end
                    
                    obj.solar_panel_data(i).maximum_power = true_solar_system.solar_constant_AU * (true_solar_system.AU_distance/norm(true_SC_navigation.position  - true_solar_system.position_Sun))^2 ...
                        * obj.solar_panel_data(i).shape_model.area * obj.solar_panel_data(i).packing_fraction * obj.solar_panel_data(i).solar_cell_efficiency; % [Watts]
                    
                    obj.solar_panel_data(i).instantaneous_power_consumed = obj.solar_panel_data(i).instantaneous_power_consumption;
                end
            end
            
        end
    end
end

