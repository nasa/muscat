classdef True_SC_Body < handle
    %TRUE_SC_BODY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name % [string] = ‘SC j’ for jth SC
        time % [sec] : Current true time
        time_step % [sec] : Time step
        date % [sec from J2000] : Current true date
        mass % [kg] : Mass of the SC
        %             – dry_mass % [kg] : Dry mass of the SC, that doesn’t change position/attitude
        %             – propellant_mass % [kg] : Propellant mass of the SC, that does change value
        %             – supplement_mass % [kg] : Positive/negative mass that is added/removed from the SC. (e.g. sample collection or projectile)
        %             – # solar_panel_mass % [kg] : Total solar panel mass of the SC, that doesn’t change value
        %             – # total_mass % [kg] : Total mass of the SC
        %             – location_propellant_mass % [m] : Location of the propellant mass of the SC, in body
        %             frame B
        %             – location_supplement_mass % [m] : Location of the positive/negative supplement mass of the SC, in body frame B
        
        shape_model % : Cell of SC shape models
        %             – Vertices % [m] : Position of vertices in body frame B
        %             – Faces : Triplet of vertex indices define a face
        %             – reflectance_factor % in [0, 1] for ith face (used for SRP)
        %             – orientation % [unit vector] normal out vector (used
        %               for SRP)
        
        location_center_of_mass % [m] : Compute CM from above data
        
        flag_hardware_exists % [Boolean] : Data structure that denotes if a hardware exists on a SC or not (1 = HW exists, else = It doesn t exist)
        %         – adc_sun_sensor [Boolean]
        %         – adc_star_tracker [Boolean]
        %         – adc_imu [Boolean]
        %         – adc_micro_thruster [Boolean]
        %         – adc_reaction_wheel_assembly [Boolean]
        %         – navigation_camera [Boolean]
        %         – navigation_chemical_thruster [Boolean]
        %         – navigation_ep_thruster [Boolean]
        %         – power_solar_panel [Boolean]
        %         – power_battery [Boolean]
        %         – power_pdc [Boolean]
        
        volume_total % [m^3] : Total volume of the SC
    end
    
    methods
        function obj = True_SC_Body(mission_true_time,i_SC,sc_body_init_data)
            %TRUE_SC_BODY Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = ['SC ',num2str(i_SC)];
            obj.time = mission_true_time.time;
            obj.time_step = mission_true_time.time_step;
            obj.date = mission_true_time.date;
            
            % mass
            obj.mass = sc_body_init_data.mass;
            obj.mass.solar_panel_mass = 0; % [kg]
            obj = func_update_total_mass(obj);
            
            % shape model
            if iscell(sc_body_init_data.shape_model)
                obj.shape_model = sc_body_init_data.shape_model;
            elseif isstruct(sc_body_init_data.shape_model)
                warning('Shape model should be a cell of structs with fields (r_CM, I_over_m, volume, etc)');
                shape = sc_body_init_data.shape_model;
                
                % Center of mass
                r_CM = mean(shape.Vertices, 1)'; % [m]
                
                % Inertia matrix (assume cuboid)
                L = max(shape.Vertices(:,1)) - min(shape.Vertices(:,1)); % [m]
                W = max(shape.Vertices(:,2)) - min(shape.Vertices(:,2)); % [m]
                H = max(shape.Vertices(:,3)) - min(shape.Vertices(:,3)); % [m]
                I_over_m = diag([1/12*(W^2+H^2), 1/12*(L^2+H^2), 1/12*(L^2+W^2)]); % [m^2]
                
                % Volume
                volume = L*W*H; % [m^3]
                
                % Update
                sc_body_init_data.shape_model.r_CM = r_CM;
                sc_body_init_data.shape_model.I_over_m = I_over_m;
                sc_body_init_data.shape_model.volume = volume;
                
                obj.shape_model = {sc_body_init_data.shape_model};
            end
            
            % compute face orientation + normal vector + area
            for i_shape = 1:length(obj.shape_model)
                
                shape = obj.shape_model{i_shape};
                Face_center = zeros(size(shape.Faces));
                Face_normal = zeros(size(shape.Faces));
                area = zeros(size(shape.Faces,1),1);
                
                SC_centroid = mean(shape.Vertices);
                
                for i=1:size(shape.Faces,1)
                    % face center : [(V1x+V2x+V3x)/3 ; (V1y+V2y+V3y)/3 : (V1z+V2z+V3z)/3 ]
                    Face_center(i,:) = [
                        (shape.Vertices(shape.Faces(i,1),1)+shape.Vertices(shape.Faces(i,2),1)+shape.Vertices(shape.Faces(i,3),1))/3;
                        (shape.Vertices(shape.Faces(i,1),2)+shape.Vertices(shape.Faces(i,2),2)+shape.Vertices(shape.Faces(i,3),2))/3;
                        (shape.Vertices(shape.Faces(i,1),3)+shape.Vertices(shape.Faces(i,2),3)+shape.Vertices(shape.Faces(i,3),3))/3];
                    % normal vector : cross(V1-V3,V2-V3)
                    normal_vector_unsigned = cross(...
                        shape.Vertices(shape.Faces(i,1),:)-shape.Vertices(shape.Faces(i,3),:), ...
                        shape.Vertices(shape.Faces(i,2),:)-shape.Vertices(shape.Faces(i,3),:));
                    normal_vector_unsigned = normal_vector_unsigned/norm(normal_vector_unsigned);
                    % correct to get normal pointing outward
                    out_vector = (Face_center(i,:)-SC_centroid)/norm(Face_center(i,:)-SC_centroid); % vector_pointing_out : form SC centroid to face centroid
                    if acos(dot(out_vector,normal_vector_unsigned)) > pi/2
                        Face_normal(i,:) = -normal_vector_unsigned;
                    else
                        Face_normal(i,:) = normal_vector_unsigned;
                    end
                    % Face area
                    vertex_index = shape.Faces(i,:); % index of vertices for this face
                    a = norm(shape.Vertices(vertex_index(1),:) - shape.Vertices(vertex_index(2),:));
                    b = norm(shape.Vertices(vertex_index(2),:) - shape.Vertices(vertex_index(3),:));
                    c = norm(shape.Vertices(vertex_index(3),:) - shape.Vertices(vertex_index(1),:));
                    s = (a+b+c)/2;                                          % semi perimeter
                    area(i) = sqrt(s*(s-a)*(s-b)*(s-c));    % Heron formula
                end
                
                obj.shape_model{i_shape}.Face_center = Face_center;
                obj.shape_model{i_shape}.Face_normal = Face_normal;
                obj.shape_model{i_shape}.area = area;
            end
            
            obj.volume_total = sum(cellfun(@(x) x.volume, obj.shape_model));
            obj.flag_hardware_exists = sc_body_init_data.flag_hardware_exists;
            obj = func_update_center_of_mass(obj);
        end
        
        function obj = func_update_total_mass(obj)
            obj.mass.total_mass = obj.mass.dry_mass + obj.mass.propellant_mass + obj.mass.supplement_mass + obj.mass.solar_panel_mass;
        end
        
        function obj = func_update_center_of_mass(obj)
            % CM of Dry Mass
            % Old method: mean of vertices
            % vertices = cell2mat(cellfun(@(x) x.Vertices, obj.shape_model, 'UniformOutput', false));
            % vertices_without_duplicate = unique(vertices,'rows');
            % CM_dry_mass = mean(vertices)';
            
            % New method: volume weighted average of CM of each shape
            CM_dry_mass = sum( ...
                cell2mat(cellfun(@(x) x.volume*x.r_CM', obj.shape_model, 'UniformOutput', false)), ...
                1)' / obj.volume_total;
            
            obj.location_center_of_mass = ( ...
                obj.mass.dry_mass * CM_dry_mass + ...
                obj.mass.propellant_mass * obj.mass.location_propellant_mass + ...
                obj.mass.supplement_mass * obj.mass.location_supplement_mass) / ...
                (obj.mass.dry_mass + obj.mass.propellant_mass + obj.mass.supplement_mass);
        end
        
    end
end

