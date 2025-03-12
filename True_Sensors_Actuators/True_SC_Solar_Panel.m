%% Class: True_SC_Solar_Panel
% SC's Solar Panels

classdef True_SC_Solar_Panel < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous power consumed (irrespective of whether it is generating power or not)

        instantaneous_data_rate_generated % [kbps] : Data rate generated during current time step, in kilo bits (kb) per sec

        mass % [kg] : Mass of ith solar panel

        shape_model % : Shape model of Solar Panel
        % - Vertices [m] : Position of vertices in body frame B
        % - Faces : Triplet of vertex indices define a face
        % - Face_reflectance_factor_solar_cell_side : ∈ [0, 1] for ith face (used for SRP)
        % - Face_reflectance_factor_opposite_side : ∈ [0, 1] for ith face (used for SRP)
        % - Face_orientation_solar_cell_side [unit vector] : Normal vector in body frame B
        % - # Face_center [m] : Center of this Face
        % - # Face_area [m^2]
        % - type [string] : Type of shape is used for MI and volume calculations

        type % [string] : Solar panel type
        % 'body_mounted' : Stuck to SC side (only solar cell side is used for SRP)
        % 'passive_deployed' : Passively deployed (orientation in body frame B does not change, i.e. it is static)
        % 'active_deployed_gimballed' : Actively gimballed (orientation in body frame B changes)

        packing_fraction % [float] ∈ [0, 1] Packing fraction of solar cells in solar panel

        solar_cell_efficiency % [float] ∈ [0, 1] Efficiency of each solar cell


        %% [ ] Properties: Variables Computed Internally

        name % [string] 'SP i_SP'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        instantaneous_power_generated % [Watts] : Instantaneous power produced by ith solar panel

        maximum_power % [Watts] : Maximum power that could have been produced by ith solar panel is the Sun was exactly along SP’s orientation
        
        Sun_incidence_angle % [deg] : Angle between Sun vector and Face_orientation_solar_cell_side

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Solar_Panel(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['Solar Panel ',num2str(i_HW)];
            end
            
            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]

            % Initialize Shape
            obj.shape_model = init_data.shape_model;

            Face_center = zeros(size(obj.shape_model.Faces));
            for i=1:size(obj.shape_model.Faces,1)
                % face center : [(V1x+V2x+V3x)/3 ; (V1y+V2y+V3y)/3 : (V1z+V2z+V3z)/3 ]
                Face_center(i,:) = [
                    (obj.shape_model.Vertices(obj.shape_model.Faces(i,1),1) + obj.shape_model.Vertices(obj.shape_model.Faces(i,2),1) + obj.shape_model.Vertices(obj.shape_model.Faces(i,3),1))/3;
                    (obj.shape_model.Vertices(obj.shape_model.Faces(i,1),2) + obj.shape_model.Vertices(obj.shape_model.Faces(i,2),2) + obj.shape_model.Vertices(obj.shape_model.Faces(i,3),2))/3;
                    (obj.shape_model.Vertices(obj.shape_model.Faces(i,1),3) + obj.shape_model.Vertices(obj.shape_model.Faces(i,2),3) + obj.shape_model.Vertices(obj.shape_model.Faces(i,3),3))/3];
            end
            obj.shape_model.Face_center = Face_center;

            % SP area from vertices
            area = 0; % [m^2]
            for f=1:size(obj.shape_model.Faces,1)
                vertex_index = obj.shape_model.Faces(f,:);  % index of vertices for this face
                a = norm(obj.shape_model.Vertices(vertex_index(1),:) - obj.shape_model.Vertices(vertex_index(2),:));
                b = norm(obj.shape_model.Vertices(vertex_index(2),:) - obj.shape_model.Vertices(vertex_index(3),:));
                c = norm(obj.shape_model.Vertices(vertex_index(3),:) - obj.shape_model.Vertices(vertex_index(1),:));
                s = (a+b+c)/2;  % semi perimeter
                area = area + sqrt(s*(s-a)*(s-b)*(s-c)); % Heron formula
            end

            obj.shape_model.Face_area = area;

            % Center of mass
            obj.shape_model.r_CM = mean(obj.shape_model.Vertices, 1); % [m]

            switch obj.shape_model.type

                case 'cuboid'

                    % Inertia matrix (assume cuboid)
                    L = max(obj.shape_model.Vertices(:,1)) - min(obj.shape_model.Vertices(:,1)); % [m]
                    W = max(obj.shape_model.Vertices(:,2)) - min(obj.shape_model.Vertices(:,2)); % [m]
                    H = max(obj.shape_model.Vertices(:,3)) - min(obj.shape_model.Vertices(:,3)); % [m]
                    obj.shape_model.I_through_r_CM = diag([1/12*(W^2+H^2), 1/12*(L^2+H^2), 1/12*(L^2+W^2)]); % [m^2]

                    % Volume
                    obj.shape_model.volume = L*W*H; % [m^3]

                otherwise
                    error('Havent written yet!')

            end

            obj.mass = init_data.mass; % [kg]

            obj.packing_fraction = init_data.packing_fraction;

            obj.type = init_data.type;

            obj.solar_cell_efficiency = init_data.solar_cell_efficiency;

            obj.instantaneous_power_generated = 0; % [W]

            obj.maximum_power = 0; % [W]

            obj.Sun_incidence_angle = 180; % [deg]

            obj = func_update_SP_instantaneous_power_generated(obj, mission, i_SC);

            % Initialize Variables to store: instantaneous_power_generated maximum_power Sun_incidence_angle
            obj.store = [];

            obj.store.instantaneous_power_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_generated));
            obj.store.maximum_power = zeros(mission.storage.num_storage_steps, length(obj.maximum_power));
            obj.store.Sun_incidence_angle = zeros(mission.storage.num_storage_steps, length(obj.Sun_incidence_angle));

            obj = func_update_true_SC_SP_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            func_initialize_list_HW_energy_generated(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_SP_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_power_generated(mission.storage.k_storage,:) = obj.instantaneous_power_generated; % [W]
                obj.store.maximum_power(mission.storage.k_storage,:) = obj.maximum_power; % [W]
                obj.store.Sun_incidence_angle(mission.storage.k_storage,:) = obj.Sun_incidence_angle; % [deg]
            end

        end

        %% [ ] Methods: Main
        % Update SP's instantaneous_power_generated

        function obj = func_main_true_SC_solar_panel(obj, mission, i_SC)

            obj = func_update_SP_instantaneous_power_generated(obj, mission, i_SC);

            obj = func_update_true_SC_SP_store(obj, mission);

            % Update Power Generated Consumed
            func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            func_update_instantaneous_power_generated(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end


        %% [ ] Methods: Instantaneous Power Generated
        % Update SP's instantaneous_power_generated

        function obj = func_update_SP_instantaneous_power_generated(obj, mission, i_SC)

            if (obj.health == 1) && (mission.true_SC{i_SC}.true_SC_navigation.flag_visible_Sun == 1)

                Sun_vector = mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.true_SC_navigation.position; % [km]
                
                Sun_vector_normalized = func_normalize_vec(Sun_vector); % [unit vector]

                obj.Sun_incidence_angle = real(acosd(dot(Sun_vector_normalized', mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.shape_model.Face_orientation_solar_cell_side'))); % [deg]

                obj.maximum_power = mission.true_solar_system.solar_constant_AU * (mission.true_solar_system.AU_distance/norm(Sun_vector))^2 * obj.shape_model.Face_area * obj.packing_fraction * obj.solar_cell_efficiency; % [W]

                if obj.Sun_incidence_angle <= 90 % [deg]

                    obj.instantaneous_power_generated = obj.maximum_power * cosd(obj.Sun_incidence_angle); % [W]

                else
                    % No power generated
                    obj.instantaneous_power_generated = 0; % [W]
                    
                end

            else
                % Unhealthy Solar Panel

                obj.maximum_power = 0; % [W]

                obj.instantaneous_power_generated = 0; % [W]

                obj.Sun_incidence_angle = inf; % [deg]

            end

        end

        

    end
end

