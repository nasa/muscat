%% Class: True_SC_Body
% Tracks the SC Body

classdef True_SC_Body < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        name % [string] = 'SC j' for jth SC

        mass % : Mass of the SC
        % – dry : Dry mass of the SC, that doesn't change position/attitude
        % - - mass [kg] : Actual mass
        % - - location [m] : Location of this mass in body frame B
        % - - MI_over_m [m^2] : Computed once during initialization, shouldn't change with time (NOTE: NOT MULTIPLED BY MASS)

        % – supplement : Positive/negative mass that is added/removed from the SC. (e.g. sample collection or projectile) (Optional)

        % – # propellant : Propellant mass of the SC that does change value (Optional)
        % Take propellant mass from True_SC_Fuel_Tank

        % – # solar_panel : Solar panel mass of the SC, that doesnot change value (Optional)
        % Take solar panel mass from True_SC_Solar_Panel

        total_mass % [kg] : Total mass of the SC

        mode_COM_selector % Select which COM to use
        % 'given' : Give apriori
        % 'update' : Computed by the code

        location_COM % [m] : Compute CM from above data

        shape_model % : Cell of SC shape models
        % – Vertices [m] : Position of vertices in body frame B
        % – Faces : Triplet of vertex indices define a face
        % – Face_reflectance_factor in [0, 1] : Used for ith face (used for SRP)
        % - r_CM [m] : CM of this shape
        % - I_through_r_CM [kg m^2] : Intertia matrix of this shape, about its CM
        % - volume [m^2] : Volume of this shape
        % - Face_center [m] : Center of each Face
        % – Face_normal [unit vector] : Normal out vector of each face (used for SRP)
        % - Face_area [m^2] : Area of each face
        % - type [string] : Type of shape is used for MI and volume calculations

        total_volume % [m^3] : Total volume of the SC

        mode_MI_selector % Select which MI to use
        % 'given' : Give apriori
        % 'update' : Computed by the code

        total_MI % [kg m^2] : Total MI of the SC

        num_hardware_exists % Data structure that denotes if a hardware exists on a SC or not (1 >= HW exists, 0 = It doesn t exist)
        % Initialized to zero using init_num_hardware_exists.m file
        % - num_onboard_clock [integer]
        % - num_sun_sensor [integer]
        % - num_star_tracker [integer]
        % - num_imu [integer]
        % - num_micro_thruster [integer]
        % - num_reaction_wheel [integer]
        % - num_magnetorquer [integer]
        % - num_camera [integer]
        % - num_chemical_thruster [integer]
        % - num_ep_thruster [integer]
        % - num_solar_panel [integer]
        % - num_RTG [integer]
        % - num_battery [integer]
        % - num_PDC [integer]
        % - num_radio_antenna [integer]
        % - num_dte_communication [integer]
        % - num_intersat_communication [integer]
        % - num_onboard_memory [integer]
        % - num_science_radar [integer]
        % - num_science_altimeter [integer]
        % - num_science_telescope [integer]
        % - num_science_camera [integer]


        %% [ ] Properties: Variables Computed Internally

        flag_update_SC_body_total_mass_COM_MI % [Boolean] Flag sets if these variables should be computed again

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Body(init_data, mission)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['SC ',num2str(init_data.i_SC)];
            end

            % mass
            obj.mass = [];
            obj.mass.dry{1}.mass = 0; % [kg]
            obj.mass.dry{1}.location = [0 0 0]; % [m]
            obj.mass.dry{1}.MI_over_m = zeros(3,3); % [m^2]

            if isfield(init_data.mass, 'supplement')
                obj.mass.supplement = init_data.mass.supplement;

            else
                obj.mass.supplement{1}.mass = 0;
                obj.mass.supplement{1}.location = [0 0 0]; % [m]
                obj.mass.supplement{1}.MI_over_m = zeros(3,3); % [m^2]
            end

            obj.mass.propellant{1}.mass = 0; % [kg]
            obj.mass.propellant{1}.location = [0 0 0]; % [m]
            obj.mass.propellant{1}.MI_over_m = zeros(3,3); % [m^2]

            obj.mass.solar_panel{1}.mass = 0; % [kg]
            obj.mass.solar_panel{1}.location = [0 0 0]; % [m]
            obj.mass.solar_panel{1}.MI_over_m = zeros(3,3); % [m^2]

            obj.mode_COM_selector = init_data.mode_COM_selector;

            if strcmp(obj.mode_COM_selector, 'given')
                obj.total_mass = init_data.total_mass; % [kg]
                obj.location_COM = init_data.location_COM; % [m]
                obj.total_volume = init_data.total_volume; % [m^3]
            else

                obj.total_mass = 0; % [kg]
                obj.location_COM = [0 0 0]; % [m]
                obj.total_volume = 0; % [m^3]
            end

            % shape model
            obj.shape_model = init_data.shape_model;

            
            for i_shape = 1:length(obj.shape_model)
                % warning('Shape model should be a cell of structs with fields (r_CM, I_over_m, volume, etc)');
                shape = obj.shape_model{i_shape};

                % Center of mass
                r_CM = mean(shape.Vertices, 1); % [m]

                switch obj.shape_model{i_shape}.type

                    case 'cuboid'

                        % Inertia matrix (assume cuboid)
                        L = max(shape.Vertices(:,1)) - min(shape.Vertices(:,1)); % [m]
                        W = max(shape.Vertices(:,2)) - min(shape.Vertices(:,2)); % [m]
                        H = max(shape.Vertices(:,3)) - min(shape.Vertices(:,3)); % [m]
                        I_through_r_CM = diag([1/12*(W^2+H^2), 1/12*(L^2+H^2), 1/12*(L^2+W^2)]); % [m^2]

                        % Volume
                        volume = L*W*H; % [m^3]

                    otherwise
                        error('Havent written yet!')

                end

                % Update
                obj.shape_model{i_shape}.r_CM = r_CM;
                obj.shape_model{i_shape}.I_through_r_CM = I_through_r_CM;
                obj.shape_model{i_shape}.volume = volume;

                % obj.total_volume = obj.total_volume + obj.shape_model{i_shape}.volume; % [m^3]

            end

            % compute face orientation + normal vector + area
            for i_shape = 1:length(obj.shape_model)

                shape = obj.shape_model{i_shape};
                Face_center = zeros(size(shape.Faces));
                Face_normal = zeros(size(shape.Faces));
                Face_area = zeros(size(shape.Faces,1),1);

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
                    Face_area(i) = sqrt(s*(s-a)*(s-b)*(s-c));    % Heron formula
                end

                obj.shape_model{i_shape}.Face_center = Face_center;
                obj.shape_model{i_shape}.Face_normal = Face_normal;
                obj.shape_model{i_shape}.Face_area = Face_area;
            end

            % Add all HW on SC
            obj.num_hardware_exists = init_data.num_hardware_exists;

            % Moment of Intertia
            obj.mode_MI_selector = init_data.mode_MI_selector;

            if strcmp(obj.mode_MI_selector, 'given')
                obj.total_MI = init_data.total_MI; % [kg m^2]
            end

            % Compute dry mass
            if ~strcmp(obj.mode_MI_selector, 'given')
   
                for i_shape = 1:length(obj.shape_model)
                    obj.mass.dry{i_shape}.mass = obj.shape_model{i_shape}.mass; % [kg]
                    obj.mass.dry{i_shape}.location = obj.shape_model{i_shape}.r_CM; % [m]
                    obj.mass.dry{i_shape}.MI_over_m = obj.shape_model{i_shape}.I_through_r_CM; % [m^2]
                end
    
                % Compute total mass and COM and MI
                obj = func_update_SC_body_total_mass_COM_MI(obj);
            end
            
            % Reset Flag
            obj.flag_update_SC_body_total_mass_COM_MI = 0;

            % Initialize Variables to store: total_mass location_COM total_MI
            obj.store = [];

            obj.store.total_mass = zeros(mission.storage.num_storage_steps, length(obj.total_mass));
            obj.store.location_COM = zeros(mission.storage.num_storage_steps, length(obj.location_COM));
            obj.store.total_MI = zeros(mission.storage.num_storage_steps, 9);

            obj = func_update_true_SC_body_store(obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_body_store(obj, mission)
            if mission.storage.flag_store_this_time_step == 1
                obj.store.total_mass(mission.storage.k_storage,:) = obj.total_mass; % [kg]
                obj.store.location_COM(mission.storage.k_storage,:) = obj.location_COM; % [m]
                obj.store.total_MI(mission.storage.k_storage,:) = reshape(obj.total_MI,1,9); % [kg m^2]
            end
        end


        %% [ ] Methods: Main
        % Main function

        function obj = func_main_true_SC_body(obj, mission, i_SC)

            % First update fuel and solar panel masses if needed
            if obj.flag_update_SC_body_total_mass_COM_MI == 1
                % Update mass values from fuel tanks and solar panels first
                obj = func_update_SC_body_mass(obj, mission, i_SC);
                
                % Then compute total mass and COM and MI
                obj = func_update_SC_body_total_mass_COM_MI(obj);
            end

            % Update Store
            obj = func_update_true_SC_body_store(obj, mission);
            
        end


        %% [ ] Methods: Update Total Mass, COM, MI
        % Update total mass and COM

        function obj = func_update_SC_body_total_mass_COM_MI(obj)

            % Reset Flag
            obj.flag_update_SC_body_total_mass_COM_MI = 0;

            % Update Total Mass and COM
            if strcmp(obj.mode_COM_selector, 'update')

                obj.total_mass = 0; % [kg]
                obj.location_COM = [0 0 0]; % [m]

                for i_mass_class = 1:1:4

                    this_mass_class = [];

                    switch i_mass_class

                        case 1
                            this_mass_class = obj.mass.dry;
                        case 2
                            this_mass_class = obj.mass.supplement;
                        case 3
                            this_mass_class = obj.mass.propellant;
                        case 4
                            this_mass_class = obj.mass.solar_panel;
                        otherwise
                            error('this_mass_class does not exist!')
                    end

                    for i=1:1:length(this_mass_class)
                        obj.location_COM = ((obj.total_mass * obj.location_COM) + (this_mass_class{i}.mass * this_mass_class{i}.location)); % [kg m]
                        obj.total_mass = obj.total_mass + this_mass_class{i}.mass; % [kg]
                        obj.location_COM = (obj.location_COM / obj.total_mass); % [m]
                    end

                end

            end


            % Update MI
            if strcmp(obj.mode_MI_selector, 'update')

                obj.total_MI = zeros(3,3); % [kg m^2]

                for i_mass_class = 1:1:4

                    this_mass_class = [];

                    switch i_mass_class

                        case 1
                            this_mass_class = obj.mass.dry;
                        case 2
                            this_mass_class = obj.mass.supplement;
                        case 3
                            this_mass_class = obj.mass.propellant;
                        case 4
                            this_mass_class = obj.mass.solar_panel;
                        otherwise
                            error('this_mass_class does not exist!')
                    end

                    for i=1:1:length(this_mass_class)

                        % Displacement vector from the shape CM to the SC CM
                        r_CM_i = obj.location_COM - this_mass_class{i}.location;

                        % Parallel Axis Theorem: I_cm_sc = I_cm_shape + m * (r' * r * I_3 - r * r')
                        I_i = (this_mass_class{i}.mass * this_mass_class{i}.MI_over_m) + (this_mass_class{i}.mass * ( (r_CM_i * r_CM_i' * eye(3)) - (r_CM_i' * r_CM_i)));

                        % Add inertia matrix to the total inertia matrix
                        obj.total_MI = obj.total_MI + I_i;

                    end

                end

            end

        end


        
        %% [ ] Methods: Update Mass 
        % Update the mass class with solar panel and propellant mass

        function obj = func_update_SC_body_mass(obj, mission, i_SC)

            if isfield(mission.true_SC{i_SC}, 'true_SC_solar_panel')
                for i_SP = 1:1:obj.num_hardware_exists.num_solar_panel
                    obj.mass.solar_panel{i_SP}.mass = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.mass; % [kg]
                    obj.mass.solar_panel{i_SP}.location = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.r_CM; % [m]
                    obj.mass.solar_panel{i_SP}.MI_over_m = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.I_through_r_CM; % [kg m^2]
                end
            end

            if isfield(mission.true_SC{i_SC}, 'true_SC_fuel_tank')
                for i_FT = 1:1:obj.num_hardware_exists.num_fuel_tank
                    % Update propellant mass from fuel tank
                    obj.mass.propellant{i_FT}.mass = mission.true_SC{i_SC}.true_SC_fuel_tank{i_FT}.instantaneous_fuel_mass; % [kg]
                    obj.mass.propellant{i_FT}.location = mission.true_SC{i_SC}.true_SC_fuel_tank{i_FT}.location; % [m]
                    
                    % Calculate approximate moment of inertia for a simple cuboid fuel mass
                    % This is a simplified approach - for greater accuracy, a detailed shape model would be better
                    if ~isempty(mission.true_SC{i_SC}.true_SC_fuel_tank{i_FT}.shape_model)
                        % If shape model exists, use it
                        if isfield(mission.true_SC{i_SC}.true_SC_fuel_tank{i_FT}.shape_model, 'I_through_r_CM')
                            obj.mass.propellant{i_FT}.MI_over_m = mission.true_SC{i_SC}.true_SC_fuel_tank{i_FT}.shape_model.I_through_r_CM;
                        else
                            % Approximate as cuboid if dimensions available
                            obj.mass.propellant{i_FT}.MI_over_m = zeros(3,3);
                        end
                    else
                        % Fallback to simple approximation - treat as point mass with small inertia
                        obj.mass.propellant{i_FT}.MI_over_m = 1e-3 * eye(3); % [m^2]
                    end
                    
                    % Set flag to update total mass, COM, and MI
                    obj.flag_update_SC_body_total_mass_COM_MI = 1;
                end
            end

        end

    end

end
