%% Class: True_Ground_Station
% Tracks the data sent and recieved by Ground Station

classdef True_Ground_Station < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        num_GS_radio_antenna % [integer] Number of GS Radio Antenna

        %% [ ] Properties: Variables Computed Internally

        instantaneous_data_transmitted % [kb] Data transmitted by GS over mission.true_time.time_step sec

        total_data_transmitted % [kb] Data transmitted by GS over time

        instantaneous_data_received % [kb] Data received by GS over mission.true_time.time_step sec

        total_data_received % [kb] Data received by GS over time

        list_HW_data_transmitted % List of HW that transmitted data

        array_HW_data_transmitted % [kb] Total data transmitted by this HW

        list_HW_data_received % List of HW that received data

        array_HW_data_received % [kb] Total data received by this HW

        warning_counter % [integer] Counter stops the warning after 10 displays

        data % Other useful data

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_Ground_Station(init_data, mission)

            obj.num_GS_radio_antenna = init_data.num_GS_radio_antenna;

            obj.instantaneous_data_transmitted = 0; % [kb]
            obj.total_data_transmitted = 0; % [kb]
            obj.instantaneous_data_received = 0; % [kb]
            obj.total_data_received = 0; % [kb]

            obj.list_HW_data_transmitted = [];
            obj.array_HW_data_transmitted = []; 
            obj.list_HW_data_received = []; 
            obj.array_HW_data_received = []; 

            obj.warning_counter = 0; 

            if isfield(init_data, 'data')
                obj.data = init_data.data;
            else
                obj.data = [];
            end

            % Initialize Variables to store
            obj.store = [];

            obj.store.instantaneous_data_transmitted = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_transmitted));
            obj.store.instantaneous_data_received = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_data_received));

            obj.store.total_data_transmitted = zeros(mission.storage.num_storage_steps, length(obj.total_data_transmitted));
            obj.store.total_data_received = zeros(mission.storage.num_storage_steps, length(obj.total_data_received));

        end

        %% [ ] Methods: Initialize list_HW_data_transmitted
        % Initialize list_HW_data_transmitted for HW and Classes

        function obj = func_initialize_list_HW_data_transmitted(obj, equipment, mission)

            this_name = equipment.name;
            flag_name_exisits = 0;

            for i = 1:1:length(obj.list_HW_data_transmitted)
                if strcmp( obj.list_HW_data_transmitted{i}, this_name )
                    flag_name_exisits = 1;
                end
            end

            if flag_name_exisits == 0
                i = length(obj.list_HW_data_transmitted);
                obj.list_HW_data_transmitted{i+1} = this_name;

                if isprop(equipment, 'instantaneous_data_rate_transmitted')
                    this_instantaneous_data_transmitted = (equipment.instantaneous_data_rate_transmitted * mission.true_time.time_step); % [kb]
                elseif isprop(equipment, 'instantaneous_data_transmitted_per_sample')
                    this_instantaneous_data_transmitted = equipment.instantaneous_data_transmitted_per_sample; % [kb]
                else
                    error('Data transmitted incorrect!')
                end

                obj.array_HW_data_transmitted(1,i+1) = this_instantaneous_data_transmitted; % [kb]
            end

        end


        %% [ ] Methods: Initialize list_HW_data_received
        % Initialize list_HW_data_received for HW and Classes

        function obj = func_initialize_list_HW_data_received(obj, equipment, mission)

            this_name = equipment.name;
            flag_name_exisits = 0;

            for i = 1:1:length(obj.list_HW_data_received)
                if strcmp( obj.list_HW_data_received{i}, this_name )
                    flag_name_exisits = 1;
                end
            end

            if flag_name_exisits == 0
                i = length(obj.list_HW_data_received);
                obj.list_HW_data_received{i+1} = this_name;                

                if isprop(equipment, 'instantaneous_data_rate_received')
                    this_instantaneous_data_received = (equipment.instantaneous_data_rate_received * mission.true_time.time_step); % [kb]
                elseif isprop(equipment, 'instantaneous_data_received_per_sample')
                    this_instantaneous_data_received = equipment.instantaneous_data_received_per_sample; % [kb]
                else
                    error('Data received incorrect!')
                end

                obj.array_HW_data_received(1,i+1) = this_instantaneous_data_received; % [kb]
            end

        end



        %% [ ] Methods: Initialize Store
        % Initialize store of array_HW_data_transmitted and array_HW_data_received

        function obj = func_initialize_store_HW_data_transmitted_received(obj, mission)

            obj.store.list_HW_data_transmitted = obj.list_HW_data_transmitted;
            obj.store.list_HW_data_received = obj.list_HW_data_received;

            obj.store.array_HW_data_transmitted = zeros(mission.storage.num_storage_steps, length(obj.array_HW_data_transmitted));
            obj.store.array_HW_data_received = zeros(mission.storage.num_storage_steps, length(obj.array_HW_data_received));

            obj = func_update_true_ground_station_store(obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_ground_station_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_data_transmitted(mission.storage.k_storage,:) = obj.instantaneous_data_transmitted; % [kb]
                obj.store.instantaneous_data_received(mission.storage.k_storage,:) = obj.instantaneous_data_received; % [kb]

                obj.store.total_data_transmitted(mission.storage.k_storage,:) = obj.total_data_transmitted; % [kb]
                obj.store.total_data_received(mission.storage.k_storage,:) = obj.total_data_received; % [kb]

                obj.store.array_HW_data_transmitted(mission.storage.k_storage,:) = obj.array_HW_data_transmitted; % [kb]
                obj.store.array_HW_data_received(mission.storage.k_storage,:) = obj.array_HW_data_received; % [kb]
            end

        end

        %% [ ] Methods: Main
        % Main Ground Station code

        function obj = func_main_true_ground_station(obj, mission, i_SC)

            obj.total_data_transmitted = obj.total_data_transmitted + obj.instantaneous_data_transmitted; % [kb]
            obj.total_data_received = obj.total_data_received + obj.instantaneous_data_received; % [kb]

            % Store
            obj = func_update_true_ground_station_store(obj, mission);

            % Reset All Variables
            obj.instantaneous_data_transmitted = 0; % [kb]
            obj.instantaneous_data_received = 0; % [kb]

        end


        %% [ ] Methods: Update Instantaneous Data Transmitted
        % Updates instantaneous_data_transmitted by all HW and Classes

        function obj = func_update_instantaneous_data_transmitted(obj, equipment, mission)

            if isprop(equipment, 'instantaneous_data_rate_transmitted')
                this_instantaneous_data_transmitted = (equipment.instantaneous_data_rate_transmitted * mission.true_time.time_step); % [kb]
            elseif isprop(equipment, 'instantaneous_data_transmitted_per_sample')
                this_instantaneous_data_transmitted = equipment.instantaneous_data_transmitted_per_sample; % [kb]
            else
                error('Data transmitted incorrect!')
            end

            obj.instantaneous_data_transmitted = obj.instantaneous_data_transmitted + this_instantaneous_data_transmitted; % [kb]

            this_name = equipment.name;
            flag_name_exisits = 0;
            this_idx = 0;

            for i = 1:1:length(obj.list_HW_data_transmitted)
                if strcmp( obj.list_HW_data_transmitted{i}, this_name )
                    flag_name_exisits = 1;
                    this_idx = i;
                end
            end

            if flag_name_exisits == 0
                error('HW not found!')
            else
                obj.array_HW_data_transmitted(1,this_idx) = obj.array_HW_data_transmitted(1,this_idx) + this_instantaneous_data_transmitted; % [kb]
            end

        end



        %% [ ] Methods: Update Instantaneous Data Received
        % Updates instantaneous_data_received by all HW and Classes

        function obj = func_update_instantaneous_data_received(obj, equipment, mission)

            if isprop(equipment, 'instantaneous_data_rate_received')
                this_instantaneous_data_received = (equipment.instantaneous_data_rate_received * mission.true_time.time_step); % [kb]
            elseif isprop(equipment, 'instantaneous_data_received_per_sample')
                this_instantaneous_data_received = equipment.instantaneous_data_received_per_sample; % [kb]
            else
                error('Data received incorrect!')
            end

            obj.instantaneous_data_received = obj.instantaneous_data_received + this_instantaneous_data_received; % [kb]

            this_name = equipment.name;
            flag_name_exisits = 0;
            this_idx = 0;

            for i = 1:1:length(obj.list_HW_data_received)
                if strcmp( obj.list_HW_data_received{i}, this_name )
                    flag_name_exisits = 1;
                    this_idx = i;
                end
            end

            if flag_name_exisits == 0
                error('HW not found!')
            else
                obj.array_HW_data_received(1,this_idx) = obj.array_HW_data_received(1,this_idx) + this_instantaneous_data_received; % [kb]
            end

        end


        
    end
end

