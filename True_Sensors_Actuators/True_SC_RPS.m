%% Class: True_SC_RPS
% SC's Radioisotope Power Source (RPS)

classdef True_SC_RPS < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        instantaneous_power_consumed % [Watts] : Instantaneous housekeeping power consumed by this unit (e.g. controller/heater), irrespective of power generated

        instantaneous_data_rate_generated % [kbps] : Data rate generated during current time step, in kilo bits (kb) per sec

        power_BOL % [Watts] : Electrical power output at Beginning Of Life (BOL)

        half_life % [years] : Effective half-life used to decay power_BOL over time
        % This can be set to the radioisotope's physical half-life (e.g. 87.7 years for Pu-238),
        % or to an empirical "effective" half-life that also captures thermocouple degradation,
        % as is common practice for RPS power budgeting at low mission-fidelity (e.g. GPHS-RTG,
        % MMRTG). Defaults to 87.7 years (Pu-238) if not specified.

        BOL_date_string % [string] : Calendar date of Beginning Of Life, format = [DD-MMM-YYYY HH:MM:SS] (Optional)
        % If not specified, BOL is assumed to coincide with mission.true_time.t_initial_date_string,
        % i.e. the RPS is fresh at the start of the simulation.

        %% [ ] Properties: Variables Computed Internally

        name % [string] 'RPS i'

        health % [integer] Health of sensor/actuator
        % - 0. Switched off
        % - 1. Switched on, works nominally

        temperature % [deg C] : Temperature of sensor/actuator

        BOL_date % [sec] : Beginning Of Life date, in the same time base as mission.true_time.date #

        instantaneous_power_generated % [Watts] : Instantaneous electrical power produced by ith RPS

        maximum_power % [Watts] : Maximum power this RPS could produce at the current time, i.e. power_BOL decayed to the current date (irrespective of health)

        decay_fraction % [float] ∈ [0, 1] : Fraction of BOL power still available at the current time, due to radioisotope decay (and, if lumped into half_life, thermocouple degradation)

        elapsed_time % [sec] : Time elapsed since Beginning Of Life

        %% [ ] Properties: Storage Variables

        store

    end

    %% Methods
    methods

        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_RPS(init_data, mission, i_SC, i_HW)

            if isfield(init_data, 'name')
                obj.name = init_data.name;
            else
                obj.name = ['RPS ',num2str(i_HW)];
            end

            obj.health = 1;
            obj.temperature = 10; % [deg C]

            obj.instantaneous_power_consumed = init_data.instantaneous_power_consumed; % [W]
            obj.instantaneous_data_rate_generated = init_data.instantaneous_data_rate_generated; % [kbps]

            obj.mass = init_data.mass; % [kg]

            obj.power_BOL = init_data.power_BOL; % [W]

            if isfield(init_data, 'half_life')
                obj.half_life = init_data.half_life; % [years]
            else
                obj.half_life = 87.7; % [years] Pu-238 physical half-life
            end

            if isfield(init_data, 'BOL_date_string')
                obj.BOL_date = cal2sec(init_data.BOL_date_string); % [sec]
            else
                obj.BOL_date = mission.true_time.t_initial_date; % [sec] : BOL == simulation start
            end

            obj.elapsed_time = 0; % [sec]
            obj.decay_fraction = 1; % [float]
            obj.maximum_power = obj.power_BOL; % [W]
            obj.instantaneous_power_generated = 0; % [W]

            obj = func_update_RPS_instantaneous_power_generated(obj, mission);

            % Initialize Variables to store: instantaneous_power_generated maximum_power decay_fraction
            obj.store = [];

            obj.store.instantaneous_power_generated = zeros(mission.storage.num_storage_steps, length(obj.instantaneous_power_generated));
            obj.store.maximum_power = zeros(mission.storage.num_storage_steps, length(obj.maximum_power));
            obj.store.decay_fraction = zeros(mission.storage.num_storage_steps, length(obj.decay_fraction));

            obj = func_update_true_SC_RPS_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            func_initialize_list_HW_energy_generated(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_RPS_store(obj, mission)

            if mission.storage.flag_store_this_time_step == 1
                obj.store.instantaneous_power_generated(mission.storage.k_storage,:) = obj.instantaneous_power_generated; % [W]
                obj.store.maximum_power(mission.storage.k_storage,:) = obj.maximum_power; % [W]
                obj.store.decay_fraction(mission.storage.k_storage,:) = obj.decay_fraction; % [float]
            end

        end

        %% [ ] Methods: Main
        % Update RPS's instantaneous_power_generated

        function obj = func_main_true_SC_RPS(obj, mission, i_SC)

            obj = func_update_RPS_instantaneous_power_generated(obj, mission);

            obj = func_update_true_SC_RPS_store(obj, mission);

            % Update Power Generated Consumed
            func_update_instantaneous_power_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);
            func_update_instantaneous_power_generated(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update Data Generated
            func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end


        %% [ ] Methods: Instantaneous Power Generated
        % Update RPS's instantaneous_power_generated based on radioisotope decay since BOL
        % Note: unlike solar panels, RPS output does not depend on Sun visibility, incidence
        % angle, or spacecraft attitude -- it decays monotonically with elapsed time since BOL.        

        function obj = func_update_RPS_instantaneous_power_generated(obj, mission)

            obj.elapsed_time = max(mission.true_time.date - obj.BOL_date, 0); % [sec]

            half_life_sec = obj.half_life * 365.25 * 86400; % [sec]

            obj.decay_fraction = 2^( -obj.elapsed_time / half_life_sec ); % [float] ∈ [0, 1]

            obj.maximum_power = obj.power_BOL * obj.decay_fraction; % [W]

            if obj.health == 1              

                obj.instantaneous_power_generated = obj.maximum_power; % [W]

            else
                % Unhealthy RPS
                
                obj.instantaneous_power_generated = 0; % [W]

            end

        end

    end
end
