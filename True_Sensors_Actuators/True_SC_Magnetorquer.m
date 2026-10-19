%% Class: True_SC_Magnetorquer
% Represents a single magnetorquer for spacecraft attitude control in Earth orbit
% Computes the Earth magnetic field at the SC location (tilted dipole model)
% and the resulting control torque = commanded_dipole_moment x B_field

classdef True_SC_Magnetorquer < handle

    %% Properties
    properties

        %% [ ] Properties: Initialized Variables

        location                     %[m] : Location of the magnetorquer in the body frame
        orientation                  %[Unit vector] : Axis of the magnetic dipole moment in the body frame
        maximum_dipole_moment         %[A m^2] : Maximum magnetic dipole moment the torquer can produce
        moment_noise                  %[A m^2] : Random noise added to commanded dipole moment

        mass                          %[kg] : Mass of the magnetorquer

        power_consumed_dipole_moment_array % [power_array ; dipole_moment_array]

        instantaneous_data_generated_per_sample %[Kb] : Data generated during the current time step

        instantaneous_data_type = 1 % 0 = Data rate, 1 = Data per sample
                                    % used to avoid calling isprop() every time
                                    % step to determine if using instantaneous_data_rate_generated or
                                    % instantaneous_data_generated_per_sample, which is slow

        dipole_field_constant        %[T m^3] : mu0*M_Earth/(4*pi), strength of the tilted dipole model
        dipole_tilt_angle             %[deg] : Angle of the dipole axis from Earth's rotation (+Z) axis
        dipole_tilt_longitude         %[deg] : East longitude of the dipole axis in the Earth body-fixed frame

        %% [ ] Properties: Variables Computed Internally

        name                          % Name of the magnetorquer
        health                        % Health status (0: Not working, 1: Working)

        temperature                   %[deg C]

        commanded_dipole_moment       %[A m^2] : Desired dipole moment magnitude along orientation, comes from the SC's software
        true_commanded_dipole_moment  %[A m^2] : Add moment noise to commanded_dipole_moment. This is the actual moment of this torquer, which the SC software doesn't know.

        actual_dipole_moment_vector   %[A m^2] : Actual dipole moment vector in the body frame
        magnetic_field_body           %[T] : Earth's magnetic field vector at the SC location, in the body frame

        actual_torque                 %[Nm] : Actual torque = actual_dipole_moment_vector x magnetic_field_body
        saturated                     %[Boolean] Is the commanded dipole moment currently saturated ?

        instantaneous_power_consumption %[Watts] : Power consumed during operation
        instantaneous_power_consumed    %[Watts] : Alias for power tracking system

        flag_executive                %[Bool] : Is the magnetorquer currently executing a command ?

        dipole_axis_ECEF               %[Unit vector] : Tilted dipole axis, in the Earth body-fixed (ECEF) frame
        index_Earth_target             %[integer] : Index into mission.true_target corresponding to Earth

        instantaneous_data_volume        % [kb]

        %% [ ] Properties: Storage Variables

        store                        %[Struct] : Store of the magnetorquer
    end

    %% Methods
    methods
        %% [ ] Methods: Constructor
        % Construct an instance of this class

        function obj = True_SC_Magnetorquer(init_data, mission, i_SC, i_MTQ)

            obj.name = ['Magnetorquer ', num2str(i_MTQ)];

            obj.health = true;
            obj.temperature = 20;

            obj.location = init_data.location;
            obj.orientation = init_data.orientation / norm(init_data.orientation);

            obj.maximum_dipole_moment = init_data.maximum_dipole_moment;
            obj.moment_noise = init_data.moment_noise;

            if isfield(init_data, 'mass')
                obj.mass = init_data.mass;
            else
                obj.mass = 0;
            end

            obj.power_consumed_dipole_moment_array = init_data.power_consumed_dipole_moment_array;

            obj.instantaneous_data_generated_per_sample = init_data.instantaneous_data_generated_per_sample; % [kb]
            obj.instantaneous_data_volume = obj.instantaneous_data_generated_per_sample; % [kb]

            % Tilted dipole Earth magnetic field model parameters
            % Defaults are the standard first-order geomagnetic dipole approximation
            if isfield(init_data, 'dipole_field_constant')
                obj.dipole_field_constant = init_data.dipole_field_constant; % [T m^3]
            else
                obj.dipole_field_constant = 7.94e15; % [T m^3] : mu0*M_Earth/(4*pi)
            end

            if isfield(init_data, 'dipole_tilt_angle')
                obj.dipole_tilt_angle = init_data.dipole_tilt_angle; % [deg]
            else
                obj.dipole_tilt_angle = 169.2; % [deg] : from Earth's rotation (+Z) axis
            end

            if isfield(init_data, 'dipole_tilt_longitude')
                obj.dipole_tilt_longitude = init_data.dipole_tilt_longitude; % [deg]
            else
                obj.dipole_tilt_longitude = -72.68; % [deg] : East longitude, in Earth body-fixed frame
            end

            obj.dipole_axis_ECEF = [
                sind(obj.dipole_tilt_angle) * cosd(obj.dipole_tilt_longitude);
                sind(obj.dipole_tilt_angle) * sind(obj.dipole_tilt_longitude);
                cosd(obj.dipole_tilt_angle)];

            % This class requires the SC to be navigating relative to Earth
            obj.index_Earth_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
            assert(strcmp(mission.true_target{obj.index_Earth_target}.name, 'Earth'), ...
                'True_SC_Magnetorquer requires the SC navigation target to be Earth')

            % Initialize dynamic state
            obj.commanded_dipole_moment = 0;
            obj.true_commanded_dipole_moment = 0;
            obj.actual_dipole_moment_vector = [0, 0, 0];
            obj.magnetic_field_body = [0, 0, 0];
            obj.actual_torque = [0, 0, 0];
            obj.saturated = false;
            obj.flag_executive = false;
            obj.instantaneous_power_consumption = 0;
            obj.instantaneous_power_consumed = 0;

            % Initialize storage
            obj.store = [];
            obj.store.commanded_dipole_moment = zeros(mission.storage.num_storage_steps_attitude, 1);
            obj.store.actual_torque = zeros(mission.storage.num_storage_steps_attitude, 3);
            obj.store.magnetic_field_body = zeros(mission.storage.num_storage_steps_attitude, 3);
            obj.store.saturated = zeros(mission.storage.num_storage_steps_attitude, 1);

            % Update storage
            func_update_true_SC_magnetorquer_store(obj, mission);

            % Update SC Power Class
            func_initialize_list_HW_energy_consumed(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update SC Data Handling Class
            func_initialize_list_HW_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

        end

        %% [ ] Methods: Store
        % Update the store variable

        function obj = func_update_true_SC_magnetorquer_store(obj, mission)
            obj.store.commanded_dipole_moment(mission.storage.k_storage_attitude, :) = obj.commanded_dipole_moment;
            obj.store.actual_torque(mission.storage.k_storage_attitude, :) = obj.actual_torque;
            obj.store.magnetic_field_body(mission.storage.k_storage_attitude, :) = obj.magnetic_field_body;
            obj.store.saturated(mission.storage.k_storage_attitude, :) = obj.saturated;
        end

        %% [ ] Methods: Magnetic Field
        % Compute Earth's tilted-dipole magnetic field at the SC location, in the body frame

        function obj = func_update_true_SC_magnetorquer_magnetic_field(obj, mission, i_SC)

            target = mission.true_target{obj.index_Earth_target};

            % SC position relative to Earth's center, in J2000 [m]
            r_ECI = 1e3 * mission.true_SC{i_SC}.true_SC_orbit_propagation.position_relative_target(:); % [m]

            % Rotate into the Earth body-fixed (ECEF) frame
            % target.rotation_matrix converts Body Frame -> J2000, so its transpose does the reverse
            r_ECEF = target.rotation_matrix' * r_ECI; % [m]

            r_norm = norm(r_ECEF); % [m]
            r_hat = r_ECEF / r_norm;

            % Tilted dipole magnetic field, in ECEF [T]
            B_ECEF = (obj.dipole_field_constant / r_norm^3) * ...
                (3 * dot(obj.dipole_axis_ECEF, r_hat) * r_hat - obj.dipole_axis_ECEF);

            % Rotate to the inertial (J2000) frame
            B_ECI = target.rotation_matrix * B_ECEF; % [T]

            % Rotate to the SC body frame
            % adc.rotation_matrix converts Body Frame -> Inertial Frame, so its transpose does the reverse
            B_body = mission.true_SC{i_SC}.true_SC_adc.rotation_matrix' * B_ECI; % [T]

            obj.magnetic_field_body = B_body';
        end

        %% [ ] Methods: Main
        % Update Magnetorquer

        function func_main_true_SC_magnetorquer(obj, mission, i_SC, ~)
            % Main function that runs the magnetorquer simulation
            if ~obj.health
                obj.commanded_dipole_moment = 0;
                obj.true_commanded_dipole_moment = 0;
                obj.actual_dipole_moment_vector = [0, 0, 0];
                obj.actual_torque = [0, 0, 0];
                obj.saturated = false;
                obj.instantaneous_power_consumption = 0;
                obj.instantaneous_power_consumed = 0;
                obj.instantaneous_data_generated_per_sample = 0;
                return;
            end

            % Earth's magnetic field is an environment quantity, update it regardless of actuation
            obj = func_update_true_SC_magnetorquer_magnetic_field(obj, mission, i_SC);

            if (obj.flag_executive && abs(obj.commanded_dipole_moment) > 0)

                % Limit commanded dipole moment
                if abs(obj.commanded_dipole_moment) > obj.maximum_dipole_moment
                    obj.commanded_dipole_moment = sign(obj.commanded_dipole_moment) * obj.maximum_dipole_moment;
                    obj.saturated = true;
                else
                    obj.saturated = false;
                end

                % Randomize noise
                current_moment_noise = obj.moment_noise * (2*rand() - 1);

                % Compute the true dipole moment with noise
                obj.true_commanded_dipole_moment = obj.commanded_dipole_moment + current_moment_noise;

                % Compute the dipole moment vector, in the body frame
                obj.actual_dipole_moment_vector = obj.orientation * obj.true_commanded_dipole_moment;

                % Compute the torque generated by the magnetorquer, in the body frame
                % tau = m x B
                obj.actual_torque = cross(obj.actual_dipole_moment_vector, obj.magnetic_field_body);

                % Apply the generated torque to the spacecraft ADC
                mission.true_SC{i_SC}.true_SC_adc.control_torque = ...
                    mission.true_SC{i_SC}.true_SC_adc.control_torque + obj.actual_torque;

                % Update power consumption
                power_consumptions = obj.power_consumed_dipole_moment_array(1, :);
                dipole_moments = obj.power_consumed_dipole_moment_array(2, :);

                % Ensure the array is sorted by dipole moment
                [dipole_moments, sortIdx] = sort(dipole_moments);
                power_consumptions = power_consumptions(sortIdx);

                % Interpolate to find the power consumption for the current dipole moment
                abs_moment = abs(obj.true_commanded_dipole_moment);
                if abs_moment < min(dipole_moments)
                    obj.instantaneous_power_consumption = power_consumptions(1);
                elseif abs_moment > max(dipole_moments)
                    obj.instantaneous_power_consumption = power_consumptions(end);
                else
                    obj.instantaneous_power_consumption = interp1(dipole_moments, power_consumptions, abs_moment, 'linear');
                end
                % Set property for power tracking system
                obj.instantaneous_power_consumed = obj.instantaneous_power_consumption;

                % Update the data generated
                obj.instantaneous_data_generated_per_sample = obj.instantaneous_data_volume; % [kb]

            else
                obj.commanded_dipole_moment = 0;
                obj.true_commanded_dipole_moment = 0;
                obj.actual_dipole_moment_vector = [0, 0, 0];
                obj.actual_torque = [0, 0, 0];
                obj.saturated = false;
                obj.instantaneous_power_consumption = 0;
                obj.instantaneous_power_consumed = 0;
                obj.instantaneous_data_generated_per_sample = 0;
            end

            % Update Power Consumed
            func_update_instantaneous_power_consumed_attitude(mission.true_SC{i_SC}.true_SC_power, obj, mission);

            % Update Data Generated
            func_update_instantaneous_data_generated_attitude(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);

            % Update storage
            func_update_true_SC_magnetorquer_store(obj, mission);

            % Reset command flags at the end of each cycle
            % This prevents commands from persisting indefinitely between cycles
            obj.flag_executive = false;
            obj.commanded_dipole_moment = 0;
        end
    end
end
