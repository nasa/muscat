%% [ ] Methods: Main for DART

function obj = func_update_software_SC_control_orbit_DART(obj, mission, i_SC)

% Main control loop for spacecraft orbit


first_pass = false;

% Always verify the capacity of the fuel tanks.
total_fuel = 0;
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
    for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
        total_fuel = total_fuel + mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.instantaneous_fuel_mass;
    end

    % Log fuel level if it falls below certain thresholds
    if total_fuel < 0.5
        warning('CRITICAL: Spacecraft fuel level below 0.5 kg. Remaining: %.3f kg', total_fuel);
    elseif total_fuel < 1.0
        warning('WARNING: Spacecraft fuel level below 1.0 kg. Remaining: %.3f kg', total_fuel);
    end
end

if ~obj.desired_DeltaV_computed


    % First, assess whether a maneuver calculation is actually needed
    [is_maneuver_needed, reason] = func_assess_maneuver_necessity(obj, mission, i_SC);

    if ~is_maneuver_needed
        % Log the reason we're skipping the maneuver calculation
        disp(['Skipping maneuver calculation: ', reason]);

        % If we're skipping, make sure to reset the flags
        func_reset_after_completion(obj, mission, i_SC);
        return;
    end

    % Maneuver seems necessary, proceed with calculations
    func_estimate_target_intercept_location_time(obj, mission, i_SC);
    func_compute_TCM_Lambert_Battin(obj, mission, i_SC);

    % Log the computed maneuver for mission awareness
    if obj.desired_DeltaV_computed
        disp(['Computed maneuver: DeltaV magnitude = ', num2str(norm(obj.desired_control_DeltaV)), ' m/s']);
        disp(['Execution time: ', datestr(datetime('now') + seconds(obj.time_DeltaV - mission.true_time.time))]);
        disp(['Estimated fuel required: ', num2str(obj.estimated_fuel_required), ' kg']);
    end

    first_pass = true;
end



% Validate attitude and execute DeltaV if conditions are met
if obj.desired_DeltaV_needs_to_be_executed && ...
        (mission.true_time.time >= obj.time_DeltaV)  && ...
        ~first_pass && ...
        (norm(func_get_attitude_error(mission.true_SC{i_SC}.software_SC_control_attitude, mission, i_SC)) < 0.03) && ... % rad
        ~obj.flag_insufficient_fuel
    func_command_DeltaV(obj, mission, i_SC);
end

% Handle thruster warm-up logic if maneuver is planned
if obj.desired_DeltaV_needs_to_be_executed && obj.desired_DeltaV_computed
    % Check if remaining DeltaV is above threshold
    remaining_DeltaV = obj.desired_control_DeltaV - obj.total_DeltaV_executed;
    if norm(remaining_DeltaV) < obj.threshold_minimum_deltaV
        % If DeltaV is below threshold, consider it complete
        disp(['Skipping maneuver - remaining DeltaV (', num2str(norm(remaining_DeltaV)), ...
            ' m/s) below threshold (', num2str(obj.threshold_minimum_deltaV), ' m/s)']);

        % Stop warming up thruster if needed
        for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
            if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = false;
                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_executive = false;
            end
        end

        func_reset_after_completion(obj, mission, i_SC);
        return;
    end

    % Check for sufficient fuel before proceeding
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
        total_fuel = 0;
        for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
            total_fuel = total_fuel + mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.instantaneous_fuel_mass;
        end

        % Add safety margin
        fuel_with_margin = obj.estimated_fuel_required * 1.1; % 10% margin

        if total_fuel < fuel_with_margin
            warning('Insufficient fuel for DeltaV execution. Required: %.3f kg (with margin), Available: %.3f kg', ...
                fuel_with_margin, total_fuel);

            % Cancel the maneuver
            obj.flag_insufficient_fuel = true;
            obj.desired_DeltaV_needs_to_be_executed = false;
            obj.desired_DeltaV_computed = false;

            % Stop warming up thruster
            for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                    mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = false;
                end
            end
            return;
        end
    end

    % Get time until DeltaV execution
    time_to_DeltaV = obj.time_DeltaV - mission.true_time.time;

    % Get thruster warm-up time
    thruster_warm_up_time = 30; % Default value
    for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
        if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
            thruster_warm_up_time = mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).thruster_warm_up_time;
            break;
        end
    end

    % Start thruster warm-up with a safety margin (45 sec before needed)
    thruster_warming = false;
    for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
        if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up
            thruster_warming = true;
            break;
        end
    end

    if time_to_DeltaV > 0 && time_to_DeltaV <= (thruster_warm_up_time + 45) && ~thruster_warming
        % Time to start warming up the thruster
        for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
            if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = true;
                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).func_start_warm_up(mission);
            end
        end
        disp(['Starting thruster warm-up at T-', num2str(time_to_DeltaV), ' seconds before maneuver']);
    end
end

% Check if maneuver is complete - simplified approach
if obj.desired_DeltaV_needs_to_be_executed && obj.desired_DeltaV_computed
    % Check remaining DeltaV - this is updated directly by the thruster
    remaining_DeltaV = obj.desired_control_DeltaV - obj.total_DeltaV_executed;

    % Check timeout conditions
    maneuver_timeout = false;
    time_since_planned = mission.true_time.time - obj.time_DeltaV;

    % Timeout if running too long since start
    if obj.maneuver_start_time > 0 && (mission.true_time.time - obj.maneuver_start_time) > 120
        maneuver_timeout = true;
        warning('Maneuver timeout reached after %d seconds from maneuver start', ...
            mission.true_time.time - obj.maneuver_start_time);
    end

    % Timeout if waiting too long after planned execution
    if time_since_planned > 120 && ~maneuver_timeout
        maneuver_timeout = true;
        warning('Maneuver timeout: %d seconds since planned execution time', time_since_planned);
    end

    % Check if we should complete the maneuver
    is_deltaV_complete = norm(remaining_DeltaV) < obj.threshold_minimum_deltaV;
    if is_deltaV_complete || maneuver_timeout || time_since_planned > 60
        % Report reason for completion
        if is_deltaV_complete
            disp(['Maneuver completed: Target DeltaV achieved within ', ...
                num2str(obj.threshold_minimum_deltaV), ' m/s threshold']);
        elseif maneuver_timeout
            disp(['Maneuver timeout after ', num2str(mission.true_time.time - obj.maneuver_start_time), ' seconds']);
        else
            disp(['Maneuver forced to complete after ', num2str(time_since_planned), ' seconds from planned time']);
        end

        % Reset everything
        func_reset_after_completion(obj, mission, i_SC);

        % Turn off thruster warm-up
        for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
            if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).flag_warming_up = false;
                mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).commanded_thrust = 0;
            end
        end

        disp('Maneuver completed and all flags reset');
    end
end


end