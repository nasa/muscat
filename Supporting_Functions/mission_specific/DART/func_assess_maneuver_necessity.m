%% [ ] Methods: Check Maneuver Necessary

function [is_needed, reason] = func_assess_maneuver_necessity(obj, mission, i_SC)
% This function evaluates whether a maneuver calculation is actually needed
% based on current trajectory and intercept parameters

% Default to needing a maneuver
is_needed = true;
reason = '';

% Get orbit estimation data
position_sc = mission.true_SC{i_SC}.software_SC_estimate_orbit.position;
velocity_sc = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity;

position_target = mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target;
velocity_target = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target;

% Calculate distance to target
rel_position = position_target - position_sc;
distance_to_target = norm(rel_position);

% Calculate relative velocity
rel_velocity = velocity_target - velocity_sc;
relative_speed = norm(rel_velocity);

approach_projection = dot(rel_position, rel_velocity) / (distance_to_target * relative_speed);


% Check if we're already very close to target (less than 3 km)
if distance_to_target < 3
    % Check if we're on a reasonable approach (approaching, not departing)

    % If projection is negative, we're approaching
    if approach_projection < 0
        % Calculate miss distance approximation using simple projection
        is_needed = false;
        reason = ['Already on successful intercept trajectory. Distance: ', num2str(distance_to_target), ' km'];
    end
end

% Check available fuel vs expected requirements
if is_needed && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
    % Estimate using very rough heuristic - actual calculation will happen in Lambert-Battin
    % This is just a pre-check to avoid unnecessary calculation
    available_fuel = mission.true_SC{i_SC}.true_SC_fuel_tank{1}.instantaneous_fuel_mass;

    % Very rough estimate of fuel needed (not accurate, just for pre-screening)
    approximate_fuel_required = relative_speed * 0.005 * mission.true_SC{i_SC}.true_SC_body.total_mass / 3000;

    % If clearly insufficient fuel (with large margin to be safe)
    if approximate_fuel_required > 10 * available_fuel
        is_needed = false;
        reason = ['Insufficient fuel for likely maneuver. Est. required: ', num2str(approximate_fuel_required), ' kg, Available: ', num2str(available_fuel), ' kg'];
    end
end

% Check if time-to-intercept is very short
if is_needed && approach_projection < 0
    % Estimate time to closest approach
    time_to_closest_approach = distance_to_target / relative_speed;

    % If very close to intercept (less than 1 minute)
    if time_to_closest_approach < 60
        is_needed = false;
        reason = ['Intercept imminent (', num2str(time_to_closest_approach), ' seconds). Maneuver computation skipped.'];
    end
end
end