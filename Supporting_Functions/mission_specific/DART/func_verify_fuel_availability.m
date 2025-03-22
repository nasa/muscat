%% [ ] Methods: Verify Fuel Availability

function obj = func_verify_fuel_availability(obj, mission, i_SC)

% Get distance to target
target_pos = mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target;
sc_pos = mission.true_SC{i_SC}.software_SC_estimate_orbit.position;
distance_to_target = norm(target_pos - sc_pos);

% Check if we're already so close that we can consider this an intercept
% For an impactor mission, if we're within 5km, we're essentially on an intercept course already
% This prevents unnecessary maneuvers when we're extremely close
if distance_to_target < 5.0
    % Get relative velocity to check if trajectory is favorable
    target_vel = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target;
    sc_vel = mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity;
    rel_vel = sc_vel - target_vel;

    % Calculate dot product to see if we're approaching or moving away
    approach_direction = dot(rel_vel, (target_pos - sc_pos) / distance_to_target);

    if approach_direction < 0 % Negative means approaching
        disp(['Already on intercept course at distance of ', num2str(distance_to_target), ...
            ' km with approach velocity of ', num2str(-approach_direction), ' km/s. Skipping maneuver calculation.']);

        % Skip the maneuver since we're already approaching the target
        obj.flag_insufficient_fuel = false; % Not a fuel issue
        obj.desired_DeltaV_needs_to_be_executed = false;
        obj.desired_DeltaV_computed = false;
        return;
    end
end

end
