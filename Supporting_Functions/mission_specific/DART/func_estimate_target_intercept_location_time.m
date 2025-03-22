%% [ ] Methods: Estimate Target Intercept

% This function computes the intercept position, velocity, and time for a spacecraft
% to intercept a target body. The computation involves simulating the orbital
% trajectories of both the spacecraft and the target over a defined planning horizon
% and finding the point of minimum distance between their positions.

function obj = func_estimate_target_intercept_location_time(obj, mission, i_SC)

% Retrieve the current position and velocity of the target body as estimated
% by the spacecraft's software.
this_target_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target(:);
    mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target(:)];

this_time_array = 0:60:obj.time_horizon;

% Get the position of the Sun at each time step in the J2000 frame
% for the defined time array.
Sun_pos_t0_tf = cspice_spkezr('10', mission.true_time.date + this_time_array, 'J2000', 'NONE', '10');

% Simulate the trajectory of the target body using an ODE solver.
% The function `func_orbit_SB_body` models the orbital motion of the body.
[T, X] = ode113(@(t, X) func_orbit_SB_body(t, X, ...
    mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, ...
    Sun_pos_t0_tf, this_time_array), this_time_array, this_target_pos_vel_current, obj.options);

% Store the target body's position and velocity over the simulation period.
target_pos_t0_tf = X';

% Retrieve the current position and velocity of the spacecraft
% as estimated by its onboard software.
this_SC_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position(:);
    mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity(:)];

% Simulate the spacecraft's trajectory over the same time horizon.
[T, X] = ode113(@(t, X) func_orbit_SB_body(t, X, ...
    mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, ...
    Sun_pos_t0_tf, this_time_array), this_time_array, this_SC_pos_vel_current, obj.options);

% Store the spacecraft's position and velocity over the simulation period.
SC_pos_t0_tf = X';

% Compute the Euclidean distance between the spacecraft and the target body
% at each time step. The result is an array of distances.
Distance_array = vecnorm(SC_pos_t0_tf(1:3, :) - target_pos_t0_tf(1:3, :), 2, 1);

% Find the minimum distance and the corresponding index in the time array.
% `min_index` tells us the time step where the closest approach occurs.
[min_distance, min_index] = min(Distance_array);

% Determine the time of interception based on the index of minimum distance.
if min_index == 1
    % Case 1: If the minimum distance occurs at the first time step, it implies
    % that the orbits are diverging (the closest point is now, and they are moving apart).
    obj.time_intercept = mission.true_time.time + obj.time_horizon/5 + ...
        obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff;
elseif (min_index > 1) && (min_index < length(Distance_array))
    % Case 2: The minimum distance occurs somewhere within the planning horizon.
    obj.time_intercept = mission.true_time.time + this_time_array(min_index);
    % If the calculated intercept time is too soon (before certain operational
    % thresholds like `time_horizon_DeltaV`), adjust the intercept time.
    if (obj.time_intercept - mission.true_time.time) < ...
            (obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff)
        obj.time_intercept = mission.true_time.time + obj.time_horizon/5 + ...
            obj.time_horizon_DeltaV + obj.time_horizon_data_cutoff;
    end
else
    % Case 3: If the minimum distance occurs at the last time step, it suggests
    % that the spacecraft and target body are converging slowly but don't intercept
    % within the planning horizon.
    obj.time_intercept = mission.true_time.time + obj.time_horizon;
end

% Record the target body's position and velocity at the time of interception.
% These values correspond to the time step with the minimum distance.
obj.intercept_SB_position = target_pos_t0_tf(1:3, min_index);
obj.intercept_SB_velocity = target_pos_t0_tf(4:6, min_index);

% Record the minimum distance at interception.
obj.intercept_distance = min_distance;
end
