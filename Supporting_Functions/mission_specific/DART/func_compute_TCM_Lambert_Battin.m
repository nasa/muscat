%% [ ] Methods: Compute TCM using Lambert-Battin method
% Compute Transfer Correction Maneuver (TCM) using Lambert-Battin method

function obj = func_compute_TCM_Lambert_Battin(obj, mission, i_SC)

% Verify fuel availability
func_verify_fuel_availability(obj, mission, i_SC);


% We are solving the Lambert Battin problem using the position
% of the spacecraft after 'time_horizon_DeltaV' has passed.
this_time_array = 0:10:obj.data.time_horizon_DeltaV+obj.data.time_horizon_data_cutoff;
Sun_pos_t0_tf = cspice_spkezr('10', mission.true_time.date + this_time_array, 'J2000', 'NONE', '10');
new_this_SC_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position(:); mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity(:)]; % [km, km/sec]

% Estimated SC orbit
[T,X] = ode113(@(t,X) func_orbit_SB_body(t,X, mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, Sun_pos_t0_tf, this_time_array),this_time_array,new_this_SC_pos_vel_current,obj.data.options);
SC_pos_t_DeltaV = X(end,:)';
r1 = 1e3*SC_pos_t_DeltaV(1:3);  % [m]

% Extract the positions
r2 = obj.data.intercept_SB_position * 1e3; % [m]

% Time of flight [sec]
tof = obj.data.time_intercept - mission.true_time.time - obj.data.time_horizon_DeltaV - obj.data.time_horizon_data_cutoff;

% Safety check: Ensure time of flight is reasonable
if tof <= 0
    warning('Time of flight is negative or zero. Adjusting intercept time.');
    tof = 3600; % Set a default 1 hour time of flight
    obj.data.time_intercept = mission.true_time.time + obj.data.time_horizon_DeltaV + obj.data.time_horizon_data_cutoff + tof;
end

% Solve Lambert's problem
[V1, ~] = LAMBERTBATTIN(r1, r2, 'pro', tof);

% Compute required Delta-V
obj.data.desired_control_DeltaV = V1' - 1e3*SC_pos_t_DeltaV(4:6); % [m/s]

% Now that we have the new deltaV we can update the intercept distance.
% Estimate SB orbit
this_target_pos_vel_current = [mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target(:);
    mission.true_SC{i_SC}.software_SC_estimate_orbit.velocity_target(:)];
[~, X] = ode113(@(t, X) func_orbit_SB_body(t, X, ...
    mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, ...
    Sun_pos_t0_tf, this_time_array), this_time_array, this_target_pos_vel_current, obj.data.options);
SB_pos_t0_tf = X';


% Estimate SC orbit
% We propagate using the time after the deltaV begins (after time_horizon_DeltaV)
new_this_time_array = obj.data.time_horizon_DeltaV+obj.data.time_horizon_data_cutoff : 10 : obj.data.time_intercept - mission.true_time.time; % [sec]
new_this_SC_pos_vel_current = [SC_pos_t_DeltaV(1:3); SC_pos_t_DeltaV(4:6) + (1e-3*obj.data.desired_control_DeltaV)]; % [km, km/sec]
Sun_pos_t0_tf = cspice_spkezr('10', mission.true_time.date + new_this_time_array, 'J2000', 'NONE', '10');

[~,X] = ode113(@(t,X) func_orbit_SB_body(t,X, mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.mu, Sun_pos_t0_tf, new_this_time_array),new_this_time_array,new_this_SC_pos_vel_current,obj.data.options);

SC_pos_tDeltaV_tf = X';

new_intercept_distance = norm(SC_pos_tDeltaV_tf(1:3,end) - SB_pos_t0_tf(1:3,end));
obj.data.desired_intercept_distance = new_intercept_distance; % [km]

obj.data.time_DeltaV = mission.true_time.time + obj.data.time_horizon_DeltaV + obj.data.time_horizon_data_cutoff; % [sec] since t0
obj.data.time_data_cutoff = mission.true_time.time + obj.data.time_horizon_data_cutoff; % [sec] since t0

% Calculate estimated fuel required for the maneuver
% Using the rocket equation: m_fuel = m_sc * (1 - exp(-deltaV/g0/Isp))
% But for small deltaV, we can approximate: m_fuel ≈ m_sc * deltaV/(g0*Isp)
g0 = 9.80665; % m/s^2

% Get average ISP from all available chemical thrusters
total_isp = 0;
num_thrusters = 0;
for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
    if mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.health
        total_isp = total_isp + mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.isp;
        num_thrusters = num_thrusters + 1;
    end
end

% Calculate average ISP if any thruster is healthy
if num_thrusters > 0
    avg_isp = total_isp / num_thrusters;
else
    avg_isp = 200; % Default ISP if no healthy thrusters
    warning('No healthy thrusters available. Using default ISP of 200s.');
end

% Estimate fuel required
sc_mass = mission.true_SC{i_SC}.true_SC_body.total_mass;
deltaV_magnitude = norm(obj.data.desired_control_DeltaV);
obj.data.estimated_fuel_required = sc_mass * deltaV_magnitude / (g0 * avg_isp);

% Check if we have sufficient fuel
total_available_fuel = 0;
if isfield(mission.true_SC{i_SC}, 'true_SC_fuel_tank') && ...
        mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
    for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
        total_available_fuel = total_available_fuel + ...
            mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.instantaneous_fuel_mass;
    end
end

% Set flags based on fuel availability
if total_available_fuel < (obj.data.estimated_fuel_required + obj.data.min_fuel_threshold) && ~mission.true_SC{i_SC}.true_SC_orbit_propagation.flag_SC_crashed
    obj.data.flag_insufficient_fuel = true;
    warning(['Insufficient fuel for maneuver. Required: ', ...
        num2str(obj.data.estimated_fuel_required), ' kg, Available: ', ...
        num2str(total_available_fuel), ' kg']);

    % Don't execute the maneuver if we don't have enough fuel
    obj.data.desired_DeltaV_needs_to_be_executed = false;
    obj.data.desired_DeltaV_computed = false;
else
    obj.data.flag_insufficient_fuel = false;
    obj.data.desired_DeltaV_needs_to_be_executed = true;
    obj.data.desired_DeltaV_computed = true;

end

obj.data.desired_DeltaV_achieved = false;
obj.data.total_DeltaV_executed = zeros(3,1); % [m/sec]
obj.data.desired_attitude_for_DeltaV_achieved = false;
end
