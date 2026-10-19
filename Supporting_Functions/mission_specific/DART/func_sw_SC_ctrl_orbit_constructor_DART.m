% [ ] Methods: Additional Constructor for Control Orbit for DART

function obj = func_sw_SC_ctrl_orbit_constructor_DART(obj, init_data, mission, i_SC)

obj.data.last_time_control = 0; % [s]

% Maneuver tracking
obj.data.maneuver_start_time = 0; % [s] Time when current maneuver began execution
obj.data.thruster_fired_successfully = false; % [bool] Flag to track if thruster actually fired

% Delta-V control parameters
obj.data.desired_control_DeltaV = zeros(3, 1); % [m/s]: Desired Delta-V vector
obj.data.total_DeltaV_executed = zeros(3, 1); % [m/s]: Accumulated executed Delta-V
obj.data.threshold_minimum_deltaV = 0.01; % [m/s] - the minimum deltaV that is worth executing, usually 0.001 m/s

% Thrust control parameters
obj.data.thrust_quantization = 0; % [N]: Quantization of thrust commands from hardware
if isfield(init_data, 'thrust_quantization')
    obj.data.thrust_quantization = init_data.thrust_quantization;
end
obj.data.desired_control_thrust = 0; % [N]: Desired thrust for trajectory control

% Time horizons and intercept data
obj.data.time_horizon = 10 * 24 * 60 * 60; % [sec]: Time horizon for trajectory planning. Default planning horizon (10 days)
obj.data.time_intercept = 0; % [sec]: Time of intercept with target
obj.data.time_horizon_DeltaV = 30 * 60; % [sec]: Time horizon for Delta-V execution. % Default Delta-V planning horizon (30 minutes)
obj.data.time_DeltaV = 0; % [sec]: Time of Delta-V execution
obj.data.time_horizon_data_cutoff = 0; % [sec]: Time cutoff for data integration
obj.data.time_data_cutoff = 0; % [sec]: Data cutoff time

% Intercept and trajectory details
obj.data.intercept_SB_position = zeros(3,1); % [km]: Intercept position relative to small body
obj.data.intercept_SB_velocity = zeros(3,1); % [km/s]: Intercept velocity relative to small body
obj.data.intercept_distance = 0; % [km]: Distance at intercept
obj.data.desired_intercept_distance = 0; % [km]: Desired distance at intercept

% Options for numerical integrations
obj.data.options = odeset('RelTol', 1e-14, 'AbsTol', 1e-14); % [struct]: Options for ODE solvers

% Flags and execution controls
obj.data.desired_DeltaV_needs_to_be_executed = false; % [boolean]: Whether Delta-V needs execution
obj.data.desired_DeltaV_computed = false; % [boolean]: Whether Delta-V is computed
obj.data.desired_attitude_for_DeltaV_achieved = false; % [boolean]: Attitude flag for Delta-V execution
obj.data.desired_DeltaV_achieved = false; % [boolean]: Whether Delta-V has been achieved

% Fuel management properties
obj.data.min_fuel_threshold = 0.1; % [kg]: Minimum fuel threshold to allow maneuvers
obj.data.estimated_fuel_required = 0; % [kg]: Estimated fuel required for maneuver
obj.data.flag_insufficient_fuel = false; % [boolean]: Flag indicating insufficient fuel


% Additional Storage Variables
obj.store.intercept_distance = zeros(mission.storage.num_storage_steps,length(obj.data.intercept_distance)); 
obj.store.desired_DeltaV_computed = zeros(mission.storage.num_storage_steps,length(obj.data.desired_DeltaV_computed));
obj.store.desired_attitude_for_DeltaV_achieved = zeros(mission.storage.num_storage_steps,length(obj.data.desired_attitude_for_DeltaV_achieved));

obj.store.desired_control_DeltaV = zeros(mission.storage.num_storage_steps, length(obj.data.desired_control_DeltaV));
obj.store.total_DeltaV_executed = zeros(mission.storage.num_storage_steps, length(obj.data.total_DeltaV_executed));
obj.store.flag_insufficient_fuel = zeros(mission.storage.num_storage_steps, length(obj.data.flag_insufficient_fuel));
obj.store.estimated_fuel_required = zeros(mission.storage.num_storage_steps, length(obj.data.estimated_fuel_required));



end