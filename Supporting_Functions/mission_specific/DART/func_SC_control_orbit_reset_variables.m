%% [ ] Methods: Reset Variables

function obj = func_SC_control_orbit_reset_variables(obj)

% Reset time and control flags
obj.flag_executive = 0;
%obj.desired_DeltaV_needs_to_be_executed = false;
%obj.desired_DeltaV_computed = false;
%obj.desired_attitude_for_DeltaV_achieved = 0;
%obj.desired_DeltaV_achieved = false;

% Reset Delta-V and thrust parameters
% obj.desired_control_DeltaV = zeros(3, 1);
%obj.total_DeltaV_executed = zeros(3, 1);
obj.desired_control_thrust = 0;

% Reset intercept and trajectory details
%obj.intercept_SB_position = zeros(3, 1);
%obj.intercept_SB_velocity = zeros(3, 1);
%obj.intercept_distance = 0;
%obj.desired_intercept_distance = 0;
%obj.time_intercept = 0;
%obj.time_DeltaV = 0;
%obj.time_data_cutoff = 0;

% Reset fuel management
obj.flag_insufficient_fuel = false;
end
