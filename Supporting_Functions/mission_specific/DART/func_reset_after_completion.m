%% [ ] Methods: Reset after Completion

function obj = func_reset_after_completion(obj, mission, i_SC)

% IMPORTANT: Clear all maneuver flags and reset values to zero
obj.desired_DeltaV_achieved = true;
obj.desired_DeltaV_needs_to_be_executed = false;
obj.desired_control_DeltaV = [0 0 0]';
obj.desired_DeltaV_computed = false;
obj.last_time_control = mission.true_time.time;
obj.total_DeltaV_executed = [0 0 0]';

% Reset maneuver tracking
obj.maneuver_start_time = 0;
obj.thruster_fired_successfully = false;
obj.flag_executive = 0;

% Add: Clear pending thruster commands
for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
    if mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.health
        mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.pending_fire = false;
        mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.pending_thrust = 0;
    end
end
end
