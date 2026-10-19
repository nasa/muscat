%% [ ] Methods: Reset after Completion

function obj = func_reset_after_completion(obj, mission, i_SC)

% IMPORTANT: Clear all maneuver flags and reset values to zero
obj.data.desired_DeltaV_achieved = true;
obj.data.desired_DeltaV_needs_to_be_executed = false;
obj.data.desired_control_DeltaV = [0 0 0]';
obj.data.desired_DeltaV_computed = false;
obj.data.last_time_control = mission.true_time.time;
obj.data.total_DeltaV_executed = [0 0 0]';

% Reset maneuver tracking
obj.data.maneuver_start_time = 0;
obj.data.thruster_fired_successfully = false;
obj.data.flag_executive = 0;

% Add: Clear pending thruster commands
for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
    if mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.health
        mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.pending_fire = false;
        mission.true_SC{i_SC}.true_SC_chemical_thruster{i_thruster}.pending_thrust = 0;
    end
end
end
