%% [ ] Methods: Compute Reaction Wheels Command using pseudo inverse

function func_compute_rw_command_pinv(obj, mission, i_SC)
% Compute reaction wheel commands to achieve desired control torque

% Use scaled torque for wheel commands
% Apply negative of the control torque, because the torque commanded to the
% wheels must be equal and opposite to achieve the desired torque on the
% spacecraft body
wheel_accelerations = obj.pinv_reaction_wheel_contribution_matrix * -obj.desired_control_torque';

% Apply momentum management to avoid direction reversals
wheel_accelerations = func_manage_wheel_momentum(obj, mission, i_SC, wheel_accelerations);

% Apply commands to reaction wheels
func_apply_reaction_wheel_commands(obj, mission, i_SC, wheel_accelerations);
end
