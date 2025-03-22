%% [ ] Methods: Actuator Selection DART

function obj = func_actuator_selection_DART(obj, mission, i_SC)

% Get current attitude error
attitude_error = func_get_attitude_error(obj, mission, i_SC);

% STEP 0: Check if wheels need desaturation
if func_is_desaturation_needed(obj, mission, i_SC)
    % If wheels are already saturated, handle desaturation
    disp('Performing wheel desaturation');
    func_handle_desaturation(obj, mission, i_SC);
    return;
end

% STEP 1: Check if attitude error is too large for wheels
if norm(attitude_error) > obj.reaction_wheel_attitude_control_threshold
    % For large angle corrections, always use thrusters - simple rule
    func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC);
    return;
end

% STEP 2: Check if torque request exceeds wheel capability
if norm(obj.desired_control_torque) > obj.max_rw_torque
    % If torque is too high for wheels, use thrusters
    func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC);
    return;
end

% STEP 3: Check for predicted wheel saturation (simplified)
[will_saturate_quickly, ~] = func_predict_wheel_saturation(obj, mission, i_SC);
if will_saturate_quickly
    % If wheels will saturate soon, use thrusters
    disp('Using thrusters due to predicted wheel saturation');
    func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC);
    return;
end


% STEP 4: If we reached here, use reaction wheels (all conditions satisfied)
func_reset_thrusters(obj, mission, i_SC);
func_compute_rw_command_pinv(obj, mission, i_SC);
end