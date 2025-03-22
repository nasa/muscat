%% [ ] Methods: Control Attitude DART

function obj = func_update_software_SC_control_attitude_DART(obj, mission, i_SC)

obj.instantaneous_data_generated_per_sample = (1e-3)*8*7; % [kb] i.e. 7 Bytes per sample

% Navigation and Guidance
% Set pointing vectors based on SC mode
obj = func_set_pointing_vectors_DART(obj, mission, i_SC);

% Compute Desired Attitude using Array Search
obj = func_compute_desired_attitude_Array_Search(obj);
obj.desired_angular_velocity = zeros(1,3); % [rad/sec]

% Control
switch obj.mode_software_SC_control_attitude_selector

    case 'DART Oracle'
        % Use Oracle
        obj = func_update_software_SC_control_attitude_Oracle(obj, mission, i_SC);

    case 'DART Control Asymptotically Stable send to ADC directly'
        obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC);
        mission.true_SC{i_SC}.true_SC_adc.control_torque = obj.desired_control_torque';

    case 'DART Control Asymptotically Stable send to actuators'
        obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC);
        obj = func_actuator_selection_DART(obj, mission, i_SC);

    case 'DART Control PD'
        obj = function_update_desired_control_torque_PD(obj, mission, i_SC);
        obj = func_actuator_selection_DART(obj, mission, i_SC);

    case 'DART Control Asymptotically Stable send to thrusters'
        obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC);
        func_decompose_control_torque_into_thrusters_optimization_kkt(obj,mission, i_SC);

    otherwise
        error('DART Attitude Control mode not defined!')
end
end
