%% [ ] Methods: Control Attitude Oracle
% Oracle directly moves the SC's attitude and rotation matrix, without the control actuators
function obj = func_update_software_SC_control_attitude_Oracle(obj, mission, i_SC)
% Oracle Move!
mission.true_SC{i_SC}.true_SC_adc.attitude = obj.desired_attitude; % [quaternion]
mission.true_SC{i_SC}.true_SC_adc.angular_velocity = obj.desired_angular_velocity; % [rad/sec]

% Compute Rotation Matrix
func_update_true_SC_ADC_rotation_matrix(mission.true_SC{i_SC}.true_SC_adc);
end