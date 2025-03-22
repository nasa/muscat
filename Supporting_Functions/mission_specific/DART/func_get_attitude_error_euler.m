%% [ ] Methods: Get Attitude Error Euler

function error = func_get_attitude_error_euler(obj, mission, i_SC)
% This method is used by the orbit control
% Decide which actuator to use
actual_euler = ConvertAttitude(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude'/norm(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude)', 'quaternion','321');
desired_euler = ConvertAttitude(obj.desired_attitude'/norm(obj.desired_attitude)', 'quaternion','321');
error = abs(actual_euler - desired_euler);
end