%% [ ] Methods: Desired Control Torque Asymptotically Stable

function obj = function_update_desired_control_torque_asymptotically_stable(obj, mission, i_SC)

omega_est = mission.true_SC{i_SC}.software_SC_estimate_attitude.angular_velocity; % [rad/sec]
omega_desired = obj.desired_angular_velocity; % [rad/sec]
beta_est = func_quaternion_properize(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude); % [quaternion]
beta_desired = func_quaternion_properize(obj.desired_attitude); % [quaternion]

Zbeta = zeros(4,3);
Zbeta(1:3,1:3) =  beta_est(4) * eye(3) + skew(beta_est(1:3));
Zbeta(4,  1:3) = -beta_est(1:3)';

% output desired control torque
omega_r = omega_desired + ( pinv(0.5 * Zbeta) * obj.data.control_gain(2) * (beta_desired - beta_est)' )';
obj.desired_control_torque = - obj.data.control_gain(1)*(omega_est - omega_r);

end
