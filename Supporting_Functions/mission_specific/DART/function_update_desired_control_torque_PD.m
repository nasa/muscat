%% [ ] Methods: Desired Control Torque PD

function obj = function_update_desired_control_torque_PD(obj, mission, i_SC)

% Parameters
J = mission.true_SC{i_SC}.software_SC_executive.data.inertia_BF; % [kg-m^2] Spacecraft inertia matrix

Kp = eye(3) * obj.data.control_gain(1);
Kd = eye(3) * obj.data.control_gain(2);

% Estimated and final values
omega_est = mission.true_SC{i_SC}.software_SC_estimate_attitude.angular_velocity; % [rad/sec]
omega_desired = obj.desired_angular_velocity; % [rad/sec]
delta_omega = omega_desired - omega_est;


beta_est = func_quaternion_properize(mission.true_SC{i_SC}.software_SC_estimate_attitude.attitude); % [quaternion]
beta_desired = func_quaternion_properize(obj.desired_attitude); % [quaternion]

% Calculate Delta q and Delta theta
% (this doesnt work since norm(delta_beta) = 1 even when error is 0)
%             delta_beta = func_quaternion_multiply(beta_desired, func_quaternion_conjugate(beta_est));
%             delta_beta = func_quaternion_properize(delta_beta);
%             delta_theta = 2 * delta_beta(1:3); % Assuming small angle approximations

% New addition
% Zbeta = zeros(4,3);
% Zbeta(1:3,1:3) =  beta_est(4) * eye(3) + skew(beta_est(1:3));
% Zbeta(4,  1:3) = -beta_est(1:3)';
% 
% 
% if func_error_angle_between_quaternions(beta_desired, beta_est) < 4 % [rad]
%     delta_theta = ( pinv(Zbeta) * (beta_desired - beta_est)')';
% else
%     % Use shadow desired quaternion
%     delta_theta = ( pinv(Zbeta) * (-beta_desired - beta_est)')';
% end

% Anglular error between quaternions
delta_theta = q2error(beta_desired, beta_est, 1);

% Calculate feedback control torques
ufb = (J * ((Kd * delta_omega') + (Kp * delta_theta')) )';

% Gyroscopic terms
vc = cross(omega_est, (J * omega_est')' ); % meas_rw_momentum

% Feed-forward commanded acceleration
uff = (J*obj.desired_angular_accel(:)).';

% Compute Feed-forward disturbance torques
uff_dist = func_compute_EP_dist_torque_IBEAM(mission.true_SC{i_SC});
mission.true_SC{i_SC}.software_SC_control_orbit.data.ep_disturbance_torque_BF = uff_dist;

% Total control torque
obj.desired_control_torque = ufb + uff - uff_dist + vc;
% obj.desired_control_torque = ufb + uff + vc;

end

