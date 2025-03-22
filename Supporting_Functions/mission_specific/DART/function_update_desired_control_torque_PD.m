%% [ ] Methods: Desired Control Torque PD

function obj = function_update_desired_control_torque_PD(obj, mission, i_SC)

% Parameters
J = mission.true_SC{i_SC}.true_SC_body.total_MI; % [kg m^2] Spacecraft inertia matrix (excluding wheels)

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
Zbeta = zeros(4,3);
Zbeta(1:3,1:3) =  beta_est(4) * eye(3) + skew(beta_est(1:3));
Zbeta(4,  1:3) = -beta_est(1:3)';

delta_theta = ( pinv(Zbeta) * (beta_desired - beta_est)')';


% Calculate control torques
uc = (J * ((Kd * delta_omega') + (Kp * delta_theta')) )';

% Gyroscopic terms
vc = cross(omega_est, (J * omega_est')' ); % meas_rw_momentum

% Total control torque
obj.desired_control_torque = uc + vc;

end

