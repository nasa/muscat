%% [ ] Methods: Estimate Attitude using Kalman Filter (KF)
% Use KF

function obj = func_update_software_SC_estimate_attitude_KF(obj, mission, i_SC)

obj.instantaneous_data_generated_per_sample = (1e-3)*16*20; % [kb] i.e. 20 values per sample, each of 16-bit depth

% Take in measurements from SS, ST, IMU




% Perform KF



% Output Data

obj.attitude = mission.true_SC{i_SC}.true_SC_adc.attitude; % [quaternion]
obj.attitude_uncertainty = zeros(1,4);

obj.angular_velocity = mission.true_SC{i_SC}.true_SC_adc.angular_velocity; % [rad/sec]
obj.angular_velocity_uncertainty = zeros(1,3);

obj.dot_angular_velocity = mission.true_SC{i_SC}.true_SC_adc.dot_angular_velocity; % [rad/sec^2]
obj.dot_angular_velocity_uncertainty = zeros(1,3);


end
