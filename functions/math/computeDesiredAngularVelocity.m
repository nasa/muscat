function omega = computeDesiredAngularVelocity(q0, qf, deltaTime)
% Computes the difference quaternion
delta_q = quatmultiply(qf, quatconj(q0));

% Compute the quaternion at the next timestep using SLERP
% (assuming q0 and qf are unit quaternions for simplicity)
t = deltaTime; % assuming T=1 for simplicity, otherwise replace deltaTime with deltaTime/T
theta = acos(dot(q0, qf)); % angle between q0 and qf
slerp_q = (sin((1-t)*theta)/sin(theta)) * q0 + (sin(t*theta)/sin(theta)) * qf;

% Compute the desired angular velocity
omega_quat = 2 * quatmultiply(quatconj(slerp_q), delta_q);
omega = omega_quat(1:3) / deltaTime; % Only the vector part represents the angular velocity
end
