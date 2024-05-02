function Xdot = func_attitude_Quaternion_spacecraft_v2(t,X,J,u_control,disturbance_torque, true_rw_momentum)

% SC Quaternion
beta = X(1:4);

% SC Omega
omega = X(5:7);

% beta dot
Zbeta = zeros(4,3);
Zbeta(1:3,1:3) = beta(4) * eye(3) + skew(beta(1:3));
Zbeta(4,  1:3) = -beta(1:3)';

beta_dot = 0.5 * Zbeta * omega;

% omega dot
omega_dot = J \ (u_control - cross(omega, J * omega + true_rw_momentum) + disturbance_torque);

% Put it together
Xdot = [beta_dot; omega_dot];

end