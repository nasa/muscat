function Xdot = func_ode_attitude_dynamics(~, X, mission, i_SC)
% Spacecraft Attitude Dynamics Implementation
% This function implements the coupled differential equations for quaternion 
% kinematics and rigid body dynamics of a spacecraft.

% This is an optimized version that does not create its own variable when
% possible. 

%
% Inputs:
%   t        - Time [s]
%   X        - State vector [quaternion; angular velocity] (7x1)
%   mission  - Mission parameters struct
%   i_SC     - Spacecraft index
%
% Outputs:
%   Xdot     - State derivative vector [quaternion_dot; omega_dot] (7x1)

% Inertia Matrix [kg⋅m²]

% Control and disturbance torques [N⋅m]
% Converting to column vectors (3x1)

% Reaction wheel angular momentum [N⋅m⋅s]
% Use the precalculated total reaction wheel momentum from ADC

% Extract state vector components
beta = X(1:4);    % Quaternion [β₁; β₂; β₃; β₄] where β₄ is scalar part
omega = X(5:7);   % Angular velocity vector [ωx; ωy; ωz] [rad/s]

% Quaternion kinematics: β̇ = 1/2 Ξ(β)ω
% Where Ξ(β) is the 4x3 quaternion mapping matrix:
% Ξ(β) = [β₄I₃×₃ + β̃; -β₁₂₃ᵀ]
% β̃ is the skew-symmetric matrix of the vector part of quaternion
Zbeta = zeros(4,3);
Zbeta(1:3,1:3) = beta(4) * eye(3) + skew(beta(1:3));  % β₄I₃×₃ + β̃
Zbeta(4, 1:3) = -beta(1:3)';                          % -β₁₂₃ᵀ

% Compute quaternion derivative
beta_dot = 0.5 * Zbeta * omega;

% Euler's equation for rigid body dynamics:
% Jω̇ = u_control - ω × (Jω + h_rw) + τ_dist
% where h_rw is reaction wheel angular momentum
% and τ_dist is disturbance torque

omega_dot = mission.true_SC{i_SC}.true_SC_body.total_MI \ (mission.true_SC{i_SC}.true_SC_adc.control_torque' - ...
    cross(omega, mission.true_SC{i_SC}.true_SC_body.total_MI * omega +...
    mission.true_SC{i_SC}.true_SC_adc.total_wheel_momentum) + ...
    mission.true_SC{i_SC}.true_SC_adc.disturbance_torque');



% Combine state derivatives
Xdot = [beta_dot; omega_dot];
end


