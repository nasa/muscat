%% [ ] Methods: Optimize RWA using KKT Condition

function obj = func_compute_rw_command_kkt(obj, mission, i_SC)
% Compute reaction wheel commands to achieve desired control torque

if norm(obj.desired_control_torque) > 0

    % Get the number of reaction wheels
    N = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel;

    % Compute required wheel accelerations using optimization
    n = 4 * N; % number of variables
    p = 3 * (N + 1); % number of constraints

    % Constraint vector initialization
    d = zeros(p, 1);
    for i = 1:N
        idx1 = (i-1)*3 + 1;
        idx2 = (i-1)*3 + 3;
        % Assuming dot_angular_velocity is correctly accessed
        d(idx1:idx2, 1) = mission.true_SC{i_SC}.software_SC_estimate_attitude.dot_angular_velocity';
    end

    % Scale desired torque vector
    torque_desired = obj.desired_control_torque;
    max_torque = mission.true_SC{i_SC}.true_SC_reaction_wheel{1}.max_torque; % Assuming all RWs have the same max torque
    if max_torque < Inf
        factor = min(1, (max_torque * 1) / norm(torque_desired)); % envelope_ratio assumed as 1
    else
        factor = 1;
    end
    d(3*N + 1:p, 1) = factor * torque_desired;

    % Objective matrix setup
    A = diag([zeros(3*N, 1); ones(n - 3*N, 1)]);
    b = zeros(n, 1);

    % Constraint matrix setup
    C = zeros(p, n);
    for i = 1:N
        idx1 = (i-1)*3 + 1;
        idx2 = (i-1)*3 + 3;
        % Assuming orientation gives rotation matrix R (transposed for usage here)
        R = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.orientation';

        % Moment of inertia handling
        moment_of_inertia = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.moment_of_inertia;
        if any(diag(moment_of_inertia) == 0)
            % Regularization for singular inertia matrices
            moment_of_inertia = moment_of_inertia + eye(size(moment_of_inertia)) * 1e-6;
        end

        C(idx1:idx2, idx1:idx2) = inv(moment_of_inertia);
        C(idx1:idx2, 3*N + i) = -R(:, 1);
        C(3*N + 1:p, idx1:idx2) = eye(3, 3);
    end

    % Solve optimization problem (constrained least squares)
    % Form Karush-Kuhn-Tucker (KKT) system
    O = zeros(p, p);
    E = [A'*A, C'; C, O];
    E = E + eye(size(E)) * 1e-6; % Regularization for numerical stability
    [L, U, P] = lu(E);

    f = [A'*b; d];
    xz = U\(L\(P*f));
    x = xz(1:n);

    % Results storing
    for i = 1:N
        idx1 = (i-1)*3 + 1;
        idx2 = (i-1)*3 + 3;
        % Assign computed torque and angular acceleration
        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_torque = x(idx1:idx2);
        % disp("RWA Commanded Torque")
        % disp(x(idx1:idx2))
        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.commanded_angular_acceleration = x(3*N + i, 1);
        % disp("RWA Commanded Angular Acceleration")
        % disp(x(3*N + i, 1))

    end

end
end
