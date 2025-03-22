%% [ ] Methods: Optimize Thrusters using KKT Condition

function obj = func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC, desired_torque)


% Optimize the use of micro-thrusters for the desired control torque
if nargin < 4
    % Allows to pass another torque as parameter
    desired_torque = obj.desired_control_torque;
end

%% Step 1: Reset thruster commands
% Reset the commanded thrust and torque for each microthruster to ensure a clean start
func_reset_thrusters(obj,mission, i_SC);

%% Step 2: Retrieve pre-computed optimization data
% Extract optimization data that was initialized during the preparation phase.
A = obj.optim_data.A;                                   % Quadratic cost matrix
b = obj.optim_data.b;                                   % Right-hand side of the cost function
U = obj.optim_data.U;                                   % Upper triangular matrix from LU decomposition
L = obj.optim_data.L;                                   % Lower triangular matrix from LU decomposition
P = obj.optim_data.P;                                   % Permutation matrix from LU decomposition
n = obj.optim_data.n;                                   % Number of optimization variables
N_mt = obj.optim_data.N_mt;                             % Total number of microthrusters
N_mt_opposite = obj.optim_data.N_mt_opposite;           % Number of thruster pairs with opposite torque
N_mt_same = obj.optim_data.N_mt_same;                   % Number of unique torque directions
idxs_num_opposite_torque = obj.optim_data.idxs_num_opposite_torque; % Indices of opposite torque pairs
idxs_bool_same_torque = obj.optim_data.idxs_bool_same_torque;       % Logical array of same torque indices

%% Step 3: Solve the optimization problem using the KKT system
% Formulate the KKT system and solve for the optimal thrust values.
d = desired_torque';                         % Desired control torque for the spacecraft
f = [A'*b; d];                                        % Combine cost and constraint terms into a single vector
xz = U\(L\(P*f));                                 % Solve the KKT system using LU decomposition
x_opposite = xz(1:n);                                   % Extract the solution for thruster pairs

%% Step 4: Undo the torque pair reduction
% Recover the thrust values for all microthrusters from the reduced solution.
x_same = zeros(N_mt_same, 1);                           % Preallocate thrust values for unique torque directions
for i_mt = 1:N_mt_opposite
    idxs_num_i = idxs_num_opposite_torque(i_mt, :);     % Get the indices of the current pair
    x_same(idxs_num_i(1)) = max(0, +x_opposite(i_mt));  % Assign positive thrust to the first thruster
    x_same(idxs_num_i(2)) = max(0, -x_opposite(i_mt));  % Assign negative thrust to the second thruster
end

% Distribute the thrust values among all microthrusters with the same torque
x = zeros(N_mt, 1);                                     % Preallocate full thrust vector
for i_mt = 1:N_mt_same
    idxs_bool_i = idxs_bool_same_torque(i_mt, :);       % Logical indices for thrusters with same torque
    x(idxs_bool_i) = x_same(i_mt) / sum(idxs_bool_i);   % Evenly distribute thrust among the group
end

% Scale all thrusts proportionally if any exceed the maximum limit
if max(x) > max(obj.max_thrust)
    scaling_factor = max(x) / max(obj.max_thrust);
    x = x / scaling_factor;
end

%% Step 5: Assign computed thrust and torque to each microthruster
% Assign the thrust individually and safely
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
    if x(i) >  mission.true_SC{i_SC}.true_SC_micro_thruster{i}.minimum_thrust

        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 1;
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = x(i);
    else
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation   = 0;
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = 0;
    end
end
end