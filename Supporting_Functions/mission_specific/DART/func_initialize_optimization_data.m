%% [ ] Methods: Initialize Optimization of Micro Thrusters

function obj = func_initialize_optimization_data(obj, mission, i_SC)
% Assumption: The thrust of each microthruster can be compensated by the
% thrust of another microthruster with an equal and opposite resultant torque.

%% Step 1: Identify microthrusters with the same resultant torques
% Extract the thruster input matrix, where each row represents a microthruster's contribution.
% Columns correspond to force/torque contributions in each axis.

% Initialize the thruster input array
thruster_input_array = zeros(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster, 3);

% Get the center of mass
center_of_mass = mission.true_SC{1, 1}.true_SC_body.location_COM;

% Loop through each thruster
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
    % Calculate the relative position vector
    relative_position = mission.true_SC{i_SC}.true_SC_micro_thruster{i}.location - center_of_mass;

    % Compute the torque using the cross product
    torque = cross(relative_position, mission.true_SC{i_SC}.true_SC_micro_thruster{i}.orientation);

    % Round the result to three decimal places
    torque_rounded = round(torque, 3);

    % Store the result in the thruster input array
    thruster_input_array(i, :) = torque_rounded;
end

% Number of microthrusters
N_mt = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster;

% Flags to track checked microthrusters
flag_mt_checked = zeros(N_mt, 1);

% Preallocate for storing indices and matrix of same torque directions
idxs_bool_same_torque = []; % Logical array for MTs with the same torque
matrix_same_torque = [];   % Reduced input matrix for MTs with same torque

% Iterate through each microthruster
for i_mt = 1:N_mt
    if flag_mt_checked(i_mt)
        continue; % Skip already processed microthrusters
    end

    % Extract the torque direction vector for the current microthruster
    torque_dir_i = thruster_input_array(i_mt, :);

    % Identify microthrusters with the same torque direction
    idxs_bool = ismember(thruster_input_array, torque_dir_i, 'rows');

    % Append the identified torque direction and indices to the results
    matrix_same_torque = [matrix_same_torque; torque_dir_i];
    idxs_bool_same_torque = [idxs_bool_same_torque; idxs_bool'];

    % Mark these microthrusters as processed
    flag_mt_checked(idxs_bool) = 1;
end

% Convert the logical indices into a boolean array
idxs_bool_same_torque = boolean(idxs_bool_same_torque);

%% Step 2: Identify microthrusters with opposite resultant torques
% Extract the size of the reduced matrix containing same torques
N_mt_same = size(matrix_same_torque, 1);

% Reset flags to track processed microthrusters
flag_mt_checked = zeros(N_mt, 1);

% Preallocate for storing indices and matrix of opposite torque directions
idxs_num_opposite_torque = []; % Numerical indices for MT pairs with opposite torque
matrix_opposite_torque = [];  % Reduced input matrix for opposite torques

% Iterate through the reduced set of same torque microthrusters
for i_mt = 1:N_mt_same
    if flag_mt_checked(i_mt)
        continue; % Skip already processed microthrusters
    end

    % Extract the torque direction vector for the current microthruster
    torque_dir_i = matrix_same_torque(i_mt, :);

    % Identify microthrusters with opposite torque direction
    idxs_bool = ismember(matrix_same_torque, -torque_dir_i, 'rows');

    % Ensure there is exactly one matching opposite torque
    assert(sum(idxs_bool) == 1, 'There must be a unique opposite torque match.');

    % Find the index of the matching opposite torque
    j_mt = find(idxs_bool);

    % Append the torque direction and indices to the results
    matrix_opposite_torque = [matrix_opposite_torque; matrix_same_torque(i_mt, :)];
    idxs_num_opposite_torque = [idxs_num_opposite_torque; [i_mt, j_mt]];

    % Mark these microthrusters as processed
    flag_mt_checked([i_mt, j_mt]) = 1;
end

%% Step 3: Formulate the optimization problem

% The goal is to distribute thrusts across the
% microthrusters such that the net torque matches the desired control torque
% This is a least-squares problem

% The optimization minimizes ||Ax - b||_2^2 subject to Cx = d.

% Number of microthrusters with opposite torques
N_mt_opposite = size(matrix_opposite_torque, 1);

% Problem dimensions
n = N_mt_opposite; % Number of variables (thruster pairs)
p = 3;             % Number of constraints (force/torque balance in 3D)

% Constraint matrix: Each column corresponds to the torque contribution of a thruster pair
C = matrix_opposite_torque';

% Quadratic cost function: Ax approximates b, initialized as identity for simplicity
A = eye(n);
b = zeros(n, 1);

% Form the Karush-Kuhn-Tucker (KKT) system
% E = [A'A  C';
%      C    O]
O = zeros(p, p); % Zero block matrix for constraints
E = [A' * A, C';
    C,      O];

% Perform LU decomposition of the KKT matrix for efficient solution
[L, U, P] = lu(E);

%% Step 4: Store optimization data in obj
obj.optim_data.A = A;
obj.optim_data.b = b;
obj.optim_data.U = U;
obj.optim_data.L = L;
obj.optim_data.P = P;
obj.optim_data.n = n;
obj.optim_data.N_mt = N_mt;
obj.optim_data.N_mt_opposite = N_mt_opposite;
obj.optim_data.N_mt_same = N_mt_same;
obj.optim_data.idxs_num_opposite_torque = idxs_num_opposite_torque;
obj.optim_data.idxs_bool_same_torque = idxs_bool_same_torque;
end
