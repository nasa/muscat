%% [ ] Methods: Optimize Micro Thrusters using Pseudo Inverse

function obj = func_optimize_thruster_pinv(obj, mission, i_SC, desired_torque)

% Optimize the use of micro-thrusters for the desired control torque
if nargin < 4
    % Allows to pass another torque as parameter
    desired_torque = obj.desired_control_torque';
end

% Solve for thrusts using linear least squares
thrusts = pinv(obj.thruster_contribution_matix) * desired_torque;

% Apply thrust min limit
for i=1:length(thrusts)
    if(thrusts(i) < mission.true_SC{i_SC}.true_SC_micro_thruster{i}.minimum_thrust)
        if (rem(i, 2) == 0)
            % Add on the next one
            thrusts(i-1) = thrusts(i-1)+abs(thrusts(i));
        else
            thrusts(i+1) = thrusts(i+1)+abs(thrusts(i));
        end
        thrusts(i) = 0;
    end
end

% Scale all thrusts proportionally if any exceed the maximum limit
if max(thrusts) > max(obj.max_thrust)
    scaling_factor = max(thrusts) / max(obj.max_thrust);
    thrusts = thrusts / scaling_factor;
end


% Assign the thrust individually
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
    if thrusts(i) > mission.true_SC{i_SC}.true_SC_micro_thruster{i}.minimum_thrust
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 1;
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = thrusts(i);
    else
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.command_actuation = 0;
        mission.true_SC{i_SC}.true_SC_micro_thruster{i}.commanded_thrust = 0;
    end
end
end