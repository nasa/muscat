%% [ ] Methods: Handle wheel desaturation

function obj = func_handle_desaturation(obj, mission, i_SC)
% Perform wheel desaturation if required

% Define desaturation threshold (in radians per second)
% Change from 10 RPM to 100 RPM (~10.47 rad/s)
desaturation_threshold = 100 * 2 * pi / 60; % 100 RPM

wheels = mission.true_SC{i_SC}.true_SC_reaction_wheel;

% Check if desaturation is complete for all wheels
all_wheels_safe = true;
was_in_desaturation = obj.desaturation_procedure;

for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    wheel = wheels{i};
    if abs(wheel.angular_velocity) > desaturation_threshold
        all_wheels_safe = false;
        break;
    end
end

if all_wheels_safe
    if was_in_desaturation
        % Just exited desaturation mode - update capabilities
        obj.desaturation_procedure = false;
        obj = func_update_torque_capabilities(obj, mission, i_SC);
    end
    return;
else
    if ~was_in_desaturation
        % Just entered desaturation mode - update capabilities
        obj.desaturation_procedure = true;
        obj = func_update_torque_capabilities(obj, mission, i_SC);
    else
        obj.desaturation_procedure = true;
    end
end

wheel_torque = zeros(1,3);

% Desaturate individual wheels and compute thruster torques
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    wheel = wheels{i};

    % Calculate command to reduce wheel velocity - typically 10% per time step
    cmd = -wheel.angular_velocity * 0.1; %

    % Apply safety limit to desaturation command
    if abs(cmd) > wheel.maximum_acceleration
        % Limit the commanded acceleration to maximum safe value
        cmd = sign(cmd) * wheel.maximum_acceleration;
    end

    wheel.commanded_angular_acceleration = cmd;
    wheel.flag_executive = true;

    % Compute the resulting torque from the reaction wheel
    wheel_torque = wheel_torque + (wheel.moment_of_inertia * cmd * wheel.orientation);
end

% Compute the required thruster torque to compensate
residual_torque = obj.desired_control_torque - wheel_torque;

% Optimize thruster operation
func_decompose_control_torque_into_thrusters_optimization_kkt(obj, mission, i_SC, residual_torque);

end
