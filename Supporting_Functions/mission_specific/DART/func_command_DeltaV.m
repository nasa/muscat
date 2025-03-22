%% [ ] Methods: Command DeltaV

function obj = func_command_DeltaV(obj, mission, i_SC)
% Initialize desired control thrust to zero
obj.desired_control_thrust = 0;

% Get spacecraft parameters
sc_mass = mission.true_SC{i_SC}.true_SC_body.total_mass;
dt = mission.true_time.time_step;

% Check for healthy thrusters
healthy_thrusters = [];
for i_thruster = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
    if mission.true_SC{i_SC}.true_SC_chemical_thruster(i_thruster).health
        healthy_thrusters = [healthy_thrusters, i_thruster];
    end
end

if isempty(healthy_thrusters)
    warning('No healthy thrusters available for DeltaV execution.');
    return;
end

% Check execution conditions
if (mission.true_time.time < obj.time_DeltaV) || obj.desired_DeltaV_achieved || obj.flag_insufficient_fuel
    return;
end

% Check if thruster is ready for firing
all_thrusters_ready = true;
for i = 1:length(healthy_thrusters)
    if ~mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).func_is_thruster_ready()
        all_thrusters_ready = false;
        break;
    end
end

if ~all_thrusters_ready
    % Thrusters still warming up, wait until next cycle
    return;
end

% Calculate remaining Delta-V
remaining_DeltaV = obj.desired_control_DeltaV - obj.total_DeltaV_executed;
remaining_magnitude = norm(remaining_DeltaV);

if remaining_magnitude > 0
    % Record start time if this is the first thrust of a maneuver
    if obj.maneuver_start_time == 0
        obj.maneuver_start_time = mission.true_time.time;
        obj.thruster_fired_successfully = false;
        disp(['Starting maneuver execution at time ', num2str(mission.true_time.time), ' s']);
    end

    % Get thruster parameters
    max_thrusts = zeros(1, length(healthy_thrusters));
    for i = 1:length(healthy_thrusters)
        max_thrusts(i) = mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).maximum_thrust;
    end

    % Get current attitude matrix
    current_attitude = mission.true_SC{i_SC}.true_SC_adc.attitude;
    R = quaternionToRotationMatrix(current_attitude);

    % Rotate thruster directions to inertial frame
    thruster_body_directions = zeros(3, length(healthy_thrusters));
    for i = 1:length(healthy_thrusters)
        thruster_body_directions(:, i) = mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).orientation';
    end
    orientations = R * thruster_body_directions;
    A = orientations;

    % Calculate maximum available thrust in desired direction
    max_thrust_dir = A * max_thrusts';
    max_DeltaV_per_step = (norm(max_thrust_dir) * dt) / sc_mass;

    % Calculate required thrust scaling factors
    if remaining_magnitude > max_DeltaV_per_step
        % Use maximum thrust for each thruster
        thrust_vector = max_thrusts;
    else
        % Calculate required thrust to achieve the desired delta-V in this step
        required_thrust_magnitude = (remaining_magnitude * sc_mass) / dt;

        % Distribute thrust among thrusters (simplified approach - equal distribution)
        thrust_vector = zeros(1, length(healthy_thrusters));
        for i = 1:length(healthy_thrusters)
            thrust_vector(i) = required_thrust_magnitude / length(healthy_thrusters);

            % Clamp to thruster limits
            thrust_vector(i) = min(max(thrust_vector(i),...
                mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).minimum_thrust), ...
                mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).maximum_thrust);
        end
    end

    % Apply thrust commands to thrusters
    for i = 1:length(healthy_thrusters)
        mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).commanded_thrust = thrust_vector(i);
        mission.true_SC{i_SC}.true_SC_chemical_thruster(healthy_thrusters(i)).flag_executive = 1;

        % Save that we commanded a thrust (needed for completion checks)
        obj.desired_control_thrust = obj.desired_control_thrust + thrust_vector(i);
    end

    % Note: We no longer calculate applied DeltaV here - that's now done in the thruster itself
    % which will directly update obj.total_DeltaV_executed
end
end
