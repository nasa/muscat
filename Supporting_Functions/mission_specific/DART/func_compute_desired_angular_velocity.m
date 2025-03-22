%% [ ] Methods: Desired Angular Velocity

function obj = func_compute_desired_angular_velocity(obj, mission)

if mission.storage.time_step_storage_attitude ~= mission.true_time.time_step_attitude
    warning('Compute desired angular velocity assumes all attitude data is stored!')
end

if norm(obj.data.prev_desired_attitude) == 0
    % Do nothing! Not a quaternion.
    obj.desired_angular_velocity = zeros(1,3); % [rad/sec]

else

    min_num_prev_desired_attitude = min(mission.storage.k_storage_attitude-1,obj.data.num_prev_desired_attitude);

    angular_velocity_array = zeros(min_num_prev_desired_attitude-2,3);

    i = 1;
    dt = mission.true_time.time_step_attitude; % [sec] Attitude time step
    q1 = obj.data.prev_desired_attitude;      % Quaternion at time t(i)
    q2 = obj.desired_attitude;    % Quaternion at time t(i+1)

    % Compute quaternion difference (quaternion multiplication)
    dq = func_quaternion_multiply(q2, func_quaternion_conjugate(q1));

    % Extract vector part (first 3 elements) to get angular velocity
    angular_velocity_array(i,:) = 2 * dq(1:3) / dt; % [rad/sec]

    for i=1:1:min_num_prev_desired_attitude-2

        q1 = obj.store.desired_attitude(mission.storage.k_storage_attitude - i - 1,:);      % Quaternion at time t(i)
        q2 = obj.store.desired_attitude(mission.storage.k_storage_attitude - i,:);    % Quaternion at time t(i+1)

        % Compute quaternion difference (quaternion multiplication)
        dq = func_quaternion_multiply(q2, func_quaternion_conjugate(q1));

        % Extract vector part (first 3 elements) to get angular velocity
        angular_velocity_array(i+1,:) = 2 * dq(1:3) / dt; % [rad/sec]

    end


    obj.desired_angular_velocity = mean(angular_velocity_array,1); % [rad/sec]

end

% reset prev desired attitude
obj.data.prev_desired_attitude = obj.desired_attitude; % [quaternion]

end
