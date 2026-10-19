function Xdot = func_orbit_inertial_static_relative_Target_v2(t, X, mission, i_SC, index_relative_target, flag_stm)

if nargin < 6
    flag_stm = false;
end

% Initialize Acc with Control Force
acc = (1/mission.true_SC{i_SC}.true_SC_body.total_mass) * (1e-3) * mission.true_SC{i_SC}.true_SC_orbit_propagation.control_force'; % [km / sec^2] in (3,1) format

% Add disturbance forces to Acc
acc = acc + (1/mission.true_SC{i_SC}.true_SC_body.total_mass) * (1e-3) * mission.true_SC{i_SC}.true_SC_orbit_propagation.disturbance_force'; % [km / sec^2]  in (3,1) format

% SC Position (In Relative Frame)
r_I = X(1:3);
Rot = mission.true_target{index_relative_target}.rotation_matrix;
r_R = Rot * r_I;

% Add forces from Target bodies
if flag_stm
    [a_R, ~, ~, dadr] = mission.true_target{index_relative_target}.gravity_field.Acceleration(r_R, mission.true_target{index_relative_target}.gravity_degree_harmonics);

    D = zeros(6,6);
    D(1:3,4:6) = eye(3);
    D(4:6,1:3) = Rot' * dadr * Rot;
    
    Phi = reshape(X(7:42), [6,6]);
    dPhidt = D * Phi;
else
    [a_R, ~, ~] = mission.true_target{index_relative_target}.gravity_field.Acceleration(r_R, mission.true_target{index_relative_target}.gravity_degree_harmonics);
end

a_I = Rot' * a_R;
acc = acc + a_I;


% Add forces from Solar System bodies
for i_SS_body = 1:1:mission.true_solar_system.num_SS_body

    % R_SS_Body = (interp1(mission.true_time.time_position_array, mission.true_solar_system.SS_body{i_SS_body}.position_array, t))'; % [km]

    % First term: Gravity force from all gravitational bodies 
    body_pos_vel_this_time = cspice_spkezr(mission.true_solar_system.SS_body{i_SS_body}.spice_name, mission.true_time.prev_date + t,'J2000','NONE',mission.true_target{index_relative_target}.spice_name);
    R_this_Target = body_pos_vel_this_time(1:3,1); % [km]
    
    R_SC_SS_body = r_I - R_this_Target; % [km]

    distance_SC_SS_body = norm(R_SC_SS_body); % [km]

    acc = acc - (mission.true_solar_system.SS_body{i_SS_body}.mu * R_SC_SS_body / distance_SC_SS_body^3); % [km / sec^2] in (3,1) format

    % Second term: Force from other non-Target gravitational bodies
    R_SS_Body_to_Target = - R_this_Target; % [km]
    dist_SS_Body_to_Target = norm(R_SS_Body_to_Target); % [km]

    acc = acc + (mission.true_solar_system.SS_body{i_SS_body}.mu * R_SS_Body_to_Target / dist_SS_Body_to_Target^3); % [km/sec^2]

end


% Add forces from Target bodies
for i_target = 1:1:mission.num_target

    if i_target ~= index_relative_target

        % First term: Gravity force from all gravitational bodies
        R_this_Target = (interp1(mission.true_time.time_position_array, mission.true_target{i_target}.position_array, t))'; % [km]

        R_SC_this_Target = r_I - R_this_Target; % [km]

        distance_SC_this_Target = norm(R_SC_this_Target); % [km]

        acc = acc - (mission.true_target{i_target}.mu * R_SC_this_Target / distance_SC_this_Target^3); % [km / sec^2] in (3,1) format

        % Second term: Force from other non-Sun gravitational bodies
        R_this_Target_to_Target = - R_this_Target; % [km]
        dist_this_Target_to_Target = norm(R_this_Target_to_Target); % [km]

        acc = acc + (mission.true_target{i_target}.mu * R_this_Target_to_Target / dist_this_Target_to_Target^3); % [km/sec^2]

    end

end




% Create Xdot
% x_dot=[ X(4:6);
%         a_I + (1e-3*mission.true_SC{i_SC}.true_SC_orbit_propagation.control_force'/mission.true_SC{i_SC}.true_SC_body.total_mass) + (1e-3*mission.true_SC{i_SC}.true_SC_orbit_propagation.disturbance_force'/mission.true_SC{i_SC}.true_SC_body.total_mass)];
Xdot=[X(4:6); acc];

if flag_stm
    x_dot = [x_dot; reshape(dPhidt, [36,1])];
end

end
