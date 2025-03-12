function Xdot = func_ode_orbit_inertial_absolute_dynamics(t, X, mission, i_SC)

% SC Position
R_SC = X(1:3); % [km]

% Initialize Acc with Control Force
acc = (1/mission.true_SC{i_SC}.true_SC_body.total_mass) * (1e-3) * mission.true_SC{i_SC}.true_SC_navigation.control_force'; % [km / sec^2] in (3,1) format

% Add disturbance forces to Acc
acc = acc + (1/mission.true_SC{i_SC}.true_SC_body.total_mass) * (1e-3) * mission.true_SC{i_SC}.true_SC_navigation.disturbance_force'; % [km / sec^2]  in (3,1) format

% Add forces from Solar System bodies
for i = 1:1:mission.true_solar_system.num_SS_body

    R_Body = (interp1(mission.true_time.time_position_array, mission.true_solar_system.SS_body{i}.position_array, t))'; % [km]

    R_SC_body = R_SC - R_Body; % [km]

    distance_SC_body = norm(R_SC_body); % [km]

    acc = acc - (mission.true_solar_system.SS_body{i}.mu * R_SC_body / distance_SC_body^3); % [km / sec^2] in (3,1) format

end


% Add forces from Target bodies
for i = 1:1:mission.num_target

    R_Body = (interp1(mission.true_time.time_position_array, mission.true_target{i}.position_array, t))'; % [km]

    R_SC_body = R_SC - R_Body; % [km]

    distance_SC_body = norm(R_SC_body); % [km]

    acc = acc - (mission.true_target{i}.mu * R_SC_body / distance_SC_body^3); % [km / sec^2] in (3,1) format

end

Xdot=[X(4:6); acc];

end
