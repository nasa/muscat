function Xdot = func_ode_orbit_inertial_absolute_dynamics_v2(t, X, mission, i_SC)

% SC Position
R_SC = X(1:3); % [km]

% Initialize Acc with Control Force
acc = (1/mission.true_SC{i_SC}.true_SC_body.total_mass) * (1e-3) * mission.true_SC{i_SC}.true_SC_orbit_propagation.control_force'; % [km / sec^2] in (3,1) format

% Add disturbance forces to Acc
acc = acc + (1/mission.true_SC{i_SC}.true_SC_body.total_mass) * (1e-3) * mission.true_SC{i_SC}.true_SC_orbit_propagation.disturbance_force'; % [km / sec^2]  in (3,1) format

% Add forces from Solar System bodies
for i_SS_body = 1:1:mission.true_solar_system.num_SS_body

    % R_SS_Body = (interp1(mission.true_time.time_position_array, mission.true_solar_system.SS_body{i_SS_body}.position_array, t))'; % [km]

    % First term: Gravity force from all gravitational bodies 
    body_pos_vel_this_time = cspice_spkezr(mission.true_solar_system.SS_body{i_SS_body}.spice_name, mission.true_time.prev_date + t,'J2000','NONE','SUN');
    R_SS_Body = body_pos_vel_this_time(1:3,1); % [km]
    
    R_SC_body = R_SC - R_SS_Body; % [km]

    distance_SC_body = norm(R_SC_body); % [km]

    acc = acc - (mission.true_solar_system.SS_body{i_SS_body}.mu * R_SC_body / distance_SC_body^3); % [km / sec^2] in (3,1) format

    % Second term: Force from other non-Sun gravitational bodies
    if i_SS_body ~= mission.true_solar_system.index_Sun

        R_SS_Body_to_Sun = - R_SS_Body; % [km]
        dist_SS_Body_to_Sun = norm(R_SS_Body_to_Sun); % [km]
        
        acc = acc + (mission.true_solar_system.SS_body{i_SS_body}.mu * R_SS_Body_to_Sun / dist_SS_Body_to_Sun^3); % [km/sec^2]
    end

end


% Add forces from Target bodies
for i_target = 1:1:mission.num_target

    % First term: Gravity force from all gravitational bodies 
    R_SS_Body = (interp1(mission.true_time.time_position_array, mission.true_target{i_target}.position_array, t))'; % [km]

    R_SC_body = R_SC - R_SS_Body; % [km]

    distance_SC_body = norm(R_SC_body); % [km]

    acc = acc - (mission.true_target{i_target}.mu * R_SC_body / distance_SC_body^3); % [km / sec^2] in (3,1) format

    % Second term: Force from other non-Sun gravitational bodies
    if ~strcmp(mission.true_target{i_target}.name, 'Sun')

        R_SS_Body_to_Sun = - R_SS_Body; % [km]
        dist_SS_Body_to_Sun = norm(R_SS_Body_to_Sun); % [km]
        
        acc = acc + (mission.true_target{i_target}.mu * R_SS_Body_to_Sun / dist_SS_Body_to_Sun^3); % [km/sec^2]
    end

end

% Create Xdot
Xdot=[X(4:6); acc];

end
