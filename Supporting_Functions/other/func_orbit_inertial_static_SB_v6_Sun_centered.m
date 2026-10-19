function Xdot = func_orbit_inertial_static_SB_v6_Sun_centered(t, X, obj, mission)

% SC Position from SUN
R_Ast = X(1:3); % [km]

% Initialize accelration
acc = [0 0 0]'; % [km/sec^2]

for i_SS_body = 1:1:mission.true_solar_system.num_SS_body
 
    % First term: Gravity force from all gravitational bodies 
    body_pos_vel_this_time = cspice_spkezr(mission.true_solar_system.SS_body{i_SS_body}.spice_name, mission.true_time.prev_date + t,'J2000','NONE','SUN');
    R_SS_Body = body_pos_vel_this_time(1:3,1); % [km]
    R_SS_Body_to_Ast = (R_Ast - R_SS_Body); % [km]

    dist_SS_Body_to_Ast = norm(R_SS_Body_to_Ast); % [km]
    mu_SS_body = mission.true_solar_system.SS_body{i_SS_body}.mu; % [km^3 sec^-2]

    acc = acc - (mu_SS_body * R_SS_Body_to_Ast / dist_SS_Body_to_Ast^3); % [km/sec^2]


    % Second term: Force from other non-Sun gravitational bodies
    if i_SS_body ~= mission.true_solar_system.index_Sun

        R_SS_Body_to_Sun = - R_SS_Body; % [km]
        dist_SS_Body_to_Sun = norm(R_SS_Body_to_Sun); % [km]
        
        acc = acc + (mu_SS_body * R_SS_Body_to_Sun / dist_SS_Body_to_Sun^3); % [km/sec^2]
    end

end

% Add Control and Disturbance Forces
acc = acc + (1e-3 * obj.control_force' / obj.mass) + (1e-3 * obj.disturbance_force' / obj.mass); % [km/sec^2]

Xdot=[X(4:6); acc];

end


