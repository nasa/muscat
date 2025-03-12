function Xdot = func_orbit_SB_body(t,X,mu_Sun,Sun_pos_t0_tf,time_array)

% SB Position
R_SB = X(1:3);

% % SB Position
% this_SB_pos = [interp1(time_array,SB_pos_t0_tf(1,:),t) interp1(time_array,SB_pos_t0_tf(2,:),t) interp1(time_array,SB_pos_t0_tf(3,:),t)]';
% R_SC_SB = R_SC - this_SB_pos;
% distance_SC_SB = norm(R_SC_SB);


% Sun Position
this_Sun_pos = [interp1(time_array,Sun_pos_t0_tf(1,:),t) interp1(time_array,Sun_pos_t0_tf(2,:),t) interp1(time_array,Sun_pos_t0_tf(3,:),t)]';
R_SC_SB = R_SB - this_Sun_pos;
distance_SB_Sun = norm(R_SC_SB);



Xdot=[...
    X(4);
    X(5);
    X(6);
    - mu_Sun*R_SC_SB(1)/(distance_SB_Sun^3);
    - mu_Sun*R_SC_SB(2)/(distance_SB_Sun^3);
    - mu_Sun*R_SC_SB(3)/(distance_SB_Sun^3);
    ];

end
