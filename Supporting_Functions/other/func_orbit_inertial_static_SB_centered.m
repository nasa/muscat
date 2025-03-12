function x_dot = func_orbit_inertial_static_SB_centered( ...
    t,x, ...
    this_force,...
    this_disturbance,...
    this_SC_mass,...
    mission_true_small_body,...
    flag_stm)

if nargin < 7
    flag_stm = false;
end

r_I = x(1:3);
Rot = mission_true_small_body.rotation_matrix;
r_R = Rot * r_I;

if flag_stm
    [a_R, ~, ~, dadr] = mission_true_small_body.gravity_field.Acceleration(r_R, mission_true_small_body.gravity_degree_harmonics);

    D = zeros(6,6);
    D(1:3,4:6) = eye(3);
    D(4:6,1:3) = Rot' * dadr * Rot;
    
    Phi = reshape(x(7:42), [6,6]);
    dPhidt = D * Phi;
else
    [a_R, ~, ~] = mission_true_small_body.gravity_field.Acceleration(r_R, mission_true_small_body.gravity_degree_harmonics);
end

a_I = Rot' * a_R;
x_dot=[
    x(4:6);
    a_I + 1e-3*this_force/this_SC_mass + 1e-3*this_disturbance/this_SC_mass;
    ];

if flag_stm
    x_dot = [x_dot; reshape(dPhidt, [36,1])];
end

end
