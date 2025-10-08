function x_dot = func_orbit_inertial_static_relative_Target(t, X, mission, i_SC, i_target, flag_stm)

if nargin < 6
    flag_stm = false;
end

r_I = X(1:3);
Rot = mission.true_target{i_target}.rotation_matrix;
r_R = Rot * r_I;

if flag_stm
    [a_R, ~, ~, dadr] = mission.true_target{i_target}.gravity_field.Acceleration(r_R, mission.true_target{i_target}.gravity_degree_harmonics);

    D = zeros(6,6);
    D(1:3,4:6) = eye(3);
    D(4:6,1:3) = Rot' * dadr * Rot;
    
    Phi = reshape(X(7:42), [6,6]);
    dPhidt = D * Phi;
else
    [a_R, ~, ~] = mission.true_target{i_target}.gravity_field.Acceleration(r_R, mission.true_target{i_target}.gravity_degree_harmonics);
end

a_I = Rot' * a_R;
x_dot=[ X(4:6);
        a_I + (1e-3*mission.true_SC{i_SC}.true_SC_navigation.control_force'/mission.true_SC{i_SC}.true_SC_body.total_mass) + (1e-3*mission.true_SC{i_SC}.true_SC_navigation.disturbance_force'/mission.true_SC{i_SC}.true_SC_body.total_mass)];

if flag_stm
    x_dot = [x_dot; reshape(dPhidt, [36,1])];
end

end
