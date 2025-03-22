%% [ ] Methods: Desired Attitude Array Search
% Compute Desired Attitude using Array Search
% The array search component iteratively searches for an optimized secondary alignment by testing different rotation angles around the primary vector.

function obj = func_compute_desired_attitude_Array_Search(obj)

% 1) Aligning the Primary Vector
primary_vector = obj.data.primary_vector;
desired_primary_vector = obj.data.desired_primary_vector;

% Compute rotation to rotate pointing aligned with target (in J2000)
v = cross(primary_vector, desired_primary_vector);
Rot_primary = eye(3) + skew(v)+skew(v)^2*(1/(1+dot(primary_vector, desired_primary_vector)));

% 2) Optimizing the Secondary Vector
theta = 0:2*pi/360:2*pi;
optimized_error_SP_pointing = zeros(1,length(theta));

desired_secondary_vector = obj.data.desired_secondary_vector;
secondary_vector = obj.data.secondary_vector;

for i=1:length(theta)
    U = desired_primary_vector;
    R = func_axis_angle_rot(U, theta(i));
    optimized_error_SP_pointing(i) = func_angle_between_vectors(desired_secondary_vector, R*secondary_vector');
end

[~,index_optimized_theta] = min(optimized_error_SP_pointing);
Rot_secondary = func_axis_angle_rot(U, theta(index_optimized_theta));

% 3) Combine Rotations
r = Rot_secondary * Rot_primary;

% 4) Convert Rotation Matrix to Quaternion
[SC_e_desired, SC_Phi_desired] = func_decompose_rotation_matrix(r);

SC_beta_v_desired = SC_e_desired' * sin(SC_Phi_desired/2);
SC_beta_4_desired = cos(SC_Phi_desired/2);

obj.desired_attitude = func_quaternion_properize([SC_beta_v_desired, SC_beta_4_desired]);

end
