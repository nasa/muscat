%% [ ] Methods: Desired Attitude CVX
% Compute Desired Attitude using CVX

function obj = func_compute_desired_attitude_CVX(obj)
disp('Compute Desired Attitude using CVX')

cvx_begin
variable r(3,3)
minimize ( norm(r * obj.data.secondary_vector' - obj.data.desired_secondary_vector') + norm(r' * obj.data.desired_secondary_vector' - obj.data.secondary_vector'))
subject to

r * obj.data.primary_vector' == obj.data.desired_primary_vector';
r' * obj.data.desired_primary_vector' == obj.data.primary_vector';

cvx_end

[SC_e_desired, SC_Phi_desired] = func_decompose_Rot_matrix(r);

SC_beta_v_desired = SC_e_desired*sin(SC_Phi_desired/2);
SC_beta_4_desired = cos(SC_Phi_desired/2);
obj.desired_attitude = [SC_beta_v_desired; SC_beta_4_desired];

end
