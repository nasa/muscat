%% [ ] Methods: Generic
% Execute generic coverage code

function obj = func_coverage_Generic(obj, mission, i_SC)

this_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * obj.orientation')'; % [unit vector]

this_SC_relative_position = mission.true_SC{i_SC}.true_SC_navigation.position_relative_target; % [km]
distance_SC_Target = norm(this_SC_relative_position); % [km]
this_SC_relative_position_normalize = func_normalize_vec(this_SC_relative_position); % [unit vector]

i_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target;
this_pos_points_normalized = (mission.true_target{i_target}.rotation_matrix * obj.pos_points')'; % Rotated unit vectors
this_pos_points = mission.true_target{i_target}.radius * this_pos_points_normalized; % [km]

% Accept Points within Limb Angle

limb_angle = acosd(mission.true_target{i_target}.radius/distance_SC_Target); % [deg]

dot_prod_angle_array = real(acosd(this_pos_points_normalized * this_SC_relative_position_normalize')); % [deg] angle between the vectors SC-Target center and mesh_point-Target center
idx_array = logical(dot_prod_angle_array <= limb_angle);

% Accept Points within FOV

this_pos_points_relative_SC = this_pos_points - this_SC_relative_position; % [km]
this_pos_points_relative_SC_normalize = this_pos_points_relative_SC./vecnorm(this_pos_points_relative_SC,2,2); % [unit vector]

dot_prod_angle_array = real(acosd(this_pos_points_relative_SC_normalize * this_orientation')); % [deg] angle between the vectors SC-Target center and mesh_point-Target center

idx_array = idx_array & logical(dot_prod_angle_array <= obj.field_of_view);

index_point_science_mesh = find(idx_array);

% MONOSTATIC

if isempty(index_point_science_mesh)
    % This is empty!

    [min_angle,index_point_science_mesh] = min(dot_prod_angle_array);

end

for i = 1:1:length(index_point_science_mesh)

    if obj.monostatic_observed_point(index_point_science_mesh(i)) == 0
        obj.monostatic_num_point_observed = obj.monostatic_num_point_observed + 1;
    end

    obj.monostatic_observed_point(index_point_science_mesh(i)) = obj.monostatic_observed_point(index_point_science_mesh(i)) + 1;

end




if isfield(obj.data, 'instantaneous_data_rate_generated_per_SC_mode')
    obj.instantaneous_data_rate_generated = obj.data.instantaneous_data_rate_generated_per_SC_mode(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode_value); % [W]
end


% Update Data Generated
func_update_instantaneous_data_generated(mission.true_SC{i_SC}.true_SC_data_handling, obj, mission);


end