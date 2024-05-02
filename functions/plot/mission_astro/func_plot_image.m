function func_plot_image(...
    kd, ...
    ms, ... % mission_storage
    mission_true_solar_system, ...
    mission_true_stars, ...
    mission_true_SC)
% func_get_camera_image(obj,i,true_time,true_small_body,mission_true_SC,this_SC_index,true_solar_system,true_stars,mission_init_data)

% scale_up_factor_factor = 250;
% for j=1:mission_init_data.num_SC
%     if j ~= this_SC_index

%         % plot radar line to SB
%         pos_sb_this_sc_frame = true_small_body.position_SB - mission_true_SC{this_SC_index}.true_SC_navigation.position;
%         pos_sc_this_sc_frame = (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB + scale_up_factor_factor*(1e-3)*mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*mission_true_SC{j}.true_SC_camera_sensor.camera_data(i).location'  -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
%         plot3([pos_sb_this_sc_frame(1)', pos_sc_this_sc_frame(1)'] , [pos_sb_this_sc_frame(2)', pos_sc_this_sc_frame(2)'] , [pos_sb_this_sc_frame(3)', pos_sc_this_sc_frame(3)'], 'Color','green','Linewidth',2);
%         % plot SC shape model
%         this_SC_model = []; this_SC_model.Vertices = mission_true_SC{j}.true_SC_body.shape_model.Vertices; this_SC_model.Faces = mission_true_SC{j}.true_SC_body.shape_model.Faces;
%         this_SC_model.Vertices = (mission_true_SC{j}.true_SC_adc.rotation_matrix_SC * this_SC_model.Vertices')';
%         this_SC_model.Vertices = scale_up_factor_factor * this_SC_model.Vertices * (1e-3);
%         this_SC_model.Vertices = this_SC_model.Vertices + (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
%         patch(this_SC_model,'FaceColor',[1 1 1],'Facelighting','flat');
%         %plot Solar panels shape model
%         hold on;
%         for sp=1:mission_true_SC{j}.true_SC_solar_panel.num_solar_panels
%             this_SP_model = []; this_SP_model.Vertices = mission_true_SC{j}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Vertices; this_SP_model.Faces = mission_true_SC{j}.true_SC_solar_panel.solar_panel_data(sp).shape_model.Faces;
%             this_SP_model.Vertices = (mission_true_SC{j}.true_SC_adc.rotation_matrix_SC *  this_SP_model.Vertices')';
%             this_SP_model.Vertices = scale_up_factor_factor * this_SP_model.Vertices * (1e-3);
%             this_SP_model.Vertices = this_SP_model.Vertices + (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
%             patch(this_SP_model,'FaceColor','blue','Facelighting','flat'); % 'Facelighting','gouraud'
%             hold on;
%         end
%         % plot SC local frame
%         pos_sc_this_sc_frame = (true_small_body.position_SB + mission_true_SC{j}.true_SC_navigation.position_relative_SB -  mission_true_SC{this_SC_index}.true_SC_navigation.position)';
%         arrow_size=scale_up_factor_factor*0.8*1e-3;
%         mArrow3(pos_sc_this_sc_frame',pos_sc_this_sc_frame'+mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*[arrow_size 0 0]', 'facealpha', 1, 'color', 'r', 'stemWidth', 0.005);
%         hold on;
%         mArrow3(pos_sc_this_sc_frame',pos_sc_this_sc_frame'+mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*[0 arrow_size 0]', 'facealpha', 1, 'color', 'g', 'stemWidth', 0.005);
%         hold on;
%         mArrow3(pos_sc_this_sc_frame',pos_sc_this_sc_frame'+mission_true_SC{j}.true_SC_adc.rotation_matrix_SC*[0 0 arrow_size]', 'facealpha', 1, 'color', 'b', 'stemWidth', 0.005);
%     end
% end

% Plot SB model
% scale_up_factor_factor = 1;
% this_SB_Shape_Model = true_small_body.shape_model;
% Rotation_matrix = func_rotation_matrix(true_small_body.rotation_rate * true_time.time) * true_small_body.rotation_matrix_pole_RA_Dec;
% this_SB_Shape_Model.Vertices = (Rotation_matrix * this_SB_Shape_Model.Vertices')'*(1e-3) * scale_up_factor_factor;
% this_SB_Shape_Model.Vertices = this_SB_Shape_Model.Vertices + (true_small_body.position_SB - mission_true_SC{this_SC_index}.true_SC_navigation.position)';
% patch(this_SB_Shape_Model, 'FaceColor',0.7*[1 1 1], 'EdgeColor', 'none'); % 'Facelighting','gouraud'

% axis vis3d off
% axis equal

% material([0 1 0])

rot_matrix_SC = squeeze(ms.true_SC{1}.rotation_matrix_SC(kd,:,:));
rot_matrix_camera = eye(3);


% Manage view from current attitude
x_true_hat = rot_matrix_SC * rot_matrix_camera * [0 0 -1]';
y_true_hat = rot_matrix_SC * rot_matrix_camera * [1 0 0]';

target_distance = 1e6;

camtarget([x_true_hat(1), x_true_hat(2), x_true_hat(3)] * target_distance)
campos([0, 0, 0])
camup([y_true_hat(1), y_true_hat(2), y_true_hat(3)] * target_distance)



% Plot stars
flag_show_stars = 1;
field_of_view = 5; % [deg]
if flag_show_stars == 1
    dot_product_angle_array = acosd(mission_true_stars.all_stars_unit_vector * x_true_hat);
    flag_in_FOV = logical(dot_product_angle_array <= (field_of_view/sqrt(2)));
    flag_magnitude_limit = logical(mission_true_stars.magnitude_visible <= mission_true_stars.maximum_magnitude);
    flag_magnitude_limit_FOV = (flag_in_FOV & flag_magnitude_limit);
    idx_array = find(flag_magnitude_limit_FOV);
    hold on
    for s = 1:1:length(idx_array)
        this_star = target_distance * mission_true_stars.all_stars_unit_vector(idx_array(s),:)';
        plot3(this_star(1),this_star(2),this_star(3),'*w','MarkerSize',(1 + mission_true_stars.maximum_magnitude - mission_true_stars.magnitude_visible(idx_array(s))),'MarkerFaceColor','w','MarkerEdgeColor','w')
    end
end

% Plot targets
flag_show_targets = 1;
curr_idx = ms.true_SC{1}.SC_target_index(kd);
cs = ['y', 'g', 'r'];
mrkr_sizes = [5, 5, 5];
if flag_show_targets
    coords = [ones(size(ms.target_data.dec)), deg2rad(ms.target_data.dec), deg2rad(ms.target_data.ra)];
    all_targets_unit_vector = Planetocentric2Cartesian(coords);
    dot_product_angle_array = acosd(all_targets_unit_vector * x_true_hat);
    flag_in_FOV = logical(dot_product_angle_array <= (field_of_view/sqrt(2)));
    idx_array = find(flag_in_FOV);
    hold on
    for s = 1:1:length(idx_array)
        this_target = target_distance * all_targets_unit_vector(idx_array(s),:)';
        if s == curr_idx
            c = cs(1);
            mkrs = mrkr_sizes(1);
        elseif ismember(s, ms.true_SC{1}.SC_target_index(1:kd))
            c = cs(2);
            mkrs = mrkr_sizes(2);
        else
            c = cs(3);
            mkrs = mrkr_sizes(3);
        end
        plot3(this_target(1),this_target(2),this_target(3),['o',c],'MarkerSize',mkrs,'MarkerFaceColor',c,'MarkerEdgeColor','k')
    end
end

% Light comming from sun
%         camlight('headlight')
h = light;
h.Position = (mission_true_solar_system.position_Sun - mission_true_SC{1}.true_SC_navigation.position)';
h.Style = 'local';

% field of view
camva(field_of_view);

camproj('perspective');
axis image
axis off;
end