function func_plot_target_shape(i_target, mission)

% Plot the entire Shape Model

this_SB_Shape_Model = mission.true_target{i_target}.shape_model;
switch mission.true_target{i_target}.shape_model_type

    case 'trisurf'

        this_SB_Shape_Model.Vertices = (1e-3)*(mission.true_target{i_target}.rotation_matrix * this_SB_Shape_Model.Vertices')';
        patch(this_SB_Shape_Model, 'FaceColor', rgb('Gray'), 'EdgeColor', 'none', 'DisplayName', mission.true_target{i_target}.name)
        %,    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',    'SpecularStrength',0,'AmbientStrength',.5)

    case 'sphere'

        topo = fliplr(imread(this_SB_Shape_Model.img));
        [xP,yP,zP] = sphere(50);

        % Added rotation
        for j=1:1:size(xP,2)

            new_XYZ = (mission.true_target{i_target}.rotation_matrix * [xP(:,j), yP(:,j), zP(:,j)]')';

            xP(:,j) = new_XYZ(:,1);
            yP(:,j) = new_XYZ(:,2);
            zP(:,j) = new_XYZ(:,3);
        end

        Target_radius = mission.true_target{i_target}.radius;
        hPlanet = surf(xP*Target_radius, yP*Target_radius, zP*Target_radius,'FaceColor','blue','EdgeColor','none', 'DisplayName', mission.true_target{i_target}.name);
        set(hPlanet,'facecolor','texture','cdata',topo,'edgecolor','none');

    otherwise
        error('Shape model type not recognized')
end