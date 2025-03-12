function plot_swarm_trisurf(mission_true_SC)

fig = figure();

for i_SC = 1:length(mission_true_SC)
    subplot(2, ceil(length(mission_true_SC) / 2), i_SC)
    
    % Body shapes
    for i_shape = 1:length(mission_true_SC{i_SC}.true_SC_body.shape_model)
        shape_i = mission_true_SC{i_SC}.true_SC_body.shape_model{i_shape};
        if isfield(shape_i, 'color')
            color_i = shape_i.color;
        else
            color_i = 'white';
        end
        if isfield(shape_i, 'name')
            name_i = shape_i.name;
        else
            name_i = 'Body';
        end
        trisurf( ...
            shape_i.Faces, ...
            shape_i.Vertices(:, 1), ...
            shape_i.Vertices(:, 2), ...
            shape_i.Vertices(:, 3), ...
            'FaceColor', color_i, "DisplayName", name_i)
        hold on
    end
    
    % Solar panels
    for sp = 1:mission_true_SC{i_SC}.true_SC_solar_panel.num_solar_panels
        shape_i = mission_true_SC{i_SC}.true_SC_solar_panel.solar_panel_data(sp).shape_model;
        trisurf( ...
            shape_i.Faces, ...
            shape_i.Vertices(:, 1), ...
            shape_i.Vertices(:, 2), ...
            shape_i.Vertices(:, 3), ...
            'FaceColor', 'blue', "DisplayName", ['SP ', num2str(sp)])
    end
    
    % Micro thrusters
    if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
        for i = 1:mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.num_micro_thruster
            MT_data_i = mission_true_SC{i_SC}.true_SC_micro_thruster_actuator.MT_data(i);
            quiver3( ...
                MT_data_i.location(1), ...
                MT_data_i.location(2), ...
                MT_data_i.location(3), ...
                MT_data_i.orientation(1), ...
                MT_data_i.orientation(2), ...
                MT_data_i.orientation(3), ...
                "LineWidth", 3, "DisplayName", ['MT ', num2str(i)], "AutoScaleFactor", 0.1)
        end
    end
    
    % Chemical thrusters
    if mission_true_SC{i_SC}.true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
        for i = 1:mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.num_chemical_thruster
            ct_data_i = mission_true_SC{i_SC}.true_SC_chemical_thruster_actuator.chemical_thruster_data(i);
            quiver3( ...
                ct_data_i.location(1), ...
                ct_data_i.location(2), ...
                ct_data_i.location(3), ...
                -ct_data_i.orientation(1), ...
                -ct_data_i.orientation(2), ...
                -ct_data_i.orientation(3), ...
                "LineWidth", 10, "DisplayName", ['CT ', num2str(i)], "AutoScaleFactor", 0.15)
        end
    end
    
    axis equal
    xlabel("X_{SC}")
    ylabel("Y_{SC}")
    zlabel("Z_{SC}")
    title(['Spacecraft ', num2str(i_SC)])
end

legend('Location', 'northeast')

set(gcf, 'Position', [154, 165, 1456, 683]);
end