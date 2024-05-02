function func_plot_HEXP_components(true_SC, Rot)

if nargin < 2
    Rot = eye(3);
end

plot_colors = func_get_plot_colors();

% Body shapes
for i_shape = 1:length(true_SC.true_SC_body.shape_model)
    shape_i = true_SC.true_SC_body.shape_model{i_shape};
    if isfield(shape_i, 'color')
        color = shape_i.color;
    else
        color = 'white';
    end
    if isfield(shape_i, 'name') && shape_i.name ~= ""
        name = shape_i.name;
        handle_vis = 'on';
    else
        name = 'Body';
        handle_vis = 'off';
    end
    vert = shape_i.Vertices * Rot';
    trisurf( ...
        shape_i.Faces, vert(:, 1), vert(:, 2), vert(:, 3), ...
        'FaceColor', color, 'DisplayName', name, 'HandleVisibility', handle_vis)
    hold on
end

% Solar panels
for sp = 1:true_SC.true_SC_solar_panel.num_solar_panels
    shape_i = true_SC.true_SC_solar_panel.solar_panel_data(sp).shape_model;
    vert = shape_i.Vertices * Rot';
    trisurf( ...
        shape_i.Faces, vert(:, 1), vert(:, 2), vert(:, 3), ...
        'FaceColor', 'blue', "DisplayName", ['Solar Panel ', num2str(sp)])
end

i_count = 1;

% Micro thrusters
if true_SC.true_SC_body.flag_hardware_exists.adc_micro_thruster == 1
    for i = 1:true_SC.true_SC_micro_thruster_actuator.num_micro_thruster
        MT_data_i = true_SC.true_SC_micro_thruster_actuator.MT_data(i);
        location = Rot * MT_data_i.location(:);
        orientation = Rot * MT_data_i.orientation(:);
        quiver3( ...
            location(1), location(2), location(3), ...
            orientation(1), orientation(2), orientation(3), ...
            'color', plot_colors{i_count}, "LineWidth", 3, 'HandleVisibility', 'off', "AutoScaleFactor", 0.1)
    end
    name = 'Micro Thrusters';
    quiver3([], [], [], [], [], [], ...
        'color', plot_colors{i_count}, "LineWidth", 3, "DisplayName", name, "AutoScaleFactor", 0.1);
    i_count = i_count + 1;
end

% Chemical thrusters
if true_SC.true_SC_body.flag_hardware_exists.navigation_chemical_thruster == 1
    n_ct = true_SC.true_SC_chemical_thruster_actuator.num_chemical_thruster;
    for i = 1:n_ct
        ct_data_i = true_SC.true_SC_chemical_thruster_actuator.chemical_thruster_data(i);
        if ct_data_i.maximum_thrust == 22
            color = plot_colors{1};
            scale = 0.5;
        else
            color = plot_colors{2};
            scale = 0.25;
        end
        location = Rot * ct_data_i.location(:);
        orientation = - Rot * ct_data_i.orientation(:);
        quiver3( ...
            location(1), location(2), location(3), ...
            orientation(1), orientation(2), orientation(3), ...
            'color', color, "LineWidth", 4, 'HandleVisibility','off', "AutoScaleFactor", scale)
    end
    name_1 = 'Chemical Thrusters x4 (22.5 N)';
    quiver3([], [], [], [], [], [], ...
        'color', plot_colors{i_count}, "LineWidth", 4, "DisplayName", name_1, "AutoScaleFactor", 0.5);
    i_count = i_count + 1;
    
    name_2 = 'Chemical Thrusters x8 (4.5 N)';
    quiver3([], [], [], [], [], [], ...
        'color', plot_colors{i_count}, "LineWidth", 4, "DisplayName", name_2, "AutoScaleFactor", 0.25);
    i_count = i_count + 1;
end

% Star trackers
if true_SC.true_SC_body.flag_hardware_exists.adc_star_tracker == 1
    for i = 1:true_SC.true_SC_star_tracker_sensor.num_star_tracker
        ST_data_i = true_SC.true_SC_star_tracker_sensor.star_tracker_data(i);
        location = Rot * ST_data_i.location(:);
        orientation = Rot * ST_data_i.orientation(:);
        quiver3( ...
            location(1), location(2), location(3), ...
            orientation(1), orientation(2), orientation(3), ...
            'color', plot_colors{i_count}, ...
            "LineWidth", 3, 'HandleVisibility','off', "AutoScaleFactor", 0.2)
    end
    name = 'Star Trackers x2';
    quiver3([], [], [], [], [], [], ...
        'color', plot_colors{i_count}, "LineWidth", 3, "DisplayName", name, "AutoScaleFactor", 0.2);
    i_count = i_count + 1;
end

% Reaction wheels
if true_SC.true_SC_body.flag_hardware_exists.adc_reaction_wheel_assembly == 1
    for i = 1:true_SC.true_SC_rwa_actuator.num_reaction_wheel
        RW_data_i = true_SC.true_SC_rwa_actuator.RW_data(i);
        location = Rot * RW_data_i.shape_model.r_CM;
        direction = Rot * RW_data_i.direction(:);
        quiver3( ...
            location(1), location(2), location(3), ...
            direction(1), direction(2), direction(3), ...
            'color', plot_colors{i_count}, ...
            "LineWidth", 3, 'HandleVisibility','off', "AutoScaleFactor", 0.2)
    end
    name = 'Reaction Wheels x6';
    quiver3([], [], [], [], [], [], ...
        'color', plot_colors{i_count}, "LineWidth", 3, "DisplayName", name, "AutoScaleFactor", 0.2);
    i_count = i_count + 1;
end

end