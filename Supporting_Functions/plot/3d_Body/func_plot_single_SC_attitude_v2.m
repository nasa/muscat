%% [ ] Methods: Visualize Attitude of Single SC

function func_plot_single_SC_attitude_v2(mission, i_SC)

hold on

% Debug rotation - verify that attitude data exists and is being updated
has_attitude = isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix');
if ~has_attitude
    warning('Spacecraft %d does not have attitude data. Objects will not rotate.', i_SC);
end


% Main body visualization with attitude transformation
for i_shape = 1:length(mission.true_SC{i_SC}.true_SC_body.shape_model)
    % Get original vertices
    this_Vertices = mission.true_SC{i_SC}.true_SC_body.shape_model{i_shape}.Vertices;

    % Apply attitude rotation if available
    if has_attitude
        % Apply rotation matrix to vertices relative to COM
        this_Vertices = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (this_Vertices - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
    end

    % Draw the transformed body
    h_body = trisurf(mission.true_SC{i_SC}.true_SC_body.shape_model{i_shape}.Faces, ...
        this_Vertices(:,1), ...
        this_Vertices(:,2), ...
        this_Vertices(:,3), ...
        'FaceColor',rgb('LightGrey'));
    set(get(get(h_body,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    
end
plot3(NaN, NaN, NaN, 'Color', rgb('LightGrey'), 'LineWidth', 2, 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name, ' Body']);


% Solar panel visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
    % Get original vertices
    this_Vertices = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.shape_model.Vertices;

    % Apply attitude rotation if available
    if has_attitude
        % Apply rotation matrix to vertices relative to COM
        this_Vertices = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (this_Vertices - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
    end

    h_sp = trisurf(mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.shape_model.Faces, ...
        this_Vertices(:,1), ...
        this_Vertices(:,2), ...
        this_Vertices(:,3), ...
        'FaceColor',rgb('LightBlue'));
    set(get(get(h_sp,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

    % Hide face normals - also apply rotation if available
    for i_face = 1:1:size(mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.shape_model.Faces,1)
        face_center = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.shape_model.Face_center(i_face,:);
        face_orientation = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.shape_model.Face_orientation_solar_cell_side(1,:);

        % Apply rotation to face center and orientation if attitude is available
        if has_attitude
            face_center = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (face_center - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            face_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * face_orientation')';
        end

        q = quiver3(face_center(1), ...
            face_center(2), ...
            face_center(3), ...
            face_orientation(1), ...
            face_orientation(2), ...
            face_orientation(3), ...
            'LineWidth',1,'Color',rgb('DarkBlue'), 'AutoScaleFactor',mission.storage.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end

    plot3(NaN, NaN, NaN, 'Color', rgb('LightBlue'), 'LineWidth', 2, 'DisplayName', mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.name);
end


% Camera visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
    equipment = mission.true_SC{i_SC}.true_SC_camera{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('DarkGreen'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Star Tracker visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker
    equipment = mission.true_SC{i_SC}.true_SC_star_tracker{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Gold'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Sun Sensor visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor
    equipment = mission.true_SC{i_SC}.true_SC_sun_sensor{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Yellow'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Science Radar visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar
    equipment = mission.true_SC{i_SC}.true_SC_science_radar{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Orange'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Remote Sensing visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_remote_sensing
    equipment = mission.true_SC{i_SC}.true_SC_remote_sensing{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Red'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Radio Antenna visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna
    equipment = mission.true_SC{i_SC}.true_SC_radio_antenna{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('DarkMagenta'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end



% Reaction Wheel visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    equipment = mission.true_SC{i_SC}.true_SC_reaction_wheel{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('DarkCyan'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Micro Thruster visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
    equipment = mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Violet'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end



% Chemical Thruster visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
    equipment = mission.true_SC{i_SC}.true_SC_chemical_thruster{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Yellow'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% EP Thruster visualization with attitude transformation
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_ep_thruster
    equipment = mission.true_SC{i_SC}.true_SC_ep_thruster{i_HW};

    % Equipment location and orientation
    HW_location = equipment.location;
    HW_orientation = equipment.orientation;

    % Apply attitude rotation if available
    if has_attitude
        HW_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (HW_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
        HW_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * HW_orientation')';
    end

    % Direction arrows (hidden from legend)
    q = quiver3(HW_location(1), HW_location(2), HW_location(3), HW_orientation(1), HW_orientation(2), HW_orientation(3), ...
        'LineWidth', 2, 'Color', rgb('Yellow'), 'AutoScaleFactor', mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',equipment.name);
end


% Target Direction visualization
for i_target = 1:1:mission.num_target

    % Target location and orientation
    Target_location = mission.true_SC{i_SC}.true_SC_body.location_COM;
    Target_orientation = func_normalize_vec(mission.true_target{i_target}.position - mission.true_SC{i_SC}.true_SC_navigation.position);

    % Direction arrows (hidden from legend)
    q = quiver3(Target_location(1), Target_location(2), Target_location(3), Target_orientation(1), Target_orientation(2), Target_orientation(3), ...
        'LineWidth', 5, 'Color', rgb('Black'), 'AutoScaleFactor', 5*mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',mission.true_target{i_target}.name);

end


% Sun Direction visualization
Target_location = mission.true_SC{i_SC}.true_SC_body.location_COM;
Target_orientation = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.true_SC_navigation.position);

% Direction arrows (hidden from legend)
q = quiver3(Target_location(1), Target_location(2), Target_location(3), Target_orientation(1), Target_orientation(2), Target_orientation(3), ...
    'LineWidth', 5, 'Color', rgb('Gold'), 'AutoScaleFactor', 10*mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.name);


% Earth Direction visualization
if mission.true_solar_system.index_Earth ~= 0
    Target_location = mission.true_SC{i_SC}.true_SC_body.location_COM;
    Target_orientation = func_normalize_vec(mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.position - mission.true_SC{i_SC}.true_SC_navigation.position);

    % Direction arrows (hidden from legend)
    q = quiver3(Target_location(1), Target_location(2), Target_location(3), Target_orientation(1), Target_orientation(2), Target_orientation(3), ...
        'LineWidth', 5, 'Color', rgb('Blue'), 'AutoScaleFactor', 8*mission.storage.plot_parameters.quiver_auto_scale_factor, 'DisplayName',mission.true_solar_system.SS_body{mission.true_solar_system.index_Earth}.name);
end


% Plot settings
axis equal
grid on
% Create a legend in a more visible position
lgd = legend('Location', 'southeast');

xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type)

% Set a fixed view angle that makes rotations more visible
view(3)

hold off

end
