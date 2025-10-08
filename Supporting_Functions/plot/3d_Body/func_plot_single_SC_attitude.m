%% [ ] Methods: Visualize Attitude of Single SC

function func_plot_single_SC_attitude(mission, i_SC)

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

% Solar panel visualization with attitude transformation
for i_SP = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
    % Get original vertices
    this_Vertices = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.Vertices;

    % Apply attitude rotation if available
    if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
        % Apply rotation matrix to vertices relative to COM
        this_Vertices = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (this_Vertices - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
    end

    h_sp = trisurf(mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.Faces, ...
        this_Vertices(:,1), ...
        this_Vertices(:,2), ...
        this_Vertices(:,3), ...
        'FaceColor',rgb('LightBlue'));
    set(get(get(h_sp,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

    % Hide face normals - also apply rotation if available
    for i_face = 1:1:size(mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.Faces,1)
        face_center = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.Face_center(i_face,:);
        face_orientation = mission.true_SC{i_SC}.true_SC_solar_panel{i_SP}.shape_model.Face_orientation_solar_cell_side(1,:);

        % Apply rotation to face center and orientation if attitude is available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            face_center = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (face_center - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            face_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * face_orientation')';
        end

        q = quiver3(face_center(1), ...
            face_center(2), ...
            face_center(3), ...
            face_orientation(1), ...
            face_orientation(2), ...
            face_orientation(3), ...
            'LineWidth',1,'Color',rgb('DarkBlue'), 'AutoScaleFactor',obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Camera visualization with attitude transformation
camera_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera > 0
    for i_camera = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_camera
        camera = mission.true_SC{i_SC}.true_SC_camera{i_camera};
        camera_size = 0.04;

        % Apply attitude transformation to camera location and orientation
        base_center = camera.location;
        cam_dir = camera.orientation / norm(camera.orientation);
        cam_up = camera.orientation_up / norm(camera.orientation_up);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            base_center = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (base_center - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            cam_dir = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * cam_dir')';
            cam_up = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * cam_up')';
        end

        cam_right = cross(cam_dir, cam_up);

        % Create camera box vertices and faces with the transformed orientation
        vertices = [
            base_center - camera_size/2 * cam_right - camera_size/2 * cam_up - camera_size/2 * cam_dir;
            base_center + camera_size/2 * cam_right - camera_size/2 * cam_up - camera_size/2 * cam_dir;
            base_center + camera_size/2 * cam_right + camera_size/2 * cam_up - camera_size/2 * cam_dir;
            base_center - camera_size/2 * cam_right + camera_size/2 * cam_up - camera_size/2 * cam_dir;
            base_center - camera_size/2 * cam_right - camera_size/2 * cam_up + camera_size/2 * cam_dir;
            base_center + camera_size/2 * cam_right - camera_size/2 * cam_up + camera_size/2 * cam_dir;
            base_center + camera_size/2 * cam_right + camera_size/2 * cam_up + camera_size/2 * cam_dir;
            base_center - camera_size/2 * cam_right + camera_size/2 * cam_up + camera_size/2 * cam_dir;
            ];

        faces = [
            1 2 3 4;    % back
            5 6 7 8;    % front
            1 2 6 5;    % bottom
            4 3 7 8;    % top
            1 4 8 5;    % left
            2 3 7 6;    % right
            ];

        h_camera = patch('Vertices', vertices, 'Faces', faces, 'FaceColor', rgb('ForestGreen'), 'EdgeColor', rgb('DarkGreen'), 'FaceAlpha', 0.7);
        set(get(get(h_camera,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        camera_handles = [camera_handles; h_camera];

        % Direction arrows (hidden from legend)
        q = quiver3(base_center(1), base_center(2), base_center(3), ...
            cam_dir(1), cam_dir(2), cam_dir(3), ...
            'LineWidth', 2, 'Color', rgb('DarkGreen'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Star Tracker visualization with attitude transformation
st_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker > 0
    for i_st = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_star_tracker
        st = mission.true_SC{i_SC}.true_SC_star_tracker{i_st};
        st_radius = 0.025;
        st_length = 0.06;

        % Apply attitude transformation to star tracker location and orientation
        st_location = st.location;
        st_orientation = st.orientation / norm(st.orientation);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            st_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (st_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            st_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * st_orientation')';
        end

        [X,Y,Z] = cylinder(st_radius, 12);
        Z = Z * st_length;

        v = st_orientation;
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + st_location(1);
                Y(i,j) = p(2) + st_location(2);
                Z(i,j) = p(3) + st_location(3);
            end
        end

        h_st = surf(X, Y, Z, 'FaceColor', rgb('Gold'), 'EdgeColor', rgb('DarkGoldenrod'), 'FaceAlpha', 0.8);
        set(get(get(h_st,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        st_handles = [st_handles; h_st];

        % Direction arrow (hidden from legend)
        q = quiver3(st_location(1), st_location(2), st_location(3), ...
            v(1), v(2), v(3), ...
            'LineWidth', 2, 'Color', rgb('Gold'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Sun Sensor visualization with attitude transformation
ss_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor > 0
    for i_ss = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_sun_sensor
        ss = mission.true_SC{i_SC}.true_SC_sun_sensor{i_ss};
        ss_radius = 0.02;
        ss_thickness = 0.01;

        % Apply attitude transformation to sun sensor location and orientation
        ss_location = ss.location;
        ss_orientation = ss.orientation / norm(ss.orientation);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            ss_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (ss_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            ss_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * ss_orientation')';
        end

        [X,Y,Z] = cylinder(ss_radius, 12);
        Z = Z * ss_thickness;

        v = ss_orientation;
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + ss_location(1);
                Y(i,j) = p(2) + ss_location(2);
                Z(i,j) = p(3) + ss_location(3);
            end
        end

        h_ss = surf(X, Y, Z, 'FaceColor', rgb('Yellow'), 'EdgeColor', rgb('Gold'), 'FaceAlpha', 0.8);
        set(get(get(h_ss,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        ss_handles = [ss_handles; h_ss];

        % Direction arrow (hidden from legend)
        q = quiver3(ss_location(1), ss_location(2), ss_location(3), ...
            v(1), v(2), v(3), ...
            'LineWidth', 2, 'Color', rgb('Yellow'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Science Radar visualization with attitude transformation
radar_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar > 0
    for i_radar = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_science_radar
        radar = mission.true_SC{i_SC}.true_SC_science_radar{i_radar};
        radar_radius = 0.05;
        radar_length = 0.07;

        % Apply attitude transformation to radar location and orientation
        radar_location = radar.location;
        radar_orientation = radar.orientation / norm(radar.orientation);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            radar_location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (radar_location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            radar_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * radar_orientation')';
        end

        [X,Y,Z] = cylinder(radar_radius, 16);
        Z = Z * radar_length;

        v = radar_orientation;
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + radar_location(1);
                Y(i,j) = p(2) + radar_location(2);
                Z(i,j) = p(3) + radar_location(3);
            end
        end

        h_radar = surf(X, Y, Z, 'FaceColor', rgb('Orange'), 'EdgeColor', rgb('DarkOrange'), 'FaceAlpha', 0.8);
        set(get(get(h_radar,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        radar_handles = [radar_handles; h_radar];

        % Direction arrow (hidden from legend)
        q = quiver3(radar_location(1), radar_location(2), radar_location(3), ...
            v(1), v(2), v(3), ...
            'LineWidth', 2, 'Color', rgb('Orange'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Radio Antenna visualization with attitude transformation
antenna_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna > 0
    for i_ra = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_radio_antenna

        %             % Calculate antenna position on spacecraft body surface
        %             if length(mission.true_SC{i_SC}.true_SC_body.shape_model) > 0
        %                 vertices = mission.true_SC{i_SC}.true_SC_body.shape_model{1}.Vertices;
        %                 y_coords = vertices(:,2);
        %                 max_y = max(y_coords);
        %                 tolerance = 0.05;
        %                 y_face_indices = find(abs(y_coords - max_y) < tolerance * max_y);
        %                 y_face_vertices = vertices(y_face_indices, :);
        %                 if ~isempty(y_face_vertices)
        %                     antenna_position = mean(y_face_vertices, 1);
        %                 else
        %                     antenna_position = mission.true_SC{i_SC}.true_SC_body.location_COM + [0 0.2 0];
        %                 end
        %             else
        %                 antenna_position = [0 0.2 0];
        %             end

        antenna_position = mission.true_SC{i_SC}.true_SC_radio_antenna{i_ra}.location;

        antenna_orientation = mission.true_SC{i_SC}.true_SC_radio_antenna{i_ra}.orientation;

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            antenna_position = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (antenna_position - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            antenna_orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * antenna_orientation')';
        end

        [X,Y,Z] = cylinder([0.02, 0.01], 12);
        base_length = 0.08;
        Z = Z * base_length;

        v = antenna_orientation / norm(antenna_orientation);
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + antenna_position(1);
                Y(i,j) = p(2) + antenna_position(2);
                Z(i,j) = p(3) + antenna_position(3);
            end
        end

        h_antenna = surf(X, Y, Z, 'FaceColor', rgb('Magenta'), 'EdgeColor', 'none', 'FaceAlpha', 0.8);
        set(get(get(h_antenna,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        antenna_handles = [antenna_handles; h_antenna];

        % Direction arrow (hidden from legend)
        q = quiver3(antenna_position(1), antenna_position(2), antenna_position(3), ...
            v(1), v(2), v(3), ...
            'LineWidth', 2, 'Color', rgb('Magenta'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

        % Base sphere
        [X,Y,Z] = sphere(12);
        radius = 0.025;
        X = X * radius + antenna_position(1);
        Y = Y * radius + antenna_position(2);
        Z = Z * radius + antenna_position(3);
        h_sphere = surf(X, Y, Z, 'FaceColor', rgb('DarkMagenta'), 'EdgeColor', 'none');
        set(get(get(h_sphere,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Reaction Wheel visualization with attitude transformation
rw_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel > 0
    for i_rw = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
        RW = mission.true_SC{i_SC}.true_SC_reaction_wheel{i_rw};
        wheel_radius = RW.radius;
        wheel_thickness = wheel_radius * 0.4;

        % Apply attitude transformation to wheel location and orientation
        location = RW.location;
        orientation = RW.orientation / norm(RW.orientation);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * orientation')';
        end

        [X,Y,Z] = cylinder(wheel_radius, 24);
        Z = Z * wheel_thickness - wheel_thickness/2;

        % Rotation matrix calculation
        v = orientation;
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        % Apply rotation and translation
        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + location(1);
                Y(i,j) = p(2) + location(2);
                Z(i,j) = p(3) + location(3);
            end
        end

        h_wheel = surf(X, Y, Z, 'FaceColor', rgb('Cyan'), 'EdgeColor', rgb('DarkCyan'), 'FaceAlpha', 0.7);
        set(get(get(h_wheel,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        rw_handles = [rw_handles; h_wheel];

        % Direction arrow (hidden from legend)
        q = quiver3(location(1), location(2), location(3), ...
            orientation(1), orientation(2), orientation(3), ...
            'LineWidth', 1.5, 'Color', rgb('DarkCyan'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

% Micro Thruster visualization with attitude transformation
mt_handles = [];
if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster > 0
    for i_mt = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
        MT = mission.true_SC{i_SC}.true_SC_micro_thruster{i_mt};

        % Apply attitude transformation to thruster location and orientation
        location = MT.location;
        orientation = MT.orientation / norm(MT.orientation);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * orientation')';
        end

        % Create distinct shape for micro thrusters
        [X,Y,Z] = cylinder([0.01, 0.015], 8);
        len = 0.04;
        Z = Z * len;

        v = orientation;
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + location(1);
                Y(i,j) = p(2) + location(2);
                Z(i,j) = p(3) + location(3);
            end
        end

        % Changed color to bright purple for better visibility
        h_thruster = surf(X, Y, Z, 'FaceColor', rgb('MediumPurple'), 'EdgeColor', rgb('DarkViolet'), 'FaceAlpha', 0.9);
        set(get(get(h_thruster,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        mt_handles = [mt_handles; h_thruster];

        % Direction arrow (hidden from legend)
        q = quiver3(location(1), location(2), location(3), ...
            v(1), v(2), v(3), ...
            'LineWidth', 1.5, 'Color', rgb('DarkViolet'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

        % Add a plume effect if thruster is firing (check if property exists and status is firing)
        if isprop(MT, 'flag_executive') && MT.flag_executive
            % Create a smaller plume effect for micro thrusters
            plume_len = 0.1;
            [X_plume,Y_plume,Z_plume] = cylinder([0.005, 0.02], 8);
            Z_plume = Z_plume * plume_len - 0.005;  % Offset slightly to start at the nozzle

            % Transform plume to thruster orientation
            for i = 1:size(X_plume,1)
                for j = 1:size(X_plume,2)
                    p = R * [X_plume(i,j); Y_plume(i,j); Z_plume(i,j)];
                    X_plume(i,j) = p(1) + location(1);
                    Y_plume(i,j) = p(2) + location(2);
                    Z_plume(i,j) = p(3) + location(3);
                end
            end

            h_plume = surf(X_plume, Y_plume, Z_plume, 'FaceColor', rgb('Violet'), 'EdgeColor', 'none', 'FaceAlpha', 0.3);
            set(get(get(h_plume,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        end
    end
end

% Chemical Thruster visualization with attitude transformation
ct_handles = [];
if isfield(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists, 'num_chemical_thruster') && ...
        mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster > 0
    for i_ct = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
        CT = mission.true_SC{i_SC}.true_SC_chemical_thruster{i_ct};

        % Apply attitude transformation to thruster location and orientation
        location = CT.location;
        orientation = CT.orientation / norm(CT.orientation);

        % Apply attitude rotation if available
        if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'rotation_matrix')
            location = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * (location - mission.true_SC{i_SC}.true_SC_body.location_COM)' )' + mission.true_SC{i_SC}.true_SC_body.location_COM;
            orientation = (mission.true_SC{i_SC}.true_SC_adc.rotation_matrix * orientation')';
        end

        % Make chemical thrusters significantly larger and more distinct than micro thrusters
        % Base shape - nozzle (inverted cone)
        [X,Y,Z] = cylinder([0.05, 0.025], 16);
        nozzle_len = 0.1;
        Z = Z * nozzle_len;

        % Add thrust chamber (cylinder)
        [X2,Y2,Z2] = cylinder(0.05, 16);
        chamber_len = 0.08;
        Z2 = Z2 * chamber_len + nozzle_len;

        X = [X; X2];
        Y = [Y; Y2];
        Z = [Z; Z2];

        v = orientation;
        if abs(v(3) - 1) < 1e-10
            R = eye(3);
        elseif abs(v(3) + 1) < 1e-10
            R = diag([1, 1, -1]);
        else
            axis_of_rotation = cross([0,0,1], v);
            axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);
            angle = acos(v(3));
            K = [0, -axis_of_rotation(3), axis_of_rotation(2);
                axis_of_rotation(3), 0, -axis_of_rotation(1);
                -axis_of_rotation(2), axis_of_rotation(1), 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end

        for i = 1:size(X,1)
            for j = 1:size(X,2)
                p = R * [X(i,j); Y(i,j); Z(i,j)];
                X(i,j) = p(1) + location(1);
                Y(i,j) = p(2) + location(2);
                Z(i,j) = p(3) + location(3);
            end
        end

        % Use a bright, distinct color for chemical thrusters - bright red
        h_thruster = surf(X, Y, Z, 'FaceColor', rgb('FireBrick'), 'EdgeColor', rgb('Black'), 'FaceAlpha', 1.0);
        set(get(get(h_thruster,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        ct_handles = [ct_handles; h_thruster];

        % Direction arrow (hidden from legend) - make it more visible
        q = quiver3(location(1), location(2), location(3), ...
            v(1), v(2), v(3), ...
            'LineWidth', 3, 'Color', rgb('Black'), 'AutoScaleFactor', obj.plot_parameters.quiver_auto_scale_factor);
        set(get(get(q,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

        % Add a plume effect if thruster is firing
        if isprop(CT, 'thruster_state') && strcmp(CT.thruster_state, 'firing')
            % Create a larger, more distinctive plume effect
            plume_len = 0.4; % Longer plume for chemical thrusters
            [X_plume,Y_plume,Z_plume] = cylinder([0.02, 0.1], 16);
            Z_plume = Z_plume * plume_len - 0.02;  % Offset slightly to start at the nozzle

            % Transform plume to thruster orientation
            for i = 1:size(X_plume,1)
                for j = 1:size(X_plume,2)
                    p = R * [X_plume(i,j); Y_plume(i,j); Z_plume(i,j)];
                    X_plume(i,j) = p(1) + location(1);
                    Y_plume(i,j) = p(2) + location(2);
                    Z_plume(i,j) = p(3) + location(3);
                end
            end

            h_plume = surf(X_plume, Y_plume, Z_plume, 'FaceColor', rgb('Yellow'), 'EdgeColor', 'none', 'FaceAlpha', 0.6);
            set(get(get(h_plume,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        end
    end
end

% Create dummy invisible objects for legend
hold on;

% Body
plot3(NaN, NaN, NaN, 'Color', rgb('LightGrey'), 'LineWidth', 2, 'DisplayName', 'Body');

% Solar Panels
for i_SP = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
    plot3(NaN, NaN, NaN, 'Color', rgb('LightBlue'), 'LineWidth', 2, ...
        'DisplayName', ['Solar Panel ', num2str(i_SP)]);
end

% Cameras
if ~isempty(camera_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('ForestGreen'), 'LineWidth', 2, ...
        'DisplayName', ['Cameras (x', num2str(length(camera_handles)), ')']);
end

% Star Trackers
if ~isempty(st_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('Gold'), 'LineWidth', 2, ...
        'DisplayName', ['Star Trackers (x', num2str(length(st_handles)), ')']);
end

% Sun Sensors
if ~isempty(ss_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('Yellow'), 'LineWidth', 2, ...
        'DisplayName', ['Sun Sensors (x', num2str(length(ss_handles)), ')']);
end

% Science Radars
if ~isempty(radar_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('Orange'), 'LineWidth', 2, ...
        'DisplayName', ['Science Radars (x', num2str(length(radar_handles)), ')']);
end

% Radio Antennas
if ~isempty(antenna_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('Magenta'), 'LineWidth', 2, ...
        'DisplayName', ['Radio Antennas (x', num2str(length(antenna_handles)), ')']);
end

% Reaction Wheels
if ~isempty(rw_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('Cyan'), 'LineWidth', 2, ...
        'DisplayName', ['Reaction Wheels (x', num2str(length(rw_handles)), ')']);
end

% Micro Thrusters
if ~isempty(mt_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('MediumPurple'), 'LineWidth', 2, ...
        'DisplayName', ['Micro Thrusters (x', num2str(length(mt_handles)), ')']);
end

% Chemical Thrusters
if ~isempty(ct_handles)
    plot3(NaN, NaN, NaN, 'Color', rgb('Black'), 'LineWidth', 2, ...
        'DisplayName', ['Chemical Thrusters (x', num2str(length(ct_handles)), ')']);
end

% % Add frame coordinates that stay fixed for reference
% % Fixed coordinate axes (shown with thin lines)
% fixed_axis_length = 0.7; % Length of fixed coordinate axes
% quiver3(0, 0, 0, fixed_axis_length, 0, 0, 'LineWidth', 1, 'Color', [0.7 0 0], 'LineStyle', ':', 'AutoScaleFactor', 1, 'MaxHeadSize', 0.3);
% quiver3(0, 0, 0, 0, fixed_axis_length, 0, 'LineWidth', 1, 'Color', [0 0.7 0], 'LineStyle', ':', 'AutoScaleFactor', 1, 'MaxHeadSize', 0.3);
% quiver3(0, 0, 0, 0, 0, fixed_axis_length, 'LineWidth', 1, 'Color', [0 0 0.7], 'LineStyle', ':', 'AutoScaleFactor', 1, 'MaxHeadSize', 0.3);

% Plot settings
axis equal
%     xlim([-15 15])
%     ylim([-15 15])
%     zlim([-15 15])
grid on
% Create a legend in a more visible position
lgd = legend('Location', 'southeast');
% Adjust legend position to be more central and not overlap with dashboard
%     lgd.Position = [0.7 0.2 0.1 0.6];
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
set(gca, 'FontSize', obj.plot_parameters.standard_font_size, 'FontName', obj.plot_parameters.standard_font_type)

% Set a fixed view angle that makes rotations more visible
if is_realtime_update
    view([-37.5, 30]); % Use a fixed view angle for better viewing of rotations
else
    view(3) % Default 3D view
end

title(['SC ',num2str(i_SC), ', ConOps Mode = ',mission.true_SC{i_SC}.software_SC_executive.this_sc_mode], 'FontSize', mission.storage.plot_parameters.standard_font_size)
hold off

end
