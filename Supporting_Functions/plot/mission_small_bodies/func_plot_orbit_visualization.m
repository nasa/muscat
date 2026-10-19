function func_plot_orbit_visualization(mission)

kd = mission.storage.k_storage;

%% Orbit Vizualization
plot_handle = figure('Name','SC in Target-centered J2000 Frame');
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 3D Orbit Vizualization % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(1,2,1)
hold on

func_plot_orbit_visualization_subplot1(mission, kd)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Relative Distance from Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,2)
hold on

func_plot_orbit_visualization_subplot2(mission, kd)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Relative Speed from Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,4)
hold on

func_plot_orbit_visualization_subplot3(mission, kd)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name,'_Orbit_Vizualization.png'];
    saveas(plot_handle,plot_file_name)
end


%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*
    
    ch = Chapter;
    i_target = mission.true_SC{1}.true_SC_orbit_propagation.index_relative_target;
    ch.Title = 'Results: Orbit Vizualization';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % 3D Orbit Vizualization % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sec = Section;
    sec.Title = '3D Orbit Vizualization';

    plot_handle = figure(101);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_visualization_subplot1(mission, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.ZLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(1,2,1) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)
    
    if mission.num_SC == 1
        para = Paragraph(['SC orbit in ',mission.true_target{i_target}.name,'-centered Frame for the entire duration of the simulation are shown.']);
    else
        para = Paragraph(['All SC orbit in ',mission.true_target{i_target}.name,'-centered Frame for the entire duration of the simulation are shown.']);
    end
    append(sec,para)

    
    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    % Add to Excel
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = 'Time [sec]';
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_time.store.time(i_k);
    end

    para = Paragraph('- "Time [sec]" represents simulation time.');
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = 'Date [UTC]';
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = sec2cal(mission.true_time.store.date(i_k));
    end

    para = Paragraph('- "Date [UTC]" represents simulation time as Date and Time in UTC.');
    append(sec,para)


    for i_SC = 1:1:mission.num_SC

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Position [km]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(i_k,:); % [km];
        end

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Velocity [km/sec]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(i_k,:); % [km/sec];
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' Position [km]" represents SC''s position in Sun-centered J2000 frame (from SC''s True_SC_Orbit_Propagation class). This vector has 3 elements: [X, Y, Z]']);
        append(sec,para)

        para = Paragraph(['- "SC ',num2str(i_SC),' Velocity [km/sec]" represents SC''s velocity in Sun-centered J2000 frame (from SC''s True_SC_Orbit_Propagation class). This vector has 3 elements: [X, Y, Z]']);
        append(sec,para)

    end

    for i_target = 1:1:mission.num_target

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = [mission.true_target{i_target}.name,' Position [km]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_target{i_target}.store.position(i_k,:); % [km];
        end

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = [mission.true_target{i_target}.name,' Velocity [km/sec]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_target{i_target}.store.velocity(i_k,:); % [km/sec];
        end

        para = Paragraph(['- "',mission.true_target{i_target}.name,' Position [km]" represents Target-center''s position in Sun-centered J2000 frame (from True_Target class). This vector has 3 elements: [X, Y, Z]']);
        append(sec,para)

        para = Paragraph(['- "',mission.true_target{i_target}.name,' Velocity [km/sec]" represents Target-center''s velocity in Sun-centered J2000 frame (from True_Target class). This vector has 3 elements: [X, Y, Z]']);
        append(sec,para)


    end

    for i_SS_body = 1:1:mission.true_solar_system.num_SS_body

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = [mission.true_solar_system.SS_body{i_SS_body}.name,' Position [km]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_solar_system.store.SS_body{i_SS_body}.position(i_k,:); % [km];
        end

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = [mission.true_solar_system.SS_body{i_SS_body}.name,' Velocity [km/sec]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_solar_system.store.SS_body{i_SS_body}.velocity(i_k,:); % [km/sec];
        end

        para = Paragraph(['- "',mission.true_solar_system.SS_body{i_SS_body}.name,' Position [km]" represents Solar System body-center''s position in Sun-centered J2000 frame (from True_Solar_System class). This vector has 3 elements: [X, Y, Z]']);
        append(sec,para)

        para = Paragraph(['- "',mission.true_solar_system.SS_body{i_SS_body}.name,' Velocity [km/sec]" represents Solar System body-center''s velocity in Sun-centered J2000 frame (from True_Solar_System class). This vector has 3 elements: [X, Y, Z]']);
        append(sec,para)


    end

    append(ch,sec)



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Relative Distance from Target % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = ['Relative Distance from ',mission.true_target{i_target}.name,'''s Center'];

    plot_handle = figure(102);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_visualization_subplot2(mission, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,2,2) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph(['Distance between SC and ',mission.true_target{i_target}.name,'-center for the entire duration of the simulation are shown.']);
    append(sec,para)

    para = Paragraph('Both True values (from SC''s True_SC_Orbit_Propagation class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)

    append(ch,sec)
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Relative Speed from Target % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sec = Section;
    sec.Title = ['Relative Speed from ',mission.true_target{i_target}.name,'''s Center'];

    plot_handle = figure(103);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_visualization_subplot3(mission, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    
    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,2,4) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph(['Relative speed between SC and ',mission.true_target{i_target}.name,'-center for the entire duration of the simulation are shown.']);
    append(sec,para)

    para = Paragraph('Both True values (from SC''s True_SC_Orbit_Propagation class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)

    append(ch,sec)
    
    append(mission.storage.report_parameters.report_handle,ch);

end

end


function func_plot_orbit_visualization_subplot1(mission, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 3D Orbit Vizualization % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i_SC = 1:1:mission.num_SC
    color_index = mod(i_SC-1, length(mission.storage.plot_parameters.color_array)) + 1;
    plot3(mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(1:kd,1), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(1:kd,2), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(1:kd,3), '-','LineWidth',2, 'Color',mission.storage.plot_parameters.color_array(color_index), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)
    hold on
end
hold on

% Add delta-V indicators where chemical thrusters are firing
for i_SC = 1:1:mission.num_SC
    % Check if chemical thrusters exist
    has_chemical_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster > 0;

    if has_chemical_thrusters
        firing_points = [];
        firing_indices = [];

        % Go through each thruster - OPTIMIZED APPROACH
        for i_CT = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
            % Check if thruster_state field exists
            if isfield(mission.true_SC{i_SC}.true_SC_chemical_thruster{i_CT}.store, 'thruster_state')
                thruster_states = mission.true_SC{i_SC}.true_SC_chemical_thruster{i_CT}.store.thruster_state;

                % Pre-process all thruster states at once instead of one by one
                is_firing = false(kd, 1);

                % Faster check by avoiding the loop when possible
                if iscell(thruster_states) && length(thruster_states) >= kd
                    % Convert cell array to a logical array indicating if firing
                    is_string = cellfun(@ischar, thruster_states(1:kd));
                    if any(is_string)
                        % Only process the string entries
                        string_indices = find(is_string);
                        is_firing_strings = false(length(string_indices), 1);

                        % Vectorized string comparison
                        for j = 1:length(string_indices)
                            idx = string_indices(j);
                            is_firing_strings(j) = strcmp(thruster_states{idx}, 'firing');
                        end

                        % Assign back to the full array
                        is_firing(string_indices(is_firing_strings)) = true;

                        % Find indices where firing is happening
                        new_firing_indices = find(is_firing);

                        % Add to our collection
                        if ~isempty(new_firing_indices)
                            firing_indices = unique([firing_indices, new_firing_indices']);
                        end
                    end
                end
            end
        end

        % Now convert the indices to points
        if ~isempty(firing_indices)
            firing_points = mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(firing_indices,:);

            % Plot firing points with small red dots - sparse representation
            if size(firing_points, 1) > 50
                % If too many points, sample them to avoid overcrowding
                sample_size = min(50, round(size(firing_points, 1) / 5));
                sample_indices = round(linspace(1, size(firing_points, 1), sample_size));
                plot3(firing_points(sample_indices,1), firing_points(sample_indices,2), firing_points(sample_indices,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name, ' ΔV points']);
            else
                plot3(firing_points(:,1), firing_points(:,2), firing_points(:,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name, ' ΔV points']);
            end

            % Check if deltaV direction data exists - only for sampled points to save memory
            has_orbit_control = isfield(mission.true_SC{i_SC}, 'software_SC_control_orbit');
            if has_orbit_control && isfield(mission.true_SC{i_SC}.software_SC_control_orbit.store, 'desired_control_DeltaV')
                % Only draw arrows for a sparse subset of points
                arrow_indices = firing_indices(round(linspace(1, length(firing_indices), min(20, length(firing_indices)))));

                for idx = 1:length(arrow_indices)
                    k = arrow_indices(idx);
                    if k <= size(mission.true_SC{i_SC}.software_SC_control_orbit.store.desired_control_DeltaV, 1)
                        deltaV = mission.true_SC{i_SC}.software_SC_control_orbit.store.desired_control_DeltaV(k,:);

                        % Only draw if deltaV is non-zero
                        if norm(deltaV) > 0
                            % Normalize to a small fixed length for visualization
                            deltaV = deltaV / norm(deltaV) * 0.5;  % 0.5 unit length arrows

                            % Draw small arrow showing direction
                            quiver3(mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(k,1), ...
                                mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(k,2), ...
                                mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(k,3), ...
                                deltaV(1), deltaV(2), deltaV(3), ...
                                'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
                        end
                    end
                end
            end
        end
    end
end

i_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
func_plot_target_shape(i_target, mission);

grid on

view(3)
axis equal
legend('Location','southwest')
xlabel('X axis [km]')
ylabel('Y axis [km]')
zlabel('Z axis [km]')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
% title('SC in Target-centered Frame','FontSize',mission.storage.plot_parameters.title_font_size)
title(['SC Orbit in ',mission.true_target{i_target}.name,'-centered J2000 Frame'],'FontSize',mission.storage.plot_parameters.title_font_size)
hold off

end

function func_plot_orbit_visualization_subplot2(mission, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Relative Distance from Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i_SC = 1:1:mission.num_SC
    plot(mission.true_time.store.time(1:kd), vecnorm( mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position_relative_target(1:kd,:), 2, 2),  '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name,' (True)'])
    hold on

    if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
        plot(mission.true_time.store.time(1:kd), vecnorm( mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_relative_target(1:kd,:), 2, 2),  '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name,' (Estimated)'])
    end
end

grid on
legend('Location','northeast')
xlabel('Time [sec]')
ylabel('Distance [km]')
title('SC - Target (center) Relative Distance','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_visualization_subplot3(mission, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Relative Speed from Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i_SC = 1:1:mission.num_SC
    plot(mission.true_time.store.time(1:kd), vecnorm( mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity_relative_target(1:kd,:), 2, 2),  '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name,' (True)'])
    hold on

    if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
        plot(mission.true_time.store.time(1:kd), vecnorm( mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_relative_target(1:kd,:), 2, 2),  '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name,' (Estimated)'])
    end
end

grid on
legend('Location','northeast')
xlabel('Time [sec]')
ylabel('Speed [km/sec]')
title('SC - Target (center) Relative Speed','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off


end

