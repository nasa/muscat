function func_plot_orbit_visualization(mission)

kd = mission.storage.k_storage;

%% Orbit Vizualization
plot_handle = figure('Name','SC in Target-centered Frame');
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 3D Orbit Vizualization % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(1,2,1)

for i_SC = 1:1:mission.num_SC
    color_index = mod(i_SC-1, length(mission.storage.plot_parameters.color_array)) + 1;
    plot3(mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,1), mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,2), mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,3), '-','LineWidth',2, 'Color',mission.storage.plot_parameters.color_array(color_index), 'DisplayName',mission.true_SC{i_SC}.true_SC_body.name)
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
            firing_points = mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(firing_indices,:);
            
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
                            quiver3(mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(k,1), ...
                                   mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(k,2), ...
                                   mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(k,3), ...
                                   deltaV(1), deltaV(2), deltaV(3), ...
                                   'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'HandleVisibility', 'off');
                        end
                    end
                end
            end
        end
    end
end

i_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target;
func_plot_target_shape(i_target, mission);

grid on

view(3)
axis equal
legend('Location','southwest')
xlabel('X axis [km]')
ylabel('Y axis [km]')
zlabel('Z axis [km]')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
title('SC in Target-centered Frame','FontSize',mission.storage.plot_parameters.title_font_size)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Relative Distance from Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,2)
hold on

for i_SC = 1:1:mission.num_SC
    plot(mission.true_time.store.time(1:kd), vecnorm( mission.true_SC{i_SC}.true_SC_navigation.store.position_relative_target(1:kd,:), 2, 2),  '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name,' (True)'])

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Relative Speed from Target % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,4)
hold on

for i_SC = 1:1:mission.num_SC
    plot(mission.true_time.store.time(1:kd), vecnorm( mission.true_SC{i_SC}.true_SC_navigation.store.velocity_relative_target(1:kd,:), 2, 2),  '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(i_SC), 'DisplayName', [mission.true_SC{i_SC}.true_SC_body.name,' (True)'])

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



if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_Orbit_Vizualization.png'])
end

end