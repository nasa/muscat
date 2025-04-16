function func_plot_attitude_actuator_performance(mission, i_SC)
% FUNC_PLOT_ATTITUDE_ACTUATOR_PERFORMANCE Visualize attitude actuator performance
%
% This function generates a comprehensive plot of all attitude actuator performance 
% metrics for a specified spacecraft, including:
%   1. Torques (desired control, disturbance components, and total)
%   2. Micro thruster performance (if present)
%   3. Chemical thruster disturbance torques (if present)
%   4. Reaction wheel performance (if present)
%
% INPUTS:
%   mission - The mission data structure containing all spacecraft information
%   i_SC    - Index of the spacecraft to plot
%
% The plot dynamically adapts based on which actuators are present on the spacecraft.
% For chemical thrusters, special visualization enhancements are used to make brief
% firing events more visible in the disturbance torque plot.
%
% Author: MuSCAT Team

kd = mission.storage.k_storage_attitude;

nb_row = 1; % One row for torques

% Initialize plot position counters
current_row = 2; % Start from second row since first row is always torques

has_thrusters = (mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster) > 1;
has_reaction_wheels = (mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel) > 1;

if has_thrusters
    nb_row = nb_row + 1;
    thruster_row = current_row;
    current_row = current_row + 1;
end

if has_reaction_wheels
    nb_row = nb_row + 1;
    rw_row = current_row;
end

nb_col = 3;

%% Plot Setup
plot_handle = figure('Name',['SC ',num2str(i_SC),' Attitude Actuator Performance']);
clf;
set(plot_handle, 'Color', [1, 1, 1]);
set(plot_handle, 'units', 'normalized', 'outerposition', [0, 0, 1, 1]);
set(plot_handle, 'PaperPositionMode', 'auto');

colors = {'r', 'g', 'b', 'c', 'm', 'y', 'k', [0.5, 0.5, 0.5]}; % Adding more colors

% First row : Desired, Actual, disturbance torques
%% Desired Control Torque
subplot(nb_row, nb_col, 1);
hold on;
if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')
    % Get the number of data points available for the desired control torque
    ctrl_data_length = min(kd, size(mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_control_torque, 1));

    if ctrl_data_length > 0
        for i = 1:3
            plot(mission.true_time.store.time_attitude(1:ctrl_data_length), ...
                mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_control_torque(1:ctrl_data_length, i), ...
                '-', 'LineWidth', 2, 'Color', colors{i}, ...
                'DisplayName', [char('X' + i - 1)]);
        end
    end
end
grid on;
xlabel('Time [sec]');
ylabel('Torque [Nm]');
title('Desired Control Torque');
legend('Location', 'southeast');
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
hold off;

%% Disturbance Torque with Individual Contributions (Component-wise)
subplot(nb_row, nb_col, 2);
hold on;

% Check for different disturbance sources and if they're enabled
has_SRP = isfield(mission.true_SC{i_SC}, 'true_SRP') && ...
    mission.true_SC{i_SC}.true_SRP.enable_SRP == 1 && ...
    isfield(mission.true_SC{i_SC}.true_SRP.store, 'disturbance_torque_SRP');

has_G2 = isfield(mission.true_SC{i_SC}, 'true_gravity_gradient') && ...
    mission.true_SC{i_SC}.true_gravity_gradient.enable_G2 == 1 && ...
    isfield(mission.true_SC{i_SC}.true_gravity_gradient.store, 'disturbance_torque_G2');

has_chemical_thruster = isfield(mission.true_SC{i_SC}, 'true_SC_chemical_thruster') && ...
    mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster > 0;

% Define line styles for different disturbance sources
line_styles = {'-', '--', ':'};
axis_names = {'X', 'Y', 'Z'};

% Plot component-wise disturbance torques if available
if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isfield(mission.true_SC{i_SC}.true_SC_adc.store, 'disturbance_torque')
    % Get the number of data points available for the disturbance torque
    dist_data_length = min(kd, size(mission.true_SC{i_SC}.true_SC_adc.store.disturbance_torque, 1));

    if dist_data_length > 0
        % Plot individual disturbance source components if available

        % 1. SRP components
        if has_SRP
            % Get the number of data points available for SRP torque
            srp_data_length = min(dist_data_length, size(mission.true_SC{i_SC}.true_SRP.store.disturbance_torque_SRP, 1));

            if srp_data_length > 0
                % Plot each component of SRP disturbance torque
                for axis = 1:3
                    plot(mission.true_time.store.time(1:srp_data_length), ...
                        mission.true_SC{i_SC}.true_SRP.store.disturbance_torque_SRP(1:srp_data_length, axis), ...
                        line_styles{1}, 'LineWidth', 1.5, 'Color', colors{axis}, ...
                        'DisplayName', ['SRP-' axis_names{axis}]);
                end
            end
        end

        % 2. Gravity Gradient components
        if has_G2
            % Get the number of data points available for gravity gradient torque
            g2_data_length = min(dist_data_length, size(mission.true_SC{i_SC}.true_gravity_gradient.store.disturbance_torque_G2, 1));

            if g2_data_length > 0
                % Plot each component of Gravity Gradient disturbance torque
                for axis = 1:3
                    plot(mission.true_time.store.time(1:g2_data_length), ...
                        mission.true_SC{i_SC}.true_gravity_gradient.store.disturbance_torque_G2(1:g2_data_length, axis), ...
                        line_styles{2}, 'LineWidth', 1.5, 'Color', colors{axis}, ...
                        'DisplayName', ['GG-' axis_names{axis}]);
                end
            end
        end

        % 3. Chemical Thruster components
        if has_chemical_thruster
            % Initialize chemical thruster disturbance torque array
            ct_torque = zeros(dist_data_length, 3);
            ct_data_valid = false;

            % Process each chemical thruster
            for i_CT = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                % Get thruster data using parentheses for object access
                thruster = mission.true_SC{i_SC}.true_SC_chemical_thruster{i_CT};

                % Check if the control_torque_CT field exists in storage
                if isfield(thruster.store, 'control_torque_CT')
                    % Get the length of thruster data available
                    ct_data_length = min(dist_data_length, size(thruster.store.control_torque_CT, 1));

                    if ct_data_length > 0
                        % We have valid data for the thruster
                        ct_data_valid = true;
                        
                        % Add the control torque from this thruster (already in body frame)
                        ct_torque(1:ct_data_length, :) = ct_torque(1:ct_data_length, :) + thruster.store.control_torque_CT(1:ct_data_length, :);
                    end
                end
            end

            if ct_data_valid
                % Find time points where thruster torque exists
                nonzero_indices = find(sum(abs(ct_torque), 2) > 1e-20);  % Reduced threshold to catch smaller values

                [max_torque, max_idx] = max(sum(abs(ct_torque), 2));
                
                if ~isempty(nonzero_indices)
                    % Plot each component of thruster disturbance torque at firing times with distinct colors
                    % Use a different set of colors for chemical thrusters to distinguish from SRP
                    ct_colors = {[0, 0.5, 0], [0.8500, 0.3250, 0.0980], [0.4940, 0.1840, 0.5560]}; % Green, Orange, Purple
                    axis_names = {'X', 'Y', 'Z'};
                    
                    % Special case for a single firing event - always show all three axes
                    if length(nonzero_indices) == 1
                        % Clear previous plot to avoid confusion
                        hold on;
                        
                        % Plot one point for each axis (X, Y, Z) regardless of value
                        for axis = 1:3
                            plot(mission.true_time.store.time(nonzero_indices), ...
                                ct_torque(nonzero_indices, axis), ...
                                '.', 'MarkerSize', 20, 'Color', ct_colors{axis}, ...
                                'DisplayName', ['CT-' axis_names{axis}]);
                        end
                    else
                        % Normal case for multiple firing events
                        for axis = 1:3
                            % Use clear markers and lines but keep it simple
                            plot(mission.true_time.store.time(nonzero_indices), ...
                                ct_torque(nonzero_indices, axis), ...
                                'o-', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', ct_colors{axis}, ...
                                'Color', ct_colors{axis}, ...
                                'DisplayName', ['CT-' axis_names{axis}]);
                        end
                    end
                else
                    % No non-zero values found - add a message directly on the plot
                    text(0.5, 0.7, 'No thruster events detected with torque > 1e-20 Nm', ...
                         'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'red');
                    
                    % Add suggestion for troubleshooting
                    text(0.5, 0.6, 'Check if thrusters fired at all or if torque calculation is working', ...
                         'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                         'FontSize', 8);
                end
            end
        end

        % Set y-axis to logarithmic scale to better visualize different orders of magnitude
        set(gca, 'YScale', 'log');

        % Add grid to the logarithmic plot (more helpful)
        grid on;

        % Calculate reasonable y-axis limits based on the data
        all_data = [];

        % Collect all data for determining y-axis limits
        if has_SRP && exist('srp_data_length', 'var') && srp_data_length > 0
            all_data = [all_data; abs(reshape(mission.true_SC{i_SC}.true_SRP.store.disturbance_torque_SRP(1:srp_data_length,:), [], 1))];
        end

        if has_G2 && exist('g2_data_length', 'var') && g2_data_length > 0
            all_data = [all_data; abs(reshape(mission.true_SC{i_SC}.true_gravity_gradient.store.disturbance_torque_G2(1:g2_data_length,:), [], 1))];
        end

        if has_chemical_thruster && exist('ct_torque', 'var') && exist('nonzero_indices', 'var') && ~isempty(nonzero_indices)
            all_data = [all_data; abs(reshape(ct_torque(nonzero_indices,:), [], 1))];
        end

        % Filter out zeros and set reasonable limits
        all_data = all_data(all_data > 0);
        if ~isempty(all_data)
            min_y = min(all_data);
            max_y = max(all_data);

            % Handle case where min or max is not finite
            if ~isfinite(min_y) || min_y <= 0
                min_y = 1e-10;
            end

            if ~isfinite(max_y) || max_y <= min_y
                max_y = min_y * 1000;
            end
            
            % Make special adjustments for chemical thruster torque
            % to ensure it's visible in the plot
            if has_chemical_thruster && exist('ct_torque', 'var') && exist('nonzero_indices', 'var') && ~isempty(nonzero_indices)
                % Get the min/max of chemical thruster torque
                ct_min = min(abs(ct_torque(nonzero_indices, :)));
                ct_min = ct_min(ct_min > 0);
                
                if ~isempty(ct_min)
                    % Ensure the min y-value shows the thruster torque
                    if min(ct_min) < min_y
                        min_y = min(ct_min) / 2;  % Make room below
                    end
                    
                    % Find max of CT torque
                    ct_max = max(abs(ct_torque(nonzero_indices, :)));
                    if max(ct_max) > max_y
                        max_y = max(ct_max) * 2;  % Make room above
                    end
                end
            end

            % Set y-axis limits with padding
            ylim([min_y/10, max_y*10]);
        else
            % Default limits if no data
            ylim([1e-10, 1e-3]);
        end

        % Create a customized legend with grouping
        legend_entries = findobj(gca, 'Type', 'line');
        legend_names = get(legend_entries, 'DisplayName');

        % Only create legend if there are entries
        if ~isempty(legend_entries) && iscell(legend_names)
            legend(legend_entries, legend_names, 'Location', 'southeast', 'NumColumns', 1);
        end
    else
        % No data available - just show a message
        text(0.5, 0.5, 'Disturbance torque data not available', ...
            'HorizontalAlignment', 'center', 'FontSize', 12);
    end

    xlabel('Time [sec]');
    ylabel('Disturbance Torque [Nm]');
    
    % Use the original title
    title('Disturbance Torque Components (Log Scale)');
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
    hold off;

    %% True Torque
    subplot(nb_row, nb_col, 3);
    hold on;

    % Check if the total_torque field exists
    if isfield(mission.true_SC{i_SC}.true_SC_adc.store, 'total_torque')
        % Get available data length
        total_torque_length = min(kd, size(mission.true_SC{i_SC}.true_SC_adc.store.total_torque, 1));

        if total_torque_length > 0
            % Plot the total torque if available
            for i = 1:3
                % True Torque
                plot(mission.true_time.store.time_attitude(1:total_torque_length), ...
                    mission.true_SC{i_SC}.true_SC_adc.store.total_torque(1:total_torque_length, i), ...
                    '-', 'LineWidth', 1.5, 'Color', colors{i}, ...
                    'DisplayName', [char('X' + i - 1)]);
            end
        end
    else
        % Field is missing - check if we can calculate it from disturbance and control torques
        has_data = false;

        % Try to construct from control and disturbance torques if available
        if isfield(mission.true_SC{i_SC}.true_SC_adc.store, 'control_torque') && ...
                isfield(mission.true_SC{i_SC}.true_SC_adc.store, 'disturbance_torque')

            % Get available data length for both control and disturbance
            control_length = size(mission.true_SC{i_SC}.true_SC_adc.store.control_torque, 1);
            disturbance_length = size(mission.true_SC{i_SC}.true_SC_adc.store.disturbance_torque, 1);
            total_length = min([kd, control_length, disturbance_length]);

            if total_length > 0
                control = mission.true_SC{i_SC}.true_SC_adc.store.control_torque(1:total_length, :);
                disturbance = mission.true_SC{i_SC}.true_SC_adc.store.disturbance_torque(1:total_length, :);

                % Plot reconstructed total torque
                for i = 1:3
                    plot(mission.true_time.store.time_attitude(1:total_length), ...
                        control(:, i) + disturbance(:, i), ...
                        '-', 'LineWidth', 1.5, 'Color', colors{i}, ...
                        'DisplayName', [char('X' + i - 1)]);
                end
                has_data = true;
            end
        end

        if ~has_data
            % No data available - just show a message
            text(0.5, 0.5, 'Total torque data not available', ...
                'HorizontalAlignment', 'center', 'FontSize', 12);
            set(gca, 'Visible', 'off');
        end
    end

    grid on;
    xlabel('Time [sec]');
    ylabel('True Torque [Nm]');
    title('True Torque on Spacecraft');
    legend('Location', 'southeast');
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
    hold off;

    if has_thrusters
        %% Micro Thruster Torque
        subplot(nb_row, nb_col, (thruster_row-1)*nb_col + 1);
        hold on;

        % Get the number of thrusters
        num_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster;

        if num_thrusters > 0
            % Initialize total torque array
            total_torque = zeros(kd, 3);  % 3 for X, Y, Z components

            % Determine the minimum data length available
            min_thruster_data_length = kd;
            for i_HW = 1:num_thrusters
                if isfield(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store, 'control_torque_MT')
                    thruster_data_length = size(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.control_torque_MT, 1);
                    min_thruster_data_length = min(min_thruster_data_length, thruster_data_length);
                end
            end

            if min_thruster_data_length > 0
                % Resize array to actual data length
                total_torque = zeros(min_thruster_data_length, 3);

                % Sum torques from all thrusters
                for i_HW = 1:num_thrusters
                    if isfield(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store, 'control_torque_MT') && ...
                            size(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.control_torque_MT, 1) >= min_thruster_data_length
                        total_torque = total_torque + mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.control_torque_MT(1:min_thruster_data_length, :);
                    end
                end

                % Plot total torque components
                for i = 1:3
                    plot(mission.true_time.store.time_attitude(1:min_thruster_data_length), ...
                        total_torque(:, i), '-', 'LineWidth', 1.5, ...
                        'Color', colors{i}, ...
                        'DisplayName', [char('X' + i - 1)]);
                end
            end
        end

        grid on;
        xlabel('Time [sec]');
        ylabel('Torque [Nm]');
        title('Micro Thruster Torque');
        legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;

        %% Thrust Magnitude Plus Limits
        subplot(nb_row, nb_col, (thruster_row-1)*nb_col + 2);
        hold on;

        % Find the maximum thrust across all thrusters for global max line
        global_max_thrust = 0;

        for i_HW = 1:num_thrusters
            if isfield(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store, 'true_commanded_thrust')
                thrust_data_length = size(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.true_commanded_thrust, 1);

                if thrust_data_length > 0
                    thrust_data_length = min(thrust_data_length, kd);
                    thrust_actual = mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.true_commanded_thrust(1:thrust_data_length);

                    plot(mission.true_time.store.time_attitude(1:thrust_data_length), ...
                        thrust_actual, '-', 'LineWidth', 1.5, ...
                        'DisplayName', [mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.name]);

                    max_thrust = mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.maximum_thrust;
                    min_thrust = mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.minimum_thrust;

                    % Update global max thrust if this one is larger
                    global_max_thrust = max(global_max_thrust, max_thrust);

                    % Plot individual thruster limits without legend entries
                    plot(mission.true_time.store.time_attitude(1:thrust_data_length), ...
                        ones(1,thrust_data_length) * max_thrust, ':', 'LineWidth', 1, 'HandleVisibility', 'off');
                    plot(mission.true_time.store.time_attitude(1:thrust_data_length), ...
                        ones(1,thrust_data_length) * -min_thrust, ':', 'LineWidth', 1, 'HandleVisibility', 'off');
                end
            end
        end

        % Add a global max thrust limit line with legend entry if we found any thrusters
        if global_max_thrust > 0 && kd > 0
            plot(mission.true_time.store.time_attitude(1:min(kd, length(mission.true_time.store.time_attitude))), ...
                ones(1, min(kd, length(mission.true_time.store.time_attitude))) * global_max_thrust, ...
                ':', 'LineWidth', 2.5, 'Color', 'r', ...
                'DisplayName', 'Max Thrust Limit');
        end

        grid on;
        xlabel('Time [sec]');
        ylabel('Thrust [N]');
        title('Micro Thrusters Output thrust');
        legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;

        %% Thruster Firing Status
        subplot(nb_row, nb_col, (thruster_row-1)*nb_col + 3);
        hold on;

        % Get number of thrusters
        num_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster;

        % Plot horizontal lines for each thruster WITHOUT adding them to legend
        for i = 1:num_thrusters
            % Get time range to plot
            if kd > 0
                plot([mission.true_time.store.time_attitude(1), mission.true_time.store.time_attitude(min(kd, length(mission.true_time.store.time_attitude)))], ...
                    [i i], '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5, 'HandleVisibility', 'off');
            end
        end

        % Plot firing status dots
        for i_HW = 1:num_thrusters
            if isfield(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store, 'true_commanded_thrust')
                thruster_data_length = min(size(mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.true_commanded_thrust, 1), kd);

                if thruster_data_length > 0
                    thrust = mission.true_SC{i_SC}.true_SC_micro_thruster{i_HW}.store.true_commanded_thrust(1:thruster_data_length);

                    % Find times when thruster is firing (using norm > 0)
                    firing_indices = find(abs(thrust) > 1e-10); % Small threshold to account for numerical precision

                    if ~isempty(firing_indices)
                        % Set HandleVisibility to 'off' to exclude from legend
                        scatter(mission.true_time.store.time_attitude(firing_indices), ...
                            i_HW * ones(size(firing_indices)), ...
                            20, 'filled', ...
                            'HandleVisibility', 'off');
                    end
                end
            end
        end

        % Customize plot
        ylim([0, num_thrusters + 1]);
        yticks(1:num_thrusters);
        yticklabels(arrayfun(@(x) ['T' num2str(x)], 1:num_thrusters, 'UniformOutput', false));
        grid on;
        xlabel('Time [sec]');
        ylabel('Thruster ID');
        title('Micro Thruster Firing Status (Boolean)');
        % Legend removed as thruster IDs are shown on y-axis
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
    end

    if has_reaction_wheels
        %% Reaction Wheel Torque (Summed per Axis)
        subplot(nb_row, nb_col, (rw_row-1)*nb_col + 1);
        hold on;

        % Get number of reaction wheels
        num_rw = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel;

        if num_rw > 0
            % Initialize total torque
            total_torque = zeros(kd, 3);

            % Determine minimum data length available across all wheels
            min_rw_data_length = kd;
            for i = 1:num_rw
                if isfield(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store, 'actual_torque')
                    rw_data_length = size(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.actual_torque, 1);
                    min_rw_data_length = min(min_rw_data_length, rw_data_length);
                end
            end

            if min_rw_data_length > 0
                % Resize total_torque to match available data
                total_torque = zeros(min_rw_data_length, 3);

                % Sum torques from all reaction wheels
                for i = 1:num_rw
                    if isfield(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store, 'actual_torque') && ...
                            size(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.actual_torque, 1) >= min_rw_data_length
                        total_torque = total_torque + mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.actual_torque(1:min_rw_data_length, :);
                    end
                end

                % Plot total torque per axis
                for axis = 1:3
                    plot(mission.true_time.store.time_attitude(1:min_rw_data_length), total_torque(:, axis), ...
                        '-', 'LineWidth', 1.5, 'Color', colors{axis}, ...
                        'DisplayName', [char('X' + axis - 1)]);
                end
            end
        end

        grid on;
        xlabel('Time [sec]');
        ylabel('Torque [Nm]');
        title('Total Reaction Wheel Torque');
        legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;

        %% Commanded Angular Acceleration
        subplot(nb_row, nb_col, (rw_row-1)*nb_col + 2);
        hold on;

        for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
            if isfield(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store, 'commanded_angular_acceleration')
                rw_data_length = size(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.commanded_angular_acceleration, 1);

                if rw_data_length > 0
                    rw_data_length = min(rw_data_length, kd);

                    % Commanded Angular Acceleration
                    plot(mission.true_time.store.time_attitude(1:rw_data_length), ...
                        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.commanded_angular_acceleration(1:rw_data_length), ...
                        '-', 'LineWidth', 2, 'Color', colors{i}, ...
                        'DisplayName', [mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.name]);
                end
            end
        end

        grid on;
        xlabel('Time [sec]');
        ylabel('Angular Acceleration [rad/s^2]');
        title('Reaction Wheel Commanded Angular Acceleration');
        legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;

        %% Angular Velocity with Max Limits
        subplot(nb_row, nb_col, (rw_row-1)*nb_col + 3);
        hold on;

        for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
            if isfield(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store, 'angular_velocity')
                rw_vel_data_length = size(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.angular_velocity, 1);

                if rw_vel_data_length > 0
                    rw_vel_data_length = min(rw_vel_data_length, kd);

                    % Angular Velocity
                    plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                        mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.angular_velocity(1:rw_vel_data_length), ...
                        '-', 'LineWidth', 1.5, 'Color', colors{i}, ...
                        'DisplayName', [mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.name]);

                    % Max Angular Velocity Limits
                    if isfield(mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store, 'max_angular_velocity')
                        max_vel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.store.max_angular_velocity;

                        % Only add a legend entry for the first reaction wheel's limits
                        % to avoid cluttering the legend
                        if i == 1
                            % Add max velocity line with legend entry
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * max_vel, ':', 'LineWidth', 1.5, 'Color', 'k', ...
                                'DisplayName', 'Max Velocity Limit');

                            % Add 80% saturation margin line
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * (0.8 * max_vel), '--', 'LineWidth', 1.5, 'Color', 'k', ...
                                'DisplayName', '80% Saturation Margin');

                            % Add negative limit lines with legend entries
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * -max_vel, ':', 'LineWidth', 1.5, 'Color', 'k', ...
                                'HandleVisibility', 'off');
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * (-0.8 * max_vel), '--', 'LineWidth', 1.5, 'Color', 'k', ...
                                'HandleVisibility', 'off');
                        else
                            % For other wheels, don't add to legend
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * max_vel, ':', 'LineWidth', 1, 'Color', colors{i}, 'HandleVisibility', 'off');
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * -max_vel, ':', 'LineWidth', 1, 'Color', colors{i}, 'HandleVisibility', 'off');
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * (0.8 * max_vel), '--', 'LineWidth', 1, 'Color', colors{i}, 'HandleVisibility', 'off');
                            plot(mission.true_time.store.time_attitude(1:rw_vel_data_length), ...
                                ones(1,rw_vel_data_length) * (-0.8 * max_vel), '--', 'LineWidth', 1, 'Color', colors{i}, 'HandleVisibility', 'off');
                        end
                    end
                end
            end
        end

        grid on;
        xlabel('Time [sec]');
        ylabel('Angular Velocity [rad/s]');
        title('Reaction Wheel Angular Velocity with Saturation Limits');
        legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
    end

    sgtitle(['SC ',num2str(i_SC),' Attitude Actuator Performance'],'fontsize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

    if mission.storage.plot_parameters.flag_save_plots == 1
        saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Attitude_Actuator.png'])
    end

end
