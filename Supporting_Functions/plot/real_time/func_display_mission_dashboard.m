function func_display_mission_dashboard(ax, mission, is_realtime_update)
% FUNC_DISPLAY_MISSION_DASHBOARD - Displays a mission dashboard with key information
%
% Syntax:  func_display_mission_dashboard(ax, mission, is_realtime_update)
%
% Inputs:
%    ax - Axes handle where to place the dashboard
%    mission - Mission structure containing all mission data
%    is_realtime_update - (Optional) Boolean flag indicating if this is a real-time update
%
% Outputs:
%    None - Updates the axes directly
%
% Example:
%    func_display_mission_dashboard(gca, mission)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: func_visualize_SC

    % Check if axes handle is provided
    if nargin < 1 || isempty(ax)
        error('Axes handle must be provided');
    end
    
    % Check if mission structure is provided
    if nargin < 2 || isempty(mission)
        error('Mission structure must be provided');
    end
    
    % Default value for is_realtime_update
    if nargin < 3
        is_realtime_update = false;
    end
    
    % Create left dashboard (mission-related info)
    create_left_dashboard(ax, mission, is_realtime_update);
    
    % Create right dashboard (spacecraft-related info)
    create_right_dashboard(ax, mission, is_realtime_update);
end

function create_left_dashboard(ax, mission, is_realtime_update)
    % Define dashboard position and style - thinner panels
    dashboard_width = 0.20;  % Reduced from 0.25 to make panels thinner
    dashboard_height = 0.9;  % Height of the dashboard as fraction of figure height
    left_dashboard_pos = [0.02, 0.05, dashboard_width, dashboard_height];
    
    % Create a panel for the dashboard
    dashboard = uipanel('Parent', ax.Parent, ...
                      'Units', 'normalized', ...
                      'Position', left_dashboard_pos, ...
                      'BackgroundColor', [0.1, 0.1, 0.12], ...
                      'ForegroundColor', [1, 1, 1], ...
                      'BorderType', 'line', ...
                      'BorderWidth', 1, ...
                      'HighlightColor', [0.4, 0.4, 0.4], ...
                      'ShadowColor', [0.05, 0.05, 0.05], ...
                      'Title', 'MISSION DASHBOARD', ...
                      'TitlePosition', 'centertop', ...
                      'FontSize', 14, ...
                      'FontWeight', 'bold', ...
                      'ForegroundColor', [0.2, 0.6, 1]);
    
    % Create panel for content
    content_panel = uipanel('Parent', dashboard, ...
                         'Units', 'normalized', ...
                         'Position', [0.02, 0.02, 0.96, 0.94], ...
                         'BackgroundColor', [0.15, 0.15, 0.17], ...
                         'BorderType', 'none');
    
    % Build and add content to the dashboard
    build_left_dashboard_content(content_panel, mission, is_realtime_update);
end

function create_right_dashboard(ax, mission, is_realtime_update)
    % Define dashboard position and style - thinner panels
    dashboard_width = 0.20;  % Reduced from 0.25 to make panels thinner
    dashboard_height = 0.9;  % Height of the dashboard as fraction of figure height
    right_dashboard_pos = [0.78, 0.05, dashboard_width, dashboard_height];  % Adjusted position to maintain spacing
    
    % Create a panel for the dashboard
    dashboard = uipanel('Parent', ax.Parent, ...
                      'Units', 'normalized', ...
                      'Position', right_dashboard_pos, ...
                      'BackgroundColor', [0.1, 0.1, 0.12], ...
                      'ForegroundColor', [1, 1, 1], ...
                      'BorderType', 'line', ...
                      'BorderWidth', 1, ...
                      'HighlightColor', [0.4, 0.4, 0.4], ...
                      'ShadowColor', [0.05, 0.05, 0.05], ...
                      'Title', 'SPACECRAFT DETAILS', ...
                      'TitlePosition', 'centertop', ...
                      'FontSize', 14, ...
                      'FontWeight', 'bold', ...
                      'ForegroundColor', [0.2, 0.6, 1]);
    
    % Create panel for content
    content_panel = uipanel('Parent', dashboard, ...
                         'Units', 'normalized', ...
                         'Position', [0.02, 0.02, 0.96, 0.94], ...
                         'BackgroundColor', [0.15, 0.15, 0.17], ...
                         'BorderType', 'none');
    
    % Build and add content to the dashboard
    build_right_dashboard_content(content_panel, mission, is_realtime_update);
end

function build_left_dashboard_content(parent, mission, is_realtime_update)
    % Creates mission-related information for the left dashboard
    
    % Define text colors
    title_color = [0.2, 0.8, 0.4];  % Green for section titles
    label_color = [0.8, 0.8, 0.8];  % Light gray for labels
    value_color = [1.0, 0.8, 0.2];  % Amber/gold for values
    info_color = [0.6, 0.8, 1.0];   % Light blue for information text
    
    % Define font and spacing
    title_font = mission.storage.plot_parameters.title_font_size;
    text_font = mission.storage.plot_parameters.standard_font_size;
    info_font = mission.storage.plot_parameters.standard_font_size;
    
    % Current vertical position (start from top)
    y_pos = 0.98;
    
    % Spacing between elements
    y_step_small = 0.022;
    y_step_large = 0.035;
    section_gap = 0.04;
    
    % Add refresh rate information if this is a real-time update
    if is_realtime_update
        % Get refresh interval from mission storage
        refresh_interval = mission.storage.viz_update_interval;
        
        % Create an information box about refresh rate with proper line breaks for alignment
        info_text = sprintf(['• Real-time Visualization\n', ...
                           '• Updates every %.0f seconds\n', ...
                           '• Updates on mode changes\n', ...
                           '• Performance optimized'], refresh_interval);
        
        % Create a panel for the info text with a slightly darker background
        info_panel = uipanel('Parent', parent, ...
                           'Units', 'normalized', ...
                           'Position', [0.05, y_pos-0.08, 0.9, 0.08], ...  % Reduced height
                           'BackgroundColor', [0.12, 0.12, 0.14], ...
                           'BorderType', 'line', ...
                           'BorderWidth', 1, ...
                           'HighlightColor', [0.3, 0.3, 0.35], ...
                           'ForegroundColor', info_color);
        
        % Add the text with left alignment and adjusted position
        text_handle = uicontrol('Parent', info_panel, ...
                              'Style', 'text', ...
                              'Units', 'normalized', ...
                              'Position', [0.05, 0, 0.9, 1], ...  % Full height, no vertical padding
                              'String', info_text, ...
                              'BackgroundColor', [0.12, 0.12, 0.14], ...
                              'ForegroundColor', info_color, ...
                              'FontSize', info_font, ...
                              'FontAngle', 'italic', ...
                              'HorizontalAlignment', 'left');
        
        % Update y_pos to account for the info box
        y_pos = y_pos - 0.12;  % Adjusted offset
    end
    
    % Create mission name label
    if isfield(mission, 'name')
        createTextLabel(parent, 0.5, y_pos, mission.name, 'center', [1.0, 0.6, 0.1], 14, 'bold');
        y_pos = y_pos - section_gap;
    end
    
    % ===== MISSION PARAMETERS SECTION =====
    createTextLabel(parent, 0.08, y_pos, 'MISSION PARAMETERS', 'left', title_color, title_font, 'bold');
    y_pos = y_pos - y_step_large;
    
    % Number of spacecraft and targets
    if isfield(mission, 'num_SC')
        y_pos = addLabelValuePair(parent, 'Spacecraft:', mission.num_SC, y_pos, label_color, value_color, text_font);
    end
    
    if isfield(mission, 'num_target')
        y_pos = addLabelValuePair(parent, 'Targets:', mission.num_target, y_pos, label_color, value_color, text_font);
    end
    
    % Frame type
    if isfield(mission, 'frame')
        y_pos = addLabelValuePair(parent, 'Frame:', mission.frame, y_pos, label_color, value_color, text_font);
    end
    
    % Time information
    if isfield(mission, 'true_time')
        % Add the current date (using date property from True_Time class)
        if isprop(mission.true_time, 'date')
            date_str = sec2cal(mission.true_time.date);
            y_pos = addLabelValuePair(parent, 'Mission Date:', date_str, y_pos, label_color, value_color, text_font);
        end
        
        if isprop(mission.true_time, 'time')
            % Format time nicely if it's large
            time_val = mission.true_time.time;
            if time_val > 3600
                y_pos = addLabelValuePair(parent, 'Current Time:', sprintf('%.2f hours (%.2f sec)', time_val/3600, time_val), y_pos, label_color, value_color, text_font);
            elseif time_val > 60
                y_pos = addLabelValuePair(parent, 'Current Time:', sprintf('%.2f min (%.2f sec)', time_val/60, time_val), y_pos, label_color, value_color, text_font);
            else
                y_pos = addLabelValuePair(parent, 'Current Time:', sprintf('%.2f sec', time_val), y_pos, label_color, value_color, text_font);
            end
        end
        
        if isprop(mission.true_time, 'time_step')
            y_pos = addLabelValuePair(parent, 'Time Step:', sprintf('%.2f sec', mission.true_time.time_step), y_pos, label_color, value_color, text_font);
        end
        
        if isprop(mission.true_time, 'time_step_attitude')
            y_pos = addLabelValuePair(parent, 'Attitude Time Step:', sprintf('%.2f sec', mission.true_time.time_step_attitude), y_pos, label_color, value_color, text_font);
        end
    end
    
    y_pos = y_pos - section_gap;
    
    % ===== TARGET INFORMATION =====
    if isfield(mission, 'true_target') && ~isempty(mission.true_target)
        createTextLabel(parent, 0.08, y_pos, 'TARGET', 'left', title_color, title_font, 'bold');
        y_pos = y_pos - y_step_large;
        
        for i_target = 1:mission.num_target
            if isprop(mission.true_target{i_target}, 'name')
                y_pos = addLabelValuePair(parent, 'Name:', mission.true_target{i_target}.name, ...
                    y_pos, label_color, value_color, text_font);
            end
            
            % Distance calculations
            if isprop(mission.true_target{i_target}, 'position') && isfield(mission, 'true_SC') && ...
               numel(mission.true_SC) > 0 && isfield(mission.true_SC{1}, 'true_SC_navigation') && ...
               isprop(mission.true_SC{1}.true_SC_navigation, 'position')
                
                % Distance from spacecraft to target
                distance_to_target = norm(mission.true_SC{1}.true_SC_navigation.position - mission.true_target{i_target}.position);
                y_pos = addLabelValuePair(parent, 'Distance to Target:', sprintf('%.2f km', distance_to_target), ...
                    y_pos, label_color, value_color, text_font);
            end
        end
        
        y_pos = y_pos - y_step_small;
    end
    
    % ===== SIMULATION STATUS =====
    if isfield(mission, 'true_time')
        createTextLabel(parent, 0.08, y_pos, 'SIMULATION STATUS', 'left', title_color, title_font, 'bold');
        y_pos = y_pos - y_step_large;
        
        % Progress information
        if isprop(mission.true_time, 'k') && isprop(mission.true_time, 'num_time_steps')
            percentage = 100 * mission.true_time.k / mission.true_time.num_time_steps;
            y_pos = addLabelValuePair(parent, 'Progress:', sprintf('%.1f%% (%d/%d steps)', ...
                percentage, mission.true_time.k, mission.true_time.num_time_steps), ...
                y_pos, label_color, value_color, text_font);
        end
        
        % Time progress
        if isprop(mission.true_time, 'time') && isprop(mission.true_time, 't_initial') && isprop(mission.true_time, 't_final')
            elapsed = mission.true_time.time - mission.true_time.t_initial;
            total = mission.true_time.t_final - mission.true_time.t_initial;
            percentage = 100 * elapsed / total;
            
            % Format based on magnitude
            if total > 3600
                y_pos = addLabelValuePair(parent, 'Time Progress:', sprintf('%.1f%% (%.1f/%.1f hours)', ...
                    percentage, elapsed/3600, total/3600), ...
                    y_pos, label_color, value_color, text_font);
            elseif total > 60
                y_pos = addLabelValuePair(parent, 'Time Progress:', sprintf('%.1f%% (%.1f/%.1f minutes)', ...
                    percentage, elapsed/60, total/60), ...
                    y_pos, label_color, value_color, text_font);
            else
                y_pos = addLabelValuePair(parent, 'Time Progress:', sprintf('%.1f%% (%.1f/%.1f seconds)', ...
                    percentage, elapsed, total), ...
                    y_pos, label_color, value_color, text_font);
            end
        end
        
        % If this is a real-time update, add a "Last Updated" timestamp
        if is_realtime_update
            current_datestr = datestr(now, 'HH:MM:SS');
            y_pos = addLabelValuePair(parent, 'Dashboard Updated:', current_datestr, ...
                y_pos, label_color, [0.2, 0.9, 0.2], text_font);
        end
    end
end

function build_right_dashboard_content(parent, mission, is_realtime_update)
    % Creates spacecraft-related information for the right dashboard
    
    % Define text colors
    title_color = [0.2, 0.8, 0.4];  % Green for section titles
    label_color = [0.8, 0.8, 0.8];  % Light gray for labels
    value_color = [1.0, 0.8, 0.2];  % Amber/gold for values
    highlight_color = [0.2, 0.8, 0.2]; % Green for highlighted items
    
    % Define font and spacing
    title_font = mission.storage.plot_parameters.title_font_size;
    text_font = mission.storage.plot_parameters.standard_font_size;
    
    % Current vertical position (start from top)
    y_pos = 0.98;
    
    % Spacing between elements
    y_step_small = 0.022;
    y_step_large = 0.035;
    section_gap = 0.04;
    
    % ===== SPACECRAFT INFORMATION =====
    for i_SC = 1:mission.num_SC
        if isfield(mission, 'true_SC') && numel(mission.true_SC) >= i_SC
            
            % Position and Velocity
            if isfield(mission.true_SC{i_SC}, 'true_SC_navigation')
                createTextLabel(parent, 0.08, y_pos, sprintf('SPACECRAFT %d', i_SC), 'left', title_color, title_font, 'bold');
                y_pos = y_pos - y_step_large;
                
                if isprop(mission.true_SC{i_SC}.true_SC_navigation, 'position')
                    pos = mission.true_SC{i_SC}.true_SC_navigation.position;
                    % Format position vector with scientific notation and stack components
                    createTextLabel(parent, 0.08, y_pos, 'Position (km):', 'left', label_color, text_font);
                    y_pos = y_pos - y_step_small;
                    
                    % Display each component on its own line with scientific notation
                    createTextLabel(parent, 0.15, y_pos, sprintf('X: %.3e', pos(1)), 'left', value_color, text_font);
                    y_pos = y_pos - y_step_small;
                    createTextLabel(parent, 0.15, y_pos, sprintf('Y: %.3e', pos(2)), 'left', value_color, text_font);
                    y_pos = y_pos - y_step_small;
                    createTextLabel(parent, 0.15, y_pos, sprintf('Z: %.3e', pos(3)), 'left', value_color, text_font);
                    y_pos = y_pos - y_step_small;
                end
                
                if isprop(mission.true_SC{i_SC}.true_SC_navigation, 'velocity')
                    vel = mission.true_SC{i_SC}.true_SC_navigation.velocity;
                    % Format velocity vector with scientific notation and stack components
                    createTextLabel(parent, 0.08, y_pos, 'Velocity (km/s):', 'left', label_color, text_font);
                    y_pos = y_pos - y_step_small;
                    
                    % Display each component on its own line with scientific notation
                    createTextLabel(parent, 0.15, y_pos, sprintf('X: %.3e', vel(1)), 'left', value_color, text_font);
                    y_pos = y_pos - y_step_small;
                    createTextLabel(parent, 0.15, y_pos, sprintf('Y: %.3e', vel(2)), 'left', value_color, text_font);
                    y_pos = y_pos - y_step_small;
                    createTextLabel(parent, 0.15, y_pos, sprintf('Z: %.3e', vel(3)), 'left', value_color, text_font);
                    y_pos = y_pos - y_step_small;
                end
                
                y_pos = y_pos - y_step_small;
            end
            
            % Software Modes (Current mode)
            if isfield(mission.true_SC{i_SC}, 'software_SC_executive')
                if isprop(mission.true_SC{i_SC}.software_SC_executive, 'this_sc_mode')
                    y_pos = addLabelValuePair(parent, 'Current Mode:', mission.true_SC{i_SC}.software_SC_executive.this_sc_mode, ...
                        y_pos, label_color, highlight_color, text_font);
                    y_pos = y_pos - y_step_small;
                end
                
                % Available Modes
                if isprop(mission.true_SC{i_SC}.software_SC_executive, 'sc_modes') && ~isempty(mission.true_SC{i_SC}.software_SC_executive.sc_modes)
                    createTextLabel(parent, 0.08, y_pos, 'Available Modes:', 'left', title_color, text_font-1, 'bold');
                    y_pos = y_pos - y_step_small;
                    
                    for i_mode = 1:length(mission.true_SC{i_SC}.software_SC_executive.sc_modes)
                        % Highlight current mode if available
                        if isprop(mission.true_SC{i_SC}.software_SC_executive, 'this_sc_mode') && ...
                           strcmp(mission.true_SC{i_SC}.software_SC_executive.this_sc_mode, mission.true_SC{i_SC}.software_SC_executive.sc_modes{i_mode})
                            createTextLabel(parent, 0.15, y_pos, ['• ' mission.true_SC{i_SC}.software_SC_executive.sc_modes{i_mode} ' ◄'], 'left', highlight_color, text_font, 'bold');
                        else
                            createTextLabel(parent, 0.15, y_pos, ['• ' mission.true_SC{i_SC}.software_SC_executive.sc_modes{i_mode}], 'left', label_color, text_font);
                        end
                        y_pos = y_pos - y_step_small;
                    end
                    
                    y_pos = y_pos - y_step_small;
                end
            end
            
            % Power information - restored original access method
            if isfield(mission.true_SC{i_SC}, 'true_SC_power')
                createTextLabel(parent, 0.08, y_pos, 'Power:', 'left', title_color, text_font-1, 'bold');
                y_pos = y_pos - y_step_small;
                
                power_info_found = false;
                
                if isprop(mission.true_SC{i_SC}.true_SC_power, 'instantaneous_total_power_generated')
                    y_pos = addLabelValuePair(parent, 'Generated:', sprintf('%.2f W', mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(mission.storage.k_storage)), ...
                        y_pos, label_color, value_color, text_font, 0.15);
                    power_info_found = true;
                end
                
                if isprop(mission.true_SC{i_SC}.true_SC_power, 'instantaneous_total_power_consumed')
                    y_pos = addLabelValuePair(parent, 'Consumed:', sprintf('%.2f W', mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(mission.storage.k_storage)), ...
                        y_pos, label_color, value_color, text_font, 0.15);
                    power_info_found = true;
                end
                
                if ~power_info_found
                    y_pos = addLabelValuePair(parent, 'Status:', 'Initializing...', y_pos, label_color, value_color, text_font, 0.15);
                end
                
                y_pos = y_pos - y_step_small;
            end
            
            % Battery information - restored original access method
            if isfield(mission.true_SC{i_SC}, 'true_SC_battery') && ~isempty(mission.true_SC{i_SC}.true_SC_battery)
                createTextLabel(parent, 0.08, y_pos, 'Battery:', 'left', title_color, text_font-1, 'bold');
                y_pos = y_pos - y_step_small;
                
                % Show only the first battery for brevity
                if length(mission.true_SC{i_SC}.true_SC_battery) > 0
                    % Use correct mission.true_SC{i_SC}.true_SC_battery{1} properties
                    if isprop(mission.true_SC{i_SC}.true_SC_battery{1}, 'instantaneous_capacity') && isprop(mission.true_SC{i_SC}.true_SC_battery{1}, 'maximum_capacity')
                        % Get state of charge or calculate it
                        if isprop(mission.true_SC{i_SC}.true_SC_battery{1}, 'state_of_charge')
                            percentage = mission.true_SC{i_SC}.true_SC_battery{1}.store.state_of_charge(mission.storage.k_storage);
                        else
                            percentage = 100 * mission.true_SC{i_SC}.true_SC_battery{1}.instantaneous_capacity / mission.true_SC{i_SC}.true_SC_battery{1}.maximum_capacity;
                        end
                        y_pos = addLabelValuePair(parent, 'Status:', ...
                            sprintf('%.1f%% (%.2f/%.2f Wh)', percentage, mission.true_SC{i_SC}.true_SC_battery{1}.store.instantaneous_capacity(mission.storage.k_storage), mission.true_SC{i_SC}.true_SC_battery{1}.maximum_capacity), ...
                            y_pos, label_color, value_color, text_font, 0.15);
                    else
                        y_pos = addLabelValuePair(parent, 'Status:', 'Initializing...', y_pos, label_color, value_color, text_font, 0.15);
                    end
                end
                
                y_pos = y_pos - y_step_small;
            end
            
            % Data handling information
            if isfield(mission.true_SC{i_SC}, 'true_SC_data_handling')
                createTextLabel(parent, 0.08, y_pos, 'Data Status:', 'left', title_color, text_font-1, 'bold');
                y_pos = y_pos - y_step_small;
                
                if isprop(mission.true_SC{i_SC}.true_SC_data_handling, 'instantaneous_data_generated')
                    y_pos = addLabelValuePair(parent, 'Generated:', sprintf('%.2f kb', mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_generated(mission.storage.k_storage)), ...
                        y_pos, label_color, value_color, text_font, 0.15);
                end
                
                if isprop(mission.true_SC{i_SC}.true_SC_data_handling, 'instantaneous_data_removed')
                    y_pos = addLabelValuePair(parent, 'Removed:', sprintf('%.2f kb', mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_removed(mission.storage.k_storage)), ...
                        y_pos, label_color, value_color, text_font, 0.15);
                end
                
                if isprop(mission.true_SC{i_SC}.true_SC_data_handling, 'instantaneous_data_change')
                    y_pos = addLabelValuePair(parent, 'Change:', sprintf('%.2f kb', mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_change(mission.storage.k_storage)), ...
                        y_pos, label_color, value_color, text_font, 0.15);
                end
                
                y_pos = y_pos - y_step_small;
            end
            
            % Attitude information if available
            if isfield(mission.true_SC{i_SC}, 'true_SC_adc') && isprop(mission.true_SC{i_SC}.true_SC_adc, 'attitude')
                createTextLabel(parent, 0.08, y_pos, 'Attitude:', 'left', title_color, text_font-1, 'bold');
                y_pos = y_pos - y_step_small;
                
                % Quaternion
                attitude = mission.true_SC{i_SC}.true_SC_adc.attitude;
                y_pos = addLabelValuePair(parent, 'Quaternion:', sprintf('[%.3f, %.3f, %.3f, %.3f]', ...
                    attitude(1), attitude(2), attitude(3), attitude(4)), ...
                    y_pos, label_color, value_color, text_font, 0.15);
                
                % Angular velocity if available
                if isprop(mission.true_SC{i_SC}.true_SC_adc, 'angular_velocity')
                    omega = mission.true_SC{i_SC}.true_SC_adc.angular_velocity;
                    y_pos = addLabelValuePair(parent, 'Angular Velocity:', sprintf('[%.5f, %.5f, %.5f] rad/s', ...
                        omega(1), omega(2), omega(3)), ...
                        y_pos, label_color, value_color, text_font, 0.15);
                end
                
                y_pos = y_pos - y_step_small;
            end
        end
    end
end

function createTextLabel(parent, x, y, text, alignment, color, font_size, font_weight)
    % Helper function to create a text label
    if nargin < 7, font_size = 10; end
    if nargin < 8, font_weight = 'normal'; end
    
    uicontrol('Parent', parent, ...
              'Style', 'text', ...
              'Units', 'normalized', ...
              'Position', [x-0.05, y-0.02, 0.9, 0.04], ...
              'String', text, ...
              'BackgroundColor', get(parent, 'BackgroundColor'), ...
              'ForegroundColor', color, ...
              'FontSize', font_size, ...
              'FontWeight', font_weight, ...
              'HorizontalAlignment', alignment);
end

function new_y_pos = addLabelValuePair(parent, label, value, y_pos, label_color, value_color, font_size, label_width)
    % Helper function to add a label-value pair
    if nargin < 8, label_width = 0.3; end
    
    % Create label (left aligned)
    createTextLabel(parent, 0.08, y_pos, label, 'left', label_color, font_size);
    
    % Create value (left aligned but offset)
    if isnumeric(value)
        value_str = num2str(value);
    else
        value_str = value;
    end
    
    createTextLabel(parent, label_width + 0.2, y_pos, value_str, 'left', value_color, font_size);
    
    % Return new y position
    new_y_pos = y_pos - 0.025;
end





