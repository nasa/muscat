function func_realtime_performance_plot(mission, fig_handle, subplot_handles)
% ULTRA-OPTIMIZED REAL-TIME DASHBOARD
% Focuses only on critical status information with minimum performance impact
% Uses binary state visualization and only updates what has changed

% Current state indices
kd = mission.storage.k_storage;
i_SC = 1; % Only first spacecraft

% Skip if insufficient data points
if kd < 2
    return;
end

% Create persistent variable for tracking state changes
persistent last_data;
if isempty(last_data)
    last_data = struct('mode', 0, 'desat', 0, 'power_gen', 0, 'power_con', 0, ...
                      'memory', 0, 'fuel', 0, 'time', -inf);
end

% Check if enough time has passed (at least 0.5s between data checks)
if mission.true_time.time - last_data.time < 0.5
    return;
end

% Get latest time for display
time_sim_elapsed = mission.true_time.time - mission.true_time.t_initial;
time_days = time_sim_elapsed / 86400;
time_hours = time_sim_elapsed / 3600;

% EXTREME SAMPLING OPTIMIZATION:
% For long simulations, use just a few points with logarithmic sampling
if kd > 100
    % Create a logarithmic sampling - weights recent points heavier
    sample_range = unique(round(logspace(0, log10(kd), 15)));
else
    % For short runs, use very sparse linear sampling
    sample_rate = max(1, floor(kd/10));
    sample_range = 1:sample_rate:kd;
end

% Always include latest point for current status
if ~ismember(kd, sample_range)
    sample_range = [sample_range kd];
end

% Extract minimal time data
time_vec = mission.true_time.store.time(sample_range);

% Title with minimum computation
sgtitle(fig_handle, sprintf('Mission Status | Time: %.1f days (%.1f hrs)', time_days, time_hours), 'FontSize', 12);

%% MODE VISUALIZATION - Using color bands like func_plot_software_executive_visualization
ax = subplot_handles{1, 1};

try
    % Get current mode value
    current_mode = mission.true_SC{i_SC}.software_SC_executive.store.this_sc_mode_value(kd);
    
    % Only update if the mode changed
    if current_mode ~= last_data.mode
        cla(ax);
        
        % Get modes data
        mode_values = mission.true_SC{i_SC}.software_SC_executive.store.this_sc_mode_value(sample_range);
        sc_mode_names = mission.true_SC{i_SC}.software_SC_executive.store.sc_modes;
        
        % Define a colormap with distinct colors for modes
        cmap = lines(length(sc_mode_names));
        
        hold(ax, 'on');
        
        % Create colored bands for each mode (like in func_plot_software_executive_visualization)
        for j = 1:length(sc_mode_names)
            fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                 [(j-0.3) (j-0.3) (j+0.3) (j+0.3)], ...
                 cmap(j,:), 'EdgeColor', 'none', 'FaceAlpha', 0.3);
        end
        
        % Plot the actual mode line
        plot(ax, time_vec, mode_values, '-k', 'LineWidth', 2);
        
        % Set up axis with mode names
        yticks(ax, 1:length(sc_mode_names));
        yticklabels(ax, sc_mode_names);
        ylim(ax, [0.5, length(sc_mode_names)+0.5]);
        
        grid(ax, 'on');
        title(ax, 'Spacecraft Mode');
        hold(ax, 'off');
        
        % Update last mode for change detection
        last_data.mode = current_mode;
    end
catch
    % Silent error handling
end

%% DESATURATION STATUS - as a binary indicator
ax = subplot_handles{1, 2};

try
    % Check if desaturation data is available
    has_desaturation = false;
    
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel > 0
        % Simply check if reaction wheels exist, even if desaturation data isn't available
        has_desaturation = true;
        
        % Get reaction wheel speeds for simple visualization
        rw_speed = [];
        for i_rw = 1:min(3, mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel)
            try
                rw_data = mission.true_SC{i_SC}.true_SC_reaction_wheel{i_rw}.store.wheel_speed_current(sample_range);
                rw_speed(i_rw,:) = rw_data;
            catch
                rw_speed(i_rw,:) = zeros(size(sample_range));
            end
        end
        
        % Only update occasionally to improve performance
        if mod(kd, 20) == 0 || (kd > 2 && ~isempty(rw_speed))
            cla(ax);
            
            hold(ax, 'on');
            
            if ~isempty(rw_speed)
                % Plot each reaction wheel speed
                colors = {'r', 'g', 'b'};
                for i = 1:size(rw_speed, 1)
                    plot(ax, time_vec, rw_speed(i,:), colors{i}, 'LineWidth', 2);
                end
                
                grid(ax, 'on');
                title(ax, 'Reaction Wheel Speeds');
                ylabel(ax, 'Speed (RPM)');
                
                % Add legend
                legend_text = {};
                for i = 1:size(rw_speed, 1)
                    legend_text{i} = sprintf('RW %d', i);
                end
                legend(ax, legend_text, 'Location', 'best');
            else
                text(ax, 0.5, 0.5, 'Reaction Wheels Present', 'HorizontalAlignment', 'center');
            end
            
            hold(ax, 'off');
        end
    else
        cla(ax);
        text(ax, 0.5, 0.5, 'No Reaction Wheels', 'HorizontalAlignment', 'center');
        axis(ax, 'off');
    end
catch
    % Silent error handling
    cla(ax);
    text(ax, 0.5, 0.5, 'Reaction Wheel Data Error', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
end

%% POWER STATUS - Simplified with current values emphasized
ax = subplot_handles{1, 3};

try
    % Get current power values
    current_power_gen = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(kd);
    current_power_con = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(kd);
    
    % Only update if power changed significantly (>1W change)
    if abs(current_power_gen - last_data.power_gen) > 1 || abs(current_power_con - last_data.power_con) > 1
        cla(ax);
        
        % Get sparse power data
        power_gen = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(sample_range);
        power_con = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(sample_range);
        
        % Super minimal plot - focus on status rather than details
        hold(ax, 'on');
        area(ax, time_vec, power_gen, 'FaceColor', [0.6 1 0.6], 'EdgeColor', 'none');
        area(ax, time_vec, power_con, 'FaceColor', [1 0.6 0.6], 'EdgeColor', 'none');
        
        grid(ax, 'on');
        title(ax, 'Power Status');
        
        % Show current values as large text
        surplus = current_power_gen - current_power_con;
        if surplus >= 0
            status_color = 'g';
            status_text = sprintf('SURPLUS: %.1f W', surplus);
        else
            status_color = 'r';
            status_text = sprintf('DEFICIT: %.1f W', surplus);
        end
        
        text(ax, 0.5, 0.9, sprintf('Gen: %.1f W', current_power_gen), 'Units', 'normalized', 'HorizontalAlignment', 'center');
        text(ax, 0.5, 0.8, sprintf('Use: %.1f W', current_power_con), 'Units', 'normalized', 'HorizontalAlignment', 'center');
        text(ax, 0.5, 0.5, status_text, 'Units', 'normalized', 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', status_color, 'FontSize', 14);
        
        hold(ax, 'off');
        
        % Update for change detection
        last_data.power_gen = current_power_gen;
        last_data.power_con = current_power_con;
    end
catch
    % Silent error handling - use generic fallback
    cla(ax);
    text(ax, 0.5, 0.5, 'No Power Data', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
end

%% DATA BALANCE - Similar to power surplus
ax = subplot_handles{2, 3};

try
    % Get data generation and downlink rates
    data_gen_rate = 0;
    data_downlink_rate = 0;
    
    % Try to get data generation rate
    try
        % Calculate data generation rate from the change in stored data
        if kd > 2
            t_diff = mission.true_time.store.time(kd) - mission.true_time.store.time(kd-1);
            current_data = mission.true_SC{i_SC}.true_SC_data_handling.store.data_volume_stored(kd);
            prev_data = mission.true_SC{i_SC}.true_SC_data_handling.store.data_volume_stored(kd-1);
            downlinked_data = 0;
            
            % Try to get downlinked data
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_communication_link > 0
                try
                    downlinked_data = mission.true_SC{i_SC}.true_SC_communication_link{1}.store.data_volume_downlinked(kd) - mission.true_SC{i_SC}.true_SC_communication_link{1}.store.data_volume_downlinked(kd-1);
                catch
                    % No downlink data available
                end
            end
            
            % Calculate rates (KB/s)
            if t_diff > 0
                data_gen_rate = ((current_data - prev_data) + downlinked_data) / t_diff;
                data_downlink_rate = downlinked_data / t_diff;
            end
        end
    catch
        % Failed to calculate rates
    end
    
    % Only update occasionally or if values change significantly
    if mod(kd, 20) == 0 || kd < 5
        cla(ax);
        
        hold(ax, 'on');
        
        % Create a visualization showing data generation vs downlink
        % Similar to the power surplus/deficit
        balance = data_gen_rate - data_downlink_rate;
        
        % Show current values as large text
        if balance <= 0
            status_color = 'g';
            status_text = sprintf('KEEPING UP: %.1f KB/s', -balance);
        else
            if balance > 5
                status_color = 'r';
            else
                status_color = [1 0.5 0]; % orange
            end
            status_text = sprintf('ACCUMULATING: %.1f KB/s', balance);
        end
        
        % Create a simple bar chart
        bar_data = [data_gen_rate, data_downlink_rate];
        bar_labels = {'Generated', 'Downlinked'};
        b = bar(ax, bar_data, 0.5);
        if data_gen_rate > data_downlink_rate
            b.FaceColor = 'flat';
            b.CData(1,:) = [1 0.7 0.7]; % red for generation
            b.CData(2,:) = [0.7 1 0.7]; % green for downlink
        else
            b.FaceColor = 'flat';
            b.CData(1,:) = [0.7 0.7 1]; % blue for generation
            b.CData(2,:) = [0.7 1 0.7]; % green for downlink
        end
        
        set(ax, 'XTickLabel', bar_labels);
        
        % Show current values as text
        text(ax, 0.5, 0.9, sprintf('Gen: %.1f KB/s', data_gen_rate), 'Units', 'normalized', 'HorizontalAlignment', 'center');
        text(ax, 0.5, 0.8, sprintf('Down: %.1f KB/s', data_downlink_rate), 'Units', 'normalized', 'HorizontalAlignment', 'center');
        text(ax, 0.5, 0.5, status_text, 'Units', 'normalized', 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', status_color, 'FontSize', 14);
        
        grid(ax, 'on');
        title(ax, 'Data Balance');
        ylabel(ax, 'Rate (KB/s)');
        
        hold(ax, 'off');
    end
catch
    % Silent error handling
    cla(ax);
    text(ax, 0.5, 0.5, 'Data Rate Unavailable', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
end

%% FUEL STATUS - Simplified gauge
ax = subplot_handles{2, 1};

try
    % Check if fuel data is available
    has_fuel_data = false;
    
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0
        % Get current fuel value
        current_fuel_pct = 0;
        for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
            try
                current_mass = mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.store.fuel_mass(kd);
                initial_mass = mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.maximum_capacity;
                current_fuel_pct = current_mass / initial_mass * 100;
                has_fuel_data = true;
                break; % Just use first available tank
            catch
                % Skip if not available
            end
        end
        
        if has_fuel_data
            % Only update if fuel changed by more than 0.1% or occasionally
            if mod(kd, 20) == 0 || abs(current_fuel_pct - last_data.fuel) > 0.1 || kd < 5
                cla(ax);
                
                % Get fuel data for first tank
                fuel_mass = [];
                max_fuel = 0;
                
                for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
                    try
                        fuel_mass = mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.store.fuel_mass(sample_range);
                        max_fuel = mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.maximum_capacity;
                        break; % Just use first available tank
                    catch
                        % Skip if not available
                    end
                end
                
                if ~isempty(fuel_mass)
                    % Calculate fuel percentage
                    fuel_pct = fuel_mass / max_fuel * 100;
                    
                    % Create a simple horizontal bar/gauge
                    hold(ax, 'on');
                    
                    % Draw full gauge background (gray)
                    fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                         [0 0 100 100], [0.9 0.9 0.9], 'EdgeColor', 'k');
                    
                    % Draw actual fuel level
                    fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                         [0 0 fuel_pct(end) fuel_pct(end)], [0.2 0.7 0.2], 'EdgeColor', 'none');
                    
                    % Draw a thick line for current level
                    plot(ax, time_vec, fuel_pct, 'k-', 'LineWidth', 2);
                    
                    % Add warning levels
                    plot(ax, [time_vec(1) time_vec(end)], [20 20], 'r--', 'LineWidth', 1);
                    plot(ax, [time_vec(1) time_vec(end)], [10 10], 'r-', 'LineWidth', 2);
                    
                    grid(ax, 'on');
                    title(ax, 'Fuel Status');
                    ylim(ax, [0, 100]);
                    ylabel(ax, 'Fuel Remaining (%)');
                    
                    % Show current value
                    fuel_text = sprintf('%.1f%%', current_fuel_pct);
                    if current_fuel_pct < 10
                        text_color = 'r';
                        fuel_text = ['CRITICAL: ' fuel_text];
                    elseif current_fuel_pct < 20
                        text_color = [1 0.5 0]; % Orange
                        fuel_text = ['LOW: ' fuel_text];
                    else
                        text_color = 'g';
                        fuel_text = ['FUEL: ' fuel_text];
                    end
                    
                    text(ax, 0.5, 0.5, fuel_text, 'Units', 'normalized', 'FontWeight', 'bold', ...
                         'HorizontalAlignment', 'center', 'Color', text_color, 'FontSize', 14);
                    
                    hold(ax, 'off');
                    
                    % Update for change detection
                    last_data.fuel = current_fuel_pct;
                end
            end
        else
            % No fuel data but tank exists - show default view
            if mod(kd, 30) == 0 || kd < 5
                cla(ax);
                text(ax, 0.5, 0.5, 'Fuel Tank Present - No Data', 'HorizontalAlignment', 'center');
                axis(ax, 'off');
            end
        end
    else
        % No fuel tanks at all
        if mod(kd, 30) == 0 || kd < 5
            cla(ax);
            text(ax, 0.5, 0.5, 'No Fuel Tanks', 'HorizontalAlignment', 'center');
            axis(ax, 'off');
        end
    end
catch
    % Silent error handling
    if mod(kd, 30) == 0 || kd < 5
        cla(ax);
        text(ax, 0.5, 0.5, 'Fuel Data Error', 'HorizontalAlignment', 'center');
        axis(ax, 'off');
    end
end

%% MEMORY STATUS - simple filled gauge
ax = subplot_handles{2, 2};

try
    % Get current memory value
    current_memory_pct = 0;
    current_memory = 0;
    max_memory = 0;
    has_memory_data = false;
    
    if isfield(mission.true_SC{i_SC}.true_SC_data_handling.store, 'data_volume_stored')
        try
            current_memory = mission.true_SC{i_SC}.true_SC_data_handling.store.data_volume_stored(kd);
            has_memory_data = true;
            
            if isfield(mission.true_SC{i_SC}.true_SC_data_handling, 'maximum_data_volume')
                max_memory = mission.true_SC{i_SC}.true_SC_data_handling.maximum_data_volume;
            elseif mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0
                try
                    max_memory = mission.true_SC{i_SC}.true_SC_onboard_memory{1}.maximum_capacity;
                catch
                    % Fallback to a default value
                    max_memory = 1024; % 1 MB default
                end
            end
            
            if max_memory > 0
                current_memory_pct = current_memory / max_memory * 100;
            end
        catch
            % Failed to get memory data
        end
    end
    
    if has_memory_data
        % Only update if memory changed by more than 0.5% or occasionally
        if mod(kd, 20) == 0 || abs(current_memory_pct - last_data.memory) > 0.5 || kd < 5
            cla(ax);
            
            % Get memory usage data
            memory_used = mission.true_SC{i_SC}.true_SC_data_handling.store.data_volume_stored(sample_range);
            
            if ~isempty(memory_used) && max_memory > 0
                % Calculate memory percentage
                memory_pct = memory_used / max_memory * 100;
                
                % Simple filled area plot
                hold(ax, 'on');
                
                % Draw full capacity background (light gray)
                fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                     [0 0 100 100], [0.95 0.95 0.95], 'EdgeColor', 'k');
                
                % Draw usage level (blue, changing to red near full)
                if current_memory_pct > 90
                    fill_color = [1 0.7 0.7]; % Red tint
                elseif current_memory_pct > 75
                    fill_color = [1 0.9 0.7]; % Isaure tint
                else
                    fill_color = [0.7 0.7 1]; % Blue tint
                end
                
                fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                     [0 0 memory_pct(end) memory_pct(end)], fill_color, 'EdgeColor', 'none');
                
                % Add a black line for visibility
                plot(ax, time_vec, memory_pct, 'k-', 'LineWidth', 2);
                
                % Add warning levels
                plot(ax, [time_vec(1) time_vec(end)], [75 75], 'y--', 'LineWidth', 1);
                plot(ax, [time_vec(1) time_vec(end)], [90 90], 'r--', 'LineWidth', 1);
                
                grid(ax, 'on');
                title(ax, 'Memory Status');
                ylim(ax, [0, 100]);
                ylabel(ax, 'Memory Used (%)');
                
                % Show current value
                memory_text = sprintf('%.1f%%', current_memory_pct);
                if current_memory_pct > 90
                    text_color = 'r';
                    memory_text = ['CRITICAL: ' memory_text];
                elseif current_memory_pct > 75
                    text_color = [0.8 0.4 0]; % Dark orange
                    memory_text = ['HIGH: ' memory_text];
                else
                    text_color = 'b';
                    memory_text = ['MEMORY: ' memory_text];
                end
                
                text(ax, 0.5, 0.5, memory_text, 'Units', 'normalized', 'FontWeight', 'bold', ...
                     'HorizontalAlignment', 'center', 'Color', text_color, 'FontSize', 14);
                
                % Add total memory size
                if max_memory >= 1024
                    mem_text = sprintf('Total: %.1f MB', max_memory/1024);
                else
                    mem_text = sprintf('Total: %.1f KB', max_memory);
                end
                text(ax, 0.5, 0.9, mem_text, 'Units', 'normalized', 'HorizontalAlignment', 'center');
                
                hold(ax, 'off');
                
                % Update for change detection
                last_data.memory = current_memory_pct;
            end
        end
    else
        % No memory data
        if mod(kd, 30) == 0 || kd < 5
            cla(ax);
            
            % Check if memory hardware exists
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0
                text(ax, 0.5, 0.5, 'Memory Hardware Present - No Data', 'HorizontalAlignment', 'center');
            else
                text(ax, 0.5, 0.5, 'No Memory Hardware', 'HorizontalAlignment', 'center');
            end
            
            axis(ax, 'off');
        end
    end
catch
    % Silent error handling
    if mod(kd, 30) == 0 || kd < 5
        cla(ax);
        text(ax, 0.5, 0.5, 'Memory Data Error', 'HorizontalAlignment', 'center');
        axis(ax, 'off');
    end
end

%% MISSION PROGRESS
ax = subplot_handles{2, 3};

try
    % Calculate mission progress as % of simulation time
    progress_pct = time_sim_elapsed / (mission.true_time.t_final - mission.true_time.t_initial) * 100;
    
    % Only update occasionally
    if mod(kd, 10) == 0 || kd < 10
        cla(ax);
        
        % Create a circular progress indicator
        hold(ax, 'on');
        
        % Draw outer circle (gray)
        th = linspace(0, 2*pi, 100);
        x = cos(th);
        y = sin(th);
        plot(ax, x, y, 'k-', 'LineWidth', 2);
        
        % Draw mission progress as a partial filled circle
        progress_rad = progress_pct/100 * 2*pi;
        th_progress = linspace(0, progress_rad, 50);
        x_progress = cos(th_progress);
        y_progress = sin(th_progress);
        
        % Add center point and close the wedge
        x_progress = [0, x_progress, 0];
        y_progress = [0, y_progress, 0];
        
        % Fill with green
        fill(ax, x_progress, y_progress, 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
        
        % Add text in center
        text(ax, 0, 0, sprintf('%.1f%%', progress_pct), 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'middle', 'FontWeight', 'bold', 'FontSize', 14);
        
        axis(ax, 'equal');
        axis(ax, 'off');
        title(ax, 'Mission Progress');
        
        hold(ax, 'off');
    end
catch
    % Silent error handling
end

%% COMMUNICATION LINK STATUS
ax = subplot_handles{3, 1};

try
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_communication_link > 0
        % Check link status
        comm_link_index = 1; % Use first link
        
        % Current comms status (1=on, 0=off)
        try
            comms_on = mission.true_SC{i_SC}.true_SC_communication_link{comm_link_index}.store.flag_executive(kd);
        catch
            comms_on = 0;
        end
        
        % Only update if status changes or occasionally
        if mod(kd, 30) == 0 || (kd > 2 && comms_on ~= mission.true_SC{i_SC}.true_SC_communication_link{comm_link_index}.store.flag_executive(kd-1))
            cla(ax);
            
            % Get comms status data
            comms_status = [];
            try
                comms_status = mission.true_SC{i_SC}.true_SC_communication_link{comm_link_index}.store.flag_executive(sample_range);
            catch
                % Create fake data if not available
                comms_status = zeros(size(sample_range));
            end
            
            % Create binary view of comms status
            hold(ax, 'on');
            
            % Fill background for "off" state
            fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                 [0 0 0.5 0.5], [0.95 0.95 0.95], 'EdgeColor', 'none');
                
            % Fill background for "on" state
            fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                 [0.5 0.5 1 1], [0.8 1 0.8], 'EdgeColor', 'none');
            
            % Plot actual status line
            plot(ax, time_vec, comms_status * 0.5 + 0.25, 'k-', 'LineWidth', 2);
            
            % Add labels
            yticks(ax, [0.25, 0.75]);
            yticklabels(ax, {'OFF', 'ON'});
            ylim(ax, [0, 1]);
            
            grid(ax, 'on');
            title(ax, 'Communication Link');
            
            % Show current status with icon
            if comms_on
                text(ax, 0.5, 0.75, '📡 ACTIVE', 'HorizontalAlignment', 'center', ...
                     'FontWeight', 'bold', 'FontSize', 14, 'Color', 'g');
            else
                text(ax, 0.5, 0.25, '❌ INACTIVE', 'HorizontalAlignment', 'center', ...
                     'FontWeight', 'bold', 'FontSize', 14, 'Color', 'r');
            end
            
            hold(ax, 'off');
        end
    else
        if mod(kd, 30) == 0 % Only update occasionally
            cla(ax);
            text(ax, 0.5, 0.5, 'No Communication Link', 'HorizontalAlignment', 'center');
            axis(ax, 'off');
        end
    end
catch
    % Silent error handling
end

%% BATTERY STATUS
ax = subplot_handles{3, 2};

try
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery > 0
        % Get current battery status
        current_battery_pct = 0;
        for i_bat = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery
            try
                current_charge = mission.true_SC{i_SC}.true_SC_battery{i_bat}.store.state_of_charge(kd);
                current_battery_pct = current_charge * 100; % Convert to percentage
                break; % Just use first available battery
            catch
                % Skip if not available
            end
        end
        
        % Only update if battery changed by more than 1% or occasionally
        if mod(kd, 20) == 0 || abs(current_battery_pct - last_data.battery) > 1
            cla(ax);
            
            % Get battery data
            battery_charge = [];
            
            for i_bat = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery
                try
                    battery_charge = mission.true_SC{i_SC}.true_SC_battery{i_bat}.store.state_of_charge(sample_range) * 100;
                    break; % Just use first available battery
                catch
                    % Skip if not available
                end
            end
            
            if ~isempty(battery_charge)
                % Create battery level visualization
                hold(ax, 'on');
                
                % Draw battery outline
                plot(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1) time_vec(1)], ...
                     [0 0 100 100 0], 'k-', 'LineWidth', 2);
                
                % Fill with color based on level
                if current_battery_pct < 20
                    fill_color = [1 0.7 0.7]; % Red
                elseif current_battery_pct < 40
                    fill_color = [1 0.9 0.7]; % Orange
                else
                    fill_color = [0.7 1 0.7]; % Green
                end
                
                % Fill battery level
                fill(ax, [time_vec(1) time_vec(end) time_vec(end) time_vec(1)], ...
                     [0 0 battery_charge battery_charge], fill_color, 'EdgeColor', 'none');
                
                % Draw level line
                plot(ax, time_vec, battery_charge, 'k-', 'LineWidth', 2);
                
                % Add warning lines
                plot(ax, [time_vec(1) time_vec(end)], [20 20], 'r--', 'LineWidth', 1);
                plot(ax, [time_vec(1) time_vec(end)], [40 40], 'y--', 'LineWidth', 1);
                
                ylim(ax, [0 100]);
                grid(ax, 'on');
                title(ax, 'Battery Status');
                ylabel(ax, 'Charge (%)');
                
                % Show current value
                battery_text = sprintf('%.1f%%', current_battery_pct);
                if current_battery_pct < 20
                    text_color = 'r';
                    battery_text = ['CRITICAL: ' battery_text];
                elseif current_battery_pct < 40
                    text_color = [0.8 0.4 0]; % Dark orange
                    battery_text = ['LOW: ' battery_text];
                else
                    text_color = 'g';
                    battery_text = ['BATTERY: ' battery_text];
                end
                
                text(ax, 0.5, 0.5, battery_text, 'Units', 'normalized', 'FontWeight', 'bold', ...
                     'HorizontalAlignment', 'center', 'Color', text_color, 'FontSize', 14);
                
                hold(ax, 'off');
                
                % Update for change detection
                last_data.battery = current_battery_pct;
            end
        end
    else
        if mod(kd, 30) == 0 % Only update occasionally
            cla(ax);
            text(ax, 0.5, 0.5, 'No Battery Data', 'HorizontalAlignment', 'center');
            axis(ax, 'off');
        end
    end
catch
    % Silent error handling
end

%% ORBITAL STATUS 
ax = subplot_handles{3, 3};

try
    if mod(kd, 20) == 0 || kd < 5 % Only update occasionally
        cla(ax);
        
        % Show target-relative position
        pos = mission.true_SC{i_SC}.true_SC_navigation.position_relative_target;
        dist = norm(pos);
        
        % Make a simple 2D representation
        hold(ax, 'on');
        
        % Draw target
        plot(ax, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
        
        % Draw SC position (scaled)
        scale_factor = 1; % Base scale
        
        % Very basic auto-scaling
        if dist > 1000
            scale_factor = 1000;
        elseif dist > 100
            scale_factor = 100;
        elseif dist > 10
            scale_factor = 10;
        end
        
        pos_scaled = pos / scale_factor;
        plot(ax, pos_scaled(1), pos_scaled(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
        
        % Connect with line
        plot(ax, [0, pos_scaled(1)], [0, pos_scaled(2)], 'b-');
        
        % Add label showing scale
        text(ax, 0.05, 0.95, sprintf('Scale: 1 unit = %d km', scale_factor), ...
             'Units', 'normalized', 'HorizontalAlignment', 'left');
        
        % Add distance text
        text(ax, 0.5, 0.1, sprintf('Distance: %.1f km', dist), ...
             'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        
        axis(ax, 'equal');
        grid(ax, 'on');
        title(ax, 'Orbit Status');
        
        % Set reasonable limits
        max_val = max(abs(pos_scaled)) * 1.2;
        if max_val < 1
            max_val = 1;
        end
        xlim(ax, [-max_val, max_val]);
        ylim(ax, [-max_val, max_val]);
        
        hold(ax, 'off');
    end
catch
    % Silent error handling
end

% Update last time check
last_data.time = mission.true_time.time;

% Very lightweight update - only draws what's been changed
drawnow limitrate;
end 