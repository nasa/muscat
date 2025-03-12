function func_plot_power_visualization(mission, i_SC)

% Get the actual number of timesteps that have been completed
if isfield(mission.true_time, 'k') && mission.true_time.k > 0
    % If simulation is running or stopped early, use the current timestep
    actual_steps = mission.true_time.k;
else
    % If simulation completed or k isn't available, use storage
    actual_steps = mission.storage.k_storage;
end

% Ensure we don't exceed the available data
kd = min(actual_steps, mission.storage.k_storage);

% Check if we have any data to plot
if kd <= 0
    figure();
    text(0.5, 0.5, 'No power data available - simulation did not run', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
    axis off;
    return;
end

%% Ensure we have enough colors for all hardware components
% Check if color_array exists and has sufficient colors
if ~isfield(mission.storage.plot_parameters, 'color_array') || ...
   length(mission.storage.plot_parameters.color_array) < 12
    
    % Create a fallback color array with many distinct colors
    fallback_colors = [
        0.0000, 0.4470, 0.7410; % Default MATLAB blue
        0.8500, 0.3250, 0.0980; % Default MATLAB orange
        0.9290, 0.6940, 0.1250; % Default MATLAB yellow
        0.4940, 0.1840, 0.5560; % Default MATLAB purple
        0.4660, 0.6740, 0.1880; % Default MATLAB green
        0.3010, 0.7450, 0.9330; % Default MATLAB light blue
        0.6350, 0.0780, 0.1840; % Default MATLAB burgundy
        0.0000, 0.0000, 1.0000; % Blue
        1.0000, 0.0000, 0.0000; % Red
        0.0000, 0.5000, 0.0000; % Dark green
        0.7500, 0.0000, 0.7500; % Magenta
        0.0000, 0.7500, 0.7500; % Cyan
        0.7500, 0.7500, 0.0000; % Yellow
        0.2500, 0.2500, 0.2500; % Dark gray
        0.7500, 0.7500, 0.7500; % Light gray
        0.5000, 0.5000, 0.0000; % Olive
        0.5000, 0.0000, 0.5000; % Purple
        0.0000, 0.5000, 0.5000; % Teal
        0.5000, 0.2500, 0.0000; % Brown
        0.2500, 0.5000, 0.0000; % Dark olive
        0.0000, 0.2500, 0.5000; % Navy
        0.2500, 0.0000, 0.5000; % Indigo
        0.0000, 0.9000, 0.4000; % Spring green
        0.9000, 0.0000, 0.4000; % Crimson
    ];
    
    % Use the fallback color array instead
    color_array = fallback_colors;
else
    % Use the existing color array
    color_array = mission.storage.plot_parameters.color_array;
end

%% Get time data for plotting
% Get actual time data (only the valid portion)
time_data = mission.true_time.store.time(1:kd);

% For memory optimization with large datasets, determine if we need to downsample
plot_count = length(time_data);
if plot_count > 10000
    % Calculate downsample factor - aim for around 5000 points
    downsample_factor = max(1, floor(plot_count / 5000));
    plot_indices = 1:downsample_factor:plot_count;
    % Make sure we include the last point
    if plot_indices(end) ~= plot_count
        plot_indices = [plot_indices, plot_count];
    end
    
    % Use downsampled data for plots
    plot_time = time_data(plot_indices);
else
    % Use all data points
    plot_indices = 1:plot_count;
    plot_time = time_data;
end

%% Power Visualization - Main Figure
plot_handle = figure('Name',['SC ', num2str(i_SC), ' Power Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

% Change layout to 3x3 to accommodate battery plots
subplot_rows = 3;
subplot_cols = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Summary Plot - Power Generated & Consumed with Energy Balance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_power')
    % Get valid power data up to the current step
    power_generated = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(1:kd);
    power_consumed = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(1:kd);
    
    % Compute downsampled data for plotting
    plot_power_gen = power_generated(plot_indices);
    plot_power_con = power_consumed(plot_indices);
    
    % Calculate power deficit (negative means more consumption than generation)
    power_deficit = power_generated - power_consumed;
    plot_power_deficit = power_deficit(plot_indices);
    
    % Calculate cumulative energy balance (in Watt-hours)
    time_step = mission.true_time.time_step;
    energy_balance = cumsum(power_deficit * (time_step / 3600)); % Watt-sec to Watt-hour
    plot_energy_balance = energy_balance(plot_indices);
    
    % Calculate total energy metrics
    total_energy_generated = sum(power_generated) * (time_step / 3600);
    total_energy_consumed = sum(power_consumed) * (time_step / 3600);
    final_energy_balance = energy_balance(end);
    
    % Plot power generation and consumption
    subplot(subplot_rows, subplot_cols, 1)
    hold on
    area(plot_time, plot_power_gen, 'FaceColor', [0.4, 0.8, 0.4], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    area(plot_time, plot_power_con, 'FaceColor', [0.8, 0.4, 0.4], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    plot(plot_time, plot_power_gen, '-', 'Color', [0, 0.5, 0], 'LineWidth', 2);
    plot(plot_time, plot_power_con, '-', 'Color', [0.7, 0, 0], 'LineWidth', 2);
    
    grid on
    legend('Generated', 'Consumed', 'Location', 'NorthEast')
    xlabel('Time [sec]')
    ylabel('Power [Watts]')
    title('Power Generation vs Consumption', 'FontSize', mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type)
    
    % Add annotation showing total energy metrics
    energy_str = sprintf('Energy Generated: %.1f Whr\nEnergy Consumed: %.1f Whr\nBalance: %.1f Whr', ...
        total_energy_generated, total_energy_consumed, final_energy_balance);
    annotation('textbox', [0.15, 0.82, 0.2, 0.1], 'String', energy_str, ...
        'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);
    hold off
    
    % Plot power deficit (generation - consumption)
    subplot(subplot_rows, subplot_cols, 2)
    hold on
    
    % Use area plot to highlight deficit periods
    area(plot_time, plot_power_deficit, 'FaceColor', [0.5, 0.8, 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    area(plot_time, min(0, plot_power_deficit), 'FaceColor', [0.8, 0.5, 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    plot(plot_time, plot_power_deficit, '-', 'Color', [0, 0, 0.8], 'LineWidth', 2);
    plot(plot_time([1, end]), [0, 0], '--k', 'LineWidth', 1);
    
    grid on
    xlabel('Time [sec]')
    ylabel('Power Balance [Watts]')
    title('Power Surplus/Deficit', 'FontSize', mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type)
    
    % Calculate percentage of time with power deficit
    deficit_percentage = 100 * sum(power_deficit < 0) / length(power_deficit);
    deficit_time_str = sprintf('Time in Deficit: %.1f%%', deficit_percentage);
    annotation('textbox', [0.39, 0.82, 0.15, 0.03], 'String', deficit_time_str, ...
        'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);
    hold off
    
    % Plot cumulative energy balance
    subplot(subplot_rows, subplot_cols, 3)
    hold on
    
    % Use area plot with different colors for positive/negative
    area(plot_time, max(0, plot_energy_balance), 'FaceColor', [0.5, 0.8, 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    area(plot_time, min(0, plot_energy_balance), 'FaceColor', [0.8, 0.5, 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    plot(plot_time, plot_energy_balance, '-', 'Color', [0, 0, 0.8], 'LineWidth', 2);
    plot(plot_time([1, end]), [0, 0], '--k', 'LineWidth', 1);
    
    grid on
    xlabel('Time [sec]')
    ylabel('Energy Balance [Whr]')
    title('Cumulative Energy Balance', 'FontSize', mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type)
    
    % Add annotation showing final energy balance
    balance_str = sprintf('Final Energy Balance: %.1f Whr', final_energy_balance);
    text_color = [0 0 0]; % Default black
    if final_energy_balance < 0
        text_color = [0.8 0 0]; % Red for negative
    elseif final_energy_balance > 0
        text_color = [0 0.5 0]; % Green for positive
    end
    
    annotation('textbox', [0.64, 0.82, 0.15, 0.03], 'String', balance_str, ...
        'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7], ...
        'Color', text_color);
    hold off
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Solar Panel Performance with Sun Angle % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_solar_panel') && ...
   mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel > 0
    
    % Calculate solar panel metrics only if we have solar panel data
    num_panels = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel;
    
    % Check for valid data in the first solar panel
    if isfield(mission.true_SC{i_SC}.true_SC_solar_panel{1}.store, 'instantaneous_power_generated') && ...
       kd <= length(mission.true_SC{i_SC}.true_SC_solar_panel{1}.store.instantaneous_power_generated)
        
        % Solar Panel Efficiency Plot (subplot 4)
        subplot(subplot_rows, subplot_cols, 4)
    hold on
    
    % Calculate total power generated by all solar panels
    total_power = zeros(kd, 1);
        total_max = zeros(kd, 1);
        
        for i_HW = 1:num_panels
            try
                % Get power data and ensure it's valid
                panel_power = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.instantaneous_power_generated(1:kd);
                panel_max = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.maximum_power(1:kd);
                
                total_power = total_power + panel_power;
                total_max = total_max + panel_max;
            catch
                warning('Solar panel %d data incomplete. Skipping.', i_HW);
            end
    end
    
    % Calculate efficiency
    efficiency = zeros(kd, 1);
    for i = 1:kd
        if total_max(i) > 0
            efficiency(i) = 100 * total_power(i) / total_max(i);
        else
            efficiency(i) = 0;
        end
    end
    
        % Get downsampled data for plotting
        plot_efficiency = efficiency(plot_indices);
        
        % Plot efficiency as an area to make it more visible
        area(plot_time, plot_efficiency, 'FaceColor', [0.4, 0.6, 0.9], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
        plot(plot_time, plot_efficiency, '-b', 'LineWidth', 2);
        
        % Also plot the mean efficiency as a horizontal line
        mean_eff = mean(efficiency(efficiency > 0)); % Ignore zero values
        if ~isnan(mean_eff) && mean_eff > 0
            plot(plot_time([1, end]), [mean_eff, mean_eff], '--k', 'LineWidth', 1.5);
            text(plot_time(end) * 0.95, mean_eff * 1.05, sprintf('Mean: %.1f%%', mean_eff), ...
                'FontSize', mission.storage.plot_parameters.standard_font_size * 0.8, ...
                'HorizontalAlignment', 'right');
        end
    
    grid on
    xlabel('Time [sec]')
    ylabel('Efficiency [%]')
        title('Solar Panel Efficiency', 'FontSize', mission.storage.plot_parameters.title_font_size)
        ylim([0, max(efficiency) * 1.1 + 1]);
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
                'FontName', mission.storage.plot_parameters.standard_font_type)
    hold off
        
        % Individual Solar Panel Power Generation (subplot 5)
        subplot(subplot_rows, subplot_cols, 5)
    hold on
    
    sp_legend = {};
        for i_HW = 1:num_panels
            try
                % Get power data and ensure it's valid
                panel_power = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.instantaneous_power_generated(1:kd);
                plot_panel_power = panel_power(plot_indices);
                
        % Safe color indexing
        color_index = mod(i_HW-1, length(color_array)) + 1;
                
                % Plot solar panel power
                plot(plot_time, plot_panel_power, '-', 'LineWidth', 1.5, 'Color', color_array(color_index,:));
        sp_legend{end+1} = ['SP ', num2str(i_HW)];
            catch
                warning('Solar panel %d data incomplete. Skipping.', i_HW);
            end
    end
    
    grid on
        if ~isempty(sp_legend)
            legend(sp_legend, 'Location', 'NorthEast')
        end
    xlabel('Time [sec]')
    ylabel('Power [Watts]')
        title('Solar Panel Power Generation', 'FontSize', mission.storage.plot_parameters.title_font_size)
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
                'FontName', mission.storage.plot_parameters.standard_font_type)
    hold off
    
        % Sun incidence angle vs power scatter plot (subplot 6)
        subplot(subplot_rows, subplot_cols, 6)
    hold on
    
    % For each solar panel, plot sun angle vs power as scatter plot
        for i_HW = 1:num_panels
            try
                % Get and validate angle and power data
                sun_angle = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.Sun_incidence_angle(1:kd);
                panel_power = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.instantaneous_power_generated(1:kd);
                
                % Only plot valid data points
                valid_idx = ~isnan(sun_angle) & ~isnan(panel_power);
                if sum(valid_idx) > 0
                    % Downsample for scatter plot too - but based on valid points
                    valid_points = find(valid_idx);
                    if length(valid_points) > 5000
                        downsample_factor = max(1, floor(length(valid_points) / 5000));
                        scatter_indices = valid_points(1:downsample_factor:end);
                    else
                        scatter_indices = valid_points;
                    end
                    
        % Safe color indexing
        color_index = mod(i_HW-1, length(color_array)) + 1;
                    
                    scatter(sun_angle(scatter_indices), panel_power(scatter_indices), ...
                            10, color_array(color_index,:), 'filled', 'MarkerFaceAlpha', 0.5);
                end
            catch
                warning('Solar panel %d angle/power data incomplete. Skipping.', i_HW);
            end
    end
    
    grid on
    xlabel('Sun Incidence Angle [deg]')
    ylabel('Power [Watts]')
        title('Sun Angle vs Power Generated', 'FontSize', mission.storage.plot_parameters.title_font_size)
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
                'FontName', mission.storage.plot_parameters.standard_font_type)
        
        % Add a theoretical cosine curve for reference (if we have solar panel data)
        if num_panels > 0 && isfield(mission.true_SC{i_SC}.true_SC_solar_panel{1}, 'max_power')
            try
                max_power_overall = mission.true_SC{i_SC}.true_SC_solar_panel{1}.max_power;
                angles = 0:90;  % 0 to 90 degrees
                theoretical_power = max_power_overall * cosd(angles);
                plot(angles, theoretical_power, '--k', 'LineWidth', 1);
                text(45, max_power_overall * 0.7, 'Theoretical (cos θ)', ...
                    'FontSize', mission.storage.plot_parameters.standard_font_size * 0.7, ...
                    'HorizontalAlignment', 'center');
            catch
                % Skip if data not available
            end
        end
        
        hold off
    else
        % Display message if solar panel data is incomplete
        for i = 4:6
            subplot(subplot_rows, subplot_cols, i)
            text(0.5, 0.5, 'Solar panel data incomplete or unavailable', ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            axis off;
        end
    end
else
    % Display message if no solar panels
    for i = 4:6
        subplot(subplot_rows, subplot_cols, i)
        text(0.5, 0.5, 'No solar panel data available', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        axis off;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Battery Performance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_battery') && ...
   mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery > 0
    
    num_batteries = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery;
    
    % Battery State of Charge Plot (subplot 7)
    subplot(subplot_rows, subplot_cols, 7)
    hold on
    
    % Plot SoC for each battery
    for i_batt = 1:num_batteries
        try
            batt_soc = mission.true_SC{i_SC}.true_SC_battery{i_batt}.store.state_of_charge(1:kd);
            plot_batt_soc = batt_soc(plot_indices);
            
            % Safe color indexing
            color_index = mod(i_batt-1, length(color_array)) + 1;
            
            % Plot battery SoC
            plot(plot_time, plot_batt_soc, '-', 'LineWidth', 1.5, 'Color', color_array(color_index,:));
            batt_legend{i_batt} = ['Battery ', num2str(i_batt)];
        catch
            warning('Battery %d data incomplete. Skipping.', i_batt);
        end
    end
    
    grid on
    if exist('batt_legend', 'var')
        legend(batt_legend, 'Location', 'Best')
    end
    xlabel('Time [sec]')
    ylabel('State of Charge [%]')
    title('Battery State of Charge', 'FontSize', mission.storage.plot_parameters.title_font_size)
    ylim([0, 100])
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
            'FontName', mission.storage.plot_parameters.standard_font_type)
    hold off
    
    % Battery Capacity Plot (subplot 8)
    subplot(subplot_rows, subplot_cols, 8)
    hold on
    
    total_capacity = 0;
    current_capacity = 0;
    
    for i_batt = 1:num_batteries
        try
            batt = mission.true_SC{i_SC}.true_SC_battery{i_batt};
            if batt.health == 1
                total_capacity = total_capacity + batt.maximum_capacity;
                current_capacity = current_capacity + batt.instantaneous_capacity;
                
                batt_capacity = batt.store.instantaneous_capacity(1:kd);
                plot_batt_capacity = batt_capacity(plot_indices);
                
                % Safe color indexing
                color_index = mod(i_batt-1, length(color_array)) + 1;
                
                % Plot battery capacity
                plot(plot_time, plot_batt_capacity, '-', 'LineWidth', 1.5, 'Color', color_array(color_index,:));
            end
        catch
            warning('Battery %d data incomplete. Skipping.', i_batt);
        end
    end
    
    grid on
    xlabel('Time [sec]')
    ylabel('Capacity [Whr]')
    title('Battery Capacity', 'FontSize', mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
            'FontName', mission.storage.plot_parameters.standard_font_type)
    
    % Add annotation showing total capacity info
    capacity_str = sprintf('Total Capacity: %.1f Whr\nCurrent: %.1f Whr', total_capacity, current_capacity);
    annotation('textbox', [0.64, 0.15, 0.15, 0.05], 'String', capacity_str, ...
        'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);
    hold off
    
    % Battery Power Emergency Events (subplot 9)
    subplot(subplot_rows, subplot_cols, 9)
    hold on
    
    if isfield(mission.true_SC{i_SC}.true_SC_power, 'store') && ...
       isfield(mission.true_SC{i_SC}.true_SC_power.store, 'instantaneous_energy')
        
        % Get energy data
        energy_data = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_energy(1:kd);
        plot_energy = energy_data(plot_indices);
        
        % Create area plot for energy flow
        area(plot_time, max(0, plot_energy), 'FaceColor', [0.4, 0.8, 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        area(plot_time, min(0, plot_energy), 'FaceColor', [0.8, 0.4, 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        plot(plot_time, plot_energy, '-', 'Color', [0, 0, 0.8], 'LineWidth', 1.5);
        
        % Add zero line
        plot(plot_time([1, end]), [0, 0], '--k', 'LineWidth', 1);
        
        % Calculate statistics
        discharge_time = sum(energy_data < 0) * (mission.true_time.time_step / 3600); % hours
        charge_time = sum(energy_data > 0) * (mission.true_time.time_step / 3600); % hours
        
        % Add annotation with statistics
        stats_str = sprintf('Charging: %.1f hrs\nDischarging: %.1f hrs', charge_time, discharge_time);
        annotation('textbox', [0.89, 0.15, 0.15, 0.05], 'String', stats_str, ...
            'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);
    end
    
    grid on
    xlabel('Time [sec]')
    ylabel('Energy Flow [Whr]')
    title('Battery Emergency Events (if any)', 'FontSize', mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
            'FontName', mission.storage.plot_parameters.standard_font_type)
    hold off
    
else
    % Display message if no batteries
    for i = 7:9
        subplot(subplot_rows, subplot_cols, i)
        text(0.5, 0.5, 'No battery data available', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        axis off;
    end
end

% Add overall title
sgtitle(['SC ', num2str(i_SC), ' Power Performance'], ...
    'FontSize', mission.storage.plot_parameters.title_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)

% Save the plot if flag is set
if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle, [mission.storage.output_folder, mission.name, '_SC', num2str(i_SC), '_Power.png'])
end

end