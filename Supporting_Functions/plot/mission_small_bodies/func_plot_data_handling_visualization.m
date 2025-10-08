%% Data Handling Vizualization

function func_plot_data_handling_visualization(mission, i_SC)

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
    text(0.5, 0.5, 'No data handling data available - simulation did not run', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
    axis off;
    return;
end

plot_handle = figure('Name',['SC ', num2str(i_SC), ' Data Handling Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Data Generated Removed % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_data_handling')

    subplot(2,2,1)
    hold on

    % Get the actual data sizes to ensure we don't exceed array limits
    if isfield(mission.true_SC{i_SC}.true_SC_data_handling.store, 'instantaneous_data_generated')
        data_generated = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_generated;
        data_removed = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_removed;

        % Determine the number of valid data points
        valid_points = min([kd, length(data_generated), length(data_removed)]);

        % Get the time vector for the valid points
        valid_time = mission.true_time.store.time(1:valid_points);

        % Plot with appropriate units and labels
        area(valid_time, data_generated(1:valid_points), 'FaceColor', [0.4, 0.8, 0.4], 'FaceAlpha', 0.3, 'DisplayName', 'Generated');
        area(valid_time, data_removed(1:valid_points), 'FaceColor', [0.8, 0.4, 0.4], 'FaceAlpha', 0.3, 'DisplayName', 'Removed');
        plot(valid_time, data_generated(1:valid_points), '-', 'Color', [0, 0.5, 0], 'LineWidth', 1.5, 'DisplayName', 'Gen (Line)');
        plot(valid_time, data_removed(1:valid_points), '-', 'Color', [0.7, 0, 0], 'LineWidth', 1.5, 'DisplayName', 'Rem (Line)');

        % Calculate some statistics for annotations
        total_generated = sum(data_generated(1:valid_points));
        total_removed = sum(data_removed(1:valid_points));

        % Convert to appropriate units
        [gen_value, gen_unit] = convert_data_units(total_generated);
        [rem_value, rem_unit] = convert_data_units(total_removed);

        % Average rates
        avg_gen_rate = total_generated/valid_time(end);
        avg_rem_rate = total_removed/valid_time(end);
        [gen_rate_val, gen_rate_unit] = convert_data_rate(avg_gen_rate);
        [rem_rate_val, rem_rate_unit] = convert_data_rate(avg_rem_rate);

        % Add annotation with totals and rates
        data_str = sprintf('Total Generated: %.2f %s\nTotal Removed: %.2f %s\nAvg Gen Rate: %.2f %s\nAvg Rem Rate: %.2f %s', ...
            gen_value, gen_unit, rem_value, rem_unit, gen_rate_val, gen_rate_unit, rem_rate_val, rem_rate_unit);

        text(valid_time(end)*0.05, max(data_generated(1:valid_points))*0.9, data_str, ...
            'FontSize', mission.storage.plot_parameters.standard_font_size*0.8, ...
            'BackgroundColor', [1 1 1 0.7]);

        %         % Identify transmission events (large data removal spikes)
        %         transmission_threshold = 5 * mean(data_removed(data_removed > 0)); % 5x average non-zero removal
        %         transmission_indices = find(data_removed > transmission_threshold);
        %
        %         % Mark transmission events
        %         if ~isempty(transmission_indices)
        %             for i = 1:length(transmission_indices)
        %                 if transmission_indices(i) <= valid_points
        %                     idx = transmission_indices(i);
        %                     plot(valid_time(idx), data_removed(idx), 'rv', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Transmission');
        %
        %                     % Only label first few to avoid clutter
        %                     if i <= 3
        %                         text(valid_time(idx), data_removed(idx)*1.1, ' Transmission', ...
        %                             'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
        %                             'Color', [0.7, 0, 0], 'Rotation', 45);
        %                     end
        %                 end
        %             end
        %         end

    else
        text(0.5, 0.5, 'No data generation/removal information available', ...
            'HorizontalAlignment', 'center', 'FontSize', 12);
    end

    grid on
    legend('Location','northwest')
    xlabel('Time [sec]')
    ylabel('Data [kb/timestep]')
    title('Data Generation vs Removal','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Onboard Memory % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_onboard_memory')

    subplot(2,2,2)
    hold on

    % Check if we have any onboard memory devices
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0

        % Track total memory capacity and usage for annotation
        total_max_capacity = 0;
        total_current_usage = 0;

        for i_HW = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory
            % Get memory data safely
            memory = mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW};

            % Check valid data length
            valid_points = min(kd, length(memory.store.instantaneous_capacity));
            valid_time = mission.true_time.store.time(1:valid_points);

            % Plot memory capacity
            semilogy(valid_time, memory.store.instantaneous_capacity(1:valid_points), '-', ...
                'LineWidth', 2, 'Color', mission.storage.plot_parameters.color_array(i_HW), ...
                'DisplayName', memory.name)

            % Plot maximum capacity as dashed line
            semilogy(valid_time([1 end]), memory.store.maximum_capacity*[1 1]', '--', ...
                'LineWidth', 1, 'Color', mission.storage.plot_parameters.color_array(i_HW), ...
                'DisplayName', [memory.name ' Max'])

            %             % Update totals for annotation
            %             total_max_capacity = total_max_capacity + memory.store.maximum_capacity;
            %             if valid_points > 0
            %                 total_current_usage = total_current_usage + memory.store.instantaneous_capacity(valid_points);
            %             end

            %             % Identify memory resets (transmission events)
            %             mem_data = memory.store.instantaneous_capacity(1:valid_points);
            %             reset_indices = find(diff(mem_data) < -0.5*max(mem_data)); % Look for 50% drops
            %
            %             % Mark memory reset points
            %             if ~isempty(reset_indices)
            %                 for i = 1:length(reset_indices)
            %                     idx = reset_indices(i) + 1; % Point after the drop
            %                     if idx <= valid_points
            %                         plot(valid_time(idx), mem_data(idx), 'v', 'MarkerSize', 8, ...
            %                             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', ...
            %                             'DisplayName', 'Memory Reset');
            %                     end
            %                 end
            %             end
        end        

        % Can be used for more informations :

        % % Add annotation with total memory statistics
        % [max_cap_value, max_cap_unit] = convert_data_units(total_max_capacity);
        % [curr_usage_value, curr_usage_unit] = convert_data_units(total_current_usage);
        % usage_percent = 100 * total_current_usage / total_max_capacity;
        %
        % mem_str = sprintf('Total Capacity: %.2f %s\nCurrent Usage: %.2f %s\nUsage: %.1f%%', ...
        %     max_cap_value, max_cap_unit, curr_usage_value, curr_usage_unit, usage_percent);
        %
        % Position annotation in upper right corner
        %annotation('textbox', [0.46, 0.80, 0.15, 0.08], 'String', mem_str, ...
        %   'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);


    else
        text(0.5, 0.5, 'No onboard memory devices', ...
            'HorizontalAlignment', 'center', 'FontSize', 12);
    end

    grid on
    ylim([1 inf])
    set(gca, 'YScale', 'log')
    legend('Location','southeast')
    xlabel('Time [sec]')
    ylabel('Data [kb]')
    title('Onboard Memory Status','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off


    subplot(2,2,3)
    hold on

    % Plot memory state of data storage (SoDS)
    if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0
        for i_HW = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory
            memory = mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW};

            % Check valid data length
            valid_points = min(kd, length(memory.store.state_of_data_storage));
            valid_time = mission.true_time.store.time(1:valid_points);

            % Plot state of data storage percentage
            plot(valid_time, memory.store.state_of_data_storage(1:valid_points), '-', ...
                'LineWidth', 2, 'Color', mission.storage.plot_parameters.color_array(i_HW), ...
                'DisplayName', memory.name)

            % Add fill level label at the end of each line
            if valid_points > 0
                current_sods = memory.store.state_of_data_storage(valid_points);
                text(valid_time(end), current_sods, sprintf(' %.1f%%', current_sods), ...
                    'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
                    'Color', mission.storage.plot_parameters.color_array(i_HW));
            end

            % Identify memory resets (transmission events)
            mem_data = memory.store.state_of_data_storage(1:valid_points);
            if length(mem_data) > 1
                reset_indices = find(diff(mem_data) < -10); % Look for 10% drops

                % Mark memory reset points
                if ~isempty(reset_indices)
                    for i = 1:length(reset_indices)
                        idx = reset_indices(i) + 1; % Point after the drop
                        if idx <= valid_points
                            plot(valid_time(idx), mem_data(idx), 'v', 'MarkerSize', 8, ...
                                'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', ...
                                'DisplayName', 'Data Transmitted');

                            % Calculate transmission metrics for first few events
                            if i <= 3
                                before_idx = reset_indices(i);
                                after_idx = idx;
                                data_sent_pct = mem_data(before_idx) - mem_data(after_idx);

                                % Only annotate significant transmissions
                                if data_sent_pct > 10
                                    text(valid_time(after_idx), mem_data(after_idx) + 3, ...
                                        sprintf('%.1f%% sent', data_sent_pct), ...
                                        'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
                                        'Rotation', 45);
                                end
                            end
                        end
                    end
                end
            end
        end

        % Add memory fill thresholds
        plot(valid_time([1 end]), [80 80], '--r', 'LineWidth', 1, 'DisplayName', 'Warning (80%)');
        plot(valid_time([1 end]), [90 90], '--m', 'LineWidth', 1, 'DisplayName', 'Critical (90%)');

    else
        text(0.5, 0.5, 'No memory state data available', ...
            'HorizontalAlignment', 'center', 'FontSize', 12);
    end

    grid on
    legend('Location','northwest')
    xlabel('Time [sec]')
    ylabel('Memory Usage [%]')
    title('Memory Fill Level','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Transmission Analysis % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,4)
hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_data_handling') && ...
        isfield(mission.true_SC{i_SC}.true_SC_data_handling.store, 'instantaneous_data_generated')

    data_generated = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_generated;
    data_removed = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_removed;

    % Determine the number of valid data points
    valid_points = min([kd, length(data_generated), length(data_removed)]);

    % Get the time vector for the valid points
    valid_time = mission.true_time.store.time(1:valid_points);

    % Calculate cumulative generation and removal
    cum_generated = cumsum(data_generated(1:valid_points));
    cum_removed = cumsum(data_removed(1:valid_points));

    % Plot cumulative trends
    plot(valid_time, cum_generated, '-', 'LineWidth', 2, 'Color', [0, 0.5, 0], 'DisplayName', 'Cumulative Generated');
    plot(valid_time, cum_removed, '-', 'LineWidth', 2, 'Color', [0.7, 0, 0], 'DisplayName', 'Cumulative Removed');

    % Calculate and plot data accumulation (difference between generation and removal)
    data_accumulated = cum_generated - cum_removed;
    plot(valid_time, data_accumulated, '-', 'LineWidth', 2, 'Color', [0, 0, 0.7], 'DisplayName', 'Net Accumulated');

    %     % Identify transmission events (large data removal increases)
    %     transmission_threshold = 5 * mean(diff(cum_removed(diff(cum_removed) > 0))); % 5x average increase
    %     transmission_indices = find(diff(cum_removed) > transmission_threshold) + 1;
    %
    %     % Extract transmission times and amounts
    %     transmission_times = [];
    %     transmission_amounts = [];
    %
    %     % Mark transmission events
    %     if ~isempty(transmission_indices)
    %         for i = 1:length(transmission_indices)
    %             idx = transmission_indices(i);
    %             if idx <= valid_points && idx > 1
    %                 % Find where transmission starts (look for slope change)
    %                 start_idx = idx - 1;
    %                 while start_idx > 1 && abs(diff(cum_removed(start_idx-1:start_idx))) > 0.1 * transmission_threshold
    %                     start_idx = start_idx - 1;
    %                 end
    %
    %                 % Calculate actual amount transmitted
    %                 if idx <= length(cum_removed) && start_idx >= 1
    %                     amount_removed = cum_removed(idx) - cum_removed(start_idx);
    %
    %                     % Store for statistics
    %                     transmission_times = [transmission_times; valid_time(idx)];
    %                     transmission_amounts = [transmission_amounts; amount_removed];
    %
    %                     % Mark transmission event
    %                     plot(valid_time(idx), cum_removed(idx), 'rv', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Transmission');
    %
    %                     % Draw vertical line indicating transmission
    %                     plot([valid_time(idx), valid_time(idx)], [0, cum_removed(idx)], ':', 'Color', [0.7, 0, 0], 'HandleVisibility', 'off');
    %
    %                     % Only annotate first few transmissions to avoid clutter
    %                     if i <= 5
    %                         [amount_value, amount_unit] = convert_data_units(amount_removed);
    %                         text(valid_time(idx), cum_removed(idx), sprintf(' %.1f %s', amount_value, amount_unit), ...
    %                             'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
    %                             'Color', [0.7, 0, 0]);
    %                     end
    %                 end
    %             end
    %         end
    %     end

    %     % Add transmission statistics
    %     if length(transmission_times) >= 2
    %         % Calculate average time between transmissions
    %         time_between = diff(transmission_times);
    %         avg_time_between = mean(time_between);
    %         hours_between = avg_time_between / 3600;
    %
    %         % Calculate average transmission size
    %         avg_size = mean(transmission_amounts);
    %         [avg_size_value, avg_size_unit] = convert_data_units(avg_size);
    %
    %         % Calculate overall data generation and transmission rates
    %         overall_gen_rate = cum_generated(end) / valid_time(end);
    %         overall_trans_rate = cum_removed(end) / valid_time(end);
    %
    %         % Estimate time to next transmission based on recent pattern
    %         if length(transmission_times) >= 2
    %             next_transmission = transmission_times(end) + avg_time_between;
    %             time_to_next = next_transmission - valid_time(end);
    %
    %             if time_to_next > 0
    %                 mins_to_next = time_to_next / 60;
    %                 if mins_to_next < 60
    %                     next_str = sprintf('Est. time to next transmission: %.0f min', mins_to_next);
    %                 else
    %                     next_str = sprintf('Est. time to next transmission: %.1f hours', mins_to_next/60);
    %                 end
    %             else
    %                 next_str = 'Transmission due now';
    %             end
    %         else
    %             next_str = '';
    %         end
    %
    %         % Add statistics annotation
    %         stats_str = sprintf(['Detected %d transmissions\nAvg interval: %.1f hours\n' ...
    %             'Avg size: %.1f %s\n%s'], ...
    %             length(transmission_times), hours_between, avg_size_value, avg_size_unit, next_str);
    %
    %         text(valid_time(1) + (valid_time(end)-valid_time(1))*0.05, max(cum_generated)*0.9, stats_str, ...
    %             'FontSize', mission.storage.plot_parameters.standard_font_size*0.8, ...
    %             'BackgroundColor', [1 1 1 0.7]);
    %     elseif length(transmission_times) == 1
    %         % Only one transmission detected
    %         [amount_value, amount_unit] = convert_data_units(transmission_amounts);
    %
    %         stats_str = sprintf('1 transmission detected\nAmount: %.1f %s', amount_value, amount_unit);
    %
    %         text(valid_time(1) + (valid_time(end)-valid_time(1))*0.05, max(cum_generated)*0.9, stats_str, ...
    %             'FontSize', mission.storage.plot_parameters.standard_font_size*0.8, ...
    %             'BackgroundColor', [1 1 1 0.7]);
    %     else
    %         % No transmissions detected
    %         text(valid_time(1) + (valid_time(end)-valid_time(1))*0.05, max(cum_generated)*0.9, 'No transmissions detected', ...
    %             'FontSize', mission.storage.plot_parameters.standard_font_size*0.8, ...
    %             'BackgroundColor', [1 1 1 0.7]);
    %     end

else
    text(0.5, 0.5, 'No transmission data available', ...
        'HorizontalAlignment', 'center', 'FontSize', 12);
end

grid on
legend('Location', 'Northwest')
xlabel('Time [sec]')
ylabel('Cumulative Data [kb]')
title('Transmission Cycle Analysis', 'FontSize', mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)
hold off

% Add overall title
sgtitle(['SC ', num2str(i_SC), ' Data Handling Performance'], ...
    'FontSize', mission.storage.plot_parameters.title_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)

% Save the plot if flag is set
if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle, [mission.storage.output_folder, mission.name, '_SC', num2str(i_SC), '_Data_Handling.png'])
end

end

%% Helper function to convert data units for display
function [value, unit] = convert_data_units(kb_value)
% Convert kb to appropriate unit based on magnitude
if kb_value < 1
    value = kb_value * 1000; % Convert to bits
    unit = 'bits';
elseif kb_value < 1000
    value = kb_value;
    unit = 'kb';
elseif kb_value < 1000000
    value = kb_value / 1000;
    unit = 'MB';
elseif kb_value < 1000000000
    value = kb_value / 1000000;
    unit = 'GB';
else
    value = kb_value / 1000000000;
    unit = 'TB';
end
end

%% Helper function to convert data rate units
function [value, unit] = convert_data_rate(kbps_value)
% Convert kbps to appropriate unit based on magnitude
if kbps_value < 0.001
    value = kbps_value * 1000000; % Convert to bps
    unit = 'bps';
elseif kbps_value < 1
    value = kbps_value * 1000; % Convert to bps
    unit = 'bps';
elseif kbps_value < 1000
    value = kbps_value;
    unit = 'kbps';
elseif kbps_value < 1000000
    value = kbps_value / 1000;
    unit = 'Mbps';
else
    value = kbps_value / 1000000;
    unit = 'Gbps';
end
end
