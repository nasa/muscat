function func_plot_individual_data_usage(mission, i_SC)

kd = mission.storage.k_storage;

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

%% Individual Data Usage Visualization - Main Figure
% Create figure with specific tag for callback functionality
plot_handle = figure('Tag', 'DataUsageFigure', 'Name', ['SC ', num2str(i_SC), ' Individual Data Generation']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

% Store font sizes in the figure's application data for callbacks to use
setappdata(plot_handle, 'std_font_size', mission.storage.plot_parameters.standard_font_size);
setappdata(plot_handle, 'std_font_type', mission.storage.plot_parameters.standard_font_type);
setappdata(plot_handle, 'title_font_size', mission.storage.plot_parameters.title_font_size);

% Check if we have data usage information available
if ~isfield(mission.true_SC{i_SC}, 'true_SC_data_handling') || ~isfield(mission.true_SC{i_SC}.true_SC_data_handling.store, 'list_HW_data_generated')
    % No data handling information available - show an empty plot with a message
    text(0.5, 0.5, 'No data usage information available', 'HorizontalAlignment', 'center');
    axis off;
    return;
end

% Get the list of hardware and corresponding data generation
hw_list = mission.true_SC{i_SC}.true_SC_data_handling.store.list_HW_data_generated;
num_hw = length(hw_list);

if num_hw == 0
    % No hardware in the list - just show an empty plot with a message
    text(0.5, 0.5, 'No hardware components with data generation information', 'HorizontalAlignment', 'center');
    axis off;
    return;
end

% Get hardware data generation information
hw_data_generated = mission.true_SC{i_SC}.true_SC_data_handling.store.array_HW_data_generated(1:kd, :);

% Sort hardware items by total data generation (for better organization)
total_hw_data = sum(hw_data_generated, 1);
[sorted_data, sort_idx] = sort(total_hw_data, 'descend');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Create a 2x4 layout with subplots % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create a figure with tight subplots
figure_margin = 0.05;

% Create the main plot in a 2x3 grid, spanning [1 2 4 5]
% This gives the main plot more horizontal space
main_plot = subplot(2, 3, [1 2 4 5]);
% Adjust position to maximize space usage
main_pos = get(main_plot, 'Position');
set(main_plot, 'Position', [figure_margin, main_pos(2), main_pos(3)*1.2, main_pos(4)]);
hold on;

% Store line handles for interactivity
line_handles = zeros(num_hw, 1);

% Determine if we need to downsample for very large datasets
time_data = mission.true_time.store.time(1:kd);
plot_count = length(time_data);

% Downsampling for large datasets (more than 10000 points)
if plot_count > 10000
    % Calculate downsample factor - aim for around 5000 points
    downsample_factor = max(1, floor(plot_count / 5000));
    plot_indices = 1:downsample_factor:plot_count;
    % Make sure we include the last point
    if plot_indices(end) ~= plot_count
        plot_indices = [plot_indices, plot_count];
    end
else
    plot_indices = 1:plot_count;
end

downsampled_time = time_data(plot_indices);

% Loop through all hardware components and plot them using scatter for better performance
for i = 1:num_hw
    hw_idx = sort_idx(i);
    
    % Safe color indexing
    color_index = mod(i-1, length(color_array)) + 1;
    
    % Use scatter for better performance with large datasets
    line_handles(i) = scatter(downsampled_time, hw_data_generated(plot_indices, hw_idx), 5, ...
        'MarkerEdgeColor', color_array(color_index), ...
        'MarkerFaceColor', color_array(color_index), ...
        'MarkerFaceAlpha', 0.5, ...
        'DisplayName', hw_list{hw_idx}, ...
        'Tag', ['hw_line_', num2str(i)]);
    
    % Connect points with a line - use a thin line for performance
    if length(plot_indices) > 1
        line(downsampled_time, hw_data_generated(plot_indices, hw_idx), ...
            'Color', [color_array(color_index), 0.3], 'LineWidth', 0.5);
    end
end

grid on;
xlabel('Time [sec]');
ylabel('Data [kb]');
title('All Data Generation', 'FontSize', mission.storage.plot_parameters.title_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);

% Set y-axis to logarithmic scale 
set(gca, 'YScale', 'log');

% Add the standard legend back to the left plot
% Only show top components to avoid extremely large legend
num_legend_items = min(10, num_hw); % Show at most 10 items in legend
legend_items = line_handles(1:num_legend_items);
legend_labels = arrayfun(@(i) hw_list{sort_idx(i)}, 1:num_legend_items, 'UniformOutput', false);
if num_hw > num_legend_items
    legend_extra = legend([legend_items; line_handles(num_legend_items+1)], [legend_labels, {'(More...)'}], 'Location', 'NorthEast');
else
    legend_extra = legend(legend_items, legend_labels, 'Location', 'NorthEast');
end
% Make legend more compact
legend_extra.FontSize = mission.storage.plot_parameters.standard_font_size * 0.6;
legend_extra.ItemHitFcn = @(~,~)disp('Use the hardware list to select items');

% Create the hardware selection panel in column 3
legend_pos = [0.68, 0.1, 0.10, 0.82]; % [x y width height] - narrower width
legend_panel = uipanel('Position', legend_pos, ...
                      'Title', 'Hardware Components', ...
                      'FontSize', mission.storage.plot_parameters.standard_font_size*0.9, ...
                      'FontName', mission.storage.plot_parameters.standard_font_type);

% Create the top right plot for selected hardware
top_right_plot = subplot(2, 3, 3);
top_right_pos = get(top_right_plot, 'Position');
% Adjust position for better alignment
set(top_right_plot, 'Position', [0.83, top_right_pos(2), 0.15, top_right_pos(4)]);
selected_handle = plot(time_data, ones(length(time_data), 1), '-', 'LineWidth', 3, 'Color', [0, 0.5, 0, 1]);
grid on;
xlabel('Time [sec]');
ylabel('Data [kb]');
title('Selected Hardware', 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size*0.8, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);
% Add initial instruction text
% text(0.5, 0.5, 'Select a hardware component from the list', ...
%     'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
%     'FontSize', mission.storage.plot_parameters.standard_font_size*0.8, ...
%     'FontName', mission.storage.plot_parameters.standard_font_type, ...
%     'Tag', 'instruction_text');

% Create the bottom right plot for pie chart
bottom_right_plot = subplot(2, 3, 6);
bottom_right_pos = get(bottom_right_plot, 'Position');
% Adjust position for better alignment
set(bottom_right_plot, 'Position', [0.83, bottom_right_pos(2), 0.15, bottom_right_pos(4)]);
% Only include top contributors in the pie chart to keep it readable
num_pie_items = min(8, num_hw);  % Maximum 8 items in pie chart for readability
top_hw_idx = sort_idx(1:num_pie_items);

% Calculate values for pie chart
top_data = total_hw_data(top_hw_idx);
total_data = sum(total_hw_data);
other_data = total_data - sum(top_data);

if num_hw > num_pie_items
    % If we have more hardware than we're showing in the pie chart,
    % add an "Other" category for the remaining hardware
    pie_data = [top_data, other_data];
    pie_labels = [hw_list(top_hw_idx), {'Other'}];
else
    pie_data = top_data;
    pie_labels = hw_list(top_hw_idx);
end

% Calculate percentages for labeling
percentages = 100 * pie_data / total_data;

% Create labels with percentages
pie_labels_with_pct = cell(size(pie_labels));
for i = 1:length(pie_labels)
    if percentages(i) >= 1
        % Shorten labels if too long for better visibility
        short_label = pie_labels{i};
        if length(short_label) > 15
            short_label = [short_label(1:12), '...'];
        end
        pie_labels_with_pct{i} = sprintf('%s (%.1f%%)', short_label, percentages(i));
    else
        pie_labels_with_pct{i} = '';  % Don't label very small slices
    end
end

% Create pie chart
p = pie(pie_data, pie_labels_with_pct);

% Style the pie chart text to match other plots
for i = 1:2:length(p)
    if ishghandle(p(i+1), 'text')  % Check if it's a text handle
        set(p(i+1), 'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
                   'FontName', mission.storage.plot_parameters.standard_font_type);
    end
end

title('Total Data by Component', 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);

% Add overall title for the figure
annotation('textbox', [0.25, 0.96, 0.5, 0.03], ...
    'String', ['SC ', num2str(i_SC), ' Individual Data Generation'], ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', ...
    'FontSize', mission.storage.plot_parameters.title_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);

% Create a listbox of hardware items (removed filter functionality)
% Store the data for the listbox
setappdata(plot_handle, 'hw_list', hw_list);
setappdata(plot_handle, 'hw_data', hw_data_generated);
setappdata(plot_handle, 'time_data', time_data);
setappdata(plot_handle, 'sort_idx', sort_idx);
setappdata(plot_handle, 'line_handles', line_handles);
setappdata(plot_handle, 'selected_plot', selected_handle);
setappdata(plot_handle, 'color_array', color_array);
setappdata(plot_handle, 'top_right_plot', top_right_plot);
setappdata(plot_handle, 'instruction_text', findobj(top_right_plot, 'Tag', 'instruction_text'));
% Store camera plot status flag - used to prevent overriding camera plot
setappdata(plot_handle, 'is_camera_plot', false);

% Calculate positions for controls inside the panel
listbox_height = 0.85;
listbox_width = 0.9;
listbox_x = 0.05;
listbox_y = 0.12;

% Create the listbox with hardware items (relative to panel)
hw_listbox = uicontrol('Parent', legend_panel, ...
    'Style', 'listbox', ...
    'Units', 'normalized', ...
    'Position', [listbox_x, listbox_y, listbox_width, listbox_height], ...
    'String', hw_list(sort_idx), ...
    'Callback', @(src, event) selectHardware(src, event, plot_handle), ...
    'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);
setappdata(plot_handle, 'hw_listbox', hw_listbox);

% Add reset button (relative to panel)
reset_pos = [listbox_x, 0.02, listbox_width, 0.09];
uicontrol('Parent', legend_panel, ...
    'Style', 'pushbutton', ...
    'Units', 'normalized', ...
    'Position', reset_pos, ...
    'String', 'Reset Highlight', ...
    'Callback', @(src, event) resetAllHighlights(src, event, plot_handle), ...
    'FontSize', mission.storage.plot_parameters.standard_font_size*0.7, ...
    'FontName', mission.storage.plot_parameters.standard_font_type);

% Save the plot if flag is set
if mission.storage.plot_parameters.flag_save_plots == 1
    % Use exportapp instead of saveas to properly include UI components
    output_file = [mission.storage.output_folder, mission.name, '_SC', num2str(i_SC), '_Individual_Data_Usage.png'];
    try
        exportapp(plot_handle, output_file);
    catch
        % Fallback to print if exportapp is not available (older MATLAB versions)
        %print(plot_handle, '-dpng', '-r150', output_file);
    end
end

end

% Callback function for hardware selection
function selectHardware(src, ~, fig_handle)
    % Get the selected item
    items = get(src, 'String');
    selected_idx = get(src, 'Value');
    
    if selected_idx <= 0 || selected_idx > length(items)
        return; % Invalid selection
    end
    
    % Get data from figure
    hw_list = getappdata(fig_handle, 'hw_list');
    hw_data = getappdata(fig_handle, 'hw_data');
    time_data = getappdata(fig_handle, 'time_data');
    sort_idx = getappdata(fig_handle, 'sort_idx');
    line_handles = getappdata(fig_handle, 'line_handles');
    selected_plot = getappdata(fig_handle, 'selected_plot');
    color_array = getappdata(fig_handle, 'color_array');
    top_right_plot = getappdata(fig_handle, 'top_right_plot');
    instruction_text = getappdata(fig_handle, 'instruction_text');
    std_font_size = getappdata(fig_handle, 'std_font_size');
    std_font_type = getappdata(fig_handle, 'std_font_type');
    
    % The actual index in the original data
    hw_idx = sort_idx(selected_idx);
    
    % Check if this is a camera plot (to protect from override)
    hw_name = hw_list{hw_idx};
    is_camera = contains(lower(hw_name), 'camera');
    
    % Reset all lines to normal state (only modify highlight, not visibility)
    for i = 1:length(line_handles)
        if ishandle(line_handles(i))
            color = get(line_handles(i), 'Color');
            set(line_handles(i), 'LineWidth', 1);
            % Ensure all lines have proper alpha
            if length(color) == 3
                set(line_handles(i), 'Color', [color, 0.3]);
            end
            % Push non-selected lines to back
            if i ~= selected_idx
                uistack(line_handles(i), 'bottom');
            end
        end
    end
    
    % Highlight the selected line
    % Get the color without transparency
    color_index = mod(selected_idx-1, size(color_array, 1)) + 1;
    color = color_array(color_index, :);
    
    % Set the highlighted line properties
    if ishandle(line_handles(selected_idx))
        set(line_handles(selected_idx), 'LineWidth', 3, 'Color', color);
        % Bring to front
        uistack(line_handles(selected_idx), 'top');
    end
    
    % Update the selected hardware plot in the top right
    axes(top_right_plot);
    % Hide the instruction text if it exists
    if ishandle(instruction_text)
        set(instruction_text, 'Visible', 'off');
    end
    
    set(selected_plot, 'XData', time_data, 'YData', hw_data(:, hw_idx), 'Color', color);
    set(top_right_plot, 'YScale', 'linear'); 
    
    % Update the plot title
    % Truncate very long hardware names for title to prevent overlap
    if length(hw_name) > 25
        hw_name = [hw_name(1:22), '...'];
    end
    title(['Selected: ', hw_name], 'FontSize', std_font_size, ...
        'FontName', std_font_type);
    
    % Adjust Y-axis limits with a bit of margin
    max_val = max(hw_data(:, hw_idx));
    if max_val > 0
        ylim([0, max_val*1.1]);
    else
        ylim([0, 1]); % Default if no data
    end
    
    % Store camera plot status
    setappdata(fig_handle, 'is_camera_plot', is_camera);
end

% Reset all highlights (but not the list)
function resetAllHighlights(~, ~, fig_handle)
    % Get data from figure
    line_handles = getappdata(fig_handle, 'line_handles');
    selected_plot = getappdata(fig_handle, 'selected_plot');
    top_right_plot = getappdata(fig_handle, 'top_right_plot');
    instruction_text = getappdata(fig_handle, 'instruction_text');
    time_data = getappdata(fig_handle, 'time_data');
    std_font_size = getappdata(fig_handle, 'std_font_size');
    std_font_type = getappdata(fig_handle, 'std_font_type');
    
    % Reset all lines to original state
    for i = 1:length(line_handles)
        if ishandle(line_handles(i))
            color = get(line_handles(i), 'Color');
            if length(color) >= 3
                set(line_handles(i), 'LineWidth', 1, 'Color', [color(1:3), 0.3]);
            end
        end
    end
    
    % Clear the selected hardware plot
    axes(top_right_plot);
    % Reset the plot data - use ones for log scale
    set(selected_plot, 'XData', time_data, 'YData', ones(length(time_data), 1));

    % Show the instruction text
    if ishandle(instruction_text)
        set(instruction_text, 'Visible', 'on');
    else
        % Recreate the instruction text if it doesn't exist
        text(0.5, 0.5, 'Select a hardware component from the list', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', std_font_size*0.8, ...
            'FontName', std_font_type, ...
            'Tag', 'instruction_text');
        setappdata(fig_handle, 'instruction_text', findobj(top_right_plot, 'Tag', 'instruction_text'));
    end
    title('Selected Component', 'FontSize', std_font_size, ...
        'FontName', std_font_type);
        
    % Reset camera plot status
    setappdata(fig_handle, 'is_camera_plot', false);
end 