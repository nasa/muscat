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

subplot(2,2,1)

func_plot_data_handling_visualization_subplot1(mission, i_SC, kd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Onboard Memory % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,2)

func_plot_data_handling_visualization_subplot2(mission, i_SC, kd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Memory Fill Level % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,3)

func_plot_data_handling_visualization_subplot3(mission, i_SC, kd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Transmission Analysis % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,2,4)

func_plot_data_handling_visualization_subplot4(mission, i_SC, kd);

% Add overall title
sgtitle(['SC ', num2str(i_SC), ' Data Handling Performance'], ...
    'FontSize', mission.storage.plot_parameters.title_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name, '_SC', num2str(i_SC), '_Data_Handling.png'];
    saveas(plot_handle, plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*

    ch = Chapter;
    ch.Title = ['Results: SC ',num2str(i_SC),' Data Handling Performance'];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Data Generated Removed % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if isfield(mission.true_SC{i_SC}, 'true_SC_data_handling')

        sec = Section;
        sec.Title = 'Data Generated vs Removed';

        plot_handle = figure(101);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_data_handling_visualization_subplot1(mission, i_SC, kd);

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(2,2,1) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Instantaneous data generated and removed by SC''s data handling system (from SC''s True_SC_Data_Handling class), at every time step, are shown.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        if isfield(mission.true_SC{i_SC}.true_SC_data_handling.store, 'instantaneous_data_generated')

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Data Generated [kb]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            data_generated = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_generated;
            valid_points = min(kd, length(data_generated));

            for i_k = 1:1:valid_points
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = data_generated(i_k);
            end

            para = Paragraph(['- "SC ',num2str(i_SC),' Data Generated [kb]" represents instantaneous data generated by the spacecraft at every time step.']);
            append(sec,para)

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Data Removed [kb]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            data_removed = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_removed;
            valid_points = min(kd, length(data_removed));

            for i_k = 1:1:valid_points
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = data_removed(i_k);
            end

            para = Paragraph(['- "SC ',num2str(i_SC),' Data Removed [kb]" represents instantaneous data removed (e.g., downlinked) from the spacecraft at every time step.']);
            append(sec,para)

        end

        append(ch,sec)

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Onboard Memory % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if isfield(mission.true_SC{i_SC}, 'true_SC_onboard_memory') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0

        sec = Section;
        sec.Title = 'Onboard Memory Status';

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory == 1
            para = Paragraph(['There is ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory),' onboard memory device:']);
            append(sec,para)
        else
            para = Paragraph(['There are ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory),' onboard memory devices:']);
            append(sec,para)
        end

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory
            para = Paragraph(['- ',mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW}.name,' has maximum capacity of ',num2str(mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW}.maximum_capacity),' kb.']);
            append(sec,para)
        end

        plot_handle = figure(102);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_data_handling_visualization_subplot2(mission, i_SC, kd);

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

        para = Paragraph('Instantaneous stored data (solid line) and maximum capacity (dashed line) of each onboard memory device (from SC''s True_SC_Onboard_Memory class) are shown.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory

            memory = mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW};
            valid_points = min(kd, length(memory.store.instantaneous_capacity));

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = [memory.name,' Capacity [kb]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:valid_points
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = memory.store.instantaneous_capacity(i_k);
            end

            para = Paragraph(['- "',memory.name,' Capacity [kb]" represents instantaneous stored data in ',memory.name,' at every time step.']);
            append(sec,para)

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Memory Fill Level % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        plot_handle = figure(103);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_data_handling_visualization_subplot3(mission, i_SC, kd);

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(2,2,3) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Percentage fill level (state of data storage, SoDS) of each onboard memory device is shown, with Warning (80%) and Critical (90%) threshold lines marked.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory

            memory = mission.true_SC{i_SC}.true_SC_onboard_memory{i_HW};
            valid_points = min(kd, length(memory.store.state_of_data_storage));

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = [memory.name,' SoDS [%]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:valid_points
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = memory.store.state_of_data_storage(i_k);
            end

            para = Paragraph(['- "',memory.name,' SoDS [%]" represents percentage fill level of ',memory.name,' at every time step.']);
            append(sec,para)

        end

        append(ch,sec)

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Transmission Analysis % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if isfield(mission.true_SC{i_SC}, 'true_SC_data_handling') && ...
            isfield(mission.true_SC{i_SC}.true_SC_data_handling.store, 'instantaneous_data_generated')

        sec = Section;
        sec.Title = 'Transmission Cycle Analysis';

        plot_handle = figure(104);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_data_handling_visualization_subplot4(mission, i_SC, kd);

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

        para = Paragraph('Cumulative data generated, cumulative data removed, and the net accumulated data (their difference) over the entire simulation duration are shown.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        data_generated = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_generated;
        data_removed = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_removed;
        valid_points = min([kd, length(data_generated), length(data_removed)]);

        cum_generated = cumsum(data_generated(1:valid_points));
        cum_removed = cumsum(data_removed(1:valid_points));
        data_accumulated = cum_generated - cum_removed;

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Cumulative Data Generated [kb]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:valid_points
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = cum_generated(i_k);
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' Cumulative Data Generated [kb]" represents cumulative data generated by the spacecraft over the simulation.']);
        append(sec,para)

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Cumulative Data Removed [kb]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:valid_points
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = cum_removed(i_k);
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' Cumulative Data Removed [kb]" represents cumulative data removed (e.g., downlinked) from the spacecraft over the simulation.']);
        append(sec,para)

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Net Accumulated Data [kb]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:valid_points
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = data_accumulated(i_k);
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' Net Accumulated Data [kb]" is given by "Cumulative Data Generated" - "Cumulative Data Removed".']);
        append(sec,para)

        append(ch,sec)

    end

    append(mission.storage.report_parameters.report_handle,ch);

end

end

function func_plot_data_handling_visualization_subplot1(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Data Generated Removed % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_data_handling')

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

    else
        text(0.5, 0.5, 'No data generation/removal information available', ...
            'HorizontalAlignment', 'center', 'FontSize', 12);
    end

else
    text(0.5, 0.5, 'No data handling information available', ...
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

function func_plot_data_handling_visualization_subplot2(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Onboard Memory % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_onboard_memory') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0

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

    end

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

end

function func_plot_data_handling_visualization_subplot3(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Memory Fill Level % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_onboard_memory') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_onboard_memory > 0

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

function func_plot_data_handling_visualization_subplot4(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Transmission Analysis % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
