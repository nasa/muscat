function func_plot_power_visualization(mission, i_SC)

kd = mission.storage.k_storage;

%% Ensure we have enough colors for all hardware components
color_array = mission.storage.plot_parameters.color_array;

%% Power Visualization - Main Figure
plot_handle = figure('Name',['SC ', num2str(i_SC), ' Power Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Summary Plot - Power Generated & Consumed with Energy Balance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3, 3, 1)

func_plot_power_visualization_subplot1(mission, i_SC, kd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Power Surplus & Deficit % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3, 3, 2)

func_plot_power_visualization_subplot2(mission, i_SC, kd);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Cumulative Energy Balance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3, 3, 3)

func_plot_power_visualization_subplot3(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Solar Panel Performance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_solar_panel') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel > 0

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Solar Panel Power Generation % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3, 3, 4)

    func_plot_power_visualization_subplot4(mission, i_SC, kd);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Solar Panel Efficiency % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3, 3, 5)

    func_plot_power_visualization_subplot5(mission, i_SC, kd);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Sun Incidence Angle % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3, 3, 6)

    func_plot_power_visualization_subplot6(mission, i_SC, kd)


else
    % Display message if no solar panels
    for i = 4:6
        subplot(3, 3, i)
        text(0.5, 0.5, 'No solar panel data available', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        axis off;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Battery Performance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(mission.true_SC{i_SC}, 'true_SC_battery') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery > 0

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Battery Capacity % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3, 3, 7)

    func_plot_power_visualization_subplot7(mission, i_SC, kd)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Battery State of Charge % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3, 3, 8)

    func_plot_power_visualization_subplot8(mission, i_SC, kd)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Battery Power Emergency Events % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    subplot(3, 3, 9)

    func_plot_power_visualization_subplot9(mission, i_SC, kd)



else
    % Display message if no batteries
    for i = 7:9
        subplot(3, 3, i)
        text(0.5, 0.5, 'No battery data available', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        axis off;
    end
end

% Add overall title
sgtitle(['SC ', num2str(i_SC), ' Power Performance'], ...
    'FontSize', mission.storage.plot_parameters.title_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name, '_SC', num2str(i_SC), '_Power.png'];
    saveas(plot_handle, plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*

    ch = Chapter;
    ch.Title = ['Results: SC ',num2str(i_SC),' Power Performance'];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Summary Plot - Power Generated & Consumed with Energy Balance % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Power Generated and Consumed';

    plot_handle = figure(101);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_power_visualization_subplot1(mission, i_SC, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(3,3,1) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('Total power generated and consumed by the spacecraft at every time step.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Power Generated [Watts]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(i_k);
    end

    para = Paragraph(['- "Power Generated [Watts]" represents the total power generated by the spacecraft.']);
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Power Consumed [Watts]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(i_k);
    end

    para = Paragraph(['- "Power Consumed [Watts]" represents the total power consumed by the spacecraft.']);
    append(sec,para)

    % append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Power Surplus & Deficit % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % sec = Section;
    % sec.Title = 'Power Surplus and Deficit';

    plot_handle = figure(102);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_power_visualization_subplot2(mission, i_SC, kd);

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(3,3,2) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('Extra power generated by the spacecraft at every time step, where "Power Surplus" is given by "Power Generated" - "Power Consumed".');
    append(sec,para)

    para = Paragraph('- Positive "Power Surplus" means more power is generated by the spacecraft than power consumed, at this time step.');
    append(sec,para)

    para = Paragraph('- Negative "Power Surplus" means less power is generated by the spacecraft than power consumed, at this time step.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Power Surplus [Watts]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    % Get valid power data up to the current step
    power_generated = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(1:kd);
    power_consumed = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(1:kd);

    % Calculate power deficit (negative means more consumption than generation)
    power_surplus = power_generated - power_consumed;

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = power_surplus(i_k);
    end

    para = Paragraph(['- "Power Surplus [Watts]" is given by "Power Generated" - "Power Consumed".']);
    append(sec,para)

    % append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Cumulative Energy Balance % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % sec = Section;
    % sec.Title = 'Cumulative Energy Balance';

    plot_handle = figure(103);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_power_visualization_subplot3(mission, i_SC, kd);

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(3,3,3) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('Cumulative Energy Balance represents the total energy accumulated over the entire simulation duration. It is calculated as the integral of the "Power Surplus" over time.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Cumulative Energy Balance [Whr]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    % Calculate cumulative energy balance (in Watt-hours)
    time_step = mission.true_time.time_step;
    energy_balance = cumsum(power_surplus * (time_step / 3600)); % Watt-sec to Watt-hour

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = energy_balance(i_k);
    end

    para = Paragraph(['- "Cumulative Energy Balance [Whr]" is given by Sum_{all time steps} ("Power Surplus" * time step).']);
    append(sec,para)

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Solar Panel Performance % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if isfield(mission.true_SC{i_SC}, 'true_SC_solar_panel') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel > 0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Solar Panel Power Generation % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        sec = Section;
        sec.Title = 'Solar Panel Power Generation';

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel == 1
            para = Paragraph(['There is ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel),' solar panel:']);
            append(sec,para)
        else
            para = Paragraph(['There are ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel),' solar panels:']);
            append(sec,para)
        end

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
            para = Paragraph(['- SP ',num2str(i_HW),' has total area ',num2str(mission.true_SC{1}.true_SC_solar_panel{i_HW}.shape_model.Face_total_area),'meter^2.' ]);
            append(sec,para)
        end

        plot_handle = figure(104);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_power_visualization_subplot4(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(3,3,4) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Power generated by each solar panel at every time step.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SP',num2str(i_HW),' Power Generated [Watts]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:kd
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.instantaneous_power_generated(i_k);
            end

            para = Paragraph(['- "SP ',num2str(i_HW),' Power Generated [Watts]" represents power generated by SP ',num2str(i_HW),' at every time step.']);
            append(sec,para)

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Solar Panel Efficiency % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        plot_handle = figure(105);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_power_visualization_subplot5(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(3,3,5) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Efficiency is defined as the ratio of the actual power generated by all the solar panels by the maxiumum power that could have been generated if the Sun was orthogonal to the solar panels.');
        append(sec,para)

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Sun Incidence Angle % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        plot_handle = figure(106);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_power_visualization_subplot6(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(3,3,6) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Sun Incidence Angle [deg] is the angle between the actual Sun rays and the normal vector of the solar panels.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SP',num2str(i_HW),' Sun Incidence Angle [deg]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:kd
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.Sun_incidence_angle(i_k);
            end

            para = Paragraph(['- "SP',num2str(i_HW),' Sun Incidence Angle [deg]" represents Sun Incidence Angle on SP ',num2str(i_HW),' at every time step.']);
            append(sec,para)

        end

        append(ch,sec)

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Battery Performance % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if isfield(mission.true_SC{i_SC}, 'true_SC_battery') && mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery > 0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Battery Capacity % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        sec = Section;
        sec.Title = 'Battery Performance';

        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery == 1
            para = Paragraph(['There is ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery),' battery:']);
            append(sec,para)
        else
            para = Paragraph(['There are ',num2str(mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery),' batteries:']);
            append(sec,para)
        end

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery
            para = Paragraph(['- Battery ',num2str(i_HW),' has maximum capacity of ',num2str(mission.true_SC{i_SC}.true_SC_battery{i_HW}.maximum_capacity),' Whr.' ]);
            append(sec,para)
        end

        plot_handle = figure(107);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_power_visualization_subplot7(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(3,3,7) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Battery capacity at every time step.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Batt',num2str(i_HW),' Capacity [Whr]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:kd
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_battery{i_HW}.store.instantaneous_capacity(i_k);
            end

            para = Paragraph(['- "Batt',num2str(i_HW),' Capacity [Whr]" represents capacity of Battery ',num2str(i_HW),' at every time step.']);
            append(sec,para)

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Battery State of Charge % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        plot_handle = figure(108);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_power_visualization_subplot8(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(3,3,8) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Battery state of charge (SoC) at every time step.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Batt',num2str(i_HW),' SoC [%]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:kd
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_battery{i_HW}.store.state_of_charge(i_k);
            end

            para = Paragraph(['- "Batt',num2str(i_HW),' SoC [%]" represents Battery ',num2str(i_HW),' SoC at every time step.']);
            append(sec,para)

        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % Battery Power Emergency Events % %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        plot_handle = figure(109);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_power_visualization_subplot9(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(3,3,9) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('Battery charging and discharging cycles are shown by tracking the energy flow into and out of the battery/batteries at every time step.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Energy Flow [Whr]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_energy(i_k);
        end

        para = Paragraph(['- "Energy Flow [Whr]" represents battery charging (positive Energy Flow) and battery discharging (negative Energy Flow).']);
        append(sec,para)

        
        append(ch,sec)
    end


    append(mission.storage.report_parameters.report_handle,ch);

end

end

function func_plot_power_visualization_subplot1(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Power Generated & Consumed with Energy Balance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Get valid power data up to the current step
power_generated = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(1:kd);
power_consumed = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(1:kd);

% Calculate power deficit (negative means more consumption than generation)
power_surplus = power_generated - power_consumed;

% Calculate cumulative energy balance (in Watt-hours)
time_step = mission.true_time.time_step;
energy_balance = cumsum(power_surplus * (time_step / 3600)); % Watt-sec to Watt-hour

% Calculate total energy metrics
total_energy_generated = sum(power_generated) * (time_step / 3600);
total_energy_consumed = sum(power_consumed) * (time_step / 3600);
final_energy_balance = energy_balance(end);

% Plot power generation and consumption

hold on
area(mission.true_time.store.time(1:kd), power_generated, 'FaceColor', [0.4, 0.8, 0.4], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
area(mission.true_time.store.time(1:kd), power_consumed, 'FaceColor', [0.8, 0.4, 0.4], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
plot(mission.true_time.store.time(1:kd), power_generated, '-', 'Color', [0, 0.5, 0], 'LineWidth', 2);
plot(mission.true_time.store.time(1:kd), power_consumed, '-', 'Color', [0.7, 0, 0], 'LineWidth', 2);

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



end


function func_plot_power_visualization_subplot2(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Power Surplus & Deficit % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Get valid power data up to the current step
power_generated = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(1:kd);
power_consumed = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(1:kd);

% Calculate power deficit (negative means more consumption than generation)
power_surplus = power_generated - power_consumed;


% Use area plot to highlight deficit periods
area(mission.true_time.store.time(1:kd), power_surplus, 'FaceColor', [0.5, 0.8, 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
area(mission.true_time.store.time(1:kd), min(0, power_surplus), 'FaceColor', [0.8, 0.5, 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
plot(mission.true_time.store.time(1:kd), power_surplus, '-', 'Color', [0, 0, 0.8], 'LineWidth', 2);
plot([mission.true_time.store.time(1) mission.true_time.store.time(kd)], [0, 0], '--k', 'LineWidth', 1);

grid on
xlabel('Time [sec]')
ylabel('Power Surplus [Watts]')
title('Power Surplus or Deficit', 'FontSize', mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type)

% Calculate percentage of time with power deficit
deficit_percentage = 100 * sum(power_surplus < 0) / length(power_surplus);
deficit_time_str = sprintf('Time in Deficit: %.1f%%', deficit_percentage);
annotation('textbox', [0.39, 0.82, 0.15, 0.03], 'String', deficit_time_str, ...
    'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);



end

function func_plot_power_visualization_subplot3(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Cumulative Energy Balance % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Get valid power data up to the current step
power_generated = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(1:kd);
power_consumed = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(1:kd);

% Calculate power deficit (negative means more consumption than generation)
power_surplus = power_generated - power_consumed;

% Calculate cumulative energy balance (in Watt-hours)
time_step = mission.true_time.time_step;
energy_balance = cumsum(power_surplus * (time_step / 3600)); % Watt-sec to Watt-hour

% Calculate total energy metrics
total_energy_generated = sum(power_generated) * (time_step / 3600);
total_energy_consumed = sum(power_consumed) * (time_step / 3600);
final_energy_balance = energy_balance(end);


% Use area plot with different colors for positive/negative
area(mission.true_time.store.time(1:kd), max(0, energy_balance), 'FaceColor', [0.5, 0.8, 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
area(mission.true_time.store.time(1:kd), min(0, energy_balance), 'FaceColor', [0.8, 0.5, 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
plot(mission.true_time.store.time(1:kd), energy_balance, '-', 'Color', [0, 0, 0.8], 'LineWidth', 2);
plot([mission.true_time.store.time(1) mission.true_time.store.time(kd)], [0, 0], '--k', 'LineWidth', 1);

grid on
xlabel('Time [sec]')
ylabel('Cumulative Energy Balance [Whr]')
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


end

function func_plot_power_visualization_subplot4(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Solar Panel Power Generation % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

sp_legend = {};
for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
    % Get power data and ensure it's valid
    panel_power = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.instantaneous_power_generated(1:kd);

    % Safe color indexing
    color_index = mod(i_HW, length(mission.storage.plot_parameters.color_array)) + 1;

    % Plot solar panel power
    plot(mission.true_time.store.time(1:kd), panel_power, '-', 'LineWidth', 1.5, 'Color', mission.storage.plot_parameters.color_array(color_index));
    sp_legend{end+1} = ['SP ', num2str(i_HW)];
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


end

function func_plot_power_visualization_subplot5(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Solar Panel Efficiency % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Calculate total power generated by all solar panels
total_power = zeros(kd, 1);
total_max = zeros(kd, 1);

for i_HW = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel
    % Get power data and ensure it's valid
    panel_power = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.instantaneous_power_generated(1:kd);
    panel_max = mission.true_SC{i_SC}.true_SC_solar_panel{i_HW}.store.maximum_power(1:kd);

    total_power = total_power + panel_power;
    total_max = total_max + panel_max;
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

% Plot efficiency as an area to make it more visible
area(mission.true_time.store.time(1:kd), efficiency, 'FaceColor', [0.4, 0.6, 0.9], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
plot(mission.true_time.store.time(1:kd), efficiency, '-b', 'LineWidth', 2);

% Also plot the mean efficiency as a horizontal line
mean_eff = mean(efficiency(efficiency > 0)); % Ignore zero values
if ~isnan(mean_eff) && mean_eff > 0
    plot([mission.true_time.store.time(1) mission.true_time.store.time(kd)], [mean_eff, mean_eff], '--k', 'LineWidth', 1.5);
    text(mission.true_time.store.time(kd) * 0.95, mean_eff * 1.05, sprintf('Mean: %.1f%%', mean_eff), ...
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

end

function func_plot_power_visualization_subplot6(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Sun Incidence Angle % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

sp_legend = {};
% For each solar panel, plot sun angle vs power as scatter plot
for i_HW = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_solar_panel

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
        color_index = mod(i_HW, length(mission.storage.plot_parameters.color_array)) + 1;

        scatter(sun_angle(scatter_indices), panel_power(scatter_indices), ...
            10, mission.storage.plot_parameters.color_array(color_index), 'filled', 'MarkerFaceAlpha', 0.5);

        sp_legend{end+1} = ['SP ', num2str(i_HW)];
    end

end

grid on
if ~isempty(sp_legend)
    legend(sp_legend, 'Location', 'NorthEast')
end
xlabel('Sun Incidence Angle [deg]')
ylabel('Power Generated [Watts]')
title('Sun Angle vs Power Generated', 'FontSize', mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)

% % Add a theoretical cosine curve for reference (if we have solar panel data)
% max_power_overall = mission.true_SC{i_SC}.true_SC_solar_panel{1}.max_power;
% angles = 0:90;  % 0 to 90 degrees
% theoretical_power = max_power_overall * cosd(angles);
% plot(angles, theoretical_power, '--k', 'LineWidth', 1);
% text(45, max_power_overall * 0.7, 'Theoretical (cos θ)', ...
%     'FontSize', mission.storage.plot_parameters.standard_font_size * 0.7, ...
%     'HorizontalAlignment', 'center');

end


function func_plot_power_visualization_subplot7(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Battery Capacity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

total_capacity = 0;
current_capacity = 0;

for i_batt = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

    batt = mission.true_SC{i_SC}.true_SC_battery{i_batt};
    if batt.health == 1
        total_capacity = total_capacity + batt.maximum_capacity;
        current_capacity = current_capacity + batt.instantaneous_capacity;

        batt_capacity = batt.store.instantaneous_capacity(1:kd);

        % Safe color indexing
        color_index = mod(i_batt, length(mission.storage.plot_parameters.color_array)) + 1;

        % Plot battery capacity
        plot(mission.true_time.store.time(1:kd), batt_capacity, '-', 'LineWidth', 1.5, 'Color', mission.storage.plot_parameters.color_array(color_index));
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

end

function func_plot_power_visualization_subplot8(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Battery State of Charge % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

for i_batt = 1:1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_battery

    batt_soc = mission.true_SC{i_SC}.true_SC_battery{i_batt}.store.state_of_charge(1:kd);

    % Safe color indexing
    color_index = mod(i_batt, length(mission.storage.plot_parameters.color_array)) + 1;

    % Plot battery SoC
    plot(mission.true_time.store.time(1:kd), batt_soc, '-', 'LineWidth', 1.5, 'Color', mission.storage.plot_parameters.color_array(color_index));
    batt_legend{i_batt} = ['Battery ', num2str(i_batt)];

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



end

function func_plot_power_visualization_subplot9(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Battery Power Emergency Events % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

% Get energy data
energy_data = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_energy(1:kd);

% Create area plot for energy flow
area(mission.true_time.store.time(1:kd), max(0, energy_data), 'FaceColor', [0.4, 0.8, 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
area(mission.true_time.store.time(1:kd), min(0, energy_data), 'FaceColor', [0.8, 0.4, 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(mission.true_time.store.time(1:kd), energy_data, '-', 'Color', [0, 0, 0.8], 'LineWidth', 1.5);

% Add zero line
plot([mission.true_time.store.time(1) mission.true_time.store.time(kd)], [0, 0], '--k', 'LineWidth', 1);

% Calculate statistics
discharge_time = sum(energy_data < 0) * (mission.true_time.time_step / 3600); % hours
charge_time = sum(energy_data > 0) * (mission.true_time.time_step / 3600); % hours

% Add annotation with statistics
stats_str = sprintf('Charging: %.1f hrs\nDischarging: %.1f hrs', charge_time, discharge_time);
annotation('textbox', [0.89, 0.15, 0.15, 0.05], 'String', stats_str, ...
    'FitBoxToText', true, 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);


grid on
xlabel('Time [sec]')
ylabel('Energy Flow [Whr]')
title('Battery Charging and Discharging', 'FontSize', mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, ...
    'FontName', mission.storage.plot_parameters.standard_font_type)


end