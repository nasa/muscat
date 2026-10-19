function func_plot_telecom(mission, i_SC)

kd = mission.storage.k_storage;

plot_handle = figure('Name',['SC ',num2str(i_SC),' Telecom Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

% Check if we should plot in days or seconds
plot_in_days = isfield(mission.true_time, 'data') && isfield(mission.true_time.data, 'time_fraction');

% Determine the number of subplots (4 if we have antenna data, 3 if not)
if isfield(mission.true_SC{i_SC}, 'true_SC_radio_antenna') && ~isempty(mission.true_SC{i_SC}.true_SC_radio_antenna)
    flag_has_antenna = true;
    num_subplots = 4;
else
    flag_has_antenna = false;
    num_subplots = 3;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Link Status % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(num_subplots,1,1);

func_plot_telecom_subplot_link_status(mission, i_SC, kd, plot_in_days);

% Plot Antenna Mode if available
current_subplot = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Antenna Mode % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_has_antenna
    current_subplot = current_subplot + 1;
    subplot(num_subplots,1,current_subplot);

    func_plot_telecom_subplot_antenna_mode(mission, i_SC, kd, plot_in_days);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Data Rate % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

current_subplot = current_subplot + 1;
subplot(num_subplots,1,current_subplot);

func_plot_telecom_subplot_data_rate(mission, i_SC, kd, plot_in_days);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Visibility % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

current_subplot = current_subplot + 1;
subplot(num_subplots,1,current_subplot);

func_plot_telecom_subplot_visibility(mission, i_SC, kd, plot_in_days);

sgtitle(['SC ',num2str(i_SC),' Telecom Performance'],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Telecom.png'];
    saveas(plot_handle,plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*

    ch = Chapter;
    ch.Title = ['Results: SC ',num2str(i_SC),' Telecom Performance'];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Link Status % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Communication Link Status';

    plot_handle = figure(101);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_telecom_subplot_link_status(mission, i_SC, kd, plot_in_days);

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(',num2str(num_subplots),',1,1) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('Whether each communication link (from SC''s True_SC_Communication_Link class) is commanded On or Off by the executive, at every time step, is shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)

        commlink = mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink};
        link_label = [char(commlink.name),': ',char(commlink.TX_name),' to ',char(commlink.RX_name)];

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' ',link_label,' Link Status [Bool]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = commlink.store.flag_executive(i_k);
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' ',link_label,' Link Status [Bool]" represents whether this link is commanded On (1) or Off (0) at every time step.']);
        append(sec,para)

    end

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Antenna Mode % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if flag_has_antenna

        sec = Section;
        sec.Title = 'Radio Antenna Mode';

        plot_handle = figure(102);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_telecom_subplot_antenna_mode(mission, i_SC, kd, plot_in_days);

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(',num2str(num_subplots),',1,2) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('The mode of each SC radio antenna (from SC''s True_SC_Radio_Antenna class) at every time step is shown: Off, Tx (transmit), or Rx (receive).');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        for idx_radio_antenna = 1:1:length(mission.true_SC{i_SC}.true_SC_radio_antenna)

            antenna = mission.true_SC{i_SC}.true_SC_radio_antenna{idx_radio_antenna};

            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
            mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' ',char(antenna.name),' Mode [0=Off,1=Tx,2=Rx]'];
            this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

            for i_k = 1:1:kd
                mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = antenna.store.mode_TX_RX(i_k);
            end

            para = Paragraph(['- "SC ',num2str(i_SC),' ',char(antenna.name),' Mode [0=Off,1=Tx,2=Rx]" represents this antenna''s mode at every time step.']);
            append(sec,para)

        end

        append(ch,sec)

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Data Rate % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Communication Link Data Rate';

    plot_handle = figure(103);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_telecom_subplot_data_rate(mission, i_SC, kd, plot_in_days);

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(',num2str(num_subplots),',1,',num2str(num_subplots-1),') inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('The instantaneous data rate (from SC''s True_SC_Communication_Link class) of each communication link, at every time step, is shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)

        commlink = mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink};
        link_label = [char(commlink.name),': ',char(commlink.TX_name),' to ',char(commlink.RX_name)];

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' ',link_label,' Data Rate [kbps]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = commlink.store.this_data_rate(i_k);
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' ',link_label,' Data Rate [kbps]" represents the instantaneous data rate of this link at every time step.']);
        append(sec,para)

    end

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Visibility % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Communication Link Visibility';

    plot_handle = figure(104);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_telecom_subplot_visibility(mission, i_SC, kd, plot_in_days);

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(',num2str(num_subplots),',1,',num2str(num_subplots),') inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('Whether each communication link (from SC''s True_SC_Communication_Link class) has line-of-sight visibility between its TX and RX ends, at every time step, is shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)

        commlink = mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink};
        link_label = [char(commlink.name),': ',char(commlink.TX_name),' to ',char(commlink.RX_name)];

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' ',link_label,' Visibility [Bool]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = commlink.store.flag_TX_RX_visible(i_k);
        end

        para = Paragraph(['- "SC ',num2str(i_SC),' ',link_label,' Visibility [Bool]" represents whether this link''s TX and RX ends have line-of-sight visibility (1) or not (0) at every time step.']);
        append(sec,para)

    end

    append(ch,sec)

    append(mission.storage.report_parameters.report_handle,ch);

end

end

function func_plot_telecom_subplot_link_status(mission, i_SC, kd, plot_in_days)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Link Status % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on
grid on
xlabel(func_plot_telecom_conditional_label(plot_in_days))
ylabel('Link Status')
if plot_in_days
    xlim([0 mission.true_time.time])
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
end
ylim([0 1.15])
yticks([0,1])
yticklabels({'Off', 'On'});
set(gca,'TickLabelInterpreter', 'tex')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.store.flag_executive(1:kd),'-','LineWidth',2,'DisplayName',strcat(mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.name,": ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.TX_name," to ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.RX_name))
end
legend()
hold off

end

function func_plot_telecom_subplot_antenna_mode(mission, i_SC, kd, plot_in_days)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Antenna Mode % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on
grid on
ylabel('Antenna Mode')
xlabel(func_plot_telecom_conditional_label(plot_in_days))
if plot_in_days
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
    xlim([0 mission.true_time.time])
end
ylim([0 2.15])
yticks([0,1,2])
yticklabels({'Off', 'Tx', 'Rx'});
set(gca,'TickLabelInterpreter', 'tex')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

for idx_radio_antenna = 1:1:length(mission.true_SC{i_SC}.true_SC_radio_antenna)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_radio_antenna{idx_radio_antenna}.store.mode_TX_RX(1:kd),'-','LineWidth',2,'DisplayName',mission.true_SC{i_SC}.true_SC_radio_antenna{idx_radio_antenna}.name)
end
legend()
hold off

end

function func_plot_telecom_subplot_data_rate(mission, i_SC, kd, plot_in_days)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Data Rate % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on
grid on
if plot_in_days
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
    xlim([0 mission.true_time.time])
end
ylabel('Datarate [kbps]')
xlabel(func_plot_telecom_conditional_label(plot_in_days))
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.store.this_data_rate(1:kd),'-','LineWidth',2,'DisplayName',strcat(mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.name,": ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.TX_name," to ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.RX_name))
end
legend()
hold off

end

function func_plot_telecom_subplot_visibility(mission, i_SC, kd, plot_in_days)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Visibility % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on
grid on
xlabel(func_plot_telecom_conditional_label(plot_in_days))
ylabel('Visibility')
if plot_in_days
    xticks(mission.true_time.data.time_fraction)
    xticklabels(mission.true_time.data.days)
    xlim([0 mission.true_time.time])
end
yticks([0,1])
ylim([0 1.1])
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

for idx_commlink = 1:1:length(mission.true_SC{i_SC}.true_SC_communication_link)
    plot(mission.true_time.store.time(1:kd),mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.store.flag_TX_RX_visible(1:kd),'-','LineWidth',2,'DisplayName',strcat(mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.name,": ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.TX_name," to ",mission.true_SC{i_SC}.true_SC_communication_link{idx_commlink}.RX_name))
end
legend()
hold off

end

function label = func_plot_telecom_conditional_label(plot_in_days)
if plot_in_days
    label = 'Time [days]';
else
    label = 'Time [sec]';
end
end
