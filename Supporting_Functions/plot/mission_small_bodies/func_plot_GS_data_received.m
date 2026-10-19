%% Plot GS Data Received

function func_plot_GS_data_received(mission)

% Define a colormap (jet, parula, turbo)
cmap = turbo(mission.true_ground_station.num_GS_radio_antenna);
kd = mission.storage.k_storage;

plot_handle = figure('Name','GS Telecom Performance');
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Instantaneous Data Rate Received [kbps] % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3,1,1)

func_plot_GS_data_received_subplot1(mission, kd, cmap)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Cumulative Data Received [kb] % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3,1,2)

func_plot_GS_data_received_subplot2(mission, kd, cmap)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Data Received per Telecom Pass [kb] % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3,1,3)

GS_pass_data = func_plot_GS_data_received_subplot3(mission, kd, cmap);

sgtitle('GS Telecom Performance (Received)','FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name,'_GS_Telecom_Received.png'];
    saveas(plot_handle,plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*

    ch = Chapter;
    ch.Title = 'Results: Ground Station Telecom Performance';

    sec = Section;
    sec.Title = 'Ground Station (GS) Locations';

    para = Paragraph(['The names and locations of ',num2str(mission.true_ground_station.num_GS_radio_antenna),' GSs are:']);
    append(sec,para)

    for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna
        para = Paragraph([num2str(i_GS),'. ',mission.true_GS_radio_antenna{i_GS}.name,' is located at (Latitude: ',num2str(mission.true_GS_radio_antenna{i_GS}.latitude),'N, Longitude: ',num2str(mission.true_GS_radio_antenna{i_GS}.longitude),'E)' ]);
        append(sec,para)
    end

    append(ch,sec)


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Instantaneous Data Rate Received [kbps] % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Instantaneous Data Rate Received';

    plot_handle = figure(101);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_GS_data_received_subplot1(mission, kd, cmap)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(3,1,1) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('The instantaneous data rate recieved at each GS from all SC are shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    % Add to Excel
    for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['GS ',num2str(i_GS),' Inst. Data Rate Rec. [kbps]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(i_k); 
        end

        para = Paragraph(['- "GS ',num2str(i_GS),' Inst. Data Rate Rec. [kbps]" represents instantaneous data rate received by GS ',num2str(i_GS),' (',mission.true_GS_radio_antenna{i_GS}.name,').']);
        append(sec,para)
    end

    append(ch,sec)


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Cumulative Data Received [kb] % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Cumulative Data Received';

    plot_handle = figure(102);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_GS_data_received_subplot2(mission, kd, cmap)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(3,1,2) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('The cumulative data recieved at each GS from all SC are shown.');
    append(sec,para)

    para = Paragraph(['The cumulative data recieved at each GS at the end of the simulation are:']);
    append(sec,para)

    total_cum_data_received = 0;

    for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna

        cum_data_received = cumsum(mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(1:kd))*mission.true_time.time_step; % [kb]
        total_cum_data_received = total_cum_data_received + cum_data_received(end); % [kb]

        para = Paragraph([num2str(i_GS),'. ',mission.true_GS_radio_antenna{i_GS}.name,' recieved ',num2str(cum_data_received(end)),' kb of data.' ]);
        append(sec,para)
    end

    para = Paragraph(['All GSs recieved total ',num2str(total_cum_data_received),' kb of data.' ]);
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    % Add to Excel
    for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['GS ',num2str(i_GS),' Cum. Data Rec. [kb]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        cum_data_received = cumsum(mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(1:kd))*mission.true_time.time_step; % [kb]
        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = cum_data_received(i_k); % [kb];
        end

        para = Paragraph(['- "GS ',num2str(i_GS),' Cum. Data Rec. [kb]" represents cumulative data received by GS ',num2str(i_GS),' (',mission.true_GS_radio_antenna{i_GS}.name,').']);
        append(sec,para)
    end

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = 'All GS Cum. Data Rec. [kb]';
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_ground_station.store.total_data_received(i_k); % [kb];
    end

    para = Paragraph('- "All GS Cum. Data Rec. [kb]" represents cumulative data received by all GSs.');
    append(sec,para)

    append(ch,sec)


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Data Received per Telecom Pass [kb] % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'Data Received per Telecom Pass';

    plot_handle = figure(103);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    GS_pass_data = func_plot_GS_data_received_subplot3(mission, kd, cmap);

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(3,1,3) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('The data recieved per telecom pass at each GS from all SC are shown.');
    append(sec,para)

    for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna

        % disp(['GS ',num2str(i_GS),' (',mission.true_GS_radio_antenna{i_GS}.name,') has ',num2str(GS_pass_data{i_GS}.num_pass),' passes.'])
        para = Paragraph(['GS ',num2str(i_GS),' (',mission.true_GS_radio_antenna{i_GS}.name,') has ',num2str(GS_pass_data{i_GS}.num_pass),' passes.']);
        para.Bold = true;
        append(sec,para)

        if GS_pass_data{i_GS}.num_pass > 0

            % disp('They are:')
            para = Paragraph('They are:');
            append(sec,para)

            % disp('Pass   Start Time [UTC]              Stop Time [UTC]               Duration [sec]      Data Recieved [kb]')                
            para = Paragraph('Pass | Start Time [UTC] |            Stop Time [UTC] |             Duration [sec] |    Data Recieved [kb]');
            append(sec,para)

            for i_pass = 1:1:GS_pass_data{i_GS}.num_pass

                % disp([num2str(i_pass),'.     ',sec2cal(mission.true_time.store.date(GS_pass_data{i_GS}.kd_start_array(i_pass))),'     ',sec2cal(mission.true_time.store.date(GS_pass_data{i_GS}.kd_end_array(i_pass))), ...
                %     '     ',num2str(GS_pass_data{i_GS}.duration_array(i_pass)),'                 ',num2str(GS_pass_data{i_GS}.data_recieved_array(i_pass))])
                para = Paragraph([num2str(i_pass),'. |   ',sec2cal(mission.true_time.store.date(GS_pass_data{i_GS}.kd_start_array(i_pass))),' |   ',sec2cal(mission.true_time.store.date(GS_pass_data{i_GS}.kd_end_array(i_pass))), ...
                    ' |   ',num2str(GS_pass_data{i_GS}.duration_array(i_pass)),' |               ',num2str(GS_pass_data{i_GS}.data_recieved_array(i_pass))]);
                append(sec,para)
        
            end

        end

    end

    append(ch,sec)

    append(mission.storage.report_parameters.report_handle,ch);
end

end


function func_plot_GS_data_received_subplot1(mission, kd, cmap)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Instantaneous Data Rate Received [kbps] % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna
    plot(mission.true_time.store.time(1:kd), mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(1:kd),'-','LineWidth',3, 'Color',cmap(i_GS,:), 'DisplayName',mission.true_GS_radio_antenna{i_GS}.name)
end
legend()
xlim([mission.true_time.store.time(1) mission.true_time.store.time(kd)])
xlabel('Time [sec]')
ylabel('Instantaneous Data Rate Received [kbps]')

grid on
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

end

function func_plot_GS_data_received_subplot2(mission, kd, cmap)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Cumulative Data Received [kb] % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna

    cum_data_received = cumsum(mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(1:kd))*mission.true_time.time_step; % [kb]

    plot(mission.true_time.store.time(1:kd), cum_data_received(1:kd),'-','LineWidth',3,'Color',cmap(i_GS,:), 'DisplayName',mission.true_GS_radio_antenna{i_GS}.name)
end

plot(mission.true_time.store.time(1:kd), mission.true_ground_station.store.total_data_received(1:kd),'--','LineWidth',3,'Color','k', 'DisplayName','All GS')

legend('Location','northwest')
xlim([mission.true_time.store.time(1) mission.true_time.store.time(kd)])
xlabel('Time [sec]')
ylabel('Cumulative Data Received [kb]')

grid on
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

end

function GS_pass_data = func_plot_GS_data_received_subplot3(mission, kd, cmap)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Data Received per Telecom Pass [kb] % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

GS_pass_data = [];

for i_GS = 1:1:mission.true_ground_station.num_GS_radio_antenna

    cum_data_received = cumsum(mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(1:kd))*mission.true_time.time_step; % [kb]

    GS_pass_data{i_GS} = [];  
    this_data = [];
    this_data.num_pass = 0;
    this_data.kd_start_array = [];
    this_data.kd_end_array = [];
    this_data.duration_array = [];
    this_data.data_recieved_array = [];

    if cum_data_received(end) > 0

        idx = logical(mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(1:kd) > 0);

        idx_recieved = find(idx(1:end-1) - idx(2:end));

        if mod(length(idx_recieved),2) ~= 0
            warning(['GS ',num2str(i_GS),' passes are incorrect!'])
        end

        for i_pass = 1:1:length(idx_recieved)/2

            this_pass_start = 2*(i_pass-1) + 1;
            this_pass_end = 2*(i_pass-1) + 2;

            kd_start = idx_recieved(this_pass_start)+1; % +1 is due to the difference operation in idx_recieved line
            kd_end = idx_recieved(this_pass_end);

            this_data.num_pass = this_data.num_pass + 1;
            this_data.kd_start_array = [this_data.kd_start_array; kd_start];
            this_data.kd_end_array = [this_data.kd_end_array; kd_end];

            this_duration = mission.true_time.store.time(kd_end) - mission.true_time.store.time(kd_start); % [sec]
            if this_duration == 0
                this_duration = mission.true_time.time_step; % [sec]
            end
            this_data.duration_array = [this_data.duration_array; this_duration];

            if ~all(idx(kd_start:kd_end) == 1, 'all')
                error('All idx are not 1!')
            end

            this_data_recieved = sum(mission.true_GS_radio_antenna{i_GS}.store.instantaneous_data_rate_received(kd_start:kd_end))*mission.true_time.time_step; % [kb]
            this_data.data_recieved_array = [this_data.data_recieved_array; this_data_recieved];

            if kd_start == kd_end
                kd_start = kd_start - 1;
            end

            h2 = fill([mission.true_time.store.time(kd_start) mission.true_time.store.time(kd_start) mission.true_time.store.time(kd_end) mission.true_time.store.time(kd_end)],[0 this_data_recieved this_data_recieved 0],cmap(i_GS,:),'EdgeColor','none');
            h2.Annotation.LegendInformation.IconDisplayStyle = 'off';

        end

        fill([-10 -10 -5 -5],[0 1 1 0],cmap(i_GS,:),'EdgeColor','none','DisplayName',mission.true_GS_radio_antenna{i_GS}.name)
        
    end

    GS_pass_data{i_GS} = this_data;

end
legend show
xlim([mission.true_time.store.time(1) mission.true_time.store.time(kd)])
xlabel('Time [sec]')
ylabel('Data Received per Pass [kb]')

grid on
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

end