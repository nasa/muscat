function func_plot_attitude_visualization(mission, i_SC)

kd = mission.storage.k_storage_attitude;

%% Attitude Visualization
plot_handle = figure('Name',['SC ',num2str(i_SC),' Attitude Estimator Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');
axIdx = 0;
ax = nan(2*4,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude (Quaternion) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,1)

func_plot_attitude_visualization_subplot1(mission, i_SC, kd)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude (Euler Anlges) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2, 4, 2)

func_plot_attitude_visualization_subplot2(mission, i_SC, kd)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude Estimation Error % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,3)

func_plot_attitude_visualization_subplot3(mission, i_SC, kd)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Desired SC Attitude (Quaternion) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

    subplot(2,4,4)

    func_plot_attitude_visualization_subplot4(mission, i_SC, kd)

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Angular Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,5)

func_plot_attitude_visualization_subplot5(mission, i_SC, kd)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Angular Velocity (Estimation Error) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,6)

func_plot_attitude_visualization_subplot6(mission, i_SC, kd)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Desired SC Angular Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

    subplot(2,4,7)

    func_plot_attitude_visualization_subplot7(mission, i_SC, kd)
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Angle Error between Desired Attitude and Estimated Attitude % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

    subplot(2,4,8)

    func_plot_attitude_visualization_subplot8(mission, i_SC, kd)

end


sgtitle(['SC ',num2str(i_SC),' Attitude Estimator Performance'],'fontsize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Attitude_Estimator.png'];
    saveas(plot_handle,plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*

    ch = Chapter;
    ch.Title = ['Results: SC ',num2str(i_SC),' Attitude Estimator Performance'];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % SC Attitude (Quaternion) % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'SC Attitude (Quaternion)';

    plot_handle = figure(101);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_attitude_visualization_subplot1(mission, i_SC, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,4,1) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('SC attitude (quaternion), which represents orientation of inertial frame I with respect to the body frame B, for the entire duration of the simulation are shown.');
    append(sec,para)

    para = Paragraph('Both True values (from SC''s True_SC_ADC class) and Estimated values (from SC''s Software_SC_Estimate_Attitude class) are shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Attitude [Quaternion]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_adc.store.attitude(i_k,:);
    end

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Estimated SC ',num2str(i_SC),' Attitude [Quaternion]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(i_k,:);
    end

    para = Paragraph(['- "SC ',num2str(i_SC),' Attitude [Quaternion]" represents SC''s attitude (from SC''s True_SC_ADC class). This vector has 4 elements: [beta_1, beta_2, beta_3, beta_4], where beta_4 is the scalar term.']);
    append(sec,para)

    para = Paragraph(['- "Estimated SC ',num2str(i_SC),' Attitude [Quaternion]" represents estimated SC''s attitude (from SC''s Software_SC_Estimate_Attitude class). This vector has 4 elements: [beta_1, beta_2, beta_3, beta_4], where beta_4 is the scalar term.']);
    append(sec,para)


    plot_handle = figure(102);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_attitude_visualization_subplot2(mission, i_SC, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,4,2) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('SC attitude (Euler Angles) for the entire duration of the simulation are shown.');
    append(sec,para)


    plot_handle = figure(103);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_attitude_visualization_subplot3(mission, i_SC, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,4,3) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('SC attitude estimation error for the entire duration of the simulation are shown.');
    append(sec,para)

    para = Paragraph('Estimation error in SC attitude between True values (from SC''s True_SC_ADC class) and Estimated values (from SC''s Software_SC_Estimate_Attitude class) are shown.');
    append(sec,para)


    append(ch,sec)


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Desired SC Attitude % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

        sec = Section;
        sec.Title = 'Desired SC Attitude (Quaternion)';

        plot_handle = figure(104);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_attitude_visualization_subplot4(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(2,4,4) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('If attitude control is present, then Desired SC Attitude (quaternion) for the entire duration of the simulation are shown.');
        append(sec,para)

        para = Paragraph('These values (from SC''s Software_SC_Control_Attitude class) are shown.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Desired SC ',num2str(i_SC),' Attitude [Quaternion]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(i_k,:);
        end

        para = Paragraph(['- "Desired SC ',num2str(i_SC),' Attitude [Quaternion]" represents SC''s desired attitude (from SC''s Software_SC_Control_Attitude class). This vector has 4 elements: [beta_1, beta_2, beta_3, beta_4], where beta_4 is the scalar term.']);
        append(sec,para)

        plot_handle = figure(108);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_attitude_visualization_subplot8(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(2,4,8) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('If attitude control is present, then the angle error between Desired SC Attitude values (from SC''s Software_SC_Control_Attitude class) and Estimated SC Attitude values (from SC''s Software_SC_Estimate_Attitude class) for the entire duration of the simulation are shown.');
        append(sec,para)

        append(ch,sec)

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % SC Angular Velocity % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'SC Angular Velocity';

    plot_handle = figure(105);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_attitude_visualization_subplot5(mission, i_SC, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,4,5) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('SC Angular Velocity (rad/sec) for the entire duration of the simulation are shown.');
    append(sec,para)

    para = Paragraph('Both True values (from SC''s True_SC_ADC class) and Estimated values (from SC''s Software_SC_Estimate_Attitude class) are shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['SC ',num2str(i_SC),' Angular Velocity [rad/sec]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(i_k,:);
    end

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Estimated SC ',num2str(i_SC),' Angular Velocity [rad/sec]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(i_k,:);
    end

    para = Paragraph(['- "SC ',num2str(i_SC),' Angular Velocity [rad/sec]" represents SC''s angular velocity (from SC''s True_SC_ADC class). This vector has 3 elements: [omega_1, omega_2, omega_3].']);
    append(sec,para)

    para = Paragraph(['- "Estimated SC ',num2str(i_SC),' Angular Velocity [rad/sec]" represents estimated SC''s angular velocity (from SC''s Software_SC_Estimate_Attitude class). This vector has 3 elements: [omega_1, omega_2, omega_3].']);
    append(sec,para)

    plot_handle = figure(106);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_attitude_visualization_subplot6(mission, i_SC, kd)

    set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
    ax = gca;
    ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
    ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

    drawnow limitrate

    fig = Figure(gcf);
    fig.Scaling = "auto";
    fig.Snapshot.Caption = ['Location: Subplot(2,4,6) inside ',plot_file_name];
    append(sec,fig);

    para = Paragraph('Explanation of this plot:');
    para.Bold = true;
    append(sec,para)

    para = Paragraph('SC angular velocity estimation error for the entire duration of the simulation are shown.');
    append(sec,para)

    para = Paragraph('Estimation error in SC angular velocity between True values (from SC''s True_SC_ADC class) and Estimated values (from SC''s Software_SC_Estimate_Attitude class) are shown.');
    append(sec,para)

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Desired SC Angular Velocity % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isfield(mission.true_SC{i_SC}, 'software_SC_control_attitude')

        sec = Section;
        sec.Title = 'Desired SC Angular Velocity (rad/sec)';

        plot_handle = figure(107);
        clf
        set(plot_handle,'Color',[1 1 1]);
        set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
        set(plot_handle,'PaperPositionMode','auto');

        func_plot_attitude_visualization_subplot7(mission, i_SC, kd)

        set(gca, 'FontSize', mission.storage.report_parameters.report_plot_font_size)
        ax = gca;
        ax.Title.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.XLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;
        ax.YLabel.FontSize = mission.storage.report_parameters.report_plot_font_size;

        drawnow limitrate

        fig = Figure(gcf);
        fig.Scaling = "auto";
        fig.Snapshot.Caption = ['Location: Subplot(2,4,7) inside ',plot_file_name];
        append(sec,fig);

        para = Paragraph('Explanation of this plot:');
        para.Bold = true;
        append(sec,para)

        para = Paragraph('If attitude control is present, then Desired SC Angular Velocity (rad/sec) for the entire duration of the simulation are shown.');
        append(sec,para)

        para = Paragraph('These values (from SC''s Software_SC_Control_Attitude class) are shown.');
        append(sec,para)

        para = Paragraph('Variables in Excel file:');
        para.Bold = true;
        append(sec,para)

        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
        mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Desired SC ',num2str(i_SC),' Angular Velocity [rad/sec]'];
        this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

        for i_k = 1:1:kd
            mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(i_k,:);
        end

        para = Paragraph(['- "Desired SC ',num2str(i_SC),' Angular Velocity [rad/sec]" represents SC''s desired angular velocity (from SC''s Software_SC_Control_Attitude class). This vector has 3 elements: [omega_1, omega_2, omega_3].']);
        append(sec,para)

        append(ch,sec)

    end


    append(mission.storage.report_parameters.report_handle,ch);

end

end


function func_plot_attitude_visualization_subplot1(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude (Quaternion) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\beta_1 (True)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\beta_2 (True)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\beta_3 (True)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,4), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\beta_4 (True)')

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,1),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\beta_1 (Estimated)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,2),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\beta_2 (Estimated)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,3),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\beta_3 (Estimated)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,4),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\beta_4 (Estimated)')

ylim([-1 1])

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Quaternion')
title('Quaternion','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot2(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude (Euler Anlges) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[roll,pitch,yaw] = func_quat2euler(mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,:));

% Plot Euler
hold on
plot(mission.true_time.store.time_attitude(1:kd), roll(:)*180/pi, '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Roll (True)')
plot(mission.true_time.store.time_attitude(1:kd), pitch(:)*180/pi, '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pitch (True)')
plot(mission.true_time.store.time_attitude(1:kd), yaw(:)*180/pi, '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Yaw (True)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Angle (deg)')
title('Euler Angles','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot3(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Attitude Estimation Error % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \beta_1')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \beta_2')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \beta_3')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.attitude(1:kd,4) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude(1:kd,4), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\Delta \beta_4')

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,1),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \beta_1 (Uncertainty)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,2),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \beta_2 (Uncertainty)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,3),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \beta_3 (Uncertainty)')
% plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.attitude_uncertainty(1:kd,4),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\Delta \beta_4 (Uncertainty)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Estimation Error')
title('Estimation Error in Attitude','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot4(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Desired SC Attitude (Quaternion) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\beta_1 (Desired)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\beta_2 (Desired)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\beta_3 (Desired)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_attitude(1:kd,4), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(4), 'DisplayName','\beta_4 (Desired)')

ylim([-1 1])

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Quaternion')
title('Desired Quaternion','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot8(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Angle Error between Desired Attitude and Estimated Attitude % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.error_angle_desired_attitude(1:kd), '-','LineWidth',2,'Color','k')

grid on
%     legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Error angle [rad]')
title('Angle between {\beta}_{desired} and {\beta}_{estimated}','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot5(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Angular Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\omega_1 (True)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\omega_2 (True)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\omega_3 (True)')

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,1),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\omega_1 (Estimated)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,2),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\omega_2 (Estimated)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,3),'--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\omega_3 (Estimated)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('\omega [rad/sec]')
title('Angular Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot6(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Angular Velocity (Estimation Error) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \omega_1')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \omega_2')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.true_SC_adc.store.angular_velocity(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \omega_3')

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta \omega_1 (Uncertainty)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta \omega_2 (Uncertainty)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_estimate_attitude.store.angular_velocity_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta \omega_3 (Uncertainty)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Estimation Error [rad/sec]')
title('Estimation Error in Angular Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_attitude_visualization_subplot7(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Desired SC Angular Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


hold on

plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\omega_1 (Desired)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\omega_2 (Desired)')
plot(mission.true_time.store.time_attitude(1:kd), mission.true_SC{i_SC}.software_SC_control_attitude.store.desired_angular_velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\omega_3 (Desired)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('\omega [rad/sec]')
title('Desired Angular Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'fontsize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end