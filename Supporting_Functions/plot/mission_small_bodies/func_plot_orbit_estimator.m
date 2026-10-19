function func_plot_orbit_estimator(mission, i_SC)

kd = mission.storage.k_storage;

%% Orbit Vizualization
plot_handle = figure('Name',['SC ',num2str(i_SC),' and Target Position and Velocity']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,1)

func_plot_orbit_estimator_subplot1(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in SC Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,5)

func_plot_orbit_estimator_subplot5(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,2)

func_plot_orbit_estimator_subplot2(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in SC Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,6)

func_plot_orbit_estimator_subplot6(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Target Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,3)

func_plot_orbit_estimator_subplot3(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in Target Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,7)

func_plot_orbit_estimator_subplot7(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Target Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,4)

func_plot_orbit_estimator_subplot4(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in Target Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,8)

func_plot_orbit_estimator_subplot8(mission, i_SC, kd)


sgtitle(['SC ',num2str(i_SC),' and Target Position and Velocity'],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

%% Save Image

if mission.storage.plot_parameters.flag_save_plots == 1
    plot_file_name = [mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Position_Velocity.png'];
    saveas(plot_handle,plot_file_name)
end

%% Write Report

if mission.storage.report_parameters.flag_write_report == 1

    import mlreportgen.report.*
    import mlreportgen.dom.*

    ch = Chapter;
    ch.Title = ['Results: SC ',num2str(i_SC),' and Target Position Estimator Performance'];


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % SC Position % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sec = Section;
    sec.Title = 'SC Position';

    plot_handle = figure(101);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot1(mission, i_SC, kd)

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

    para = Paragraph('SC position in Sun-centered J2000 frame for the entire duration of the simulation are shown.');
    append(sec,para)

    para = Paragraph('Both True values (from SC''s True_SC_Orbit_Propagation class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)


    plot_handle = figure(105);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot5(mission, i_SC, kd)

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

    para = Paragraph('Estimation error in SC position between True values (from SC''s True_SC_Orbit_Propagation class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Estimated SC ',num2str(i_SC),' Position [km]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(i_k,:); % [km];
    end

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Estimated SC ',num2str(i_SC),' Velocity [km/sec]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(i_k,:); % [km];
    end

    para = Paragraph(['- "Estimated SC ',num2str(i_SC),' Position [km]" represents SC''s position in Sun-centered J2000 frame (from SC''s Software_SC_Estimate_Orbit class).']);
    append(sec,para)

    para = Paragraph(['- "Estimated SC ',num2str(i_SC),' Velocity [km/sec]" represents SC''s velocity in Sun-centered J2000 frame (from SC''s Software_SC_Estimate_Orbit class).']);
    append(sec,para)

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % SC Velocity % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sec = Section;
    sec.Title = 'SC Velocity';

    plot_handle = figure(102);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot2(mission, i_SC, kd)

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

    para = Paragraph('SC velocity in Sun-centered J2000 frame for the entire duration of the simulation are shown.');
    append(sec,para)

    para = Paragraph('Both True values (from SC''s True_SC_Orbit_Propagation class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)


    plot_handle = figure(106);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot6(mission, i_SC, kd)

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

    para = Paragraph('Estimation error in SC velocity between True values (from SC''s True_SC_Orbit_Propagation class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Target Position % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    i_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
    sec = Section;
    sec.Title = ['Target (',mission.true_target{i_target}.name,') Position'];

    plot_handle = figure(103);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot3(mission, i_SC, kd)

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

    para = Paragraph(['Target (',mission.true_target{i_target}.name,') position in Sun-centered J2000 frame for the entire duration of the simulation are shown.']);
    append(sec,para)

    para = Paragraph('Both True values (from True_Target class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)


    plot_handle = figure(107);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot7(mission, i_SC, kd)

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

    para = Paragraph(['Estimation error in Target (',mission.true_target{i_target}.name,') position between True values (from True_Target class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.']);
    append(sec,para)

    para = Paragraph('Variables in Excel file:');
    para.Bold = true;
    append(sec,para)

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Estimated SC',num2str(i_SC),'s Target Position [km]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(i_k,:); % [km];
    end

    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);
    mission.storage.report_parameters.Excel_Variable_Names{this_Excel_variable_count+1} = ['Estimated SC',num2str(i_SC),'s Target Velocity [km/sec]'];
    this_Excel_variable_count = length(mission.storage.report_parameters.Excel_Variable_Names);

    for i_k = 1:1:kd
        mission.storage.report_parameters.Excel_cellData{i_k,this_Excel_variable_count} = mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(i_k,:); % [km];
    end

    para = Paragraph(['- "Estimated SC',num2str(i_SC),'s Target Position [km]" represents Target''s position in Sun-centered J2000 frame (from SC''s Software_SC_Estimate_Orbit class).']);
    append(sec,para)

    para = Paragraph(['- "Estimated SC',num2str(i_SC),'s Target Velocity [km/sec]" represents Target''s velocity in Sun-centered J2000 frame (from SC''s Software_SC_Estimate_Orbit class).']);
    append(sec,para)

    append(ch,sec)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Target Velocity % %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sec = Section;
    sec.Title = ['Target (',mission.true_target{i_target}.name,') Velocity'];

    plot_handle = figure(104);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot4(mission, i_SC, kd)

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

    para = Paragraph(['Target (',mission.true_target{i_target}.name,') velocity in Sun-centered J2000 frame for the entire duration of the simulation are shown.']);
    append(sec,para)

    para = Paragraph('Both True values (from True_Target class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.');
    append(sec,para)


    plot_handle = figure(108);
    clf
    set(plot_handle,'Color',[1 1 1]);
    set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
    set(plot_handle,'PaperPositionMode','auto');

    func_plot_orbit_estimator_subplot8(mission, i_SC, kd)

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

    para = Paragraph(['Estimation error in Target (',mission.true_target{i_target}.name,') velocity between True values (from True_Target class) and Estimated values (from SC''s Software_SC_Estimate_Orbit class) are shown.']);
    append(sec,para)

    append(ch,sec)


    append(mission.storage.report_parameters.report_handle,ch);

end

end

function func_plot_orbit_estimator_subplot1(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_orbit_propagation')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (True)')
end

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (Estimated)')
end

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Position [km]')
title('SC Position','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot5(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in SC Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.position(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z')

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z (Uncertainty)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Position Error [km]')
title('Estimation Error in SC Position','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot2(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_orbit_propagation')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (True)')
end

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (Estimated)')
end

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Velocity [km/sec]')
title('SC Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot6(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in SC Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_orbit_propagation.store.velocity(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z')

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z (Uncertainty)')


grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Velocity Error [km/sec]')
title('Estimation Error in SC Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot3(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Target Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

i_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (True)')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (True)')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (True)')

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (Estimated)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (Estimated)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (Estimated)')

grid on

legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Position [km]')
title('Target Position','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot7(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in Target Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

i_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z')

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z (Uncertainty)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Position Error [km]')
title('Estimation Error in Target Position')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot4(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Target Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

i_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (True)')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (True)')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (True)')


plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (Estimated)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (Estimated)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (Estimated)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Velocity [km/sec]')
title('Target Velocity')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end

function func_plot_orbit_estimator_subplot8(mission, i_SC, kd)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Estimation Error in Target Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on

i_target = mission.true_SC{i_SC}.true_SC_orbit_propagation.index_relative_target;
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y')
plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z')

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y (Uncertainty)')
plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z (Uncertainty)')

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Velocity Error [km/sec]')
title('Estimation Error in Target Velocity', 'FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

end