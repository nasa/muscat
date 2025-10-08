%% Postprocess NISAR Mission: Plot Ground Track

function func_post_process_NISAR_Ground_Tracks(mission, i_SC)


kd = mission.storage.k_storage;

plot_handle = figure('Name', ['SC ',num2str(i_SC),' Ground Track']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

% Region Of Interest (ROI)
ROI_idx = logical(mission.true_SC{i_SC}.software_SC_executive.store.latitude > 24) & logical(mission.true_SC{i_SC}.software_SC_executive.store.latitude < 40) ...
    & logical(mission.true_SC{i_SC}.software_SC_executive.store.longitude < -80) & logical(mission.true_SC{i_SC}.software_SC_executive.store.longitude > -100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Ground Track (Latitude) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3,1,1)
hold on

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_executive.store.latitude(1:kd), '-b','LineWidth',2)

plot(mission.true_time.store.time(ROI_idx), mission.true_SC{i_SC}.software_SC_executive.store.latitude(ROI_idx), 'or','MarkerSize',10, 'MarkerFaceColor','r')

grid on
xlabel('Time [sec]')
ylabel('Latitude [deg]')
% title('Latitude','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Ground Track (Longitude) % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3,1,2)
hold on

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_executive.store.longitude(1:kd), '-b','LineWidth',2)

plot(mission.true_time.store.time(ROI_idx), mission.true_SC{i_SC}.software_SC_executive.store.longitude(ROI_idx), 'or','MarkerSize',10, 'MarkerFaceColor','r')

grid on
xlabel('Time [sec]')
ylabel('Longitude [deg]')
% title('Longitude','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Sun-Earth-SC Angle % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(3,1,3)
hold on

plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_executive.store.Sun_Earth_SC_Angle(1:kd), '-b','LineWidth',2)

plot(mission.true_time.store.time(ROI_idx), mission.true_SC{i_SC}.software_SC_executive.store.Sun_Earth_SC_Angle(ROI_idx), 'or','MarkerSize',10, 'MarkerFaceColor','r')

grid on
xlabel('Time [sec]')
ylabel('Sun-Earth-SC Angle [deg]')
% title('Longitude','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off


if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_GroundTrack.png'])
end


ROI_UTC_Time = sec2cal(mission.true_time.store.date(find(ROI_idx,1))) 
% Should be around 11am/11pm UTC time for 6am/6pm CDT local time