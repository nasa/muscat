function func_plot_orbit_estimator(mission, i_SC)

kd = mission.storage.k_storage;

%% Orbit Vizualization
plot_handle = figure('Name',['SC ',num2str(i_SC),' and Target Position Estimator Performance']);
clf
set(plot_handle,'Color',[1 1 1]);
set(plot_handle,'units','normalized','outerposition',[0 0 1 1])
set(plot_handle,'PaperPositionMode','auto');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,1)
hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_navigation')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.position(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.position(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.position(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (True)')
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


if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
    
    subplot(2,4,5)
    hold on

    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.position(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.position(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.position(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z (True)')

    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z (Estimated)')

    %     semilogy(mission.storage.true_time.time_array_dynamics(1:kd), abs(mission.storage.true_SC{plot_SC_number}.position_array(1:kd,1)-mission.storage.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,1)),'-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1))

    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Position Error [km]')
    title('Estimation Error in SC Position','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SC Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,2)
hold on

if isfield(mission.true_SC{i_SC}, 'true_SC_navigation')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (True)')
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

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
    
    subplot(2,4,6)
    hold on

    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.velocity(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.velocity(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.true_SC_navigation.store.velocity(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z (True)')

    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z (Estimated)')


    %     semilogy(mission.storage.true_time.time_array_dynamics(1:kd), abs(mission.storage.true_SC{plot_SC_number}.velocity_array(1:kd,1)-mission.storage.true_SC{plot_SC_number}.SC_Estimate_Position_Velocity(1:kd,4)),'-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1))

    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Velocity Error [km/sec]')
    title('Estimation Error in SC Velocity','FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Target Position % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,3)
hold on

if isfield(mission, 'true_target')
    for i_target = 1:1:mission.num_target
        plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (True)')
        plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (True)')
        plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.position(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (True)')
    end
end

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Pos X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Pos Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Pos Z (Estimated)')
end

grid on

legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Position [km]')
title('Target Position','FontSize',mission.storage.plot_parameters.title_font_size)
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')

    for i_target = 1:1:mission.num_target
        if strcmp(mission.true_target{i_target}.name, mission.true_SC{i_SC}.software_SC_estimate_orbit.name_relative_target)
            this_i_target = i_target;
        end
    end

    subplot(2,4,7)
    hold on

    plot(mission.true_time.store.time(1:kd), mission.true_target{this_i_target}.store.position(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_target{this_i_target}.store.position(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_target{this_i_target}.store.position(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z (True)')

    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Pos X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Pos Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.position_target_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Pos Z (Estimated)')

    %     semilogy(mission.storage.true_time.time_array_dynamics(1:kd), abs(mission.storage.true_small_body.position_array(1:kd,1)-mission.storage.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,1)),'-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1))
    
    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Position Error [km]')
    title('Estimation Error in Target Position')
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Target Velocity % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,4,4)
hold on

if isfield(mission, 'true_target')
    for i_target = 1:1:mission.num_target
        plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (True)')
        plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (True)')
        plot(mission.true_time.store.time(1:kd), mission.true_target{i_target}.store.velocity(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (True)')
    end
end

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','Vel X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','Vel Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','Vel Z (Estimated)')
end

grid on
legend('Location','southwest')
xlabel('Time [sec]')
ylabel('Velocity [km/sec]')
title('Target Velocity')
set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
hold off

if isfield(mission.true_SC{i_SC}, 'software_SC_estimate_orbit')

    subplot(2,4,8)
    hold on

    plot(mission.true_time.store.time(1:kd), mission.true_target{this_i_target}.store.velocity(1:kd,1) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,1), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_target{this_i_target}.store.velocity(1:kd,2) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,2), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y (True)')
    plot(mission.true_time.store.time(1:kd), mission.true_target{this_i_target}.store.velocity(1:kd,3) - mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target(1:kd,3), '-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z (True)')

    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target_uncertainty(1:kd,1), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1), 'DisplayName','\Delta Vel X (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target_uncertainty(1:kd,2), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(2), 'DisplayName','\Delta Vel Y (Estimated)')
    plot(mission.true_time.store.time(1:kd), mission.true_SC{i_SC}.software_SC_estimate_orbit.store.velocity_target_uncertainty(1:kd,3), '--','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(3), 'DisplayName','\Delta Vel Z (Estimated)')

    %     semilogy(mission.storage.true_time.time_array_dynamics(1:kd), abs(mission.storage.true_small_body.velocity_array(1:kd,1)-mission.storage.true_SC{plot_SC_number}.SB_Estimate_Position_Velocity(1:kd,4)),'-','LineWidth',2,'Color',mission.storage.plot_parameters.color_array(1))
    
    grid on
    legend('Location','southwest')
    xlabel('Time [sec]')
    ylabel('Velocity Error [km/sec]')
    title('Estimation Error in Target Velocity', 'FontSize',mission.storage.plot_parameters.title_font_size)
    set(gca, 'FontSize',mission.storage.plot_parameters.standard_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)
    hold off
end


sgtitle(['SC ',num2str(i_SC),' and Target Position Estimator Performance'],'FontSize',mission.storage.plot_parameters.title_font_size,'FontName',mission.storage.plot_parameters.standard_font_type)

if mission.storage.plot_parameters.flag_save_plots == 1
    saveas(plot_handle,[mission.storage.output_folder, mission.name,'_SC',num2str(i_SC),'_Orbit_Estimator.png'])
end

end