function func_visualize_spacecraft_DARE(mission_true_SC, mission_init_data, flag_save_plot)
    fig = figure(1);
    set(fig, 'Color', [1, 1, 1]);
    set(fig, 'units', 'normalized', 'outerposition', [0, 0, 0.7, 1]);
    set(fig, 'PaperPositionMode', 'auto');
    clf;
    
    subplot(2, 1, 1)
    plot_spacecraft_components_DARE(mission_true_SC{1});
    axis equal
    view([-25, 10])
    xlabel("X_{SC} [m]")
    ylabel("Y_{SC} [m]")
    zlabel("Z_{SC} [m]")
    set(gca, 'fontsize', 14, 'FontName', 'Times New Roman')
    
    subplot(2, 1, 2)
    plot_spacecraft_components_DARE(mission_true_SC{1});
    axis equal
    xlim([-2, 2])
    ylim([-3, 3])
    view([-50, 3])
    % legend westoutside
    legend("Location", "westoutside")
    xlabel("X_{SC} [m]")
    ylabel("Y_{SC} [m]")
    zlabel("Z_{SC} [m]")
    set(gca, 'fontsize', 14, 'FontName', 'Times New Roman')
    
    if flag_save_plot == 1
        filename = [mission_init_data.output_folder, filesep, 'SC_overview.png'];
        exportgraphics(fig, filename)
    end
end