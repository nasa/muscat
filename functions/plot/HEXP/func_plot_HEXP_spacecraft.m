function func_plot_HEXP_spacecraft(mission_true_SC, mission_init_data, flag_save_plot)
fig = figure(1);
set(fig,'Color',[1 1 1]);
set(fig,'units','normalized','outerposition',[0 0 0.7 1])
set(fig,'PaperPositionMode','auto');
clf;

R_x2z = [
    0, 0, -1;
    0, 1, 0;
    1, 0, 0;
    ];
Rot = R_x2z;
Rot = eye(3);

subplot(2,1,1)
func_plot_HEXP_components(mission_true_SC{1}, Rot);
axis equal
campos([2, 10, 7])
camup([1 0 0])

xlabel("X_{SC} [m]")
ylabel("Y_{SC} [m]")
zlabel("Z_{SC} [m]")
set(gca, 'fontsize',14,'FontName','Times New Roman')

subplot(2,1,2)
func_plot_HEXP_components(mission_true_SC{1}, Rot);
axis equal
zlim([-2,2])
% ylim([-6,6])
campos([1, 2, -5])
camup([1 0 0])
legend('Location','eastoutside');
xlabel("X_{SC} [m]")
ylabel("Y_{SC} [m]")
zlabel("Z_{SC} [m]")
set(gca, 'fontsize',14,'FontName','Times New Roman')

if flag_save_plot == 1
    filename = [mission_init_data.output_folder filesep 'SC_overview.png'];
    exportgraphics(fig, filename)
end
end