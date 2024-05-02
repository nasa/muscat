function func_plot_coe(tspan, coe)
% Plot Classical Orbital Elements
% Input:
%  tspan: time span [s] (N x 1) or [1 x N]
%  coe: classical orbital elements
%       [a, e, i, O, w, M] [km, -, rad] (N_d x 6)

fig = figure();
set(fig, 'units', 'normalized', 'outerposition', [0 0 1 1]);
set(fig, 'Color', [1 1 1]);
set(fig, 'PaperPositionMode', 'auto');

coe(:, 3:6) = rad2deg(coe(:, 3:6));
ylabels = {'a [km]', 'e', 'i [deg]', '\Omega [deg]', '\omega [deg]', 'M [deg]' };

for ii = 1:6
    subplot(2, 3, ii);
    set(gca, 'FontSize', 14)
    grid on; hold on;
    xlabel('Time [hours]')
    ylabel(ylabels{ii})
    plot(tspan / 3600, coe(:, ii), 'LineWidth', 1.5)
end
set(gca, 'fontsize', 16, 'FontName', 'Times New Roman')

end