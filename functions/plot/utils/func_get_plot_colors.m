function colors = func_get_plot_colors(n_colors)
if nargin < 1
    n_colors = 10;
end
fig = figure();
cmap = colormap(lines(n_colors));
colors = cell(n_colors,1);
for i = 1:n_colors
    colors{i} = cmap(i, :);
end
close(fig);
end