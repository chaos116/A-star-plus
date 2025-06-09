function plot_grid_map(grid)
    % plot_grid_map(grid)
    % 输入：
    %   grid - 10000x10000 的 0-1 栅格地图，1 为可通行，0 为障碍
    %
    % 输出：
    %   在当前图窗中绘制地图

    imagesc(grid);              % 使用颜色图绘制矩阵
    colormap([0 0 0; 1 1 1]);   % 0 -> 黑色（障碍），1 -> 白色（通行）
    axis equal tight;          % 保持比例，紧凑显示
    set(gca, 'YDir', 'normal');% 保证Y轴从上到下
    xlabel('Column');
    ylabel('Row');
    title('Grid Map');

    % 可选：关闭坐标轴刻度（适用于高分辨率地图）
    % set(gca, 'xtick', [], 'ytick', []);
end


