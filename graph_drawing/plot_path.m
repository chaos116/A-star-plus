function plot_path(path)
    % plot_grid_path(grid, path1, path2)
    % 输入：
    %   path - 路径 1 的节点编号数组

    hold on;

    % 将节点编号转换为坐标（行列）
    [r1, c1] = node_ids_to_coords(path);

    % 绘制路径
    plot(c1, r1, 'r-', 'LineWidth', 1.5);  % 路径1 红色线

    % 标记起点和终点
    plot(c1(1), r1(1), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g'); % 起点
    plot(c1(end), r1(end), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); % 终点

    hold off;
end

function [rows, cols] = node_ids_to_coords(ids)
    % 将节点编号转换为行列坐标（从编号转为矩阵下标）
    % 注意：编号是从左下角(1,1)开始，行从下往上编号
    grid_size = 10000;
    rows = mod(ids - 1, grid_size) + 1;
    cols = floor((ids - 1) / grid_size) + 1;
end
