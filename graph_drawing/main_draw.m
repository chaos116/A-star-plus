clear
close all
clc

filename = 'map_nodes_output.txt';
fid = fopen(filename, 'r');
mat = zeros(10000, 10000); 

line =fgetl(fid);
for i = 1:10000
    line = fgetl(fid);        
    line = line(2:10001);
    mat(i, :) = line - '0';     
end

fclose(fid);



filename = 'AstarPath.txt';
fid = fopen(filename, 'r');
astarPath = fscanf(fid, '%d'); 
filename = 'AstarPlusPath.txt';
fid = fopen(filename, 'r');
astarplusPath = fscanf(fid, '%d');  
subplot(1,2,1)
plot_grid_map(mat)
plot_path(astarPath)
title('A* algorithm');
subplot(1,2,2)
plot_grid_map(mat)
plot_path(astarplusPath)
title('A*-plus algorithm');

figure
plot_grid_map(mat)
plot_path(astarplusPath)
