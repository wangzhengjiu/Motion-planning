% Used for Motion Planning for Mobile Robots
%% initialize 
close all; clear all; clc;
tic   %计时  

set(gcf, 'Renderer', 'painters');   %gcf 表示当前图窗句柄  
set(gcf, 'Position', [1000, 0, 700, 700]);  %设置输出尺寸  前两个参数表示输出图像在屏幕的x轴位置 y轴位置（原点为屏幕左下角）  后面表示，图像长度 图像高度set size of the picture
%https://www.jianshu.com/p/c8c9a8185dcf
%% Environment map in 2D space 
xStart = 1.0;  %set starting point 
yStart = 1.0;
xTarget = 9.0; %set target
yTarget = 9.0;
MAX_X = 10;    %set size of the map
MAX_Y = 10;
%% 
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);  %create  arandom map


% Waypoint Generator Using the A* 
path= A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
visualize_map(map, path);
toc
