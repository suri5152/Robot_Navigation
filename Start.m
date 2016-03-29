clear; clc; close all;
posn = [0,0,0];      %Initialization of the variable(ydim, xdim, angle)
rad = 2;             %Robot's body radius   
wdia = 3;            %Distance between robot's wheels                  
dtDefault = 0.5;     %Timestep between driving and colecting sensor data
course = rgb2gray(imread('Environment.png')); %Image of the Environment
imagesc(course), axis image off; colormap gray;
title('Click to specify robots starting position');
[posn(1) posn(2)] = ginput(1);
drawbot(posn, rad, course, 0);
title('Click to specify robots inital heading');
[y x] = ginput(1);
y = y - posn(1);
x = x - posn(2);
[posn(3), r] = cart2pol(x,y);
drawbot(posn, rad, course, 0);   %Show start position of the Robot
pause(.01);
wander(posn, rad, wdia, course, dtDefault);   %Begin to wander

