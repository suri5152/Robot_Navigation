function drawbot(posn, rad, course, plotTrajectory)

if plotTrajectory == 1
    hold on;
else
    hold off;
    imagesc(course), axis image off; colormap gray;
    hold on;
end
%Draw a circle on the Environment
angs = 0:pi/10:2*pi;
y = posn(1) + rad*sin(angs);
x = posn(2) + rad*cos(angs);
plot(y,x);
plot([posn(1) posn(1)+rad*sin(posn(3))], [posn(2) posn(2)+rad*cos(posn(3))],'r');


