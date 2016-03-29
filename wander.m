function wander(posn, rad, wdia, course, dtDefault)
vL_target = 5;
vR_target = 5;
reltrans=0;
dy=posn(1);dx=posn(2);
thetaprev = posn(3);
yprev = posn(1);
xprev = posn(2);
prompt='Press y/n to switch on/off path tracking = ';
plotTrajectory = input(prompt,'s');
if plotTrajectory=='y'
    plotTrajectory = 1;
else
    plotTrajectory = 0;
end
numSensors = 8;
minDistFromObst = 5;
collisionAlarmDist = rad+2;
deltaAngularDispMax = 2*pi/8;

for i = 1:1:500
    dt = dtDefault;
    % Finding range
    for indDirection = 1:numSensors
        rangeArr(indDirection) = rangefinder([posn(1), posn(2), posn(3)+ (indDirection-1)*2*pi/numSensors], rad, course);
    end
    
    % Check if there is any possible collision
    apparentCollisionIndices = find(rangeArr <= minDistFromObst);
    apparentCollisionIndic = double(sum(apparentCollisionIndices)>0);
    
    if (apparentCollisionIndic == 1) && (rangeArr(1) < 30 || min(rangeArr) < collisionAlarmDist)
        % if yes - find the direction (relative to posn(3)) corresponding to the highest range
        highestRangeIndex = (find(rangeArr == max(rangeArr)));
        % if two directions are max consider one of them
        highestRangeIndex = highestRangeIndex(1);
        highestRangeVal = rangeArr(highestRangeIndex);
        % depending on the direction change, dt determine vL and vR
        deltaAngularDisp = (highestRangeIndex-1)*2*pi/numSensors;
        
        if deltaAngularDisp > deltaAngularDispMax && deltaAngularDisp <= pi
            deltaAngularDisp = deltaAngularDispMax;
        elseif deltaAngularDisp < 2*pi-deltaAngularDispMax && deltaAngularDisp > pi
            deltaAngularDisp = 2*pi - deltaAngularDispMax;
        end
        if min(rangeArr) < collisionAlarmDist
            almostCollideIndex = find(rangeArr == min(rangeArr));
            almostCollideIndex = almostCollideIndex(1);
            forceRotateDirection = mod(almostCollideIndex-1+numSensors/2, numSensors)+1;
            deltaAngularDisp = forceRotateDirection*2*pi/numSensors;
        end
        vL = -(deltaAngularDisp*wdia/2/dt)/2;
        vR = -vL;
        posn = drive(posn, wdia, vL, vR, dt);
        vL = 1;
        vR = 1;
    else
        % if not - continue moving in the forward direction
        dt = dtDefault;
        vL = vL_target;
        vR = vR_target;
    end
    
    % Every 15 iterations, determine a new random-ish arc
    
    if(mod(i,15) == 0)
        vL = 1;
        vR = sign(.5-rand);
        dt = dtDefault;
    end
    posn = drive(posn, wdia, vL, vR, dt);
    
    drawbot(posn,rad,course, plotTrajectory);
    M(i)=getframe;
    yy = posn(1);
    xx = posn(2);
    dtheta = posn(3)-thetaprev;
    if dtheta ~= 0
        dy = posn(1)-yprev;
        dx = posn(2)-xprev;
        yprev = posn(1);
        xprev = posn(2);
        [angle,trsl]=cart2pol(dx,dy);
        reltrans = trsl;
    end
    y(i) = posn(1);
    x(i) = posn(2);
    deltatheta(i) = posn(3)-thetaprev;
    if dtheta ~= 0
        deltay(i) = dy;
        deltax(i) = dx;
        rtrans(i) = trsl;
    end
    if plotTrajectory == 1
        hold on
        plot([yprev posn(1)], [xprev posn(2)]);
    end
    thetaprev = posn(3);
    drawnow;
    mTextBox = uicontrol('style','text');
    set(mTextBox,'String',{['Relative rotation = ' num2str(dtheta*180/pi) ' degrees'], ...
        ['(x,y)= (' num2str(xx) ', ' num2str(yy) ')'], ['(dx, dy) = (' num2str(dx) ', ' num2str(dy) ')'], ['Rel translation = ' num2str(reltrans)]});
    set(mTextBox, 'Position', [15 15 200 60]);
    set(mTextBox,'Units','characters')
    
    hold off
end
prompt='Press y to display the saved data and recorded video = ';
video = input(prompt,'s');
if video=='y'
    movie(M);
    fprintf('The value of absolute position y is %f\n');
    disp(y);
    fprintf('The value of absolute position x is %f\n');
    disp(x);
    fprintf('The value of relative rotation(Radians) is %f\n');
    disp(deltatheta);
    fprintf('The value of relative position y is %f\n');
    disp(deltay);
    fprintf('The value of relative position x is %f\n');
    disp(deltax);
    fprintf('The value of relative translation is %f\n');
    disp(rtrans);
    movie2avi(M, 'simulation.avi', 'compression', 'none');
end


function dist = rangefinder(posn, rad, course)
[dimy, dimx] = size(course);
for i = rad:(100+rad)
    y = posn(1)+ i*sin(posn(3));
    x = posn(2)+ i*cos(posn(3));
    x(x < 1) = 1;
    x(x > dimx) = dimx;
    y(y < 1) = 1;
    y(y > dimy) = dimy;
    if(course(round(x),round(y)) == 0)
        dist = i-rad;
        return;
    end
end
dist = 100;
return;

function [newposn] = drive(posn, wdia, vL, vR, t)
vdiff = vR-vL;
vsum = vR+vL;
newposn(3) = posn(3) + 2*vdiff*t/wdia;
if(vdiff == 0)
    newposn(1) = vL*t*sin(posn(3))+posn(1);
    newposn(2) = vR*t*cos(posn(3))+posn(2);
else
    newposn(1) = posn(1) - wdia*vsum/(2*vdiff)*(cos(vdiff*t/wdia+posn(3))-cos(posn(3)));
    newposn(2) = posn(2) + wdia*vsum/(2*vdiff)*(sin(vdiff*t/wdia+posn(3))-sin(posn(3)));
end



