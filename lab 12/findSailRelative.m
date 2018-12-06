function res = findSailRelative(robot, thetaLow, thetaHigh)

if nargin < 3
    thetaLow = 1;
    thetaHigh = 360;
end

while true
    rangeImg = rangeImage(robot, 1, true, thetaLow, thetaHigh);
    [x,y,th] = rangeImg.findLineCandidate(true);
    
    if (th == 0)&&(x == 0)&&(y == 0)
        pause(0.2)
        disp("couldn't find sail, retrying")
    else
        break
    end
end

res = [x;y;th];
end