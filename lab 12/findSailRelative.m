function res = findSailRelative(robot, axisInterval, isX, low, high, altLow)

while true
    rangeImg = rangeImage(robot, 1, true);
    
    if nargin == 6
        [x,y,th] = rangeImg.findLineCandidate(true, axisInterval, isX, low, high, altLow);
    else
        [x,y,th] = rangeImg.findLineCandidate(true);
    end
    
    if (th == 0)&&(x == 0)&&(y == 0)
        pause(0.2)
        disp("couldn't find sail, retrying")
    else
        break
    end
end

res = [x;y;th];
end