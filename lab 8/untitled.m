robot = raspbot("raspbot9");
robot.startLaser();
pause(5)

while true
    ranges = robot.laser.LatestMessage.Ranges;
    tmpRangeImage = rangeImage(ranges,1,true);
    tmpRangeImage.findLineCandidate();
    pause(3);
    
    
end

