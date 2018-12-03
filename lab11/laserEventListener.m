function laserEventListener(~, ~)

global robot
global robotPose
global map
global mapUpdatePercentage

image = rangeImage(robot, 1, true);

x = image.xArray(1:9:end);
y = image.yArray(1:9:end);
w = ones(1, length(y));
modelPts = [x; y; w];

inPose = robotPose(1:3);
[success, outPose] = map.refinePose(pose(inPose), modelPts, 40);

if success
    outPoseVec = outPose.getPoseVec();
    mapUpdate = outPoseVec - inPose;
    inTh = inPose(3);
    
    mapUpdateTh = atan2(sin(outPoseVec(3) - inTh), ...
        cos(outPoseVec(3) - inTh));
    rawTh = inTh + mapUpdatePercentage * mapUpdateTh;
    
    newPose = inPose + mapUpdatePercentage * mapUpdate;
    newPose(3) = atan2(sin(rawTh), cos(rawTh));
    
    robotPose(1:3) = newPose;
end

end