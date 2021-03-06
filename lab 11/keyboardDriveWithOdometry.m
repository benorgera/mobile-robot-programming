% drive by keyboard using map and odometry fused to estimate position

global robot
global robotPose

robot = raspbot();
robot.startLaser();
pause(5)

p0 = pose(15*0.0254, 9*0.0254, pi/2.0); % initial pose estimate on field
robotPose = [0; 0; 0; 0; 0];
robotPose(1:3) = p0.getPoseVec();

% Set up lines
f = 0.6096; % 2 feet in m
p1 = [-f ; -f];
p2 = [ f ; -f];
p3 = [ f ; f];
p4 = [-f ; f];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];

gain = 0.5;
errThresh = 0.01;
gradThresh = 0.0005;
pointFraction = 8;

% percentage of change in pose suggested by map to be applied to
% deadreackoning pose estimate
mapUpdatePercentage = 1.0;

figure

map = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh, true);

driver = robotKeypressDriver(gcf);

robot.encoders.NewMessageFcn=@encoderEventListener;

while 1
    
    image = rangeImage(robot, 1, true);
    x = image.xArray(1:pointFraction:end);
    y = image.yArray(1:pointFraction:end);
    w = ones(1, length(y));
    modelPts = [x; y; w];
    
    robotKeypressDriver.drive(robot, 9);
    inPose = robotPose(1:3);
    [success, outPose] = map.refinePose(pose(inPose), modelPts, 40);
    
    
    
    
    outPoseVec = outPose.getPoseVec();
    mapUpdate = outPoseVec - inPose;
    mapUpdate(3) = atan2(sin(outPoseVec(3) - inPose(3)), ...
        cos(outPoseVec(3) - inPose(3)));
    
    if success
        newPose = inPose + mapUpdatePercentage * mapUpdate;
        robotPose(1:3) = newPose;
    else
        newPose = robotPose(1:3);
    end
    
    hold off
    plot(lines_p1, lines_p2)
    hold on
    transform = pose(newPose).bToA();
    worldPts = transform*modelPts;
    scatter(worldPts(1, :), worldPts(2, :), 'bx')
    
    worldBodyPts = transform*map.bodyPts;
    plot(worldBodyPts(1,:),worldBodyPts(2,:),'k');
end

robot.stop();
robot.encoders.NewMessageFcn=[];
disp("broke out")