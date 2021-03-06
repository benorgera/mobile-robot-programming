% drive by keyboard using lidar map to update pose

robot = raspbot();
robot.startLaser();
pause(5)

p0 = pose(15*0.0254, 9*0.0254, pi/2.0); % initial pose estimate on field

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

figure

map = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh, true);

driver = robotKeypressDriver(gcf);

curPose = p0;

while 1
    
    image = rangeImage(robot, 1, true);
    x = image.xArray(1:pointFraction:end);
    y = image.yArray(1:pointFraction:end);
    w = ones(1, length(y));
    modelPts = [x; y; w];
    
    robotKeypressDriver.drive(robot, 6);
    [success, outPose] = map.refinePose(curPose, modelPts, 40);
    
    if ~success
        continue
    end
    
    curPose = outPose;
    
    
    hold off
    plot(lines_p1, lines_p2)
    hold on
    transform = curPose.bToA();
    worldPts = transform*modelPts;
    scatter(worldPts(1, :), worldPts(2, :), 'bx')
    
    worldBodyPts = transform*map.bodyPts;
    plot(worldBodyPts(1,:),worldBodyPts(2,:),'k');
    
end

robot.stop();
disp("broke out")