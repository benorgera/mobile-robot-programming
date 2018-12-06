global robot
global robotPose
global map
global mapUpdatePercentage

robot = raspbot();
robot.startLaser();
robot.forksDown();
pause(1)

p0 = pose(12*0.0254, 36*0.0254, -pi/4.0); % initial pose estimate on field
robotPose = [0; 0; 0; 0; 0];
robotPose(1:3) = p0.getPoseVec();

% Set up lines
f = 0.6096; % 2 feet in m
p1 = [0 ; 0];
p2 = [ 2*f ; 0];
p3 = [ 2*f ; 2*f];
p4 = [ 0 ; 2*f];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];

gain = 0.5;
errThresh = 0.01;
gradThresh = 0.0005;

% percentage of change in pose suggested by map to be applied to
% deadreackoning pose estimate
mapUpdatePercentage = 0.15;
map = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh, true);

% begin tracking position
robot.encoders.NewMessageFcn=@encoderEventListener;
robot.laser.NewMessageFcn=@laserEventListener;

% wait for map localization to work
pause(5)

pickDropObject(robot, 0.15);

robot.encoders.NewMessageFcn=[];
robot.laser.NewMessageFcn=[];