global robot
global robotPose
global map
global mapUpdatePercentage

robot = raspbot();
robot.startLaser();
pause(5)

p0 = pose(24*0.0254, 24*0.0254, pi/2.0); % initial pose estimate on field
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

vel = 0.15;

plotDriving = true;

%xf = 0.3048; yf = 0.9144; thf = pi/2.0;
xf = 0.3048; yf = 0.85; thf = pi/2.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
traj.planVelocities(vel,true,true);
fol = trajectoryFollower(traj, true, 5, plotDriving);
fol.execute(true, true);
 
xf = 0.9144; yf = 0.3048; thf = 0.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
traj.planVelocities(vel,true,true);
fol = trajectoryFollower(traj, true, 5, plotDriving);
fol.execute(true, true);

xf = 0.6096; yf = 0.6069; thf = pi/2.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
traj.planVelocities(vel,true,true);
fol = trajectoryFollower(traj, true, 6, plotDriving);
fol.execute(true, true);

robot.encoders.NewMessageFcn=[];
robot.laser.NewMessageFcn=[];