global robot
global robotPose
global map
global mapUpdatePercentage

robot = raspbot();
robot.startLaser();
robot.forksDown();
pause(1)

p0 = pose(8*0.0254, 8*0.0254, -pi/2.0); % initial pose estimate on field
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
plotDriving = false;
turnFeedbackTime = 4;

th = atan2(sin(pi/2 - robotPose(3)), cos(pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, turnFeedbackTime, plotDriving);
fol.execute(true, true)

hf = 0.1524;

pickDropObject(robot, vel, true, true, hf, 3*hf, 6*hf);

th = atan2(sin(-pi/2 - robotPose(3)), cos(-pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, 1.5, plotDriving);
fol.execute(true, true)

xf = 0.0254*21; yf = 0.0254*6; thf = -pi/2.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
traj.planVelocities(vel,true,true);
fol = trajectoryFollower(traj, true, 3, plotDriving);
fol.execute(true, true);

robot.forksDown();

refExtra = linearOrAngularTrajectory.moveRelDist(-0.08);
trajExtra = robotTrajectory(30, 0, robotPose(1:3), refExtra);
folExtra = trajectoryFollower(trajExtra, true, 0.0, true);
folExtra.execute(true, true)

th = atan2(sin(pi/2 - robotPose(3)), cos(pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, turnFeedbackTime, plotDriving);
fol.execute(true, true)

pickDropObject(robot, vel, true, true, 3*hf, 5*hf, 6*hf);

th = atan2(sin(-pi/2 - robotPose(3)), cos(-pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, turnFeedbackTime, plotDriving);
fol.execute(true, true);

xf = 0.0254*27; yf = 0.0254*6; thf = -pi/2.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
traj.planVelocities(vel,true,true);
fol = trajectoryFollower(traj, true, 1.5, plotDriving);
fol.execute(true, true);

robot.forksDown();

refExtra = linearOrAngularTrajectory.moveRelDist(-0.08);
trajExtra = robotTrajectory(30, 0, robotPose(1:3), refExtra);
folExtra = trajectoryFollower(trajExtra, true, 0.0, true);
folExtra.execute(true, true)

th = atan2(sin(pi/2 - robotPose(3)), cos(pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, turnFeedbackTime, plotDriving);
fol.execute(true, true)

pickDropObject(robot, vel, true, true, 5*hf, 7*hf, 6*hf);

th = atan2(sin(-pi/2 - robotPose(3)), cos(-pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, 1.5, plotDriving);
fol.execute(true, true)

xf = 0.0254*33; yf = 0.0254*6; thf = -pi/2.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
traj.planVelocities(vel,true,true);
fol = trajectoryFollower(traj, true, 1.5, plotDriving);
fol.execute(true, true);

robot.forksDown();

robot.encoders.NewMessageFcn=[];
robot.laser.NewMessageFcn=[];