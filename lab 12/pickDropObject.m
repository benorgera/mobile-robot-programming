function pickDropObject(robot, vel, axisInterval, isX, low, high, altLow)
global robotPose

plotDriving = false;
softening = 0.65;

robot.forksDown()

if nargin < 3
    sailPoseRel = findSailRelative(robot);
else
    sailPoseRel = findSailRelative(robot, axisInterval, isX, low, high, altLow);
end

acqDist = 0.2;

acqPoseVecRel = MRPLsystem.acquisitionPose(sailPoseRel, acqDist, true);
acqPoseVecAbs = robToWorld(acqPoseVecRel);

turn = atan2(acqPoseVecRel(2), acqPoseVecRel(1));

if (abs(turn) > pi / 4)
    ref = linearOrAngularTrajectory.turnRelAngle(turn);
    traj = robotTrajectory(201, 0, robotPose(1:3), ref);
    fol = trajectoryFollower(traj, true, 1.5, plotDriving);
    fol.execute(true, true)
end

xf = acqPoseVecAbs(1); yf = acqPoseVecAbs(2); thAcq = acqPoseVecAbs(3);
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thAcq,1,true);
traj.planVelocities(vel,false,true);

fol = trajectoryFollower(traj, true, 3, plotDriving);
fol.execute(true, true);

pause(0.1);

sailPoseRel2 = findSailRelative(robot, axisInterval, isX, low, high, altLow);
acqPoseVecRel1 = MRPLsystem.acquisitionPose(sailPoseRel2, 0.0, true);

acqPoseVecRel1(2) = acqPoseVecRel1(2) * softening;

sailPoseAbs2 = robToWorld(acqPoseVecRel1);

traj1 = cubicSpiralTrajectory.planTrajectory(sailPoseAbs2(1),...
    sailPoseAbs2(2),sailPoseAbs2(3),1,true);
traj1.planVelocities(vel * 0.4,false,false);
fol1 = trajectoryFollower(traj1, true, 2, plotDriving);
fol1.execute(true, true);

refExtra = linearOrAngularTrajectory.moveRelDist(0.015);
trajExtra = robotTrajectory(30, 0, robotPose(1:3), refExtra);
folExtra = trajectoryFollower(trajExtra, true, 0.0, true);
folExtra.execute(true, true)

robot.forksUp()
end