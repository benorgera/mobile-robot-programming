global robot

robot = raspbot("raspbot9");
robot.forksDown();
robot.startLaser();
pause(5)

robFrontOffset = 0.06;
objFaceOffset = 0.019;
extraOffset=0.05; % how far off the pallet our trajectory should lead
pushOffset = 0.01; % extra drive to scoop pallet
thetaSoftening = 0.85; % don't make hard turns
vel = 0.2;
pickupVel = 0.05;
n = 0;
runs = 3;

while n<runs % acquired a pallet 3 times
    
    tmpRangeImage = rangeImage(robot, 1, true);
    [x,y,th] = tmpRangeImage.findLineCandidate(true);
    if (th == 0)&&(x == 0)&&(y == 0)
        pause(0.2)
        continue
    end
    n = n + 1;
    robot.stopLaser();
    
    pause(3);
    
    acqPoseVec = MRPLsystem.acquisitionPose([x; y; thetaSoftening * th], ...
        extraOffset, true);
    
    xf = acqPoseVec(1);
    yf = acqPoseVec(2);
    thf = acqPoseVec(3);
    
    traj1 = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1);
    traj1.planVelocities(vel,true,true);
    traj2 = cubicSpiralTrajectory.planTrajectory(extraOffset + ...
        pushOffset,0,0,1);
    traj2.planVelocities(pickupVel,true,true);
    traj3 = cubicSpiralTrajectory.planTrajectory(extraOffset + ...
        pushOffset,0,0,-1);
    traj3.planVelocities(pickupVel,true,true);
    traj4 = cubicSpiralTrajectory.planTrajectory(xf,yf,-thf,-1);
    traj4.planVelocities(vel,true,true);
    
    folTo = trajectoryFollower(trajectoryString([ traj1 traj2 ]));
    folFrom = trajectoryFollower(trajectoryString([ traj3 traj4 ]));
    
    folTo.execute(true);
    
    robot.forksUp();
    pause(1);
    robot.forksDown();
    
    folFrom.execute(true);
    
    if n < runs
        robot.startLaser();
        pause(10);
    else
        beep
    end
end
robot.stopLaser();
robot.shutdown();

