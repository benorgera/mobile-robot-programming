global robot

robot = raspbot("raspbot9");
robot.startLaser();
robot.forksDown();
pause(5)

numImages = 3;
robFrontOffset = 0.06;
objFaceOffset = 0.019;
% moreOffset = 0.025;
moreOffset=0;
flag = false;
vel = 0.2;
n = 0;
while n<3
    
    tmpRangeImage = rangeImage(robot, 3, 1, false);
    [x,y,th] = tmpRangeImage.findLineCandidate(true);
    if (th == 0)&&(x == 0)&&(y == 0)
        continue
    end
    n = n + 1;
    
    pause(3);
    objPoseVec = [x; y; th];
    acqPoseVec = MRPLsystem.acquisitionPose(objPoseVec);
    
    xf = acqPoseVec(1);
    yf = acqPoseVec(2);
    thf = acqPoseVec(3);
    
    traj = cubicSpiralTrajectory.planTrajectory(x-robFrontOffset,y,th,1);
    traj.planVelocities(vel,true,true);
    
    fol = trajectoryFollower(traj);
    
    fol.execute(true);
    
    robot.forksUp();
    
    pause(1);
    robot.forksDown();
    traj = cubicSpiralTrajectory.planTrajectory((x-robFrontOffset),(y),th,-1);
    traj.planVelocities(vel,true,true);
    
    fol = trajectoryFollower(traj);
    
    fol.execute(true);
    pause(15);
    
end
robot.shutdown();

