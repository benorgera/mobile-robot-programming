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
p2 = [ 8*f ; 0];
p3 = [ 8*f ; 6*f];
p4 = [ 0 ; 6*f];
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

findVel = 0.17;
dropVel = 0.25;
scanVel = 0.25;
plotDriving = false;
turnFeedbackTime = 4;
tol = 1.5;
f = 0.3048; %cm

dropNum = 0;
iteration = 0;

% back up
refExtra = linearOrAngularTrajectory.moveRelDist(-0.15);
trajExtra = robotTrajectory(30, 0, robotPose(1:3), refExtra);
folExtra = trajectoryFollower(trajExtra, true, 0.0, true);
folExtra.execute(true, true)

% turn around
th = atan2(sin(pi/2 - robotPose(3)), cos(pi/2 - robotPose(3)));
ref = linearOrAngularTrajectory.turnRelAngle(th);
traj = robotTrajectory(201, 0, robotPose(1:3), ref);
fol = trajectoryFollower(traj, true, turnFeedbackTime, plotDriving);
fol.execute(true, true)

while dropNum < 7
    
    scanLocation = mod(iteration, 3);
    
    % drive to scan location
    if scanLocation == 0
        xf = 2.5 * f; yf = 3 * f; thf = pi/2.0;
        traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
        traj.planVelocities(scanVel,true,true);
        fol = trajectoryFollower(traj, true, 3, plotDriving);
        fol.execute(true, true);
    elseif scanLocation == 1
        xf = 5.5 * f; yf = 3 * f; thf = pi/2.0;
        traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
        traj.planVelocities(scanVel,true,true);
        fol = trajectoryFollower(traj, true, 3, plotDriving);
        fol.execute(true, true);
    else
        xf = 4 * f; yf = 3 * f; thf = 0;
        traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
        traj.planVelocities(scanVel,true,true);
        fol = trajectoryFollower(traj, true, 3, plotDriving);
        fol.execute(true, true);
    end
    
    pause(0.15);
    
    rangeImg = rangeImage(robot, 1, true);
    
    foundSail = false;
    
    % scan the location, choose a sail and pick it up
    if scanLocation < 2
        if scanLocation == 0
            loop = 1:4;
            lowX = f / 2;
            highX = 1.5 * f;
        else
            loop = 4:7;
            lowX = 3.5 * f;
            highX = 4.5 * f;
        end
        
        for i=loop
            
            [x,y,th] = rangeImg.findLineCandidate(true, true, true, lowX, highX, 5*f);
            
            % get sail
            pickDropObject(robot, findVel, true, true, lowX, highX, 5*f);
            
            if (th == 0)&&(x == 0)&&(y == 0)
                pause(0.2)
                disp("couldn't find sail at pallet")
                disp(i)
            else
                foundSail = true;
                break
            end
            
            lowX = lowX + f;
            highX = highX + f;
        end
    else
        for i=1:3
            
            lowY = 1.5 * f;
            highY = 2.5 * f;
            
            [x,y,th] = rangeImg.findLineCandidate(true, true, false, lowY, highY, 6*f);
            
            % get sail
            pickDropObject(robot, findVel, true, false, lowY, highY, 6*f);
            
            if (th == 0)&&(x == 0)&&(y == 0)
                pause(0.2)
                disp("couldn't find sail at pallet")
                disp("i")
            else
                foundSail = true;
                break
            end
            
            lowY = lowY + f;
            highY = highY + f;
        end
    end
    
    if ~foundSail
        disp("could't find any pallets")
        iteration = iteration + 1;
        pause(0.2)
        continue
    end
    
    % rotate towards drop
    th = atan2(sin(-pi/2 - robotPose(3)), cos(-pi/2 - robotPose(3)));
    ref = linearOrAngularTrajectory.turnRelAngle(th);
    traj = robotTrajectory(201, 0, robotPose(1:3), ref);
    fol = trajectoryFollower(traj, true, 1.5, plotDriving);
    fol.execute(true, true)
    
    % go to drop
    xf = 1.5 * f + dropNum * 0.75 * f; yf = f; thf = -pi/2.0;
    traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1,true);
    traj.planVelocities(dropVel,true,true);
    fol = trajectoryFollower(traj, true, 3, plotDriving);
    fol.execute(true, true);
    
    robot.forksDown();
    dropNum = dropNum + 1;
    
    % back up
    refExtra = linearOrAngularTrajectory.moveRelDist(-0.08);
    trajExtra = robotTrajectory(30, 0, robotPose(1:3), refExtra);
    folExtra = trajectoryFollower(trajExtra, true, 0.0, true);
    folExtra.execute(true, true)
    
    % rotate towards scan
    th = atan2(sin(pi/2 - robotPose(3)), cos(pi/2 - robotPose(3)));
    ref = linearOrAngularTrajectory.turnRelAngle(th);
    traj = robotTrajectory(201, 0, robotPose(1:3), ref);
    fol = trajectoryFollower(traj, true, turnFeedbackTime, plotDriving);
    fol.execute(true, true)
end

robot.encoders.NewMessageFcn=[];
robot.laser.NewMessageFcn=[];