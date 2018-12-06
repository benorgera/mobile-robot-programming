function relPose = worldToRob(absPose)
global robotPose;

th = absPose(3);
absPose(3) = 1;

relPose = pose(robotPose(1:3)).aToB() * absPose;
relPose(3) = atan2(sin(th - robotPose(3)), cos(th - robotPose(3)));

end