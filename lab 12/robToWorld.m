function absPose = robToWorld(relPose)
global robotPose;

th = relPose(3);
relPose(3) = 1;

absPose = pose(robotPose(1:3)).bToA() * relPose;
absPose(3) = atan2(sin(robotPose(3) + th), cos(robotPose(3) + th));

end