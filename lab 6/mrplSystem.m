%% challenge task

xf = 0.3048; yf = 0.3048; thf = 0.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1); 
traj.planVelocities(0.2);
fol = trajectoryFollower(traj);

% 
% xf = -0.6096; yf = -0.6096; thf = -pi()/2.0;
% traj1 = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1); 
% traj1.planVelocities(0.2);
% fol1 = trajectoryFollower(traj1, true);
% 
% xf = -0.3048; yf = 0.3048; thf = pi()/2.0;
% traj2 = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1); 
% traj2.planVelocities(0.2);
% fol2 = trajectoryFollower(traj2, true);

fol.execute(false);
% fol1.execute(true);
% fol2.execute(true);

% don't need to multiply by 10 when data comes from encoders, but every
% time an w is computed from vl and vr in a reference, it has to be
% multiplied by 10!! (this occurs in controller2, controller1, and
% cubicSpiral)