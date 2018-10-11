%% challenge task

xf = 0.3048; yf = 0.3048; thf = 0.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1); 
traj.planVelocities(0.2);
fol = trajectoryFollower(traj, true);
fol.execute();

% don't need to multiply by 10 when data comes from encoders, but every
% time an w is computed from vl and vr in a reference, it has to be
% multiplied by 10!! (this occurs in controller2, controller1, and
% cubicSpiral)

% xf = -0.6096; yf = -0.6096; thf = -pi()/2.0;
% traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf); 
% traj.planVelocities(0.2);
% fol1 = trajectoryFollower(traj, true);
% 
% xf = -0.3048; yf = 0.3048; thf = pi()/2.0;
% traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf); 
% traj.planVelocities(0.2);
% fol2 = trajectoryFollower(traj, true);
% fol.execute();