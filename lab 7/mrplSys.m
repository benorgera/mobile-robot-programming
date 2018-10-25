%% challenge task

vel = 0.2;

xf = 0.3048; yf = 0.3048; thf = 0.0;
traj = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1);
traj.planVelocities(vel,true,false);

xf = -0.6096; yf = -0.6096; thf = -pi()/2.0;
traj1 = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1);
traj1.planVelocities(vel,false,false);

xf = -0.3048; yf = 0.3048; thf = pi()/2.0;
traj2 = cubicSpiralTrajectory.planTrajectory(xf,yf,thf,1);
traj2.planVelocities(vel,false,true);

trajString = trajectoryString([traj traj1 traj2]);

fol = trajectoryFollower(trajString);     

fol.execute(false); 