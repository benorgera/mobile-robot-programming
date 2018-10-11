%% figure 8 sim test

% ref = trapezoidalStepReferenceControl(0.25, 0.2, 1.5, 1, 0.25);
ref = figure8ReferenceControl(3,1,0.2);
traj = robotTrajectory(1000, 0, [0;0;0], ref);
fol = trajectoryFollower(traj);
fol.execute(false);