close all
clear variables
clc

QuinticTimeScaling(5, 3)

X_start = [1 0 0 0;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];

X_end = [0 0 1 1;
         1 0 0 2;
         0 1 0 3;
         0 0 0 1];

Tf = 10;

traj = ScrewTrajectory(X_start, X_end, Tf, 10, 3);

traj{end-1}

traj = CartesianTrajectory(X_start, X_end, Tf, 10, 5);

traj{end-1}