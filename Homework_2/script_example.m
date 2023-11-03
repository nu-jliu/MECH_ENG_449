close all
clear variables
clc

Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
T = [[0, 1, 0, -5]; [1, 0, 0, 4]; [0, 0, -1, 1.6858]; [0, 0, 0, 1]];
thetalist0 = [1.5; 2.5; 3];
eomg = 0.01;
ev = 0.001;
max_iter = 30;
filename = 'thetalist';
[thetalist, success, theta_mat, err_ang, err_lin] = IKinBodyIterates(Blist, M, T, thetalist0, max_iter, eomg, ev, filename)