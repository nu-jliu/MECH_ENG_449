% ME449 Robotic Manipulation
% Allen Liu
% Assignment 3

close all
clear variables
clc

M01 = [ 1   0   0   0; 
        0   1   0   0; 
        0   0   1   0.089159; 
        0   0   0   1];
M12 = [ 0   0   1   0.28; 
        0   1   0   0.13585; 
        -1  0   0   0; 
        0   0   0   1];
M23 = [ 1   0   0   0; 
        0   1   0 -0.1197; 
        0   0   1   0.395; 
        0   0   0   1];
M34 = [ 0   0   1   0; 
        0   1   0   0; 
        -1  0   0   0.14225; 
        0   0   0   1];
M45 = [ 1   0   0   0; 
        0   1   0   0.093; 
        0   0   1   0; 
        0   0   0   1];
M56 = [ 1   0   0   0; 
        0   1   0   0;
        0   0   1   0.09465;
        0   0   0   1];
M67 = [ 1   0   0   0; 
        0   0   1   0.0823; 
        0   -1  0   0; 
        0   0   0   1];
G1 = diag([0.010267495893 0.010267495893 0.00666 3.7 3.7 3.7]);
G2 = diag([0.22689067591 0.22689067591 0.0151074 8.393 8.393 8.393]);
G3 = diag([0.049443313556 0.049443313556 0.004095 2.275 2.275 2.275]);
G4 = diag([0.111172755531 0.111172755531 0.21942 1.219 1.219 1.219]);
G5 = diag([0.111172755531 0.111172755531 0.21942 1.219 1.219 1.219]);
G6 = diag([0.0171364731454 0.0171364731454 0.033822 0.1879 0.1879 0.1879]);
Glist = cat(3, G1, G2, G3, G4, G5, G6);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67); 
Slist = [0         0         0         0        0        0;
         0         1         1         1        0        1;
         1         0         0         0       -1        0;
         0 -0.089159 -0.089159 -0.089159 -0.10915 0.005491;
         0         0         0         0  0.81725        0;
         0         0     0.425   0.81725        0  0.81725];

theta_vec  = [0 0 0 0 0 0]';
dtheta_vec = [0 0 0 0 0 0]';
g_vec = [0 0 -9.8]';

t = 5;
dt = 1e-2;

%% Part 1. Free fall
fprintf('---------- fall ----------\n')
[theta_mat, ~] = Puppet(theta_vec, dtheta_vec, g_vec, Mlist, Slist, Glist, t, dt, 0, 0, [0 0 0]', 0);
writematrix(theta_mat, 'free_fall_a.csv')

fprintf('---------- fall dt2 ----------\n')
[theta_mat, ~] = Puppet(theta_vec, dtheta_vec, g_vec, Mlist, Slist, Glist, t, 1e-3, 0, 0, [0 0 0]', 0);
writematrix(theta_mat, 'free_fall_b.csv')
%% Part 2. Damping
fprintf('---------- daming ----------\n')
zeta = 3;

[theta_mat, ~] = Puppet(theta_vec, dtheta_vec, g_vec, Mlist, Slist, Glist, t, dt, zeta, 0, [0 0 0]', 0);
writematrix(theta_mat, 'damping_a.csv')

fprintf('---------- daming 2 ----------\n')
zeta = -5e-3;

[theta_mat, ~] = Puppet(theta_vec, dtheta_vec, g_vec, Mlist, Slist, Glist, t, 0.05, zeta, 0, [0 0 0]', 0);
writematrix(theta_mat, 'damping_b.csv')

%% Part 3. Spring
fprintf('---------- spring ----------\n')
t = 10;
K = 100;
zeta = 1;
g_vec = [0 0 0]';
restlength = 0;

[theta_mat, ~] = Puppet(theta_vec, dtheta_vec, g_vec, Mlist, Slist, Glist, t, dt, zeta, K, [1 1 1]', restlength);
writematrix(theta_mat, 'spring_b.csv')

zeta = 0;
fprintf('---------- spring no damping ----------\n')
[theta_mat, ~] = Puppet(theta_vec, dtheta_vec, g_vec, Mlist, Slist, Glist, t, dt, zeta, K, [1 1 1]', restlength);
writematrix(theta_mat, 'spring_a.csv')