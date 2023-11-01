% ME449
% Allen Liu
% Quiz ch5

close all
clear variables
clc

L1 = 1;
L2 = 1;
L3 = 1;
L4 = 1;

theta1 = 0;
theta2 = 0;
theta3 = pi/2;
theta4 = -pi/2;
s4 = sin(theta4);
s34 = sin(theta3+theta4);
s234 = sin(theta2+theta3+theta4);
c4 = cos(theta4);
c34 = cos(theta3+theta4);
c234 = cos(theta2+theta3+theta4);

Fb_vec = [0 0 10 10 10 0]';
Jb = [1 1 1 1;
    L3*s4+L2*s34+L1*s234 L3*s4+L2*s34 L3*s4 0;
    L4+L3*c4+L2*c34+L1*c234 L4+L3*c4+L2*c34 L4+L3*c4 L4];

tau_vec = Jb'*Fb_vec(3:5, :);
display(tau_vec)

S1 = [0 0 1 0 0 0]';
S2 = [1 0 0 0 2 0]';
S3 = [0 0 0 0 1 0]';

S_mat = [S1 S2 S3];
theta_vec = [pi/2 pi/2 1]';
JacobianSpace(S_mat, theta_vec)

B1 = [0 1 0 3 0 0]';
B2 = [-1 0 0 0 3 0]';
B3 = [0 0 0 0 0 1]';
B_mat = [B1 B2 B3];

JacobianBody(B_mat, theta_vec)

Jb = [0 -1 0 0 -1 0 0;
    0 0 1 0 0 1 0;
    1 0 0 1 0 0 1;
    -.105 0 .006 -.045 0 .006 0;
    -.889 .006 0 -.844 .006 0 0;
    0 -.105 .889 0 0 0 0];
Jb_v = Jb(4:6, :);

A_mat = Jb_v*Jb_v';
[e, v] = eig(A_mat)