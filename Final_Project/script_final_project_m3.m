% ME449 Robot Manipulation
% Allen Liu
% Final Project M3

close all
clear variables
clc

Tb0 = [
    1   0   0   0.1662;
    0   1   0   0;
    0   0   1   0.0026;
    0   0   0   1
];

M0e = [
    1   0   0   0.033;
    0   1   0   0;
    0   0   1   0.6536;
    0   0   0   1
];

B1 = [0 0 1 0 0.033 0]';
B2 = [0 -1 0 -0.5076 0 0]';
B3 = [0 -1 0 -0.3526 0 0]';
B4 = [0 -1 0 -0.2176 0 0]';
B5 = [0 0 1 -0.2176 0 0]';

Blist = cat(2, B1, B2, B3, B4, B5);

thetalist = [0 0 0.2 -1.6 0]';

Xd = [
    0   0   1   0.5;
    0   1   0   0;
   -1   0   0   0.5;
   0    0   0   1
];

Xd_next = [
    0   0   1   0.6;
    0   1   0   0;
   -1   0   0   0.3;
    0   0   0   1
];

X = [
    0.170   0   0.985   0.387;
    0       1   0       0;
   -0.985   0   0.170   0.570;
    0       0   0       1
];
Kp = 0*eye(6);
Ki = 0*eye(6);

dt = 0.01;

FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt);
Jb = JacobianBody(Blist, thetalist);

H_mat_0 = HMatrix(0);
F_mat   = pinv(H_mat_0);

m   = size(F_mat, 2);
m_0 = zeros(1, m);

F6_mat = [m_0; m_0; F_mat; m_0];
T0e = FKinBody(M0e, Blist, thetalist);

Te0 = TransInv(T0e);
T0b = TransInv(Tb0);

Teb = Te0*T0b;
AdTeb = Adjoint(Teb);
Jbase = AdTeb*F6_mat;

Je = [Jbase Jb];
display(Je)