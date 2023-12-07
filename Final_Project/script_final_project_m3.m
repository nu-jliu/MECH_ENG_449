% ME449 Robot Manipulation
% Allen Liu
% Final Project M3

close all
clear variables
clc

%% Test FeedbackControl function
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

%%%%%%%%%%%%%%%%%% Test Configurations %%%%%%%%%%%%%%%%%%%
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

kp = 0;
ki = 0;

Kp = kp*eye(6);
Ki = ki*eye(6);

Xerr_acc = [0; 0; 0; 0; 0; 0];

dt = 0.01;

[V_vec, ~, ~] = FeedbackControl(X, Xd, Xd_next, Xerr_acc, Kp, Ki, dt);
Jb = JacobianBody(Blist, thetalist);

H_mat_0 = KUKAHMatrix(0);
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
speed_vec = pinv(Je, 1e-4)*V_vec;
display(speed_vec)

%% Main program

%%%%%%%%%%%%%%%%% Define Config Constants %%%%%%%%%%%%%%%%
Tse_init = [
    0   0   1   0;
    0   1   0   0;
   -1   0   0   0.5;
    0   0   0   1
];

Tb0 = [
    1   0   0   0.1662;
    0   1   0   0;
    0   0   1   0.0026;
    0   0   0   1
];

M0e = [
    1   0   0   0.033;
    0   1   0   0;
    0   0   1   0.6546;
    0   0   0   1
];

q_init  = [1; 0; 0];
q_final = [0; -1; -pi/2];

Tsc_init  = TF_cube(q_init);
Tsc_final = TF_cube(q_final);

theta = deg2rad(135);

Tce_standoff = [
    cos(theta)  0   sin(theta)  0;
    0           1   0           0;
   -sin(theta)  0   cos(theta)  0.2;
    0           0   0           1
];

Tce_grasp = [
    cos(theta)  0   sin(theta)  0;
    0           1   0           0;
   -sin(theta)  0   cos(theta)  0;
    0           0   0           1
];

traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, 0.01, 1);

N = size(traj, 1);



for i = 1:N
    traj_curr = traj(i, :);
    Xd = TF_traj(traj_curr);
end

function Tsc = TF_cube(q)
    x = q(1, 1);
    y = q(2, 1);
    theta = q(3, 1);

    Tsc = [
        cos(theta) -sin(theta)  0   x;
        sin(theta)  cos(theta)  0   y;
        0           0           1   0;
        0           0           0   1
    ];
end

function X = TF_traj(traj)
    r11 = traj(1, 1);
    r12 = traj(1, 2);
    r13 = traj(1, 3);
    r21 = traj(1, 4);
    r22 = traj(1, 5);
    r23 = traj(1, 6);
    r31 = traj(1, 7);
    r32 = traj(1, 8);
    r33 = traj(1, 9);
    px  = traj(1, 10);
    py  = traj(1, 11);
    pz  = traj(1, 12);

    X = [
        r11     r12     r13     px;
        r21     r22     r23     py;
        r31     r32     r33     pz;
        0       0       0       1
    ];
end
