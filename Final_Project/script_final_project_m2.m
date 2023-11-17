% ME449 Robot Manipulation
% Allen Liu
% Final Project M2

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

q0_vec = [0; 0; 0];
Tsb_init = TF_robot(q0_vec);
Tse_init = Tsb_init*Tb0*M0e;

Tsc_init = [
    1   0   0   1;
    0   1   0   0;
    0   0   1   0.025;
    0   0   0   1
];
Tsc_goal = [
    0   1   0   0;
    -1  0   0   -1;
    0   0   1   0.025;
    0   0   0   1
];

Tce_standoff = [
   -1   0   0   0;
    0   1   0   0;
    0   0  -1   0.01;
    0   0   0   1
];

config_mat = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal, 1, Tce_standoff, 10);
writematrix(config_mat, 'm2.csv')

function Tsb = TF_robot(q_vec)
    phi = q_vec(1);
    x   = q_vec(2);
    y   = q_vec(3);

    Tsb = [
        cos(phi) -sin(phi) 0 x;
        sin(phi)  cos(phi) 0 y;
        0         0        1 .0963;
        0         0        0 1
    ];
end