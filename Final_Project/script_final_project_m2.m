% ME449 Robot Manipulation
% Allen Liu
% Final Project M2

% How to run the code: 
% Go to the directory containing the file script_final_project_m2.m, then 
% in MATLAB Command Window, type:
%
% script_final_project_m2
%
% It should run the code, and save the m2.csv file that could be used for
% simulation in Coppeliasim. 

%% Clear environment
close all
clear variables
clc

%% Define environment constants
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

%% Generate trajectory
fprintf('Generating csv file ...\n')
config_mat = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, 10);
writematrix(config_mat, 'm2.csv')
fprintf('csv file generated\n')

%% Function used for calculating body configuration
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