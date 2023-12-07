% ME449 Robot Manipulation
% Allen Liu
% Final Project All Together

close all
clear variables
clc

%% Define Config Constants
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

B1 = [0 0 1 0 0.033 0]';
B2 = [0 -1 0 -0.5076 0 0]';
B3 = [0 -1 0 -0.3526 0 0]';
B4 = [0 -1 0 -0.2176 0 0]';
B5 = [0 0 1 0 0 0]';

Blist = cat(2, B1, B2, B3, B4, B5);

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

q_init_new  = [1; 0; 0];
q_final_new = [0; 1; pi/2];

Tsc_init_new  = TF_cube(q_init_new);
Tsc_final_new = TF_cube(q_final_new);

%% Choose Initial Configuration
theta0_vec = zeros(5, 1);
% q0_vec = zeros(3, 1);
q0_vec = [0; 0; 0.2];
alpha0_vec = zeros(4, 1);

%% Generate Desired Path
k = 1;
dt = 0.01;
traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, dt, k);
traj_new = TrajectoryGenerator(Tse_init, Tsc_init_new, Tsc_final_new, Tce_grasp, Tce_standoff, dt, k);

%% Feedback Control

%%%%%%%%%% Best Control %%%%%%%%%%
max_speed = 10;
config_vec = [q0_vec; theta0_vec; alpha0_vec];
kp = 0.6;
ki = 0.0;

fprintf('--------------- BEST ---------------\n')
simulate(traj, config_vec, 'best', kp, ki, max_speed, dt, M0e, Blist, Tb0);
fprintf('--------------- NEW TASK ---------------\n')
simulate(traj_new, config_vec, 'newTask', kp, ki, max_speed, dt, M0e, Blist, Tb0);

kp = 0.3;
ki = 0.1;

fprintf('--------------- OVERSHOOT ---------------\n')
simulate(traj, config_vec, 'overshoot', kp, ki, max_speed, dt, M0e, Blist, Tb0);

%% Helper Functions
function [] = simulate(traj, config_vec, filename, kp, ki, max_speed, dt, M0e, Blist, Tb0)
    N = size(traj, 1);
    
    Kp = kp*eye(6);
    Ki = ki*eye(6);
    
    config_mat = zeros(N-1, 13);
    
    figure
    hold on
    set(gcf, 'color', 'w')
    grid minor
    xlabel('Time $t$ [s]', Interpreter='latex')
    ylabel('Error', Interpreter='latex')
    title('Error Twist $X_{err}$ vs. Time', Interpreter='latex')
    
    Xerr_mat = zeros(6, N-1);
    t_span = 0:dt:dt*(N-1);
    Xerr_acc = zeros(6, 1);
    
    fprintf('Generating animation csv file.\n')

    for i = 1:N-1
        %%%%%%%%%% Get e-e Twist %%%%%%%%%%
        q_vec = config_vec(1:3, :);
        theta_vec = config_vec(4:8, :);
    
        traj_curr = traj(i, :);
        traj_next = traj(i+1, :);
        
        Xd      = TF_traj(traj_curr);
        Xd_next = TF_traj(traj_next);
    
        gripper_state = traj_curr(1, end);
    
        T0e = FKinBody(M0e, Blist, theta_vec);
        Tsb = TF_chais(q_vec);
        Tse = Tsb*Tb0*T0e;
    
        X = Tse;
        [Ve_vec, Xerr_vec, Xerr_acc] = FeedbackControl(X, Xd, Xd_next, Xerr_acc, Kp, Ki, dt);
        
        %%%%%%%%%% Calculate the control vector %%%%%%%%%%
        Jb = JacobianBody(Blist, theta_vec);
    
        H0_mat = KUKAHMatrix(0);
        F_mat  = pinv(H0_mat);
        
        m   = size(F_mat, 2);
        m_0 = zeros(1, m);
        
        F6_mat = [m_0; m_0; F_mat; m_0];
        
        
        Te0 = TransInv(T0e);

        T0b = TransInv(Tb0);
        
        Teb = Te0*T0b;
        AdTeb = Adjoint(Teb);
        Jbase = AdTeb*F6_mat;
        
        Je = [Jbase Jb];
        speed_vec = pinv(Je, 1e-4)*Ve_vec;
    
        Xerr_mat(:, i) = Xerr_vec;
    
        %%%%%%%%%% Simulate the result %%%%%%%%%%
        config_vec = NextState(config_vec, speed_vec, dt, max_speed);
        config_mat(i, :) = [config_vec' gripper_state];
    end
    writematrix(config_mat, sprintf('%s/config.csv', filename))
    fprintf('Writing error plot data.\n')

    writematrix(Xerr_mat, sprintf('%s/error.csv', filename))

    axis([0 dt*N min(Xerr_mat, [], 'all') max(Xerr_mat, [], 'all')])
    plot(t_span(1:i), Xerr_mat(1, :), LineWidth=1.5)
    plot(t_span(1:i), Xerr_mat(2, :), LineWidth=1.5)
    plot(t_span(1:i), Xerr_mat(3, :), LineWidth=1.5)
    plot(t_span(1:i), Xerr_mat(4, :), LineWidth=1.5)
    plot(t_span(1:i), Xerr_mat(5, :), LineWidth=1.5)
    plot(t_span(1:i), Xerr_mat(6, :), LineWidth=1.5)

    legend( ...
        '$e_{\varphi}$ [rad]', ...
        '$e_{\theta}$ [rad]', ...
        '$e_{\psi}$ [rad]', ...
        '$e_x$ [m]', ...
        '$e_y$ [m]', ...
        '$e_z$ [m]', ...
        Interpreter='latex' ...
    )

    saveas(gcf, sprintf('%s/error_plot.jpg', filename))
    fprintf('Done.\n')
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

function Tsb = TF_chais(q_vec)
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
