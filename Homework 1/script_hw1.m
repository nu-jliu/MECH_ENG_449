% ME449
% Homrwork l
% Allen Liu

close all
clear variables
clc

omega_1 = [0 0 1]';
omega_2 = [0 1 0]';
omega_3 = [0 1 0]';
omega_4 = [0 1 0]';
omega_5 = [0 0 -1]';
omega_6 = [0 1 0]';


R_13 = [[-0.7071, 0, -0.7071]; [0, 1, 0]; [0.7071, 0, -0.7071]];
R_s2 = [[-0.6964, 0.1736, 0.6964]; [-0.1228, -0.9848, 0.1228]; [0.7071, 0, 0.7071]];
R_25 = [[-0.7566, -0.1198, -0.6428]; [-0.1564, 0.9877, 0]; [0.6348, 0.1005, -0.7661]];
R_12 = [[0.7071, 0, -0.7071]; [0, 1, 0]; [0.7071, 0, 0.7071]];
R_34 = [[0.6428, 0, -0.7660]; [0, 1, 0]; [0.7660, 0, 0.6428]];
R_s6 = [[0.9418, 0.3249, -0.0859]; [0.3249, -0.9456, -0.0151]; [-0.0861, -0.0136, -0.9962]];
R_6b = [[-1, 0, 0]; [0, 0, 1]; [0, 1, 0]];

%% theta1
R_21 = R_12';
R_s1 = R_s2*R_21;

theta(1, :) = get_theta(so3ToVec(MatrixLog3(R_s1))./omega_1);

%% theta2
theta(2, :) = get_theta(so3ToVec(MatrixLog3(R_12))./omega_2);

%% theta3
R_23 = R_21*R_13;

theta(3, :) = get_theta(so3ToVec(MatrixLog3(R_23))./omega_3);

%% theta4
theta(4, :) = get_theta(so3ToVec(MatrixLog3(R_34))./omega_4);

%% theta5
R_43 = R_34';
R_31 = R_13';
R_45 = R_43*R_31*R_12*R_25;

theta(5, :) = get_theta(so3ToVec(MatrixLog3(R_45))./omega_5);


%% theta6
R_52 = R_25';
R_2s = R_s2';
R_56 = R_52*R_2s*R_s6;

theta(6, :) = get_theta(so3ToVec(MatrixLog3(R_56))./omega_6);

display(theta)

% R_sb
R_sb = R_s1*R_12*R_23*R_34*R_45*R_56*R_6b;
display(R_sb)

%% Functions
function [theta] = get_theta(theta_vec)
    for i = 1:3
        if ~(isnan(theta_vec(i)) || isinf(theta_vec(i)))
            theta = theta_vec(i);
            return
        end
    end
end