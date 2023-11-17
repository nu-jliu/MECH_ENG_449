% ME449 Robot Manipulation
% Allen Liu
% Final Project M1

close all
clear variables
clc

dt = 0.01;

T = 1;
t_vec = (0:dt:T)';
N = size(t_vec, 1);

config_mat = zeros(N + 1, 13);
config_vec = zeros(12, 1);
speed_vec = zeros(9, 1);
u_vec = [10 10 10 10]';
speed_vec(1:4, 1) = u_vec;

for i = 1:N
    config_mat(i, 1:12) = config_vec';

    config_vec = NextState(config_vec, speed_vec, dt, inf);
end

config_mat(end, 1:12) = config_vec';
writematrix(config_mat, 'm1_1.csv');

config_mat = zeros(N + 1, 13);
config_vec = zeros(12, 1);
speed_vec = zeros(9, 1);
u_vec = [-10 10 -10 10]';
speed_vec(1:4, 1) = u_vec;

for i = 1:N
    config_mat(i, 1:12) = config_vec';

    config_vec = NextState(config_vec, speed_vec, dt, inf);
end

config_mat(end, 1:12) = config_vec';
writematrix(config_mat, 'm1_2.csv');

config_mat = zeros(N + 1, 13);
config_vec = zeros(12, 1);
speed_vec = zeros(9, 1);
u_vec = [-10 10 10 -10]';
speed_vec(1:4, 1) = u_vec;

for i = 1:N
    config_mat(i, 1:12) = config_vec';

    config_vec = NextState(config_vec, speed_vec, dt, inf);
end

config_mat(end, 1:12) = config_vec';
writematrix(config_mat, 'm1_3.csv');