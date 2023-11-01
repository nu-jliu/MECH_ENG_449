% MECH_ENG 449 Modern Robotics
% Allen Liu
% Homework 2

close all
clear variables
clc

%% Question 1
W1 = .109;  % m
W2 = .082;  % m
L1 = .425;  % m
L2 = .392;  % m
H1 = .089;  % m
H2 = .095;  % m

M = [-1 0 0 L1+L2;
      0 0 1 W1+W2;
      0 1 0 H1-H2;
      0 0 0 1];

Tsd = [ 0.7071  0    0.7071     -0.3;
        0.7071  0   -0.7071     -0.5;
             0  1         0      0.5;
             0  0         0        1];

B1 = [  0   1   0   W1+W2    0      L1+L2]';
B2 = [  0   0   1   H2      -L1-L2  0]';
B3 = [  0   0   1   H2      -L2     0]';
B4 = [  0   0   1   H2       0      0]';
B5 = [  0  -1   0  -W2       0      0]';
B6 = [  0   0   1   0        0      0]';

B_mat = [B1 B2 B3 B4 B5 B6];

%% Init variables
start_x = zeros(2, 1);
start_y = zeros(2, 1);
start_z = zeros(2, 1);

%% Short Iteration
fprintf('---------------------------------- Short Iteration ----------------------------------\n');

theta0_vec = [-9 36 -18 -112 0 88]';

T_init = FKinBody(M, B_mat, theta0_vec);
start_x(1) = T_init(1, 4);
start_y(1) = T_init(2, 4);
start_z(1) = T_init(3, 4);

display(theta0_vec)

[theta_vec, succ, ~, ea_short, el_short] = IKinBodyIterates(M, B_mat, Tsd, theta0_vec, 1000, 0.001, 0.0001, 'short_iterates');

if succ
    display(theta_vec)
end

%% Long Iteration

fprintf('---------------------------------- Long Iteration ----------------------------------\n');
theta0_vec = [0 0 0 0 0 0]';
display(theta0_vec)

T_init = FKinBody(M, B_mat, theta0_vec);
start_x(2) = T_init(1, 4);
start_y(2) = T_init(2, 4);
start_z(2) = T_init(3, 4);

[theta_vec, succ, ~, ea_long, el_long] = IKinBodyIterates(M, B_mat, Tsd, theta0_vec, 1000, 0.001, 0.0001, 'long_iterates');

if succ
    display(theta_vec)
end

%% Plotting

short_iter = readmatrix('short_iterates.csv');
long_iter  = readmatrix('long_iterates.csv');

n = size(long_iter, 1);

x1 = zeros(n, 1);
y1 = zeros(n, 1);
z1 = zeros(n, 1);


for i = 1:n
    thetalist = long_iter(i, :)';
    T = FKinBody(M, B_mat, thetalist);
    pos = T(1:3, 4);

    x1(i) = pos(1);
    y1(i) = pos(2);
    z1(i) = pos(3);
end


n = size(short_iter, 1);
x2 = zeros(n, 1);
y2 = zeros(n, 1);
z2 = zeros(n, 1);

for i = 1:n
    thetalist = short_iter(i, :)';
    T = FKinBody(M, B_mat, thetalist);
    pos = T(1:3, 4);

    x2(i) = pos(1);
    y2(i) = pos(2);
    z2(i) = pos(3);
end

end_x = Tsd(1, 4);
end_y = Tsd(2, 4);
end_z = Tsd(3, 4);

figure

plot3(x1, y1, z1,'b-', LineWidth=1.5)
hold on
plot3(x2, y2, z2, 'r-', LineWidth=1.5)

plot3(start_x, start_y, start_z, 'ro', MarkerSize=10, LineWidth=3)
plot3(end_x, end_y, end_z, 'kx', MarkerSize=10, LineWidth=3)

hold off
xlabel('$x$ [m]', Interpreter='latex')
ylabel('$y$ [m]', Interpreter='latex')  
zlabel('$z$ [m]', Interpreter='latex')
title('\textbf{Trajectory of the End-Effector position}', Interpreter='latex')
legend('Long Iteration', 'Short Iteration', 'Start', 'End')
grid minor

figure
hold on
plot(ea_short, LineWidth=2)
plot(ea_long, LineWidth=2)
hold off

title('Linear Error $\epsilon_v$', Interpreter='latex')
xlabel('Number of iterations', Interpreter='latex')
ylabel('Error $\epsilon$', Interpreter='latex')
legend('Short Iteration', 'Long Iteration', Interpreter='latex')
grid minor

figure
hold on
plot(el_short, LineWidth=2)
plot(el_long, LineWidth=2)
hold off

title('Angular Error $\epsilon_{\omega}$', Interpreter='latex')
xlabel('Number of iterations', Interpreter='latex')
ylabel('Error $\epsilon$', Interpreter='latex')
legend('Short Iteration', 'Long Iteration', Interpreter='latex')
grid minor