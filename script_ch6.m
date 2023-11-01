% ME449
% Allen Liu 
% Quiz ch6

close all
clear variables
clc

x0_vec = [1 1]';

x_vec = x0_vec;

for i = 1:10
    f_vec = func(x_vec);
    J_mat = jacobian(x_vec);

    fprintf('----- Iteration %d ----- \n', i)
    display(x_vec)

    dx_vec = J_mat\(-f_vec);
    display(J_mat)
    x_vec = x_vec + dx_vec;
    display(dx_vec)
    display(x_vec)
    f_new = func(x_vec);
    display(f_vec)
    x_mat(:, i) = x_vec;
    f_mat(:, i) = f_new;
end

display(x_mat)
display(f_mat)

T_sd = [-.585 -.811 0 .076;
    .811 -.585 0 2.608;
    0 0 1 0;
    0 0 0 1];

S1 = [0 0 1 0 0 0]';
S2 = [0 0 1 0 -1 0]';
S3 = [0 0 1 0 -2 0]';

S_mat = [S1 S2 S3];
M = [1 0 0 3;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

theta0_vec = [pi/4 pi/4 pi/4]';

[theta_vec, succ] = IKinSpace(S_mat, M, T_sd, theta0_vec, 1e-3, 1e-4);
display(theta_vec)

FKinSpace(M, S_mat, theta_vec)

function J = jacobian(x_vec)
    x = x_vec(1);
    y = x_vec(2);

    J = [2*x 0; 0 2*y];
end

function f_vec = func(x_vec)
    x = x_vec(1);
    y = x_vec(2);

    f_vec = [x^2-9 y^2-4]';
end