% ME449
% Allen Liu
% Quiz 13

close all
clear variables
clc

H0 = H_matrix(0);
Vb = [1 0 0]';
u1 = H0*Vb;

display(u1)

Vb = [1 2 3]';
u2 = H0*Vb;
display(u2)

u_max = 10;
display(u_max/H0(1, 2))

function [H_mat] = H_matrix(phi)
    l = 2;
    w = 2;

    x_vec = [l -l -l l]';
    y_vec = [w w -w -w]';
    gamma_vec = [0 0 0 0]';
    beta_vec = [-pi/4 pi/4 3*pi/4 -3*pi/4]';
    r_vec = 0.25*ones(4, 1);

    H_mat = [
        x_vec.*sin(beta_vec + gamma_vec) - y_vec.*cos(beta_vec + gamma_vec) ...
        cos(beta_vec + gamma_vec + phi) ...
        sin(beta_vec + gamma_vec + phi) ...
    ] ./ (r_vec.*cos(gamma_vec));
end