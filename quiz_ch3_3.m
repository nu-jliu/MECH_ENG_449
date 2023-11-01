% quiz_ch3.3.m
% Allen Liu
% ME449

close all
clear variables
clc

%% Q1
T_sa = [0 -1  0 0;
        0  0 -1 0;
        1  0  0 1;
        0  0  0 1];
display(T_sa)

%% Q2
T_sb = [1  0 0 0;
        0  0 1 2;
        0 -1 0 0;
        0  0 0 1];
T_sb_inv = inv(T_sb);
display(T_sb_inv)

%% Q3
T_ab = T_sa\T_sb;
display(T_ab)

%% Q5
p_b = [1 2 3]';
p_s = T_sb*[p_b; 1];
p_s = p_s(1:3);
display(p_s)

%% Q7
R_sa = T_sa(1:3, 1:3);
p_sa = T_sa(1:3, 4);
p_sa_skw = VecToso3(p_sa);

ADT_sa = [R_sa, zeros(3);
    p_sa_skw*R_sa, R_sa];
V_s = [3 2 1 -1 -2 -3]';
V_a = ADT_sa\V_s;
display(V_a)

%% Q8
if isequal(R_sa, eye(3))
    theta = norm(p_sa);
else
    theta = norm(MatrixLog3(R_sa));
end

display(theta)

%% Q9
S_theta = [0 1 2 3 0 0]';
% T = 
theta = norm(S_theta(1:3));
omega = S_theta(1:3, :)/theta;
v     = S_theta(4:6, :)/theta;
r11 = MatrixExp3(VecToso3(omega));

G = eye(3)*theta + (1-cos(theta))*VecToso3(omega)+(theta-sin(theta))*VecToso3(omega)^2;
display(G*v)


MatrixExp6(VecTose3([0 1 2 3 0 0]'))

%% Q10
T_bs = inv(T_sb);
R_bs = T_bs(1:3, 1:3);
p_bs = T_bs(1:3, 4);
p_bs_skw = VecToso3(p_bs);

ADT_bs = [R_bs, zeros(3);
    p_bs_skw*R_bs, R_bs];

F_b = [1 0 0 2 1 0]';
F_s = ADT_bs'*F_b;
display(F_s)

%% Q11
disp('Q11')
T = [0 -1 0 3;
    1 0 0 0;
    0 0 1 1;
    0 0 0 1];
TransInv(T)
% inv(T)

%% Q12
disp('Q12')
VecTose3([1 0 0 0 2 3]')

%% Q13
disp('Q13')
q = [0 0 2]';
h = 1;
s = [1 0 0]';

ScrewToAxis(q, s, h)

%% Q14
disp('Q14')
S_theta_mat = [0 -1.5708 0 2.3562;
    1.5708 0 0 -2.3562;
    0 0 0 1;
    0 0 0 0];

MatrixExp6(S_theta_mat)
%% Q15
disp('Q15')
T = [0 -1 0 3;
    1 0 0 0;
    0 0 1 1;
    0 0 0 1];
MatrixLog6(T)