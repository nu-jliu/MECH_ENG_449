
% ME449
% Quiz ch3.1-2
% Allen Liu

close all
clear variables
clc

%% Q1
R_sa = [0 1 0;
        0 0 1;
        1 0 0];
display(R_sa)

%% Q2
R_sb = [1  0 0; 
        0  0 1;
        0 -1 0];
R_sb_inv = inv(R_sb);
display(R_sb_inv)

%% Q3
R_ab = R_sa'*R_sb;
display(R_ab)

%% Q5
p_b = [1 2 3]';
p_s = R_sb*p_b;
display(p_s)

%% Q7
omega_s = [3 2 1]';
omega_a = R_sa'*omega_s;
display(omega_a)

%% Q8
trace_sa = R_sa(1,1)+R_sa(2,2)+R_sa(3,3);
if isequal(R_sa, eye(3))
    theta = 0;
elseif trace_sa == -1
    theta = pi;
else
    theta = acos(1/2*(trace_sa-1));
end
display(theta)
theta = norm(so3ToVec(MatrixLog3(R_sa)));
display(theta)

%% Q9
omega = [1 2 0]';
theta = norm(omega);
omega = omega/theta;
omega_mat = VecToso3(omega);
R_mat = eye(3)+sin(theta)*omega_mat + (1-cos(theta))*omega_mat*omega_mat;
display(R_mat)

%% Q10
omega_mat_skw = VecToso3([1;2;0.5]);
display(omega_mat_skw)

%% Q11
R_mat = [ 0      0.5    -1; 
         -0.5    0       2; 
          1     -2       0];
R_mat_exp = MatrixExp3(R_mat);
display(R_mat_exp)