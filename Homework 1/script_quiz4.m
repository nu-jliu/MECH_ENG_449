% ME449
% Quiz ch4
% Allen Liu

L = 1;
M_mat = [1 0 0 (2+sqrt(3))*L;
         0 1 0 0;
         0 0 1 (1+sqrt(3))*L;
         0 0 0 1];

display(M_mat)

S1 = [0 0 1 0 -L 0]';
S2 = [0 1 0 0  0 L]';
S3 = [0 1 0 L  0 (1+sqrt(3))*L]';
S4 = [0 1 0 -(sqrt(3)-1) 0 (2+sqrt(3))*L]';
S5 = [0 0 0 0 0 1]';
S6 = [0 0 1 0 -(2+sqrt(3))*L 0]';

S_mat = [S1 S2 S3 S4 S5 S6];
display(S_mat)

B1 = [0 0 1 0 (1+sqrt(3))*L 0]';
B2 = [0 1 0 (1+sqrt(3))*L 0 -(1+sqrt(3))*L]';
B3 = [0 1 0 (2+sqrt(3))*L 0 -L]';
B4 = [0 1 0 2*L 0 0]';
B5 = [0 0 0 0 0 1]';
B6 = [0 0 1 0 0 0]';

B_mat = [B1 B2 B3 B4 B5 B6];
display(B_mat)

theta_vec = [-pi/2 pi/2 pi/3 -pi/4 1 pi/6]';

T_s = FKinSpace(M_mat, S_mat, theta_vec);
display(T_s)

T_b = FKinBody(M_mat, B_mat, theta_vec);
display(T_b)