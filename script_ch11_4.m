% ME449
% Allen Liu
% Quiz Ch11.4

close all
clear variables
clc

M = 1;
b = 2;
Kd = 3;
Kp = 4;

Ki = (b + Kd)*Kp/M;
display(Ki)

M = 1;
b = 2;
s = -4;
alist = conv([1 -s], conv([1 -s], [1 -s]));
a1 = alist(2);
Kd = a1*M-b;
display(Kd)