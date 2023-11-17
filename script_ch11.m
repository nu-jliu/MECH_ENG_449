% ME449
% Allen Liu
% Quiz ch11

close all
clear variables
clc

sys = tf(1, [1 2 2]);
[~, zeta, ~] = damp(sys);
display(zeta)

sys = tf(1, [1 3 9]);
[wn, zeta, pole] = damp(sys);
wd = wn.*sqrt(1-zeta.^2);
display(wd)

Ki = 10;
zeta = 1;
Kp = zeta*2*sqrt(Ki);
display(Kp)

wn = sqrt(Ki);
wd = zeta*wn;
ts = 4/wd;
display(ts);

Kp = 20;
Ki = (Kp/2)^2;
display(Ki)
wn = sqrt(Ki);
ts = 4/wn;
display(ts)