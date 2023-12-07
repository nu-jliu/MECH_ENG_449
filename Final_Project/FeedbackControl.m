function [Ve, Xerr, Xerr_acc] = FeedbackControl(X, Xd, Xd_next, Xerr_acc, Kp, Ki, dt)
% Takes: X: The current pose
%        Xd: The desired pose
%        Xd_next: Desired pose at next time step
%        Kp: Propotional gain
%        Ki: Integral gain
%        dt: Time step
% Output: Ve: End-effector twist as control variables
%         Xerr: Error twist in current time step
%         Xerr_acc: Accumulated error twist
% 
% Example Inputs:
% X = [0.170,0,0.985,0.387;0,1,0,0;-0.985,0,0.170,0.570;0,0,0,1];
% Xd = [0,0,1,0.5;0,1,0,0;-1,0,0,0.5;0,0,0,1];
% Xd_next = [0,0,1,0.6;0,1,0,0;-1,0,0,0.3;0,0,0,1];
% Xerr_acc = [0; 0; 0; 0; 0; 0];
% Kp = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
% Ki = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];
% dt = 0.01;
% [Ve, Xerr, Xerr_acc] = FeedbackControl(X, Xd, Xd_next, Xerr_acc, Kp, Ki, dt);
%
% Output:
% Ve =
% 
%          0
%          0
%          0
%    21.4000
%          0
%     6.4500
% 
% 
% Xerr =
% 
%          0
%     0.1709
%          0
%     0.0795
%          0
%     0.1067
% 
% 
% Xerr_acc =
% 
%          0
%     0.6834
%          0
%     0.3178
%          0
%     0.4268


    Xerr_se3 = TransInv(X)*Xd;

    Xerr = se3ToVec(MatrixLog6(Xerr_se3));
    Vd_vec = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));

    AdXerr = Adjoint(Xerr_se3);
    AdXerrVd = AdXerr*Vd_vec;

    Xerr_acc = Xerr_acc + Xerr;
    Ve = AdXerrVd + Kp*Xerr + Ki*Xerr_acc*dt;
end

