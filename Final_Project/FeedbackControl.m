function [Ve_vec] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)

    Xerr = TransInv(X)*Xd;
    
    Xerr_vec = se3ToVec(MatrixLog6(Xerr));
    Vd_vec = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));

    AdXerr = Adjoint(Xerr);
    AdXerrVd = AdXerr*Vd_vec;

    Ve_vec = AdXerrVd + Kp*Xerr_vec + Ki*Xerr_vec*dt;

    display(Vd_vec)
    display(AdXerrVd)
    display(Ve_vec)
    display(Xerr_vec)
end

