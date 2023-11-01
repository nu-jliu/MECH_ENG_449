function [theta_vec, success, theta_mat, err_ang, err_lin] = IKinBodyIterates(M, Blist, Tsd, theta_init, max_iter, ew, ev, filename)
    
    theta_curr = theta_init;
    theta_mat  = [];
    err_ang    = [];
    err_lin    = [];

    for i = 1 : max_iter
        fprintf('--------------- Iteration %d ---------------\n', i)

        theta_mat(i, :) = theta_curr';

        fprintf('Joint Vector:\n')
        disp(theta_curr')
        
        Tsb = FKinBody(M, Blist, theta_curr);
        Tbd = TransInv(Tsb)*Tsd;

        fprintf('SE(3) end-effector config:\n')
        disp(Tsb)
        
        Vb_skrw = MatrixLog6(Tbd);
        Vb_vec  = se3ToVec(Vb_skrw);

        wb = Vb_vec(1:3, :);
        vb = Vb_vec(4:6, :);

        fprintf('          error twist V_b: ')

        fprintf('(')
        for j = 1:6
            if j == 6
                fprintf('%.3f', Vb_vec(j))
            else
                fprintf('%.3f, ', Vb_vec(j))
            end
        end
        fprintf(')\n')

        err_ang(i, :) = norm(wb);
        err_lin(i, :) = norm(vb);

        fprintf('angular error ||omega_b||: %d\n', norm(wb))
        fprintf(' linear error     ||v_b||: %d\n\n', norm(vb))


        if norm(wb) < ew && norm(vb) < ev
            theta_vec = theta_curr;
            success   = true;
            writematrix(theta_mat, sprintf('%s.csv', filename));
            return
        end

        J   = JacobianBody(Blist, theta_curr);
        J_p = pinv(J);
        dtheta = J_p*Vb_vec;

        theta_curr = theta_curr + dtheta;
    end

    theta_vec = NaN;
    success = false;
end