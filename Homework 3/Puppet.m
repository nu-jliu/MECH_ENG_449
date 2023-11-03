function [theta_mat, dtheta_mat] = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt, damping, stiffness, springPos, restLength)
    theta_vec = thetalist;
    dtheta_vec = dthetalist;
  
    zeta = damping;
    N = size(0:dt:t, 2);
    
    M = eye(size(Mlist, 1));
    for i = 1:size(Mlist, 3)
        M = M*Mlist(:, :, i);
    end

    for i = 1:N
        theta_mat(i, :)  = theta_vec';
        dtheta_mat(i, :) = dtheta_vec';
        
        tau_vec = -zeta*dtheta_vec;

        Tsb = FKinSpace(M, Slist, theta_vec);
        se_vec = Tsb(1:3, 4) - springPos;
        dist = norm(se_vec);
        Ftip = zeros(6, 1);
        Ftip(4:6, 1) = stiffness*(restLength - dist)*se_vec;

        ddtheta_vec = ForwardDynamics(theta_vec, dtheta_vec, tau_vec, g, Ftip, Mlist, Glist, Slist);

        theta_vec  = theta_vec  + dtheta_vec*dt;
        dtheta_vec = dtheta_vec + ddtheta_vec*dt;


    end

    theta_mat(N + 1, :)  = theta_vec';
    dtheta_mat(N + 1, :) = dtheta_vec';
end