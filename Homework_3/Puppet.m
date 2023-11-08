function [theta_mat, dtheta_mat] = Puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt, damping, stiffness, springPos, restLength)
% Takes thetalist: n-vector of initial joint variables,
%       dthetalist: n-vector of initial joint rates,
%       g: Gravity vector g,
%       Ftipmat: An N x 6 matrix of spatial forces applied by the 
%                end-effector (If there are no tip forces, the user should 
%                input a zero and a zero matrix will be used),
%       Mlist: List of link frames {i} relative to {i-1} at the home
%              position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns,
%       t: The total time of simulation
%       dt: The timestep between consecutive joint forces/torques,
%       damping: The damping ratio of the joint
%       stiffness: The spring constant
%       spingPos: The position of the spring origin
%       restLength: The length of the spring at rest condition
% Returns thetamat: The N x n matrix of robot joint angles resulting from 
%                   the specified joint forces/torques,
%         dthetamat: The N x n matrix of robot joint velocities.
% This function simulates the motion of a UR5 robot with a spring connected
% to its end-effector, and outputing the thetalist and dthetalist as joint 
% position and velocity at each time step

    theta_vec = thetalist;
    dtheta_vec = dthetalist;
  
    zeta = damping;
    t_list = 0:dt:t;
    N = size(t_list, 2);
    
    % Home comfiguration
    M = eye(size(Mlist, 1));
    for i = 1:size(Mlist, 3)
        M = M*Mlist(:, :, i);
    end

    for i = 1:N

        theta_mat(i, :)  = theta_vec';
        dtheta_mat(i, :) = dtheta_vec';
        
        % Damping force
        tau_vec = -zeta*dtheta_vec;

        Tsb = FKinSpace(M, Slist, theta_vec);

        % End-effector force
        ps_b = TransInv(Tsb)*[springPos; 1];
        se_vec = -ps_b(1:3, :);
        dist = norm(se_vec);
        Ftip_b = zeros(6, 1);
        Ftip_b(4:6, 1) = stiffness*(restLength - dist)*se_vec/dist;

        ddtheta_vec = ForwardDynamics(theta_vec, dtheta_vec, tau_vec, g, -Ftip_b, Mlist, Glist, Slist);

        [theta_vec, dtheta_vec] = EulerStep(theta_vec, dtheta_vec, ddtheta_vec, dt);
    end

    theta_mat(N + 1, :)  = theta_vec';
    dtheta_mat(N + 1, :) = dtheta_vec';
end