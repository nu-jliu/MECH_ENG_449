function [config_mat] = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)
    N1 = ceil(15*k/0.01);
    N2 = ceil(15*k/0.01);
    N3 = ceil(15*k/0.01);
    N4 = ceil(15*k/0.01);

    Tse_standoff = Tsc_init*Tce_standoff;

    traj = CartesianTrajectory(Tse_init, Tse_standoff, 30, N, 5);
    
    display(traj)

    config_mat = zeros(N, 13);

    for i = 1:N
        Tse = traj{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat(i, 1:12) = [r1 r2 r3 p];
    end
end

