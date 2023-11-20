function [config_mat] = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)
% Takes: Tse_init: Initial configuration of the end-effector in the space
%                  frame
%        Tsc_init: Initial configuration of the cube in the space frame.
%        Tsc_final: Final configuration of the cube in the space frame.
%        Tce_grap: The configuratino of the cube in end-effector frame when
%                  it is being graped by the gripper.
%        % Tce_standoff: The configuration of the cube in end-effetor frame
%                        when the end-effector is in standoff position.
% Output: config_mat: The matrix representing the configuration at each
%                     timestep.



    %%%%%%%%%%%%%%% Define Time for each trajectory %%%%%%%%%%%%%%%%
    T1 = 10;
    T2 = 5;
    T3 = 5;
    T4 = 10;
    T5 = 5;
    T6 = 5;

    %%%%%%%% Get the number of points for each trajectory %%%%%%%%%
    N1 = ceil(T1*k/0.01);
    N2 = ceil(T2*k/0.01);
    N3 = ceil(T3*k/0.01);
    N4 = ceil(T4*k/0.01);
    N5 = ceil(T5*k/0.01);
    N6 = ceil(T6*k/0.01);

    %%%%%%%%%%%%%%% Calculate the first trajectory %%%%%%%%%%%%%%%%%
    config_mat_1 = zeros(N1, 13);
    Tse_standoff_init = Tsc_init*Tce_standoff;
    traj1 = CartesianTrajectory(Tse_init, Tse_standoff_init, T1, N1, 5);

    for i = 1:N1
        Tse = traj1{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat_1(i, :) = [r1 r2 r3 p 0];
    end

    %%%%%%%%%%%%%%% Calculate the second trajectory %%%%%%%%%%%%%%%%%
    config_mat_2 = zeros(N2, 13);
    Tse_grasp_init = Tsc_init*Tce_grasp;
    traj2 = CartesianTrajectory(Tse_standoff_init, Tse_grasp_init, T2, N2, 5);

    for i = 1:N2
        Tse = traj2{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat_2(i, :) = [r1 r2 r3 p 0];
    end

    %%%%%%%%%%%%%%% Calculate the third trajectory %%%%%%%%%%%%%%%%%
    config_mat_3 = zeros(N3, 13);
    traj3 = CartesianTrajectory(Tse_grasp_init, Tse_standoff_init, T3, N3, 5);

    for i = 1:N3
        Tse = traj3{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat_3(i, :) = [r1 r2 r3 p 1];
    end

    %%%%%%%%%%%%%%% Calculate the forth trajectory %%%%%%%%%%%%%%%%%
    config_mat_4 = zeros(N4, 13);
    Tse_standoff_final = Tsc_final*Tce_standoff;
    traj4 = CartesianTrajectory(Tse_standoff_init, Tse_standoff_final, T4, N4, 5);

    for i = 1:N4
        Tse = traj4{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat_4(i, :) = [r1 r2 r3 p 1];
    end

    %%%%%%%%%%%%%%% Calculate the fifth trajectory %%%%%%%%%%%%%%%%%
    config_mat_5 = zeros(N5, 13);
    Tse_grap_final = Tsc_final*Tce_grasp;
    traj5 = CartesianTrajectory(Tse_standoff_final, Tse_grap_final, T5, N5, 5);

    for i = 1:N5
        Tse = traj5{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat_5(i, :) = [r1 r2 r3 p 1];
    end

    %%%%%%%%%%%%%%% Calculate the sixth trajectory %%%%%%%%%%%%%%%%%
    config_mat_6 = zeros(N6, 13);
    traj6 = CartesianTrajectory(Tse_grap_final, Tse_standoff_final, T6, N6, 5);

    for i = 1:N6
        Tse = traj6{i};
        r1 = Tse(1, 1:3);
        r2 = Tse(2, 1:3);
        r3 = Tse(3, 1:3);
        p  = Tse(1:3, 4)';
        config_mat_6(i, :) = [r1 r2 r3 p 0];
    end

    %%%%%%%%%%%%%%%%%%% Combine all trajectories %%%%%%%%%%%%%%%%%%%
    config_mat = cat( ...
        1, ...
        config_mat_1, ...
        config_mat_2, ...
        config_mat_3, ...
        config_mat_4, ...
        config_mat_5, ...
        config_mat_6 ...
    );
end

