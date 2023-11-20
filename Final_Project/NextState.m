function [config_new_vec] = NextState(config_vec, speed_vec, dt, max_speed)
    [r_config, c_config] = size(config_vec);
    [r_speed, c_speed] = size(speed_vec);

    if r_config ~= 12 || c_config ~= 1
        error('Invalid configuration input');
    end

    if r_speed ~= 9 || c_speed ~= 1
        error('Invalid speed input');
    end

    q_vec     = config_vec(1:3, :);
    theta_vec = config_vec(4:8, :);
    alpha_vec  = config_vec(9:12, :);

    for i = 1:9
        if speed_vec(i, 1) > max_speed
            speed_vec(i, 1) = max_speed;
        elseif speed_vec(i, 1) < -max_speed
            speed_vec(i, 1) = -max_speed;
        end
    end

    u_vec        = speed_vec(1:4, :);
    thetadot_vec = speed_vec(5:9, :);

    dtheta_vec = thetadot_vec*dt;
    dalpha_vec = u_vec*dt;

    theta_new_vec = theta_vec + dtheta_vec;
    alpha_new_vec  = alpha_vec  + dalpha_vec;

    phi       = q_vec(1, 1);
    H_mat     = HMatrix(phi);
    qdot_vec  = pinv(H_mat)*u_vec;
    q_new_vec = q_vec + qdot_vec*dt;

    config_new_vec = [q_new_vec; theta_new_vec; alpha_new_vec];
end