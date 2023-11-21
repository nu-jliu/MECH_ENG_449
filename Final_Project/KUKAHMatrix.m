function H_mat = KUKAHMatrix(phi)
    l = 0.47/2;
    w = 0.3/2;

    x_vec = [l l -l -l]';
    y_vec = [w -w -w w]';
    gamma_vec = [-pi/4 pi/4 -pi/4 pi/4]';
    beta_vec = zeros(4, 1);
    r_vec = 0.0475*ones(4, 1);

    H_mat = [
        x_vec.*sin(beta_vec + gamma_vec) - y_vec.*cos(beta_vec + gamma_vec) ...
        cos(beta_vec + gamma_vec + phi) ...
        sin(beta_vec + gamma_vec + phi) ...
    ] ./ (r_vec.*cos(gamma_vec));
end
