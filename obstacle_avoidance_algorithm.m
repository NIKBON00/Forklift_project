% Obstacle in Cartesian coordinates in robot reference frame
[x, y] = pol2cart(angles, ranges);
obstacle_positions = [x, y];

% Distance to target
distace_to_target = [robots(i).x(1),robots(i).x(2)] - target_considered;

% Initialize Repulsion force
F_rep = zeros(size(obstacle_positions));
F_rep1 = zeros(size(obstacle_positions));
F_rep2 = zeros(size(obstacle_positions));

% Attraction force
if norm(distace_to_target) <= dstar
    F_att = -k_att * distace_to_target;
else 
    F_att = -dstar/norm(distace_to_target) * k_att * distace_to_target;
end


% Repulsion vectors
for j = 1:size(obstacle_positions, 1)
    obstacle_position = obstacle_positions(j, :);
    rho = norm(obstacle_position);

    term1 = abs( (distace_to_target(1)).^n ) +  abs( (distace_to_target(2)).^n );  % (x-xd)^n
    term2 = -obstacle_position./rho;
    term3 = -obstacle_position./(distace_to_target);
    term4 = abs( (distace_to_target(1)).^(n-1) ) +  abs( (distace_to_target(2)).^(n-1) );  % (x-xd)^(n-1)

    F_rep1(j,:) = k_rep .* (1/rho - 1/rho_0).*(1/rho.^2).*term1.*term2;
    F_rep2(j,:) = -(n/2)*k_rep.* (1/rho - 1/rho_0).^2 .*term4 .* term3;


    if rho<=rho_0 
        F_rep(j,:) = -k_rep.*(1/rho  - 1/rho_0).*(1/(rho.^2)).*obstacle_position./rho;   %F_rep1(j,:) + F_rep2(j,:);

    else
        F_rep(j,:) = [0,0];
    end
end

% Sum of repulsion vectors
F_rep_tot = sum(F_rep, 1);

% Calculate final potential
F_tot = F_att + F_rep_tot;

% Calculate reference value of linear velocity (v_t) and
% orientation
theta_ref = atan2(F_tot(2), F_tot(1));

error_theta = theta_ref - robots(i).x(3);

if abs(error_theta)<=error_theta_max
    alpha = (error_theta_max - abs(error_theta)) / error_theta_max;
    v_t = Kp_v_t*min(alpha * norm(F_tot), v_max);
else 
    v_t = 0;
end

omega_ref = Kp_omega * error_theta;
omega = min( max(omega_ref, -omega_max), omega_max); % omega owns to [-omega_max, omega_max]