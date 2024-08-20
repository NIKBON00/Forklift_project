% Parameters
N = 6;  % Number of sensors
target_position = [5, 5, 5];  % True position of the target
sensor_positions = rand(N, 3) * 10;  % Random sensor positions within 10x10x10 area

% Measured distances with noise
measured_distances = sqrt(sum((sensor_positions - target_position).^2, 2)) + randn(N, 1) * 0.1;

% Initial guess of the target position for each sensor
estimates = rand(N, 3) * 10;

% Adjacency matrix (1 if connected, 0 otherwise)
% Here, for simplicity, we assume a fully connected network.
% Adjust for specific network topologies.
adj_matrix = ones(N) - eye(N);

% Degree matrix (number of neighbors for each node)
degree_matrix = diag(sum(adj_matrix, 2));

% Metropolis weights
W = zeros(N, N);
for i = 1:N
    for j = 1:N
        if adj_matrix(i, j) == 1
            W(i, j) = 1 / (max(degree_matrix(i, i), degree_matrix(j, j)) + 1);
        end
    end
end
W = W - diag(sum(W, 2));  % Adjust diagonal to ensure stochastic matrix

% DLS Algorithm Parameters
num_iter = 100;
alpha = 0.1;  % Update step size

% DLS Algorithm Iteration
for k = 1:num_iter
    new_estimates = estimates;
    for i = 1:N
        % Calculate the local least squares estimate
        H = zeros(3, 3);
        g = zeros(3, 1);
        for j = 1:N
            if adj_matrix(i, j) == 1 || i == j
                diff = estimates(i, :) - sensor_positions(j, :);
                distance_est = norm(diff);
                residual = measured_distances(j) - distance_est;
                
                H = H + W(i, j) * (diff' * diff);
                g = g + W(i, j) * residual * diff';
            end
        end
        
        % Update estimate for node i
        new_estimates(i, :) = estimates(i, :) - alpha * (H \ g)';
        
        % Combine with neighbors' estimates using Metropolis weights
        for j = 1:N
            if adj_matrix(i, j) == 1
                new_estimates(i, :) = new_estimates(i, :) + W(i, j) * (estimates(j, :) - estimates(i, :));
            end
        end
    end
    estimates = new_estimates;
end

% Final estimate (average of all node estimates)
final_estimate = mean(estimates, 1);
disp(['Final estimated position: ', num2str(final_estimate)]);
disp(['True position: ', num2str(target_position)]);
