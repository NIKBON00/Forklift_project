function [position, residual] = trilateration(antenna_positions, distances)
    % antenna_positions: Nx2 matrix of known positions of antennas (x, y)
    % distances: Nx1 vector of measured distances from the point to each antenna
    % position: 1x2 vector (x, y) - estimated position of the robot
    % residual: scalar - sum of squared differences between measured and estimated distances

    % REFERENCE: UWB Indoor Localization Using Deep LearningLSTM Networks

    % Numero di antenne
    N = size(antenna_positions, 1);

    if N < 3
        error('Devi avere almeno tre antenne per eseguire la trilaterazione in 2D.');
    end

    % Costruzione della matrice A e del vettore b del sistema lineare Ax = b
    A = zeros(N-1, 2);
    b = zeros(N-1, 1);

    for ii = 2:N
       
        A(ii-1, :) = 2 * (antenna_positions(ii, :) - antenna_positions(1, :));
        b(ii-1) = distances(1)^2 - distances(ii)^2 + ...
                 sum(antenna_positions(ii, :).^2) - sum(antenna_positions(1, :).^2);
    end

    % Risoluzione del sistema lineare per ottenere la posizione stimata
    position = A \ b;
    %disp(position);

    % Calcolo del residuo (somma dei quadrati delle differenze)
    estimated_distances = sqrt(sum((antenna_positions - position').^2, 2));
    residual = sum((distances - estimated_distances).^2);
end