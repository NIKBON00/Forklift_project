function [position] = trilateration(antenna_positions, distances)
    
    % Number of antennas
    N = size(antenna_positions, 1);
 

    if N < 3
        error('You need at least 3 UWB antennas to perform the trilateration');
    end

    % Construct matrix A and vector B to compute the calculus next
    A = zeros(N-1, 2);
    b = zeros(N-1, 1);

    % Formula LLSE algorithm
    for ii = 2:N
       
        A(ii-1, :) = 2 * (antenna_positions(ii, :) - antenna_positions(1, :));
        b(ii-1) = distances(1)^2 - distances(ii)^2 + ...
                 sum(antenna_positions(ii, :).^2) - sum(antenna_positions(1, :).^2);
    end

    % Position estimation
    position = A \ b;
end