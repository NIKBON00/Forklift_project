% Possible terna combinations of UWB antennas
terna = nchoosek(1:length(UWB_sens), 3);

% Structures for consensus
Cons.adj = zeros(size(terna,1),size(terna,1));       % Adjacency Matrix
Cons.Degree = zeros(size(terna,1), 1);               % Degree Vector
Cons.F = cell(size(terna,1),1);                      % Cell of local composite information matrix 
Cons.a = cell(size(terna,1),1);                      % Cell of local composite state matrix
Cons.nmsg = 15;                                      % Number of messages exchanged

% Control paramter, useful to see if there's an obstacle between robot and
% UWB antennas
check_occupancy = zeros(size(terna,1),1);

% Noise mean of UWB measurements
Posmu = zeros(3,1);

% Distance UWB antennas - tags UWB on robot
distance_tag1 = sqrt( (robots(i).x(1) + L/2*cos(robots(i).x(3)) - UWB_sens(:,1)).^2  + (robots(i).x(2) + L/2*sin(robots(i).x(3)) - UWB_sens(:,2)).^2 );
distance_tag2 = sqrt( (robots(i).x(1) - L/2*cos(robots(i).x(3)) - UWB_sens(:,1)).^2  + (robots(i).x(2) - L/2*sin(robots(i).x(3)) - UWB_sens(:,2)).^2 );

% Consesus rounds for position and orientation savings
StoreEst = zeros(size(terna,1),Cons.nmsg+1,3);

% Composite matrices and vectors
for r = 1:size(terna,1)
    % For each terna combination
    idx = terna(r,:);

    % Antenna positions
    antenna_positions = [UWB_sens(idx(1),:); UWB_sens(idx(2),:); UWB_sens(idx(3),:)];
    
    % Retrieve with trilateration positions of the two tags
    pos_est_tag1 = trilateration(antenna_positions, [distance_tag1(idx(1)), distance_tag1(idx(2)), distance_tag1(idx(3))]);
    pos_est_tag2 = trilateration(antenna_positions, [distance_tag2(idx(1)), distance_tag2(idx(2)), distance_tag2(idx(3))]);
    
    % Position of robot from trilateration measurements
    pos_est= [(pos_est_tag1(1) + pos_est_tag2(1))/2  ; (pos_est_tag1(2) + pos_est_tag2(2))/2 ];
    
    % Orientation of robot from trilateration measurements
    angle = atan2((pos_est_tag1(2) - pos_est_tag2(2)),(pos_est_tag1(1) - pos_est_tag2(1)));

    % For each terna combination, iterative WLS
    PosCov = diag(sigma_meas);

    % Add White Gaussian noise to measurement
    zi = [pos_est;angle] + mvnrnd(Posmu,PosCov)';
    % Jacobian of output measurement wrt state
    Hi = eye(3);
    % Measurement covariance matrix
    R_i = PosCov;

    % Compute local composite information matrix and local composite
    % information state
    Cons.F{r} = Hi'*inv(R_i)*Hi;
    Cons.a{r} = Hi'*inv(R_i)*zi;

    % Consensus rounds
    StoreEst(r,1,:) = inv(Cons.F{r})*Cons.a{r};

    % Verify is there's an obstacle between robot position and UWB antenna
    for jjj = 1:size(antenna_positions,1)
        isOccupied = check_occup([antenna_positions(jjj,:)], robots(i).x,map);
        if(isOccupied) == 1
            break;
        end
    end

    % Save the result of binary occupancy check
    check_occupancy(r) = isOccupied;
end



% Consensus network discovering
for sen = 1:size(terna,1)
    % Who has the information talks
    for sen_2 = (sen + 1) :size(terna,1)
        if (check_occupancy(sen)==0 && check_occupancy(sen_2)==0)
            weight = 1;                 % speak correctly
        else
            weight = 0.2;               % obstacle present
        end
 
        % Assign weight
        Cons.adj(sen, sen_2) = weight;      
        Cons.adj(sen_2, sen) = weight;      % Made matrix symmetric
    end

end

%{
% Make adjancency matrix sthocastic (normalize each row of the matrix, in order to have of the elements of the row = 1)
for sen = 1:size(terna,1)
    row_sum = sum(Cons.adj(sen,:));
    if row_sum>0
        Cons.adj(sen,:) = Cons.adj(sen,:)/row_sum;
    end
end
%}


% Consenus degree
for ss = 1:size(terna,1)
    Cons.Degree(ss) = sum(Cons.adj(ss,:)); 
end


% Message exchange
for nMsg = 1:Cons.nmsg

        for sen = 1:size(terna,1)
            for sen_2 = 1:size(terna,1)
                 if Cons.adj(sen,sen_2)>0
                     % Metropolis-Hastings weighting
                     q_ij = 1/max(Cons.Degree(sen), Cons.Degree(sen_2) + 1);
                     Cons.F{sen} = Cons.F{sen} + q_ij * (Cons.F{sen_2} - Cons.F{sen}); 
                     Cons.a{sen} = Cons.a{sen} + q_ij * (Cons.a{sen_2} - Cons.a{sen}); 
                 end
            end
        end

        % Store the estimate at each msg round
        for r = 1:size(terna, 1)
            StoreEst(r,nMsg+1,:) = inv(Cons.F{r})*Cons.a{r};
        end 
end
