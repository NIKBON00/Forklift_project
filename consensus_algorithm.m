% Terna of sensors
%UWB_sens = [UWB_sens(1,:); UWB_sens(2,:); UWB_sens(3,:)];
terna = nchoosek(1:length(UWB_sens), 3);

% Structures for consensus
Cons.adj = zeros(size(terna,1),size(terna,1));
Cons.nMsg = 20;
Cons.Degree = zeros(size(terna,1), 1);
Cons.F = cell(size(terna,1),1);
Cons.a = cell(size(terna,1),1);
Cons.FlagUsePrevEst = 0;
Cons.nmsg = 10;

check_occupancy = zeros(size(terna,1),1);

% Position Measurements
Posmu = zeros(3,1);

% Measured distances robot - sensor
distance_rs = sqrt( (robots(i).x(1) - UWB_sens(:,1)).^2  + (robots(i).x(2) - UWB_sens(:,2)).^2  );

% Distance UWB antenna - tags UWB on robot
distance_tag1 = sqrt( (robots(i).x(1) + L*cos(robots(i).x(3)) - UWB_sens(:,1)).^2  + (robots(i).x(2) + L*sin(robots(i).x(3)) - UWB_sens(:,2)).^2 );
distance_tag2 = sqrt( (robots(i).x(1) - L*cos(robots(i).x(3)) - UWB_sens(:,1)).^2  + (robots(i).x(2) - L*sin(robots(i).x(3)) - UWB_sens(:,2)).^2 );

% Consesus rounds
StoreEst = zeros(size(terna,1),Cons.nmsg+1,3);



% Composite matrices and vectors
for r = 1:size(terna,1)
    % For each terna
    idx = terna(r,:);

    % Antenna positions
    antenna_positions = [UWB_sens(idx(1),:); UWB_sens(idx(2),:); UWB_sens(idx(3),:)];
    
    % Retrieve with trilateration positions of the two tags
    pos_est_tag1 = trilateration(antenna_positions, [distance_tag1(idx(1)), distance_tag1(idx(2)), distance_tag1(idx(3))]);
    pos_est_tag2 = trilateration(antenna_positions, [distance_tag2(idx(1)), distance_tag2(idx(2)), distance_tag2(idx(3))]);
    
    % Position of robot
    pos_est= [(pos_est_tag1(1) + pos_est_tag2(1))/2  ; (pos_est_tag1(2) + pos_est_tag2(2))/2 ];
    
    % Orientation of robot 
    angle = atan2((pos_est_tag1(2) - pos_est_tag2(2)),(pos_est_tag1(1) - pos_est_tag2(1)));

    % For each terna of sensor, iterative WLS
    PosCov = diag(sigma_meas);

    zi = [pos_est;angle] + mvnrnd(Posmu,PosCov)';
    Hi = eye(3);
    R_i = PosCov;

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

    % Save the result
    check_occupancy(r) = isOccupied;

end



% Consensus network discovering
for sen = 1:size(terna,1)
    % Who has the information talks
    for sen_2 = 1:size(terna,1)
        if not(sen_2 == sen)
            if(check_occupancy(sen_2)==0)
                Cons.adj((sen_2), sen) = 1;     % Can speak correctly if the in terna all antennas see clearly the robot
            else
                Cons.adj((sen_2), sen) = 0.2;   % Terna has inside one base stations that see an obstacle, so diluituion of precision
            end
        end
    end
end

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
