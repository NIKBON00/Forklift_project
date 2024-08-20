% Terna of sensors
terna = nchoosek(1:length(UWB_sens), 3);

% Structures for consensus
Cons.adj = zeros(size(terna,1),size(terna,1));
Cons.nMsg = 20;
Cons.Degree = zeros(size(terna,1), 1);
Cons.F = cell(size(terna,1),1);
Cons.a = cell(size(terna,1),1);
Cons.FlagUsePrevEst = 0;
Cons.nmsg = 10;

% Position Measurements
Posmu = zeros(3,1);

% Measured distances robot - sensor
distance_rs = sqrt( (robots(i).x_est(1) - UWB_sens(:,1)).^2  + (robots(i).x_est(2) - UWB_sens(:,2)).^2  );

% Consesus rounds
StoreEst = zeros(size(terna,1),Cons.nmsg+1,3);


% Consensus network discovering
for sen = 1:size(terna,1)
    % Who has the information talks
    for sen_2 = 1:size(terna,1)
        if not(sen_2 == sen)
        Cons.adj((sen_2), sen) = 1;
        end
    end
end

% Consenus degree
for ss = 1:size(terna,1)
    Cons.Degree(ss) = sum(Cons.adj(ss,:)); 
end


% Composite matrices and vectors
for r = 1:size(terna,1)
    % For each terna
    idx = terna(r,:);

    antenna_positions = [UWB_sens(idx(1),:); UWB_sens(idx(2),:); UWB_sens(idx(3),:)];
    pos_est = trilateration(antenna_positions, [distance_rs(idx(1)), distance_rs(idx(2)), distance_rs(idx(3))]);    % OK!
    angle   = angle_trilateration(antenna_positions,robots_tags,pos_est,distance_rs,idx);

    % For each terna of sensor, iterative WLS
    PosCov = eye(3)*0.005;
    %PosCov = PosCov'*PosCov;
    zi = [pos_est;angle] + mvnrnd(Posmu,PosCov)';
    Hi = eye(3);
    R_i = PosCov;
    
    Cons.F{r} = Hi'*inv(R_i)*Hi;
    Cons.a{r} = Hi'*inv(R_i)*zi;

    % Consensus rounds
    StoreEst(r,1,:) = inv(Cons.F{r})*Cons.a{r};
end

   

for nMsg = 1:Cons.nmsg

        for sen = 1:size(terna,1)
            for sen_2 = 1:size(terna,1)
                 if Cons.adj(sen,sen_2)
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
