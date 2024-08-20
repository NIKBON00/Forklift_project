clear all;
clc;

%% Simulation setup

SetUp.Dt = 0.1;                % [s]
SetUp.Time = 0:SetUp.Dt:100;    % [s]


%% Environment

% MaxX
Env.MaxX = 30;
Env.MaxY = 30;

% Location of the gas leak
Env.GasLeakPos = rand(2,1).*[Env.MaxX; Env.MaxY];


%% Robots

% Number of robots
Rob.n = 5;

% Robot model
Rob.A = eye(2);
Rob.B = eye(2);

% Initial state
Rob.InitState = cell(Rob.n);
Rob.State = cell(Rob.n);
Rob.GLDetected = zeros(1,Rob.n);
for i=1:Rob.n
    Rob.InitState{i} = rand(2,1).*[Env.MaxX; Env.MaxY];
    Rob.State{i} = zeros(2,length(SetUp.Time));
    Rob.State{i}(:,1) = Rob.InitState{i};
end

% Robot inputs
Rob.MaxVel = 2; % [m/s]
Rob.Inputs = cell(Rob.n);
for i=1:Rob.n
    Rob.Inputs{i} = (rand(2,1)-0.5)*2*Rob.MaxVel*ones(1,length(SetUp.Time));
end

% Robot isotropic gas leak nose
Rob.GLN.MaxDist = 5;    % [m]
Rob.GLN.mu = 0;         % [m]
Rob.GLN.sigma = 0.1;    % [m]

% Gas leak position measurements
Rob.GLN.PosMu = zeros(2,1);
Rob.GLN.PosCov = rand(2,2)*0.5;
Rob.GLN.PosCov = Rob.GLN.PosCov'*Rob.GLN.PosCov;
Rob.GLN.PosMeas = zeros(2,Rob.n);
Rob.GLN.H = eye(2);

% Structures for consensus
Rob.Cons.Adj = zeros(Rob.n, Rob.n);
Rob.Cons.nMsg = 10;
Rob.Cons.Degree = zeros(1, Rob.n);
Rob.Cons.F = cell(1, Rob.n);
Rob.Cons.a = cell(1, Rob.n);
Rob.Cons.FlagUsePrevEst = 0;

% Simulation
for k=1:length(SetUp.Time)-1
    %% Detection
    for i=1:Rob.n
        % Check if we detect the gas leak
        if (sqrt(sum((Rob.State{i}(:,k) - Env.GasLeakPos).^2)) <= Rob.GLN.MaxDist - abs(randn(1)*Rob.GLN.sigma)) && ...
                (Rob.GLDetected(i) == 0)
            % Stop the robot
            Rob.Inputs{i}(:,k:end) = zeros(2,length(SetUp.Time)-k+1);
            Rob.GLDetected(i) = 1;
        end

        if Rob.GLDetected(i)
            Rob.GLN.PosMeas(:,i) = Env.GasLeakPos + mvnrnd(Rob.GLN.PosMu, Rob.GLN.PosCov)';
        end
    end
        
    %% Consensus

    % Consensus network discovering
    for i=1:Rob.n
        % Who has the information talks
        if Rob.GLDetected(i)
            % Talk
            for j=1:Rob.n
                if not(j == i)
                    Rob.Cons.Adj(j,i) = 1;
                end
            end
        end
    end

    % Consensus degree
    for i=1:Rob.n
        % Node degree
        Rob.Cons.Degree(i) = sum(Rob.Cons.Adj(i,:));
    end

    % Composite matrices and vectors
    for i=1:Rob.n
        % Consensus information to be shared
        if Rob.GLDetected(i)
            if Rob.Cons.FlagUsePrevEst == i
                % Iterative WLS
                S = Rob.GLN.H*inv(Rob.Cons.F{i})*Rob.GLN.H' + Rob.GLN.PosCov;
                W = inv(Rob.Cons.F{i})*Rob.GLN.H'*inv(S);
                z = inv(Rob.Cons.F{i})*Rob.Cons.a{i} + W*(Rob.GLN.PosMeas(:,i) - Rob.GLN.H*inv(Rob.Cons.F{i})*Rob.Cons.a{i});
                P = (eye(2) - W*Rob.GLN.H)*inv(Rob.Cons.F{i});
                Rob.Cons.F{i} = Rob.GLN.H'*inv(P)*Rob.GLN.H;
                Rob.Cons.a{i} = Rob.GLN.H'*inv(P)*z;
            else
                Rob.Cons.F{i} = Rob.GLN.H'*inv(Rob.GLN.PosCov)*Rob.GLN.H;
                Rob.Cons.a{i} = Rob.GLN.H'*inv(Rob.GLN.PosCov)*Rob.GLN.PosMeas(:,i);
            end
        end
    end
   
    % Consensus rounds
    StoreEst = zeros(Rob.n, Rob.Cons.nMsg+1, 2);
    for i=1:Rob.n
        StoreEst(i,1,:) = inv(Rob.Cons.F{i})*Rob.Cons.a{i};
    end
    for nMsg = 1:Rob.Cons.nMsg
        % Store the previous data
        F = Rob.Cons.F;
        a = Rob.Cons.a;

        for i=1:Rob.n
            for j=1:Rob.n
                if Rob.Cons.Adj(i,j)
                    % Metropolis-Hastings weighting
                    q_ij = 1/(max(Rob.Cons.Degree(i), Rob.Cons.Degree(j)) + 1);
                    Rob.Cons.F{i} = Rob.Cons.F{i} + q_ij*(F{j} - F{i});
                    Rob.Cons.a{i} = Rob.Cons.a{i} + q_ij*(a{j} - a{i});
                end
            end
        end

        % Store the estimate at each msg round
        for i=1:Rob.n
            StoreEst(i,nMsg+1,:) = inv(Rob.Cons.F{i})*Rob.Cons.a{i};
        end
    end

    % Select a robot that uses the previous consensus estiamtes
    for i=1:Rob.n
        if Rob.GLDetected(i)
            Rob.Cons.FlagUsePrevEst = i;
        end
    end

    % Plot the results of the estiamtes at each time step as a function of
    % the number of messages exchanged by the consensu protocol
    if sum(Rob.GLDetected) >= 2
        figure(2), clf, hold on;
        LegS = {};
        for i=1:Rob.n
            plot(0:Rob.Cons.nMsg, StoreEst(i,:,1));
            LegS{end+1} = ['r_{', num2str(i), '}']; 
        end
        plot(0:Rob.Cons.nMsg, Env.GasLeakPos(1)*ones(1,Rob.Cons.nMsg+1), 'r--');
        LegS{end+1} = 'Actual pos';
        xlabel('Numebr of msgs');
        ylabel('x [m]');
        legend(LegS, 'Location', 'best');

        figure(3), clf, hold on;
        LegS = {};
        for i=1:Rob.n
            plot(0:Rob.Cons.nMsg, StoreEst(i,:,2));
            LegS{end+1} = ['r_{', num2str(i), '}']; 
        end
        plot(0:Rob.Cons.nMsg, Env.GasLeakPos(2)*ones(1,Rob.Cons.nMsg+1), 'r--');
        LegS{end+1} = 'Actual pos';
        xlabel('Numebr of msgs');
        ylabel('y [m]');
        legend(LegS, 'Location', 'best');
    end

    %% Motion
    for i=1:Rob.n
        % Robot motion
        Rob.State{i}(:,k+1) = Rob.A*Rob.State{i}(:,k) + SetUp.Dt*Rob.B*Rob.Inputs{i}(:,k);
        while (Rob.State{i}(1,k+1) >= Env.MaxX) || (Rob.State{i}(1,k+1) <= 0) || ...
              (Rob.State{i}(2,k+1) >= Env.MaxY) || (Rob.State{i}(2,k+1) <= 0)
            % New inputs
            Rob.Inputs{i}(:,k:end) = (rand(2,1)-0.5)*2*Rob.MaxVel*ones(1,length(SetUp.Time)-k+1);
            Rob.State{i}(:,k+1) = Rob.A*Rob.State{i}(:,k) + SetUp.Dt*Rob.B*Rob.Inputs{i}(:,k);
        end
    end
end

%% Plot

Plot.RobCol = {'bo', 'k.', 'gd', 'rs', 'bs', 'ko'};

figure(1), clf, hold on;
LegS = {};
for i=1:Rob.n
    plot(Rob.InitState{i}(1), Rob.InitState{i}(2), Plot.RobCol{i});
    LegS{end+1} = ['r_{', num2str(i), '}'];
end
plot(Env.GasLeakPos(1), Env.GasLeakPos(2), 'rd', 'MarkerSize', 18);
LegS{end+1} = 'Gas leak';
rectangle('Position', [Env.GasLeakPos(1) - Rob.GLN.MaxDist, Env.GasLeakPos(2) - Rob.GLN.MaxDist, 2*Rob.GLN.MaxDist, 2*Rob.GLN.MaxDist], 'Curvature', [1 1]);
for i=1:Rob.n
    plot(Rob.State{i}(1,:), Rob.State{i}(2,:));
end
xlabel('[m]');
ylabel('[m]');
legend(LegS, 'Location', 'best');
