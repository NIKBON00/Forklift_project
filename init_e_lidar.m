clc;
clear all;
close all;

%% PARAMETERS OF FIRST SIMULATION

% Simulation Set up: 
Setup.dt = 0.01;                    % [s]
Setup.Time = 0:Setup.dt:100;        % [s]

% Number of steps:
Setup.steps = Setup.Time/Setup.dt;        % [-]

% Number of robots:
nRobots = 1;
robots=[];                          % Vector of robot structures

% Parameter of robot:
d = 1.5;                            % width [m]
L = 2.8;                            % length [m]
R = 0.23;                           % radius wheel [m]
weigh_init = 0;
K_r = 0.02;                        % costant parameter contributing to noise in speed right sensor 
K_l = 0.02;                        % costant parameter contributing to noise in speed left  sensor
sigma_meas = [0.4 0.4 0.2];        % noise on UWB

% Collision avoidance variables:
k_att = 1.1547;                     % attraction costant
k_rep = 0.0932;                     % repulsion costant
Kp_v_t = 10;                        % speed gain
Kp_omega = 2;                       % omega gain
dstar = 0.3;                        % threshold distance for type of attractive force [m]   
rho_0 = 2.5;                        % safe distance [m]
error_theta_max = deg2rad(45);      % error angle max [rad]
v_max = 2;                          % [m/s] 
omega_max = 0.6*pi;                 % [rad/s]
n=1;                                % exponent of repulsion force

% Entrance:
inital_state = cell(nRobots,1);                 % [m]

% Targets:
targets = [15, 40; 45,20; 50, 40; 40, 19];      % [m]

% General variables:
to_grad = 180/pi;       % [°]
to_rad = pi/180;        % [rad]

% Definition of vectors useful for plotting part:
pose_est_UWB = cell(nRobots, length(Setup.steps));
EKF_pose = cell(nRobots, length(Setup.steps));

h1 = cell(nRobots,1);                           % useful for delete precedent position of robot during plotting

% Incremental variable for plottinh:
l = 1;
nM = 1;
reached = zeros(nRobots,1);

% Location of UWB antenna:
UWB_sens = [10,10; 10,40; 70,10; 70,50; 40, 36; 51, 22];

% Position of UWB tags on the robot in robot reference frame, necessary 2
% tags
robots_tags = [0, L; 0, -L]; 


%% ROBOT INITIALIZATION

for i=1:nRobots
    inital_state{i} = [2, 2 , 0];        % random entrace
    robot = Forklift_def(inital_state{i},d,R,Setup.dt,K_r,K_l,nM,NaN);     % initialization of robot structure  
    robots = [robots,robot];                                               % Add i-th robot to global vector
end

%% EKF INITIALIZATION

for i=1:nRobots
        EKFs{i} = EKF_def();
        EKFs{i}.EKF_init(weigh_init,inital_state{i},diag([0.2, 0.2, 0.5]));
end


%% CREATE THE MAP
% Create a binary warehouse map and place obstacles at defined locations
map = helperCreateBinaryOccupancyMap;

% Visualize map with obstacles and AGV
figure();
show(map);
title('Warehouse Floor Plan With Obstacles and robot');

%% ADD ROBOTS TO THE MAP IN THEIR INITIAL POSITION
for i=1:nRobots
helperPlotRobot(gca,inital_state{i});
end

%% LIDAR PROPERTIES
% Put lidar sensor
lidar = rangeSensor;
lidar.Range = [0 5];
lidar.HorizontalAngle = [-pi/2, pi/2];
lidar.HorizontalAngleResolution = pi/180;
lidar.RangeNoise = 0.01;

% Generate lidar readings
pose = zeros(1,3);

% Set up display
display = helperVisualizer;

hRobot = cell(nRobots, 1);

for i = 1:nRobots
% Plot warehouse map in the display window
hRobot{i} = plotBinaryMap(display,map,inital_state{i});
end


%% ROBOT TO TARGETS MOVEMENT

figure();
show(map);
hold on

for k=1:length(Setup.steps)
    for i=1:nRobots
            
            % Target considered from i-th robot
            target_considered = targets(i,:);

            % Generate lidar readings
            [ranges,angles] = lidar([robots(i).x(1) robots(i).x(2) robots(i).x(3)],map);
            % Scan 2D
            scan = lidarScan(ranges,angles);
    
            % Store 2-D scan as point cloud
            cart = scan.Cartesian;
            cart(:,3) = 0;
            pc = pointCloud(cart); % the points generally represent the x,y, and z geometric coordinates for samples on a surface or of an environment
    
            % Segment point cloud into clusters based on euclidean distance
            minDistance = 0.9;
            [labels,numClusters] = pcsegdist(pc,minDistance); % segments a point cloud into clusters, with a minimum Euclidean distance of minDistance between points from different clusters
    
    
            % Plot robot trajectory
            plot(robots(i).x(1), robots(i).x(2),'.');
            h1{i} = helperPlotRobot(gca,[robots(i).x(1), robots(i).x(2) robots(i).x(3)]);
            
            % Plot UWB antenna on the map
            for s = 1:length(UWB_sens)
                plot(UWB_sens(s,1), UWB_sens(s,2),'o');
                hold on
            end
      
            % Run obstacle avoidance algorithm
            obstacle_avoidance_algorithm;

            % Consensus algorithm
            consensus_algorithm;
       
            % Save estimated position with only UWB sensor trilateration
            % terna
            pose_est_UWB{i,k} = [StoreEst(1,end,1), StoreEst(1,end,2), StoreEst(1,end,3)];
            
            % Extended Kalman Filter
            EKFs{i}.EKF_prediction(robots(i).odometry_estimation,Setup.dt,d,R);
            EKFs{i}.EKF_correction(sigma_meas, [pose_est_UWB{i,k}(1) pose_est_UWB{i,k}(2) pose_est_UWB{i,k}(3)]);
            EKFs{i}.state_history{k,1} = EKFs{i}.x;

            % Update exact kinematics and state estimation with noise
            x_next = robots(i).dynamics(v_t,omega);

            % Dynamic history real position
            robots(i).dynamics_history{k,1} = x_next;

            % Odometry 
            odometry = robots(i).odometry_step(v_t,omega);

            % Dynamic history estimated position
            robots(i).odometry_history{k,1} = robots(i).x_est;

    
            % When I'm close to first target change to second
            if (robots(i).getDistance(target_considered(1),target_considered(2)) < 0.7 )
                % i-th robot has reached the target
                reached(i) = 1;
            end

            l = l+1;
            
    end

    pause(0.01);
    disp(robots(i).x(1));
    
    for i=1:nRobots
        delete(h1{i});
    end

    % If every robot has reached it's own target exit the cycle
    if (reached)
        break;
    end

end


%% ERROR PLOTS

% Estimation errors
estimation_error = zeros(length(Setup.steps),nRobots);

for i = 1:nRobots

    tmp = cell(1, length(Setup.steps));
    for k = 1:length(Setup.steps)
    tmp{1,k} = pose_est_UWB{i,k}(1:end);
    end
    indiciNonVuoti = find(~cellfun('isempty', tmp));
    
    % Estimated robot position using terna trilateration of UWB sensor
    figure(i+3);
    for k = 1:indiciNonVuoti(end)
       plot(robots(i).dynamics_history{k,1}(1), robots(i).dynamics_history{k,1}(2),'go','MarkerSize', 2'');
       hold on
       plot(robots(i).odometry_history{k,1}(1), robots(i).odometry_history{k,1}(2),'bo','MarkerSize', 2'');
       plot(pose_est_UWB{i,k}(1), pose_est_UWB{i,k}(2),'r*','MarkerSize', 2'');
    end
    xlabel('Coordinate x [m]');
    ylabel('Coordinate y [m]');
    legend('True posture','Estimation speed sensor','Estimation only UWB sensor');
    title(['Localization with consensus algorithm UWB terna only for robot', num2str(i)]);
    grid on;
    
    figure(i+5);
    for k = 1:indiciNonVuoti(end)
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - robots(i).odometry_history{k,1}(1), robots(i).dynamics_history{k,1}(2) - robots(i).odometry_history{k,1}(2)),'go','MarkerSize', 2'')
       hold on
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - pose_est_UWB{i,k}(1), robots(i).dynamics_history{k,1}(2) - pose_est_UWB{i,k}(2)),'bo','MarkerSize', 2'');
       hold on
    end
    xlabel('Steps');
    ylabel('Error norm [m]');
    legend('Error norm only speed sensor', 'error norm only UWB')
    title('Error norm with consensus algorithm UWB terna only');
    grid on;

    
 
    figure(i+7);
    for k = 1:indiciNonVuoti(end)
        plot(robots(i).dynamics_history{k,1}(1), robots(i).dynamics_history{k,1}(2),'go','MarkerSize', 2'');
        hold on
        plot(EKFs{i}.state_history{k,1}(1), EKFs{i}.state_history{k,1}(2),'r*','MarkerSize', 2'')
    end
    xlabel('Coordinate x [m]');
    ylabel('Coordinate y [m]');
    legend('True posture','Estimation with EKF ');
    title(['Localization with EKF of speed sensor + UWB terna trilateration for robot', num2str(i)]);
    grid on;
    
    
    figure(i+9);
    for k = 1:indiciNonVuoti(end)
       estimation_error(k,i) = norm( robots(i).dynamics_history{k,1}(1) - EKFs{i}.state_history{k,1}(1),      robots(i).dynamics_history{k,1}(2) - EKFs{i}.state_history{k,1}(2));
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - EKFs{i}.state_history{k,1}(1),      robots(i).dynamics_history{k,1}(2) - EKFs{i}.state_history{k,1}(2)),'bo','MarkerSize', 2'');
       hold on
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - pose_est_UWB{i,k}(1),               robots(i).odometry_history{k,1}(2) - pose_est_UWB{i,k}(2)),'go','MarkerSize', 2'' );
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - robots(i).odometry_history{k,1}(1), robots(i).dynamics_history{k,1}(2) - robots(i).odometry_history{k,1}(2)),'ro','MarkerSize', 2'' );
    end
    xlabel('Steps');
    ylabel('Error norm [m]');
    legend('Error EKF', 'Error UWB', 'Error speed sensor')
    title(['Different types of error for robot', num2str(i)]);
    grid on;


    figure(i+11)
    for k = 1:indiciNonVuoti(end)
       plot(k, robots(i).dynamics_history{k,1}(3).*to_grad,'bo','MarkerSize', 2'');
       hold on
       plot(k, EKFs{i}.state_history{k,1}(3).*to_grad,'r*','MarkerSize', 2'');
    end
    xlabel('Step');
    ylabel('Coordinate theta [°]');
    legend('Real angle','Estimated angle with EKF of speed sensor + UWB');
    title(['Orientation with EKF of speed sensor + UWB terna trilateration for robot', num2str(i)]);
    grid on;

    figure(i+13)
    for k = 1:indiciNonVuoti(end)
       plot(k, robots(i).dynamics_history{k,1}(3).*to_grad,'bo','MarkerSize', 2'');
       hold on
       plot(k, pose_est_UWB{i,k}(3).*to_grad,'r*','MarkerSize', 2'');
    end
    xlabel('Step');
    ylabel('Coordinate theta [°]');
    legend('Real angle','Estimated angle with  UWB');
    title(['Orientation with  UWB terna trilateration for robot', num2str(i)]);
    grid on;

    hold off;
end

%% Histogram error

% For each robot
for i = 1:nRobots
    errors = [];
    for k = 1:indiciNonVuoti(end)
        errors = [errors, estimation_error(k,i)];
    end
    
    figure();
    histogram(errors, 10);
    xlabel('Error [m]');
    ylabel('Number of values');
    title(['Histogram of the error for robot ', num2str(i)]);
    grid on;
end


% Mean error
mean_error = mean(estimation_error,2);
idx = find(mean_error==0);
figure();
%plot(1:length(mean_error(1:idx(1)))-1,mean_error(1:idx(1)-1));
histogram(mean_error(1:idx(1)))
title('Histogram of mean error');
xlabel('Error [m]');
ylabel('Number of values');
grid on;


