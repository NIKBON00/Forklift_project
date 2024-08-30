clc;
clear all;
close all;

%% PARAMETERS OF FIRST SIMULATION

% Simulation Set up: 
Setup.dt = 0.01;                    % [s]
Setup.Time = 0:Setup.dt:100;        % [s]

% Number of steps:
Setup.steps = Setup.Time/Setup.dt;  % [-]

% Number of robots:
nRobots = 4;
robots=[];                          % Vector of robot structures

% Parameter of robot:
d = 1.5;                            % width [m]
L = 2.8;                            % length [m]
R = 0.23;                           % radius wheel [m]
K_r = 0.02;                         % costant parameter contributing to noise in speed right sensor 
K_l = 0.02;                         % costant parameter contributing to noise in speed left  sensor
sigma_meas = [0.1 0.1 0.06];        % noise on UWB

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
to_grad = 180/pi;                   % [°]
to_rad = pi/180;                    % [rad]

% Definition of vectors useful for plotting part:
pose_est_UWB = cell(nRobots, length(Setup.steps));
EKF_pose = cell(nRobots, length(Setup.steps));

h1 = cell(nRobots,1);               % useful for delete precedent position of robot during plotting


% Robots have reached the obstacle? YES = 1, NO = 0. At the beginning set
% the value to 0
reached = zeros(nRobots,1);

% Location of UWB antenna:
UWB_sens = [10,10; 10,40; 70,10; 70,50; 40, 36; 51, 22];

% Position of UWB tags on the robot in robot reference frame, necessary 2
% tags
robots_tags = [0, L/2; 0, -L/2]; 


%% ROBOT INITIALIZATION

for i=1:nRobots
    inital_state{i} = [2, 2 , 0];                                          % Initial state of the robot
    robot = Forklift_def(inital_state{i},d,R,Setup.dt,K_r,K_l);            % initialization of robot structure  
    robots = [robots,robot];                                               % Add i-th robot to global robot vector
end

%% EKF INITIALIZATION

for i=1:nRobots
        EKFs{i} = EKF_def();                                                    % EKF definition
        EKFs{i}.EKF_init(inital_state{i},diag([(2/3)^2, (2/3)^2, (0.3/3)^2]));  % initial uncertainty on intial robot state
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

% Define lidar sensor parameters
lidar = rangeSensor;
lidar.Range = [0 5];
lidar.HorizontalAngle = [-pi/2, pi/2];
lidar.HorizontalAngleResolution = pi/180;
lidar.RangeNoise = 0.01;

% Set up display
display = helperVisualizer;

% Create an array of cell useful to plot the robots
hRobot = cell(nRobots, 1);

% Plot warehouse map in the display window
for i = 1:nRobots
    hRobot{i} = plotBinaryMap(display,map,inital_state{i});
end


%% ROBOT TO TARGETS MOVEMENT

% Plot figure of warehouse with robots movements
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
            minDistance = 1.5*d;
            [labels,numClusters] = pcsegdist(pc,minDistance); % segments a point cloud into clusters, with a minimum Euclidean distance of minDistance between points from different clusters
    
            % Plot exact robot position and orientation
            plot(robots(i).x(1), robots(i).x(2),'.');
            h1{i} = helperPlotRobot(gca,[robots(i).x(1), robots(i).x(2) robots(i).x(3)]);
            

            % Plot targets
            for ttt = 1:size(targets,1)
                plot(targets(ttt,1),targets(ttt,2),'r*');
                hold on;
            end

            % Plot UWB antennas on the map
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
            EKFs{i}.EKF_prediction(robots(i).odometry_estimation,Setup.dt,d,R);                                     % Prediction
            EKFs{i}.EKF_correction(sigma_meas, [pose_est_UWB{i,k}(1) pose_est_UWB{i,k}(2) pose_est_UWB{i,k}(3)]);   % Update
            EKFs{i}.state_history{k,1} = EKFs{i}.x;                                                                 % Save the results

            % Update exact kinematics
            x_next = robots(i).dynamics(v_t,omega);

            % Save in kinematic history
            robots(i).dynamics_history{k,1} = x_next;

            % Update estimated state with only speed sensor
            odometry = robots(i).odometry_step(v_t,omega);

            % Kinematic history estimated position
            robots(i).odometry_history{k,1} = robots(i).x_est;

    
            % When the robot estimates that is very close to target change says
            % he has reached it
            if (norm(EKFs{i}.x(1) - target_considered(1),EKFs{i}.x(2) - target_considered(2)) < 0.3 )
                % i-th robot has reached the target
                reached(i) = 1;
            end

          
            
    end

    % Pause for plot the robots on the map
    pause(0.01);
    
    % Eliminate the previous position of robot on the map
    for i=1:nRobots
        delete(h1{i});
    end

    % If every robot has reached it's own target exit the cycle
    if (reached)
        break;
    end

end


%% ERROR PLOTS

% Estimation errors, respectively position and orientation
estimation_error = zeros(length(Setup.steps),nRobots);
estimation_error_theta = zeros(length(Setup.steps),nRobots);
estimation_error_trace = zeros(length(Setup.steps),nRobots);

for i = 1:nRobots

    tmp = cell(1, length(Setup.steps));
    for k = 1:length(Setup.steps)
    tmp{1,k} = pose_est_UWB{i,k}(1:end);
    end
    indiciNonVuoti = find(~cellfun('isempty', tmp));
    
    
    % Estimated robot position using terna trilateration of UWB sensor
    figure();
    for k = 1:indiciNonVuoti(end)
       estimation_error(k,i) = norm( robots(i).dynamics_history{k,1}(1) - EKFs{i}.state_history{k,1}(1),      robots(i).dynamics_history{k,1}(2) - EKFs{i}.state_history{k,1}(2));
       plot(robots(i).dynamics_history{k,1}(1), robots(i).dynamics_history{k,1}(2),'go','MarkerSize', 2'');
       hold on
       plot(robots(i).odometry_history{k,1}(1), robots(i).odometry_history{k,1}(2),'bo','MarkerSize', 2'');
       plot(pose_est_UWB{i,k}(1), pose_est_UWB{i,k}(2),'r*','MarkerSize', 2'');
       plot(EKFs{i}.state_history{k,1}(1), EKFs{i}.state_history{k,1}(2),'-s','MarkerSize',2'');
    end
    xlabel('Coordinate x [m]');
    ylabel('Coordinate y [m]');
    legend('True posture','Speed sensor pos. estimate','UWB sensor pos. estimate','EKF pos. estimate');
    title(['Position estimation for robot ', num2str(i)]);
    grid on
    

%{
    figure();
    for k = 1:indiciNonVuoti(end)
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - robots(i).odometry_history{k,1}(1), robots(i).dynamics_history{k,1}(2) - robots(i).odometry_history{k,1}(2)),'go','MarkerSize', 2'')
       hold on
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - pose_est_UWB{i,k}(1), robots(i).dynamics_history{k,1}(2) - pose_est_UWB{i,k}(2)),'bo','MarkerSize', 2'');
       hold on
    end
    xlabel('Steps');
    ylabel('Error norm [m]');
    legend('Error norm only speed sensor', 'error norm only UWB')
    title(['Error norm with consensus algorithm UWB terna only for robot ', num2str(i)]);
    grid on;
%}

    
 %{
    figure();
    for k = 1:indiciNonVuoti(end)
        plot(robots(i).dynamics_history{k,1}(1), robots(i).dynamics_history{k,1}(2),'go','MarkerSize', 2'');
        hold on
        plot(EKFs{i}.state_history{k,1}(1), EKFs{i}.state_history{k,1}(2),'r*','MarkerSize', 2'')
    end
    xlabel('Coordinate x [m]');
    ylabel('Coordinate y [m]');
    legend('True posture','Estimation with EKF ');
    title(['Localization with EKF of speed sensor + UWB terna trilateration for robot ', num2str(i)]);
    grid on;
 %}
    
    %{
    figure();
    for k = 1:indiciNonVuoti(end)
       estimation_error(k,i) = norm( robots(i).dynamics_history{k,1}(1) - EKFs{i}.state_history{k,1}(1),      robots(i).dynamics_history{k,1}(2) - EKFs{i}.state_history{k,1}(2));
       plot(k, estimation_error(k,i),'bo','MarkerSize', 2'');
       hold on
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - pose_est_UWB{i,k}(1),               robots(i).odometry_history{k,1}(2) - pose_est_UWB{i,k}(2)),'go','MarkerSize', 2'' );
       plot(k, norm( robots(i).dynamics_history{k,1}(1) - robots(i).odometry_history{k,1}(1), robots(i).dynamics_history{k,1}(2) - robots(i).odometry_history{k,1}(2)),'ro','MarkerSize', 2'' );
    end
    xlabel('Steps');
    ylabel('Error norm [m]');
    legend('Error EKF', 'Error UWB', 'Error speed sensor')
    title(['Different types of error for robot ', num2str(i)]);
    grid on;
    
    %}
  
    %{
    figure()
    for k = 1:indiciNonVuoti(end)
       plot(k, robots(i).dynamics_history{k,1}(3).*to_grad,'go','MarkerSize', 2'');
       hold on
       plot(k, robots(i).odometry_history{k,1}(3).*to_grad, 'bo','MarkerSize', 2'')
       plot(k, pose_est_UWB{i,k}(3).*to_grad,'r*','MarkerSize', 2'');
       plot(k, EKFs{i}.state_history{k,1}(3).*to_grad,'-s','MarkerSize', 2'');
    end
    xlabel('Step');
    ylabel('Orientation \theta [°]');
    legend('True orient.','Speed sensor orient. estimate','UWB orient. estimate','EKF orient. estimate');
    title(['Orientation estimation for robot ', num2str(i)]);
    grid on;
    
    %}

     figure()
    for k = 1:indiciNonVuoti(end)
        esti_speed = abs( (robots(i).dynamics_history{k,1}(3) -robots(i).odometry_history{k,1}(3)) ).*to_grad;
        esti_UWB = abs( (robots(i).dynamics_history{k,1}(3) - pose_est_UWB{i,k}(3) )).*to_grad;
        estimation_error_theta(k,i) = abs((robots(i).dynamics_history{k,1}(3) - EKFs{i}.state_history{k,1}(3))).*to_grad;
        estimation_error_trace(k,i) = trace(EKFs{i}.P);

       plot(k,esti_speed,'ro','MarkerSize', 2'');
       hold on
       plot(k,esti_UWB,'go','MarkerSize', 2'');
       plot(k, estimation_error_theta(k,i),'bo','MarkerSize', 2'');
       hold on
    end
    xlabel('Step');
    ylabel('Error on \theta [°]');
    legend('Error speed orient. estimate','Error UWB orient. estimate','Error EFK orient. estimate');
    title(['Different types of error for robot ', num2str(i)]);
    grid on;
   

    hold off;
end

%% Histogram error position

% Errors on position (x,y), orientation and P_trace
E_position = zeros(i,1);
E_orientation = zeros(i,1);
E_trace = zeros(i,1);

% For each robot
for i = 1:nRobots

    tmp = cell(1, length(Setup.steps));
    for k = 1:length(Setup.steps)
        tmp{1,k} = pose_est_UWB{i,k}(1:end);
    end
    indiciNonVuoti = find(~cellfun('isempty', tmp));

    errors = [];
    errors_trace = [];
    for k = 1:indiciNonVuoti(end)
        errors = [errors; estimation_error(k,i)];
        errors_trace = [errors_trace; estimation_error_trace(k,i)];
    end

    E_position(i) = sum(errors)/k;
    E_trace(i) = sum(errors_trace)/k;

    
    figure();
    histogram(errors, 10);
    xlabel('Error [m]');
    ylabel('Number of values');
    title(['Histogram of the position error for robot ', num2str(i)]);
    grid on;
    hold off;
end

% Total position and P_trace errors
E_position = sum(E_position)/i;  % [m]
E_trace = sum(E_trace)/i;


% Mean error. To compute it we remove the 0 terms in the estimation_error
% matrix
mean_error = mean(estimation_error,2);
idx = find(mean_error==0);
figure();
%plot(1:length(mean_error(1:idx(1)))-1,mean_error(1:idx(1)-1));
histogram(mean_error(1:idx(1)))
title('Histogram of position mean error of all the robots');
xlabel('Error [m]');
ylabel('Number of values');
grid on;

%% Histogram error orientation

for i = 1:nRobots
    
    tmp = cell(1, length(Setup.steps));
    for k = 1:length(Setup.steps)
        tmp{1,k} = pose_est_UWB{i,k}(1:end);
    end
    indiciNonVuoti = find(~cellfun('isempty', tmp));
    
    errors = [];
    for k = 1:indiciNonVuoti(end)
        errors = [errors, estimation_error_theta(k,i)];
    end
    
    E_orientation(i) = sum(errors)/k;

    figure();
    histogram(errors, 10);
    xlabel('Error [°]');
    ylabel('Number of values');
    title(['Histogram of the orientation error for robot ', num2str(i)]);
    grid on;
end

% Total orientation error
E_orientation = sum(E_orientation)/i;   % [°]


% Mean error. To compute it we remove the 0 terms in the estimation_error_theta
% matrix
mean_error = mean(estimation_error_theta,2);
idx = find(mean_error==0);
figure();
%plot(1:length(mean_error(1:idx(1)))-1,mean_error(1:idx(1)-1));
histogram(mean_error(1:idx(1)))
title('Histogram of orientation mean error of all the robots');
xlabel('Error [°]');
ylabel('Number of values');
grid on;





