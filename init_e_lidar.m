clc;
clear all;
close all;

%% PARAMETERS OF FIRST SIMULATION

nRobots = 1;
robots=[];
dt = 0.01;
nM = 2000;
NaN = 1;
d=1.5;
weigh_init = 0;
K_r = 0.03;
K_l = 0.03;
targets = [30,55];
Kp_v_t = 2;
Kp_omega = 50;
sigma_meas = [0.1 0.1 0.1]; 
to_grad = 180/pi;

% Time vectors
time = 0:dt:nM*dt-dt;

% Define some vectors useful for plot
x_real_robot = zeros(nRobots,nM);
y_real_robot = zeros(nRobots,nM);
theta_real_robot = zeros(nRobots,nM);

x_est_robot = zeros(nRobots,nM);
y_est_robot = zeros(nRobots,nM);
theta_est_robot = zeros(nRobots,nM);

EKF_x = zeros(nRobots,nM);
EKF_y = zeros(nRobots,nM);
EKF_theta = zeros(nRobots,nM);

inital_state = [10,40,0]; % zeros(3,1);

% Create a binary warehouse map and place obstacles at defined locations
map = helperCreateBinaryOccupancyMap;

% Visualize map with obstacles and AGV
figure();
show(map);
title('Warehouse Floor Plan With Obstacles and robot');
for l=1:nM
pose = [x_real_robot(1,l),y_real_robot(1,l),theta_real_robot(1,l)];
helperPlotRobot(gca,pose);
hold on
end

% Put lidar sensor
lidar = rangeSensor;
lidar.Range = [0 7];
lidar.HorizontalAngle = [-pi/2, pi/2];
lidar.HorizontalAngleResolution = pi/180;
lidar.RangeNoise = 0.01;

% Obstacle detection
obstacle = 0;

% Generate lidar readings
pose = zeros(1,3);

% Set up display
display = helperVisualizer;

% Plot warehouse map in the display window
hRobot = plotBinaryMap(display,map,inital_state);


%% ROBOT INITIALIZATION
for i=1:nRobots

    robot = Forklift_def(inital_state,d,dt,K_r,K_l,nM,NaN);
    robots = [robots,robot];

end

%% EKF INITIALIZATION
for i=1:nRobots
        MHEKFs(i) = EKF_def();
        MHEKFs(i).EKF_init(0,[10,40,0],zeros(3,3));
end

%% ROBOT TO TARGETS MOVEMENT
for i=1:nRobots
    
    target_considered = targets(1,:);
    
    for l=1:nM

        obstacle = 0;
        pose(i,:) = robots(i).x;
        % Generate lidar readings
        [ranges,angles] = lidar(pose(i,:),map);
        % Scan 2D
        scan = lidarScan(ranges,angles);
        
        % Store 2-D scan as point cloud
        cart = scan.Cartesian;
        cart(:,3) = 0;
        pc = pointCloud(cart); % the points generally represent the x,y, and z geometric coordinates for samples on a surface or of an environment
        
        % Segment point cloud into clusters based on euclidean distance
        minDistance = 0.9;
        [labels,numClusters] = pcsegdist(pc,minDistance); % segments a point cloud into clusters, with a minimum Euclidean distance of minDistance between points from different clusters
        
        % Update display map
        updateMapDisplay(display,hRobot,pose(i,:));
        
        % Plot 2-D lidar scans
        plotLidarScan(display,scan,pose(i,3));
    
        if exist('sc','var')
            delete(sc)
            clear sc
        end
    
    
        nearxy = zeros(numClusters,2);
        maxlevel = -inf;
        
        % Loop through all the clusters in pc
        for k = 1:numClusters
            c = find(labels == k);
            % XY coordinate extraction of obstacle
            xy = pc.Location(c,1:2);
        end
    
    
        for k = 1:numClusters
            % Display obstacles if exist in the mentioned range of axes3
            sc(k,:) = displayObstacles(display,nearxy(k,:));
        end
        updateDisplay(display)
        pause(0.01)

        ranges = isnan(ranges);

        for k = 1:length(ranges)
                
            if(ranges(k) ~= 1)
                obstacle = 1;
            end
        end

        % Search index for which obstacle is not detected
        i_index = find(ranges);
        i_index_obs = find(~ranges);
        min_distance = min(ranges(i_index_obs));

        disp(obstacle);

        % Controlled input
        [v_t,omega] = controller(Kp_v_t,Kp_omega, target_considered(1),target_considered(2),MHEKFs(i).x,obstacle,i_index,min_distance);

        % Update exact kinematics and state estimation with noise
        x_next = robots(i).dynamics(v_t,omega);
        odometry = robots(i).odometry_step(v_t,omega);

        % Save data exact position
        x_real_robot(i,l) = robots(i).x(1);
        y_real_robot(i,l) = robots(i).x(2);
        theta_real_robot(i,l) = robots(i).x(3);

        % Save data estimated position with only speed sensor
        x_est_robot(i,l) = robots(i).x_est(1);
        y_est_robot(i,l) = robots(i).x_est(2);
        theta_est_robot(i,l) = robots(i).x_est(3);


        % Do EKF prediction + correction
        MHEKFs(i).EKF_prediction(robots(i).odometry_estimation, dt,d);
        MHEKFs(i).EKF_correction(sigma_meas,[robots(i).x(1),robots(i).x(2),robots(i).x(3)]);
        EKF_x(i,l) = MHEKFs(i).x(1);
        EKF_y(i,l) = MHEKFs(i).x(2);
        EKF_theta(i,l) = MHEKFs(i).x(3);

      

        % When I'm close to first target change to second
        if (robots(i).getDistance(target_considered(1),target_considered(2)) < 1  )
            %target_considered = targets(2,:);
        end

 

    end

end


%% ERROR PLOTS
% Estimated robot position using speed sensors vs exact position
figure();
plot(x_real_robot(1,:),y_real_robot(1,:),'bo','MarkerSize', 2'');
hold on
plot(x_est_robot(1,:),y_est_robot(1,:),'c*','MarkerSize', 2'');
grid on;
xlabel('Coordinate x [m]');
ylabel('Coordinate y [m]');
legend('Real position','Estimated position with speed sens.');
title('Real position of robot vs position measured with only speed sensor');
hold off;

% USING EKF
figure();
plot(EKF_x(1,:), EKF_y(1,:),'c*','MarkerSize', 2'');
hold on
plot(x_real_robot(1,:),y_real_robot(1,:),'bo','MarkerSize', 2'');
grid on;
xlabel('Coordinate x [m]');
ylabel('Coordinate y [m]');
legend('EKF state','Real state');
title('Kalman filter');
hold off;


% Plot errors
figure()
plot(time,x_est_robot(1,:)-x_real_robot(1,:),'k--');
hold on
plot(time,y_est_robot(1,:)-y_real_robot(1,:),'g--');
grid on;
xlabel('Time [s]');
ylabel('Error [m]');
legend('Error x [m]','Error y [m]');
title('Error: estimated state using speed sensors vs Real state (x,y,theta)');
hold off;

figure()
plot(time,(theta_est_robot(1,:)-theta_real_robot(1,:)).*to_grad,'b--');
grid on;
xlabel('Time [s]');
ylabel('Error [°]');
legend('Error theta [°]');
title('Error: estimated state using speed sensors vs Real state (x,y,theta)');
hold off;


figure()
plot(time,EKF_x(1,:)-x_real_robot(1,:),'k--');
hold on
plot(time,EKF_y(1,:)-y_real_robot(1,:),'g-');
grid on;
xlabel('Time [s]');
ylabel('Error [m]');
legend('Error x [m]','Error y [m]');
title('Error: estimated state using EFK (UWB + speed sensors) vs Real position');
hold off;


figure();
plot(time,(EKF_theta(1,:)-theta_real_robot(1,:)).*to_grad,'b--');
grid on;
xlabel('Time [s]');
ylabel('Error [°]');
legend('Error theta [°]');
title('Error: estimated state using EFK (UWB + speed sensors) vs Real position');
hold off;





