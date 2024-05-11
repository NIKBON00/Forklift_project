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
lidar.Range = [0 5];
lidar.HorizontalAngle = [-pi/2, pi/2];
lidar.HorizontalAngleResolution = pi/180;
lidar.RangeNoise = 0.01;

% Generate lidar readings
pose = zeros(nM,3);
pose(:,1) = x_real_robot(1,:)';
pose(:,2) = y_real_robot(1,:)';
pose(:,3) = theta_real_robot(1,:)';

% Set up display
display = helperVisualizer;

% Plot warehouse map in the display window
hRobot = plotBinaryMap(display,map,inital_state);



for i=1:nM
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
    for i = 1:numClusters
        c = find(labels == i);
        % XY coordinate extraction of obstacle
        xy = pc.Location(c,1:2);
    end


    for i = 1:numClusters
        % Display obstacles if exist in the mentioned range of axes3
        sc(i,:) = displayObstacles(display,nearxy(i,:));
    end
    updateDisplay(display)
    pause(0.01)
end
