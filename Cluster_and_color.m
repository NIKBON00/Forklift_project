
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
   
            % Convert to normalized coordinate system (0-> minimum location of detection
            % area, 1->maximum position of detection area)
            a = [xy(:,1) xy(:,2)] - repmat(bbox([1 2]),[size(xy,1) 1]);
            b = repmat(bbox([3 4]),[size(xy,1) 1]);
            xy_org = a./b;
    
            % Coordinate system (0, 0)->(0, 0), (1, 1)->(width, height) of detArea
            idx = floor(xy_org.*repmat([size(detArea,2) size(detArea,1)],[size(xy_org,1) 1]));

            % Extracts as an index only the coordinates in detArea
            validIdx = 1 <= idx(:,1) & 1 <= idx(:,2) & ...
            idx(:,1) <= size(detArea,2) & idx(:,2) <= size(detArea,1);


            % Round the index and get the level of each obstacle from detArea
            cols = idx(validIdx,1);
            rows = idx(validIdx,2);
            levels = double(detArea(sub2ind(size(detArea),rows,cols)));
    
            if ~isempty(levels)
                level = max(levels);
                maxlevel = max(maxlevel,level);
                xyInds = find(validIdx);
                xyInds = xyInds(levels == level);
            
                % Get the nearest coordinates of obstacle in detection area
                nearxy(k,:) = helperNearObstacles(xy(xyInds,:));
            else
                % Get the nearest coordinates of obstacle in the cluster
                nearxy(k,:) = helperNearObstacles(xy);
            end
        end

        % Display a warning color representing the danger level. If the
        % obstacle does not fall in the detection area, do not display a color.
        switch maxlevel
            % Red region
            case 3
                circleDisplay(display,colors(4,:))
            % Orange region
            case 2
                circleDisplay(display,colors(3,:))
            % Yellow region
            case 1
                circleDisplay(display,colors(2,:))
            % Default case
            otherwise
                circleDisplay(display,[])
        end


         for k = 1:numClusters
            % Display obstacles if exist in the mentioned range of axes3
            sc(k,:) = displayObstacles(display,nearxy(k,:));
        end
        
        updateDisplay(display)
        pause(0.01)