function nearxy = helperNearObstacles(xy)
     % Get nearest data item of each cluster
        if(size(xy,1)==1)
            nearxy = xy;
        else
            dist = vecnorm(xy,2,2); % AGV position is always at [0 0]
            [~, id] = min(dist);
            nearxy=xy(id,:);    
        end
end