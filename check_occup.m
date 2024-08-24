function [isOccupied] = check_occup(UWB_base,robot_pose,map)

isOccupied = 0;

% Generate 100 points between robot position and UWB base station
x_points = linspace(UWB_base(1), robot_pose(1),100);
y_points = linspace(UWB_base(2), robot_pose(2),100);

% Round the points to the center of the cell of map (because otherwise we
% are not sure that the linspace points will coincide with that on the map
grid_size = map.Resolution;
xindices = round(x_points*grid_size);
yindices = round(y_points*grid_size);

% Remove duplicate values
unique_points = unique([xindices',yindices'],'rows');

for jj = 1:size(unique_points,1)
    
    point = unique_points(jj, :) / grid_size; % Convert indices in coordinate (x,y)
    
    if checkOccupancy(map, point)
        isOccupied = 1;  % TRUE!
        break;
    end
end

end