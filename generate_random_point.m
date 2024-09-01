function point = generate_random_point(center, radius)
    
    % Generate random angle and radius in order to obtain points within a
    % circle
    angle = rand()*2*pi;
    radius_center = radius*sqrt(rand());

    % Generate random point
    point = [center(1) + radius_center* cos(angle), center(2) + radius_center*sin(angle)];

end