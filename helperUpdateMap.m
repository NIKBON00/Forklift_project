function helperUpdateMap(robotHandle, pose, scale)
    if nargin < 3
        scale = .5;
    end
    
    % Update robot position
    t = pose(1:2);
    a = pose(3);
    R = [cos(a) -sin(a); sin(a) cos(a)];
    G = [R t(:); 0 0 scale];
    
    RobotBodyTriangleVertices = G*[[[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]]; ones(1,4)];
    
    robotHandle.XData = RobotBodyTriangleVertices(1,:);
    robotHandle.YData = RobotBodyTriangleVertices(2,:);
end