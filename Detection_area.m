
figure
detAxes = gca;
title(detAxes,'Define Detection Area')
axis(detAxes,[-2 10 -2 4])
xlabel(detAxes,'X')
ylabel(detAxes,'Y')
axis(detAxes,'equal')
grid(detAxes,'minor')

t = linspace(-pi/2,pi/2,30)';

% Specify color values - white, yellow, orange, red
colors = [1 1 1; 1 1 0; 1 0.5 0; 1 0 0];

% Specify radius in meters
radius = [5 2 1];

% Create a 3x1 matrix of type Polygon
detAreaHandles = repmat(images.roi.Polygon,[3 1]);

pos = [cos(t) sin(t)]*radius(1);
pos = [0 -2; pos(14:17,:); 0 2];
detAreaHandles(1) = drawpolygon( ...
	'Parent',detAxes, ...
	'InteractionsAllowed','reshape', ...
	'Position',pos, ...
	'StripeColor','black', ...
	'Color',colors(2,:));

pos = [cos(t) sin(t)]*radius(2);
pos = [0 -1.5; pos(12:19,:); 0 1.5];
detAreaHandles(2) = drawpolygon( ...
	'Parent',detAxes, ...
	'InteractionsAllowed','reshape', ...
	'Position',pos, ...
	'StripeColor','black', ...
	'Color',colors(3,:));

pos = [cos(t) sin(t)]*radius(3);
pos = [0 -1; pos(10:21,:); 0 1];
detAreaHandles(3) = drawpolygon( ...
    'Parent',detAxes, ...
	'InteractionsAllowed','reshape', ...
	'Position',pos, ...
	'StripeColor','black', ...
	'Color',colors(4,:));

% Pausing for the detection area window to load
pause(2) 