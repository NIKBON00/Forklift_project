clc;
clear all;
close all;

%%
nRobots = 1;
robots=[];
dt = 0.01;
nM = 1000;
NaN = 1;
d=0.2;
weigh_init = 0;
K = 0.001;
x_target = 10;
y_target = 10;
Kp_a_t = 1;
Kp_a_w = 2;
sigma_meas = diag([0.01*rand() 0.01*rand() 0.01*rand() 0.01*rand() 0.01*rand()]);

for i=1:nRobots

    inital_state = zeros(9,1);
    robot = Forklift_def(inital_state,d,dt,K,nM,NaN);
    robots = [robots,robot];
end


for i=1:nRobots
    for l=1:nM
        MHEKFs(i,l) = EKF_def();
    end
end


figure();
for i=1:nRobots
    for l=1:nM
        
        MHEKFs(i,l).EKF_prediction(robots(i).odometry_estimation, dt,d);
        MHEKFs(i,l).EKF_correction(sigma_meas,[robots(i).x(1),robots(i).x(2),robots(i).x(3)],d);

        [a_t,a_w] = controller(Kp_a_t,Kp_a_w, x_target, y_target, robots(i).x_est);
        x_next = robots(i).dynamics(a_t,a_w);
        
        odometry = robots(i).odometry_step(a_t,a_w);
        plot(robots(i).x(1),robots(i).x(2),'bo','MarkerSize', 4'')
        disp(robots(i).x);
        hold on;
        grid on;
    end
    hold off;
end

