clc;
clear all;
close all;

%%
nRobots = 1;
robots=[];
dt = 0.01;
nM = 1000;
NaN = 1;
d=1.5;
weigh_init = 0;
K_r = 0.5;
K_l = 0.5;
x_target = 40;
y_target = 30;
Kp_v_t = 1;
Kp_omega = 2;
sigma_meas = diag([0.4 0.4 0.2]);

for i=1:nRobots

    inital_state = [1,1,0];% zeros(3,1);
    robot = Forklift_def(inital_state,d,dt,K_r,K_l,nM,NaN);
    robots = [robots,robot];

end


for i=1:nRobots
        MHEKFs(i) = EKF_def();
        MHEKFs(i).EKF_init(0,[1,1,0],zeros(3,3));
end


x_exact_robot = zeros(nRobots,nM);
y_exact_robot = zeros(nRobots,nM);

x_est_robot = zeros(nRobots,nM);
y_est_robot = zeros(nRobots,nM);

EKF_x = zeros(nRobots,nM);
EKF_y = zeros(nRobots,nM);


for i=1:nRobots
    for l=1:nM
        
        % Controlled input
        [v_t,omega] = controller(Kp_v_t,Kp_omega, x_target, y_target, robots(i).x);
        
        % Update exact dynamics and state estimation with noise
        x_next = robots(i).dynamics(v_t,omega);
        odometry = robots(i).odometry_step(v_t,omega);

        % Save data exact position
        x_exact_robot(i,l) = robots(i).x(1);
        y_exact_robot(i,l) = robots(i).x(2);

        % Save data estimated position with only acc
        x_est_robot(i,l) = robots(i).x_est(1);
        y_est_robot(i,l) = robots(i).x_est(2);

        % Do EKF prediction + correction
        MHEKFs(i).EKF_prediction(robots(i).odometry_estimation, dt,d);
        MHEKFs(i).EKF_correction(sigma_meas,[robots(i).x(1),robots(i).x(2),robots(i).x(3)]);
        EKF_x(i,l) = MHEKFs(i).x(1);
        EKF_y(i,l) = MHEKFs(i).x(2);
        
        
    end

end

% Estimated robot position using accelerometers vs exact position
figure()
plot(x_exact_robot(1,:),y_exact_robot(1,:),'bo','MarkerSize', 3'');
hold on
plot(x_est_robot(1,:),y_est_robot(1,:),'c*');
grid on;
xlabel('Coordinate x [m]');
ylabel('Coordinate y [m]');
legend('Exact position','Estimated with acc. position');
title('Effective position of robot vs position measured with only accelerometers');
hold off;

% USING EKF
figure();
plot(EKF_x(1,:), EKF_y(1,:),'c*');
hold on
plot(x_exact_robot(1,:),y_exact_robot(1,:),'bo','MarkerSize', 3'');
grid on;
xlabel('Coordinate x [m]');
ylabel('Coordinate y [m]');
legend('EKF position','Exact position');
title('Kalman filter');
hold off;

time = 0:dt:10-dt;

% Plot errors
figure()
plot(time,x_est_robot(1,:)-x_exact_robot(1,:),'k-');
hold on
plot(time,y_est_robot(1,:)-y_exact_robot(1,:),'g--');
grid on;
xlabel('Time [s]');
ylabel('Error [m]');
legend('Error x [m]','Error y [m]');
title('Error: estimated position using accelerometers vs exact position');
hold off;


figure()
plot(time,EKF_x(1,:)-x_exact_robot(1,:),'k-');
hold on
plot(time,EKF_y(1,:)-y_exact_robot(1,:),'g--');
grid on;
xlabel('Time [s]');
ylabel('Error [m]');
legend('Error x [m]','Error y [m]');
title('Error: estimated position using EFK (UWB + accelerometers) vs exact position');
hold off;
