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
targets = [30,25];
Kp_v_t = 2;
Kp_omega = 3;
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

%% ROBOT INITIALIZATION
for i=1:nRobots

    inital_state = [0,0,0]; % zeros(3,1);
    robot = Forklift_def(inital_state,d,dt,K_r,K_l,nM,NaN);
    robots = [robots,robot];

end

%% EKF INITIALIZATION
for i=1:nRobots
        MHEKFs(i) = EKF_def();
        MHEKFs(i).EKF_init(0,[0,0,0],zeros(3,3));
end

%% ROBOT TO TARGETS MOVEMENT
for i=1:nRobots
    
    target_considered = targets(1,:);
    
    for l=1:nM

        % Controlled input
        [v_t,omega] = controller(Kp_v_t,Kp_omega, target_considered(1),target_considered(2),robots(i).x_est);
        
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













