classdef Forklift_def < handle

    properties
        x;                   % state of the robot 3*1
        x_est;               % estimated state of the robot 3*1
 
        % OTHERS
        dt;                  % Time integration step [s]
        d;                   % axes width [m]
        R;                   % radius wheel [m]
        K_r;                 % Noise parameter right wheel [-]
        K_l;                 % Noise parameter left wheel [-]

        dynamics_history;    % True Kinematics history without noise
        odometry_history;    % Estimated Kinematics history with only wheel rotational speed sensor (with noise)
        odometry_estimation; % cell with{[linear velocity, rotational velocity], input noise covariance matrix Q }

        P_cov;               % state covariance matrix

    end


    methods
        
        function obj = Forklift_def(initial_state,d,R,dt,K_r,K_l)

            % State
            obj.x=zeros(3,1);           % initialize to 0 all the states
            obj.x_est = zeros(3,1);     % initialize to 0 all the estimated states
            
            % Then assign to obj.x the initial state we pass to it
            for i =1:length(initial_state)
                obj.x(i) = initial_state(i);
                obj.x_est(i) = initial_state(i);
            end


            % Data 
            obj.d = d;                  % width [m]
            obj.R = R;                  % wheel radius [m]
            
            obj.dt = dt;                % Integration time step [s]
            obj.K_r = K_r;              % Noise parameter right wheel [-]
            obj.K_l = K_l;              % Noise parameter left wheel [-]

            obj.dynamics_history = {};  % True Kinematics history without noise
            obj.odometry_history = {};  % Estimated Kinematics history with only wheel rotational speed sensor (with noise)

            obj.odometry_estimation = {[0,0], zeros(2,2)};    % cell with{[linear velocity, rotational velocity], input noise covariance matrix Q }

            obj.P_cov = zeros(3,3);     % state covariance matrix

        end


        % True Kinematics (without wheel rotational speed noise)
        function x_next = dynamics(obj,u_v_t,u_omega)

            % Speed sensor measurements retrieve
            [omega_r,omega_l] = calculus_vec(obj,u_v_t,u_omega);

            % Linear increment for each wheel
            delta_s_l = obj.dt*obj.R*omega_l;
            delta_s_r = obj.dt*obj.R*omega_r;

            % Linear increment of the center of the robot
            delta_s = (delta_s_r + delta_s_l)/2;
            % Angular increment of the orientation of the robot
            delta_theta = (delta_s_r - delta_s_l)/obj.d;
            
            % Update kinematics
            obj.x(1) = obj.x(1) + delta_s*cos(obj.x(3) + delta_theta/2);
            obj.x(2) = obj.x(2) + delta_s*sin(obj.x(3) + delta_theta/2);
            obj.x(3) = obj.x(3) + delta_theta;

            % Update state
            x_next = [obj.x(1) obj.x(2) obj.x(3)];
        
        end


        % Kinematics considering wheel rotational speed noise
        function odometry_estimation = odometry_step(obj,u_v_t,u_omega)
            
            % Speed sensor measurements retrieve
            [omega_r,omega_l] = calculus_vec(obj,u_v_t,u_omega);

            % Adding noise to speed sensor measurements
            eta_r = normrnd(0,sqrt(obj.K_r*abs(omega_r)));
            eta_l = normrnd(0,sqrt(obj.K_l*abs(omega_l)));

            % Noisy measures of wheel rotational speed
            omega_r = omega_r + eta_r;
            omega_l = omega_l + eta_l;

            % Noisy traslational and rotational velocity of robot center
            vt_est = obj.R * (omega_r + omega_l)/2;
            omega_est = obj.R * (omega_r - omega_l)/obj.d;

            % Noisy linear increment for each wheel
            delta_s_l_est = obj.dt*obj.R*omega_l;
            delta_s_r_est = obj.dt*obj.R*omega_r;

            % Noisy linear increment of the center of the robot
            delta_s_est = (delta_s_r_est + delta_s_l_est)/2;
            % Noisy angular increment of the orientation of the robot
            delta_theta_est = (delta_s_r_est - delta_s_l_est)/obj.d;

            % Noisy kinematics
            obj.x_est(1) = obj.x_est(1) + delta_s_est*cos(obj.x_est(3) + delta_theta_est/2);
            obj.x_est(2) = obj.x_est(2) + delta_s_est*sin(obj.x_est(3) + delta_theta_est/2);
            obj.x_est(3) = obj.x_est(3) + delta_theta_est;
                    
            % Input noise covariance matrix
            Q = [obj.K_r*abs(omega_r)^2, 0; 0, obj.K_l*abs(omega_l)^2];  
            % Odometry estimation
            obj.odometry_estimation = {[vt_est omega_est], Q};
            odometry_estimation = obj.odometry_estimation;
            
        end


        % Retrieve sensor velocity measurements from input velocities
        function  [omega_r,omega_l] = calculus_vec(obj,v_t,omega)
           
           % Calculus of right and left wheels speed 
           omega_r = (2*v_t + omega*obj.d)/(2*obj.R);
           omega_l = (2*v_t - omega*obj.d)/(2*obj.R);
           
        end

    end

end

