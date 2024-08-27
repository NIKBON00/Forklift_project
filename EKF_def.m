classdef EKF_def<handle

    properties
        x;                      % filter of the state 3*1
        P;                      % covariance matrix state of estimation errors 3*3
        state_history;          % history of the state
        innovation_history;     % history of innovations
    end


    methods
        
        
        % Defition of EKF objects
        function obj = EKF_def()
            obj.x = zeros(3,1);
            obj.P = zeros(3,3);
            obj.state_history=[];
            obj.innovation_history=[];
        end


        % Initialization of EKF object
        function EKF_init(obj,robot_state,robot_cov_matrix)

            % Initialize state vehicle
            x_0 = robot_state(1);
            y_0 = robot_state(2);
            theta_0 = robot_state(3);
           
            % State
            obj.x = [x_0;y_0;theta_0];

            % Define covariance matrix P
            for i=1:length(obj.x)
                for j = 1:length(obj.x)
                        obj.P(i,j) = robot_cov_matrix(i,j); % example: obj.P(1,1) = robot_cov_matrix(1,1); obj.P(1,2) = robot_cov_matrix(1,2),...; 
                end
            end

        end


        % Function that computes the prediction step of the filter
        function EKF_prediction(obj,speed_readings,dt,d,R)

            % Input linear and angular velocities of robot center 
            o_v_t = speed_readings{1,1}(1);
            o_omega = speed_readings{1,1}(2);

            % Input noise covariance matrix
            Q = speed_readings{1,2};
                        
            % Current states
            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);
           
            % Current covariance state estimation matrix
            P_curr = obj.P;

            % Jacobian matrix of partial derivates of the model wrt the state
            F = zeros(3,3);

            % Jacobian matrix of partial derivates of the model wrt odometry noise
            W = zeros(3,2);


            %% PREDICTION STEP
            % STATE KINEMATICS
            x_next = x_curr + o_v_t*dt*cos(theta_curr + dt*o_omega/2);
            y_next = y_curr + o_v_t*dt*sin(theta_curr + dt*o_omega/2);
            theta_next = theta_curr + o_omega*dt;
            
            % Jacobian wrt State
            F(1,1) = 1;
            F(1,3) = -o_v_t*dt*sin(theta_curr + dt*o_omega/2);
            F(2,2) = 1;
            F(2,3) = o_v_t*dt*cos(theta_curr  + dt*o_omega/2);
            F(3,3) = 1;

            % Jacobian wrt Noise
            W(1,1) = 0.5*dt*R*cos(theta_curr + dt*o_omega/2) - (o_v_t*dt/2)*sin(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(1,2) = 0.5*dt*R*cos(theta_curr + dt*o_omega/2) + (o_v_t*dt/2)*sin(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(2,1) = 0.5*dt*R*sin(theta_curr + dt*o_omega/2) + (o_v_t*dt/2)*cos(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(2,2) = 0.5*dt*R*sin(theta_curr + dt*o_omega/2) - (o_v_t*dt/2)*cos(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(3,1) = R*dt/d;
            W(3,2) = -R*dt/d;
        
            % Update state
            obj.x = [x_next;y_next;theta_next];

        
           %% PREDICTION STEP COVARIANCE STATE
           P_next = F*P_curr*F' + W*Q*W';
           % Update covariance matrix
           obj.P = P_next;

        end


        % Function that does the update/correction step of the filter
        function EKF_correction(obj,sigma_meas,measurements_readings)

            % Rename state and covariance matrix
            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);
            
            P_curr = obj.P;

            % Measurement noise covariance matrix
            R = diag([sigma_meas(1) sigma_meas(2) sigma_meas(3)]); % [sigma_x, sigma_y, sigma_theta]
            
            % Jacobian matrix of measurements wrt state
            H = diag([1 1 1]);  

            % Compute the Kalman Gain
            Kalman_gain = P_curr*H'*pinv(H*P_curr*H'+R);

            % Calculation of innovation
            innovation_theta = measurements_readings(3)-theta_curr;
            innovation_theta = atan2(sin(innovation_theta),cos(innovation_theta));
            innovation = [measurements_readings(1) - x_curr, measurements_readings(2) - y_curr, innovation_theta]';

            % Save the innovation
            obj.innovation_history = [obj.innovation_history,innovation];

            % Update state
            state_next = [x_curr;y_curr; theta_curr] + Kalman_gain*innovation;

            % Update state covariance matrix using Joseph Form
            P_next = (eye(3)-Kalman_gain*H)*P_curr*(eye(3)-Kalman_gain*H)' + Kalman_gain*R*Kalman_gain';

            % Save state and covariance matrix
            x_next = state_next(1);
            y_next = state_next(2);
            theta_next = state_next(3);
            obj.x = [x_next,y_next,theta_next];

            obj.P = P_next;


        end

        end

    
end