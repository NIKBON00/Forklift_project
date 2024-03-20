classdef EKF_def<handle

    properties
        x; % filter of the state 3*1
        P; % covariance matrix state of estimation errors 3*3
        weight; % weight of EKF instance
        weight_history; % history of the weights of the EKF instance
        state_history; % history of the state
        innovation_history; % history of innovations
 
    end


    methods
        
        
        function obj = EKF_def()
            obj.x = zeros(3,1);
            obj.P = zeros(3,3);
            obj.weight = 0;
            obj.weight_history=[];
            obj.state_history=[];
            obj.innovation_history=[];
        end


        function EKF_init(obj,weight_init,robot_state,robot_cov_matrix)

            
            % Initialize state vehicle
            x_0 = robot_state(1);
            y_0 = robot_state(2);
            theta_0 = robot_state(3);
           
            % State
            obj.x = [x_0;y_0;theta_0];

            % Define covariance matrix P
            for i=1:length(obj.x)
                for j = 1:length(obj.x)
           
                        obj.P(i,j) = robot_cov_matrix(i,j); % obj.P(1,1) = robot_cov_matrix(1,1); obj.P(1,2) = robot_cov_matrix(1,2),...; 

                end
            end



            obj.weight = weight_init;
            obj.weight_history = [obj.weight_history; obj.weight];

        end


        % Function that computes the prediction step of state
        function EKF_prediction(obj,accelerometer_readings,dt,d)

            o_v_t = accelerometer_readings{1,1}(1);
            o_omega = accelerometer_readings{1,1}(2);

  
            % process noise covariance matrix
            Q = accelerometer_readings{1,2};
            
            
            % Current states
            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);

            % Current covariance state estimation errors
            P_curr = obj.P;

  

            %% PREDICTION STEP
            % STATE DYNAMICS

            % Change reference frame from t-n to x-y
            v_x_curr = o_v_t*cos(theta_curr);
            v_y_curr = o_v_t*sin(theta_curr);


            a_x_curr = (o_v_t/dt)*cos(theta_curr);
            a_y_curr = (o_v_t/dt)*sin(theta_curr);
            

            % State Dynamics
            x_next = x_curr +  dt*v_x_curr + 0.5*a_x_curr*dt^2;
            y_next = y_curr +  dt*v_y_curr + 0.5*a_y_curr*dt^2;
            theta_next = theta_curr + dt*o_omega + 0.5*o_omega*dt;


            % State
            obj.x = [x_next;y_next;theta_next];


            %% JACOBIAN OF STATE
           F = zeros(3,3);
           F(1,1) = 1;
           F(2,2) = 1;
           F(3,3) = 1;


           %% JACOBIAN OF NOISE
           W = zeros(3,2);
           W(1,1) = 0.25*cos(theta_curr)*dt^2 + cos(theta_curr)*dt/2;
           W(1,2) = 0.25*cos(theta_curr)*dt^2 + cos(theta_curr)*dt/2;
           W(2,1) = 0.25*sin(theta_curr)*dt^2 + sin(theta_curr)*dt/2;
           W(2,2) = 0.25*sin(theta_curr)*dt^2 + sin(theta_curr)*dt/2;
           W(3,1) = 1.5*(1/d)*dt^2;
           W(3,2) = - 1.5*(1/d)*dt^2;
           

           
           %% PREDICTION STEP COVARIANCE STATE

           P_next = F*P_curr*F'+ W*Q*W';
           obj.P = P_next;

        end


        % Function that does the correction step of the filter
        function EKF_correction(obj,sigma_meas,measurements_readings)

            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);

            P_curr = obj.P;

            % Measurement noise covariance matrix
            R = diag([sigma_meas(1) sigma_meas(2) sigma_meas(3)]); 
            
            % Jacobian matrix of output
            H = zeros(3,3);
            H(1,1) = 1;
            H(2,2) = 1;
            H(3,3) = 1;


            Kalman_gain = P_curr*H'*pinv(H*P_curr*H'+R);


            % Calculation of innovation
            innovation = [measurements_readings(1) - x_curr, measurements_readings(2) - y_curr, measurements_readings(3)-theta_curr]';

            obj.innovation_history = [obj.innovation_history;innovation];

            state_next = [x_curr;y_curr;theta_curr] + Kalman_gain*innovation;

            P_next = (eye(3)-Kalman_gain*H)*P_curr;

            % Update the weight
            obj.weight = obj.weight*exp(-0.5*innovation.^2./(H*P_next*H'+R));
            %obj.weight_history = [obj.weight_history;obj.weight];

            x_next = state_next(1);
            y_next = state_next(2);
            theta_next = state_next(3);
            
            

            obj.x = [x_next,y_next,theta_next];

            obj.P = P_next;


        end

        end

    
end