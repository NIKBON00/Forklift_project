classdef EKF_def<handle

    properties
        x; % filter of the state 9*1
        P; % covariance matrix state of estimation errors 9*9
        weight; % weight of EKF instance
        weight_history; % history of the weights of the EKF instance
        state_history; % history of the state
        innovation_history; % history of innovations
 
    end


    methods
        
        
        function obj = EKF_def()
            obj.x = zeros(9,1);
            obj.P = zeros(9,9);
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
            v_t_0 = robot_state(4);
            v_n_0 = robot_state(5);
            omega_0 = robot_state(6);
            a_t_0 = robot_state(7);
            a_n_0 = robot_state(8);
            a_omega_0 = robot_state(9);

            obj.x = [x_0,y_0,theta_0,v_t_0,v_n_0,omega_0,a_t_0,a_n_0,a_omega_0];


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

            o_a_t_r = accelerometer_readings{1,1}(1);
            o_a_t_l = accelerometer_readings{1,1}(2);
  
            % process noise covariance matrix
            Q = accelerometer_readings{1,2};
            
            
            % Current states
            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);
            v_t_curr = obj.x(4);
            v_n_curr = obj.x(5);
            omega_curr = obj.x(6);
            a_t_curr = 0.5*o_a_t_r+0.5*o_a_t_l;
            a_n_curr = obj.x(8);
            a_w_curr = (1/d)*o_a_t_r - (1/d)*o_a_t_l;

            % Current covariance state estimation errors
            P_curr = obj.P;

  

            %% PREDICTION STEP
            % STATE DYNAMICS

            % Change reference frame from t-n to x-y
            v_x_curr = v_t_curr*cos(theta_curr) - v_n_curr*sin(theta_curr);
            v_y_curr = v_t_curr*sin(theta_curr) + v_n_curr*cos(theta_curr);

            a_x_curr = a_t_curr*cos(theta_curr) - a_n_curr*sin(theta_curr);
            a_y_curr = a_t_curr*sin(theta_curr) + a_n_curr*cos(theta_curr);
            

            % State Dynamics
            x_next = x_curr +  dt*v_x_curr + 0.5*a_x_curr*dt^2;
            y_next = y_curr +  dt*v_y_curr + 0.5*a_y_curr*dt^2;
            theta_next = theta_curr + dt*omega_curr + 0.5*a_w_curr*dt^2;
            v_x_next = v_x_curr + dt*a_x_curr; 
            v_y_next = v_y_curr + dt*a_y_curr;
            omega_next = omega_curr + a_w_curr*dt ;
            a_t_next = a_t_curr;
            a_n_next = a_n_curr;
            a_w_next = a_w_curr;



            % Change velocity reference frame from x-y to t-n
            v_t_next = v_x_next*cos(theta_next) + v_y_next*sin(theta_next);
            v_n_next = - v_x_next*sin(theta_next) + v_y_next*cos(theta_next);

            % State
            obj.x = [x_next,y_next,theta_next,v_t_next,v_n_next,omega_next,a_t_next,a_n_next,a_w_next];


            %% JACOBIAN OF STATE
           F = zeros(9,9);
           F(1,4) = cos(theta_curr);
           F(1,5) = -sin(theta_curr);
           F(1,7) = dt*cos(theta_curr);
           F(1,8) = -dt*sin(theta_curr);
           F(2,4) = sin(theta_curr);
           F(2,5) = cos(theta_curr);
           F(2,7) = dt*sin(theta_curr);
           F(2,8) = dt*cos(theta_curr);
           F(3,6) = 1;
           F(4,7) = 1;
           F(5,8) = 1;
           F(6,9) = 1;


           %% JACOBIAN OF NOISE
           W = zeros(9,2);
           W(1,1) = 0.25*cos(theta_curr)*dt^2;
           W(1,2) = 0.25*cos(theta_curr)*dt^2;
           W(2,1) = 0.25*sin(theta_curr)*dt^2;
           W(2,2) = 0.25*sin(theta_curr)*dt^2;
           W(3,1) = 0.5*(1/d)*dt^2;
           W(3,2) = - 0.5*(1/d)*dt^2;
           W(4,1) = 0.5*cos(theta_curr)*dt;
           W(4,2) = 0.5*cos(theta_curr)*dt;
           W(5,1) = 0.5*sin(theta_curr)*dt;
           W(5,2) = 0.5*sin(theta_curr)*dt;
           W(6,1) = (1/d)*dt;
           W(6,2) = -(1/d)*dt;
           W(7,1) = 0.5;
           W(7,2) = 0.5;
           W(8,1) = 0;
           W(8,2) = 0;
           W(9,1) = 1/d;
           W(9,2) = -1/d;

           
           %% PREDICTION STEP COVARIANCE STATE

           P_next = F*P_curr*F'+ W*Q*W';
           obj.P = P_next;

        end


        % Function that does the correction step of the filter
        function EKF_correction(obj,sigma_meas,measurements_readings,d)

            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);

            P_curr = obj.P;

            % Measurement noise covariance matrix
            R = diag([sigma_meas(1) sigma_meas(2) sigma_meas(3)]); 
            
            % Jacobian matrix of output
            H = zeros(3,9);
            H(1,1) = 1;
            H(2,2) = 1;
            H(3,3) = 1;


            Kalman_gain = P_curr*H'*pinv(H*P_curr*H'+R);


            % Calculation of innovation
            innovation = [measurements_readings(1) - x_curr, measurements_readings(2) - y_curr, measurements_readings(3)-theta_curr]';

            obj.innovation_history = [obj.innovation_history;innovation];

            state_next = [x_curr;y_curr;theta_curr;obj.x(4); obj.x(5); obj.x(6); obj.x(7); obj.x(8); obj.x(9)] + Kalman_gain*innovation;

            P_next = (eye(9)-Kalman_gain*H)*P_curr;

            % Update the weight
            obj.weight = obj.weight*exp(-0.5*innovation.^2./(H*P_next*H'+R));
            %obj.weight_history = [obj.weight_history;obj.weight];

            x_next = state_next(1);
            y_next = state_next(2);
            theta_next = state_next(3);
            v_t_next = state_next(4);
            v_n_next = state_next(5);
            omega_next = state_next(6);
            a_t_next = state_next(7);
            a_n_next = state_next(8);
            a_w_next = state_next(9);
            

            obj.x = [x_next,y_next,theta_next,v_t_next,v_n_next,omega_next,a_t_next,a_n_next,a_w_next];

            obj.P = P_next;


        end

        end

    
end