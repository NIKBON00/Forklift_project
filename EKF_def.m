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
        function EKF_prediction(obj,speed_readings,dt,d,R, eta_r, eta_l)

            o_v_t = speed_readings{1,1}(1);
            o_omega = speed_readings{1,1}(2);

            %vt_r = (2*o_v_t + d*o_omega)/2;
            %vt_l = (2*o_v_t - d*o_omega)/2;

            % process noise covariance matrix
            Q = speed_readings{1,2};
            
            
            % Current states
            x_curr = obj.x(1);
            y_curr = obj.x(2);
            theta_curr = obj.x(3);
           

            % Current covariance state estimation errors
            P_curr = obj.P;

            % Jacobian of state initialization
            F = zeros(3,3);

            % Jacobian of noise initialization
            W = zeros(3,2);

  

            %% PREDICTION STEP
            % STATE KINEMATICS

            x_next = x_curr + o_v_t*dt*cos(theta_curr + dt*o_omega/2);
            y_next = y_curr + o_v_t*dt*sin(theta_curr + dt*o_omega/2);
            theta_next = theta_curr + o_omega*dt;
            % Jacobian of State
            F(1,1) = 1;
            F(1,3) = -o_v_t*dt*sin(theta_curr + dt*o_omega/2);
            F(2,2) = 1;
            F(2,3) = o_v_t*dt*cos(theta_curr + dt*o_omega/2);
            F(3,3) = 1;

            % Jacobian of Noise
            W(1,1) = 0.5*dt*R*cos(theta_curr + dt*o_omega/2) - (o_v_t*dt/2)*sin(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(1,2) = 0.5*dt*R*cos(theta_curr + dt*o_omega/2) + (o_v_t*dt/2)*sin(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(2,1) = 0.5*dt*R*sin(theta_curr + dt*o_omega/2) + (o_v_t*dt/2)*cos(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(2,2) = 0.5*dt*R*sin(theta_curr + dt*o_omega/2) - (o_v_t*dt/2)*cos(theta_curr + dt*o_omega/2)*dt*R/(2*d);
            W(3,1) = R*dt/d;
            W(3,2) = -R*dt/d;

            %end
        
            % State
            obj.x = [x_next;y_next;theta_next];

        
           
           %% PREDICTION STEP COVARIANCE STATE

           P_next = F*P_curr*F'+ W*Q*W';
           obj.P = P_next;

        end


        % Function that does the correction step of the filter
        function EKF_correction(obj,sigma_meas,measurements_readings)

            % Rename state and covariance matrix
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