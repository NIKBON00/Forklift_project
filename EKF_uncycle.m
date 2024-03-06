classdef EKF_uncycle<handle

    properties
        x; % filter of the state
        P; % covariance matrix state of estimation errors
        weight; % weight of EKF instance
        weight_history; % history of the weights of the EKF instance
        state_history; % history of the state
        innovation_history; % history of innovations
 
    end


    methods
        
        
        function obj = EKF_uncycle()
            obj.x = zeros(5,1);
            obj.P = zeros(5,5);
            obj.weight = 0;
            obj.weight_history=[];
            obj.state_history=[];
            obj.innovation_history=[];
        end


        function EKF_init(obj,phi_0,n,lambda,sigma_phi,weight_init,robot_state,robot_cov_matrix)


            K = (2*pi)/lambda;
            ro_0 = -phi_0/(2*K) + (n*lambda)/2;
            beta_0= 0;
            
            % Initialize state vehicle
            x_0 = robot_state(1);
            y_0 = robot_state(2);
            theta_0 = robot_state(3);

            obj.x = [ro_0,beta_0,x_0,y_0,theta_0];

            sigma_rho = sigma_phi/(2*K);
            obj.P(1,1) = sigma_rho^2;
            obj.P(2,2) = (pi/3)^2;

            for i=3:length(obj.x)
                for j = 3:length(obj.x)
           
                        obj.P(i,j) = robot_cov_matrix(i-2,j-2); % obj.P(3,3) = robot_cov_matrix(1,1); obj.P(3,4) = robot_cov_matrix(1,2),...; 

                end
            end



            obj.weight = weight_init;
            obj.weight_history = [obj.weight_history; obj.weight];

        end


        % Function that computes the prediction step of state and
        % parameters
        function EKF_prediction(obj,measurements_readings,d)

            u = measurements_readings{1,1}(1);
            omega = measurements_readings{1,1}(2);
            Q = measurements_readings{1,2};



            % Current states
            rho_curr = obj.x(1);
            beta_curr = obj.x(2);
            x_curr = obj.x(3);
            y_curr = obj.x(4);
            theta_curr = obj.x(5);

            % Current covariance state estimation errors
            P_curr = obj.P;

  

            %% PREDICTION STEP
            % STATE DYNAMICS

            rho_next = rho_curr - u*cos(beta_curr);
            beta_next = beta_curr + omega + (u/rho_curr)*sin(beta_curr);
            x_next = x_curr + u*cos(beta_curr);
            y_next = y_curr + u*sin(beta_curr);
            theta_next = theta_curr + omega;


            obj.x = [rho_next,beta_next,x_next,y_next,theta_next];


            %% JACOBIAN OF STATE
            F = [1  u*sin(beta_curr)  0   0  0; -(u/rho_curr^2)*sin(beta_curr)   1+(u/rho_curr)*cos(beta_curr)  0  0  0;...
                 0    0   1    0   -u*sin(theta_curr); 0 0 0 1 u*cos(theta_curr);  0 0 0 0 1];

            W = [-0.5*cos(beta_curr)   -0-5*cos(beta_curr); 1/d + (1/(2*rho_curr))*sin(beta_curr)    -1/d+(1/(2*rho_curr))*sin(beta_curr);...
                  0.5*cos(theta_curr)     0.5*cos(theta_curr); 0.5*sin(theta_curr)   0.5*cos(theta_curr);
                  1/d     -1/d];

        %% PREDICTION STEP COVARIANCE
        % State
        P_next = F*P_curr*F' + W*Q*W';
        obj.P = P_next;

        end


        % Function that does the correction step of the filter
        function EKF_correction(obj,K,sigma_phi,phi_meas)

            rho_curr = obj.x(1);
            beta_curr = obj.x(2);
            x_curr = obj.x(3);
            y_curr = obj.x(4);
            theta_curr = obj.x(5);

            P_curr = obj.P;
            R =sigma_phi^2; 
            
            % Jacobian matrix of output
            H = [-2*K 0 0 0 0];

            Kalman_gain = P_curr*H'*pinv(H*P_curr*H'+R);

            phi_expected = mod(-2*K*rho_curr,2*pi);

            % Calculation of innovation
            innovation = phi_meas - phi_expected;
            innovation = atan2(sin(innovation),cos(innovation));

            obj.innovation_history = [obj.innovation_history;innovation];

            state_next = [rho_curr;beta_curr;x_curr;y_curr;theta_curr] + Kalman_gain*innovation;

            P_next = (eye(5)-Kalman_gain*H)*P_curr;

            % Update the weight
            obj.weight = obj.weight*exp(-0.5*innovation^2/(H*P_next*H'+R));
            obj.weight_history = [obj.weight_history;obj.weight];

            rho_next = state_next(1);
            beta_next = state_next(2);
            x_next = state_next(3);
            y_next = state_next(4);
            theta_next = state_next(5);
            
        

            obj.x = [rho_next,beta_next,x_next,y_next,theta_next];

            obj.P = P_next;


        end
        

        function weight_tmp = EKF_weight_tmp(obj,k,odometry_history,phase_history,Ns,weight_prec,c1,c2,K,l)

            sum_phase_diff=0;

            for i= (k-Ns+1):(k-1)
               
    
            x_tag =  odometry_history{i,1}(1) + obj.state_history{i,1}(1)*cos(odometry_history{i,1}(3) - obj.state_history{i,1}(2));
            y_tag =  odometry_history{i,1}(2) + obj.state_history{i,1}(1)*sin(odometry_history{i,1}(3) - obj.state_history{i,1}(2));
     
    
                 if i == (k-Ns+1)
                    x_min_tag = x_tag;
                    y_min_tag = y_tag;
                    x_max_tag = x_tag;
                    y_max_tag = y_tag;
                 end
     
                 if x_tag < x_min_tag
                    x_min_tag = x_tag;
                end
                if y_tag < y_min_tag
                     y_min_tag = y_tag;
                end
                if x_tag > x_max_tag
                     x_max_tag = x_tag;
                end
                if y_tag > y_max_tag
                     y_max_tag = y_tag;
                end
     
                D = sqrt((odometry_history{i,1}(1) - x_tag)^2 + (odometry_history{i,1}(2) - y_tag)^2);
                phi_expected = mod(-2*K*D,2*pi);
     
                phase_diff = (phase_history(i) - phi_expected)^2;
     
                sum_phase_diff = sum_phase_diff + phase_diff;
     
             end
     
            M1 = 1/(1 + sqrt((x_max_tag - x_min_tag)^2 + (y_max_tag - y_min_tag)^2));
     
            M2 = 1/sqrt(sum_phase_diff);
     
         % Print x_max_tag, x_min_tag, y_max_tag, y_min_tag, phi_expected
         % fprintf('EKF INSTANCE: %d \n',l);
         % fprintf('x_max_tag = %f, x_min_tag = %f, y_max_tag = %f, y_min_tag = %f \n',x_max_tag, x_min_tag, y_max_tag, y_min_tag);
         % fprintf('D: %f, -2*K*D: %f, Phi expected = %f\n',D, -2*K*D, phi_expected);
     
            weight_tmp = weight_prec + c1*M1 + c2*M2; 


            end
        end


    %  function weight = EKF_weight(obj, weight_tmp, eta)
    %     weight = eta*weight_tmp;
    % end

 
    
    
end



