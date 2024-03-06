classdef Forklift_unycicle < handle

    properties
        x; % state of the robot 3*1
        x_est; % estimated state of the robot 3*1
 
        % OTHERS
        dt; % Time integration step
        KR;
        KL;
        d;
        R;

        instance_selected;
        steps_in_range;
        weights_vec;
        
        best_tag_estimation;

        dynamics_history;
        odometry_history;
        tag_estimation_history;

        last_nonNaN_estimation;

        init_flag;

        odometry_estimation;

        phase_measured;

        P_cov;

    end


    methods
        
        function obj = Forklift_unycicle(initial_state,R,d,KR,KL, dt,nM,NaN)

            % State
            obj.x=zeros(3,1); % initialize to 0 all the state
            
            % Then assign to obj.x the initial state we pass to it
           obj.x(1) = initial_state(1);
           obj.x(2) = initial_state(2);
           obj.x(3) = initial_state(3);

            % Data
            obj.R = R;
            obj.d = d;
            obj.KR = KR;
            obj.KL = KL;


            obj.dt = dt; % Integration time step

            obj.instance_selected = nM;
            obj.steps_in_range = 0;

            obj.weights_vec = (1/nM) *ones(nM,1);

            obj.best_tag_estimation = [NaN;NaN];

            obj.dynamics_history = {};
            obj.odometry_history = {};
            obj.tag_estimation_history = {};

            obj.last_nonNaN_estimation = [NaN;NaN];

            obj.init_flag = false;

            obj.odometry_estimation = {[0,0], diag([0,0])};

            obj.P_cov = zeros(3,3);

        end



        function state = get_state(obj)
            state = obj.x;
        end




        function state_odometry = get_odometry_state(obj)
            state_odometry = obj.x_est;
        end



        function x_next = dynamics(obj,v,omega)
            [u_r,u_l] = wheel_displacements(obj,v,omega);
    
            obj.x(1) = obj.x(1) + ((u_r+u_l)/2)*cos(obj.x(3));
            obj.x(2) = obj.x(2) + ((u_r+u_l)/2)*sin(obj.x(3));
            obj.x(3) = obj.x(3) + (1/obj.d)*(u_r-u_l);
            
            x_next = obj.x;
        
        end


        function odometry_est = odom_process(obj,v,omega)
    
             [u_r,u_l] = wheel_displacements(obj,v,omega);
             
             % Add noise
             u_r = u_r + normrnd(0,sqrt(obj.KR*abs(u_r)));
             u_l = u_l + normrnd(0,sqrt(obj.KL*abs(u_l)));

             % Center displacement and rotation
             u_est = (u_r + u_l)/2;
             omega_est = (u_r-u_l)/obj.d;

             obj.x_est(1) = obj.x_est(1) + u_est*cos(obj.x_est(3));
             obj.x_est(2) = obj.x_est(2) + u_est*sin(obj.x_est(3));
             obj.x_est(3) = obj.x_est(3) + omega_est;

             % Covariance matrix of noise process
             Q = [obj.KR*abs(u_r) 0; 0 obj.KL*abs(u_l)];

             obj.odometry_estimation = {[u_est,omega_est],Q};
             odometry_est = obj.odometry_estimation;
     
        end

        function covariace_next(obj)
            u = obj.odometry_estimation{1,1}(1);
            Q = obj.odometry_estimation{1,2};

            % Jacobian of state dynamics and encoder noise

            A = [1    0   -u*sin(obj.x(3)); 0   1   u*cos(obj.x(3)); 0     0     1];

            G = [0.5*cos(obj.x(3))    0.5*cos(obj.x(3)); 0.5*sin(obj.x(3))    0.5*sin(obj.x(3));  1/obj.d    -1/obj.d];

            obj.P_cov = A*obj.P_cov*A' + G*Q*G';

        end


        
       

        function [u_r,u_l] = wheel_displacements(obj,v,omega)
    
            u_r = (v+obj.d*omega/2)*obj.dt;
            u_l = (v-obj.d*omega/2)*obj.dt;

        end



        % Funzione per range
         function inRange = inTagRange(obj,tag_position, max_range)
            dist = obj.getTagDistance(obj,tag_position);

            if dist <= max_range
                inRange = true;
            else 
                inRange = false;
            end
        end

        
        % Phase measurement for RFID system
        function phaseMeasured(obj, tag_position, lambda , sigma_phi)
            distance = obj.getTagDistance(obj,tag_position);
        
            phase = (distance * 4 * pi)/lambda;

            obj.phase_measured = mod(-phase + normrnd(0,sigma_phi) , 2*pi) ;
        end




        % Funzione calcolo distanza robot-tag
        function distance = getTagDistance(obj,tag_position)
            x_tag = tag_position(1);
            y_tag = tag_position(2);

            distance = sqrt((obj.x(1) - x_tag)^2 + (obj.x(2) - y_tag)^2);
            
        end
        
    
    
    end

end


