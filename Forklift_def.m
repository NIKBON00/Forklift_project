classdef Forklift_def < handle

    properties
        x; % state of the robot 9*1
        x_est; % estimated state of the robot 9*1
 
        % OTHERS
        dt; % Time integration step
        d; % axes length

        K;

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

        P_cov;

    end


    methods
        
        function obj = Forklift_def(initial_state,d,dt,K,nM,NaN)

            % State
            obj.x=zeros(9,1); % initialize to 0 all the state
            
            % Then assign to obj.x the initial state we pass to it
            for i =1:length(initial_state)
                obj.x(i) = initial_state(i);
                obj.x_est(i) = initial_state(i);
            end


            % Data 
            obj.d = d;
            
            obj.dt = dt; % Integration time step
            obj.K = K;

            obj.instance_selected = nM;
            obj.steps_in_range = 0;

            obj.weights_vec = (1/nM) *ones(nM,1);

            obj.best_tag_estimation = [NaN;NaN];

            obj.dynamics_history = {};
            obj.odometry_history = {};
            obj.tag_estimation_history = {};

            obj.last_nonNaN_estimation = [NaN;NaN];

            obj.init_flag = false;

            obj.odometry_estimation = {[0,0], zeros(2,2)};

            obj.P_cov = zeros(9,9);

        end



        function state = get_state(obj)
            state = obj.x;
        end




        function state_odometry = get_odometry_state(obj)
            state_odometry = obj.x_est;
        end



        function x_next = dynamics(obj,u_a_t,u_a_omega)

            [a_t_r,a_t_l] = calculus_acc(obj,u_a_t,u_a_omega); % measurements
           

            % Change velocity and acceleration reference frame from t-n to x-y
            v_x = obj.x(4)*cos(obj.x(3)) - obj.x(5)*sin(obj.x(3));
            v_y = obj.x(4)*sin(obj.x(3)) + obj.x(5)*cos(obj.x(3));

            a_x = u_a_t *cos(obj.x(3)) - obj.x(8)*sin(obj.x(3));
            a_y = u_a_t *sin(obj.x(3)) + obj.x(8)*cos(obj.x(3));


            % State dynamics
            obj.x(1) = obj.x(1) + obj.dt*v_x + 0.5*a_x*obj.dt^2;
            obj.x(2) = obj.x(2) + obj.dt*v_y + 0.5*a_y*obj.dt^2;
            obj.x(3) = obj.x(3) + obj.dt*obj.x(6) + 0.5*(u_a_omega)*obj.dt^2;
            v_x = v_x + obj.dt*a_x; 
            v_y = v_y + obj.dt*a_y;
            obj.x(6) = obj.x(6) + obj.dt*u_a_omega;

            % Change velocity reference frame from x-y to t-n
            obj.x(4) = v_x*cos(obj.x(3)) + v_y*sin(obj.x(3));
            obj.x(5) = - v_x*sin(obj.x(3)) + v_y*cos(obj.x(3));
            
            % Input
            obj.x(7) = u_a_t;
            obj.x(9) = u_a_omega;

            
            x_next = [obj.x(1) obj.x(2) obj.x(3) obj.x(4) obj.x(5) obj.x(6) obj.x(7) obj.x(8) obj.x(9)];
        
        end


        function odometry_estimation = odometry_step(obj,u_a_t,u_a_omega)


            [a_t_r,a_t_l] = calculus_acc(obj,u_a_t,u_a_omega);

            a_t_r = a_t_r + normrnd(0,sqrt(obj.K*abs(a_t_r)));
            a_t_l = a_t_l + normrnd(0,sqrt(obj.K*abs(a_t_l)));
            

            a_t_est = (a_t_r + a_t_l)/2;
            a_w_est = (a_t_r - a_t_l)/obj.d;
         

            % Change velocity and acceleration reference frame from t-n to x-y
            v_x_est = obj.x_est(4)*cos(obj.x_est(3)) - obj.x_est(5)*sin(obj.x_est(3));
            v_y_est = obj.x_est(4)*sin(obj.x_est(3)) + obj.x_est(5)*cos(obj.x_est(3));

            a_x_est = a_t_est *cos(obj.x_est(3)) - obj.x(8)*sin(obj.x_est(3));
            a_y_est = a_t_est *sin(obj.x_est(3)) + obj.x(8)*cos(obj.x_est(3));

            % State dynamics
            obj.x_est(1) = obj.x_est(1) + obj.dt*v_x_est + 0.5*a_x_est*obj.dt^2;
            obj.x_est(2) = obj.x_est(2) + obj.dt*v_y_est + 0.5*a_y_est*obj.dt^2;
            obj.x_est(3) = obj.x_est(3) + obj.dt*obj.x_est(6) + 0.5*a_w_est*obj.dt^2;
            v_x_est = v_x_est + obj.dt*a_x_est; 
            v_y_est = v_y_est + obj.dt*a_y_est;
            obj.x_est(6) = obj.x_est(6) + obj.dt*a_w_est;

            % Change velocity reference frame from x-y to t-n
            obj.x_est(4) = v_x_est*cos(obj.x_est(3)) + v_y_est*sin(obj.x_est(3));
            obj.x_est(5) = - v_x_est*sin(obj.x_est(3)) + v_y_est*cos(obj.x_est(3));

            obj.x_est(7) = a_t_est;
            obj.x_est(9) = a_w_est;

           
            Q = [obj.K*abs(a_t_r), 0; 0, obj.K*abs(a_t_l)];
            obj.odometry_estimation = {[a_t_est a_w_est], Q};
            odometry_estimation = obj.odometry_estimation;
            

        end


        function  [a_t_r,a_t_l] = calculus_acc(obj,a_t,a_w)
            
            a_t_r = a_t + obj.d/2 *a_w;
            a_t_l = a_t - obj.d/2*a_w;
           
        end

        function distance = getDistance (obj,x_t,y_t)

            x_r = obj.x(1);
            y_r = obj.x(2);

            distance = sqrt((x_r - x_t)^2 + (y_r - y_t)^2);

        end

    
    end

end

