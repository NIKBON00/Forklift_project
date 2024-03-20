classdef Forklift_def < handle

    properties
        x; % state of the robot 9*1
        x_est; % estimated state of the robot 9*1
 
        % OTHERS
        dt; % Time integration step
        d; % axes length

        K_r;
        K_l;

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
        
        function obj = Forklift_def(initial_state,d,dt,K_r,K_l,nM,NaN)

            % State
            obj.x=zeros(3,1); % initialize to 0 all the state
            
            % Then assign to obj.x the initial state we pass to it
            for i =1:length(initial_state)
                obj.x(i) = initial_state(i);
                obj.x_est(i) = initial_state(i);
            end


            % Data 
            obj.d = d;
            
            obj.dt = dt; % Integration time step
            obj.K_r = K_r;
            obj.K_l = K_l;

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

            obj.P_cov = zeros(3,3);

        end



        function state = get_state(obj)
            state = obj.x;
        end




        function state_odometry = get_odometry_state(obj)
            state_odometry = obj.x_est;
        end



        function x_next = dynamics(obj,u_v_t,u_omega)

            [a_t_r,a_t_l] = calculus_acc(obj,u_v_t,u_omega); % measurements

            % Acceleration in frame t-n (along n no acceleration/velocity because supposing no slippage)
            a_t = (a_t_r + a_t_l)/2; 
           

            % Change velocity and acceleration reference frame from t-n to x-y
            v_x = u_v_t *cos(obj.x(3));
            v_y = u_v_t *sin(obj.x(3));

            a_x = a_t *cos(obj.x(3));
            a_y = a_t *sin(obj.x(3));


            % State dynamics
            obj.x(1) = obj.x(1) + obj.dt*v_x + 0.5*a_x*obj.dt^2;
            obj.x(2) = obj.x(2) + obj.dt*v_y + 0.5*a_y*obj.dt^2;
            obj.x(3) = obj.x(3) + obj.dt*u_omega + 0.5*u_omega*obj.dt;
      
            
            
            x_next = [obj.x(1) obj.x(2) obj.x(3)];
        
        end


        function odometry_estimation = odometry_step(obj,u_v_t,u_omega)


            [a_t_r,a_t_l] = calculus_acc(obj,u_v_t,u_omega);

            % Adding noise to accelerometers
            a_t_r = a_t_r + normrnd(0,sqrt(obj.K_r*abs(a_t_r)));
            a_t_l = a_t_l + normrnd(0,sqrt(obj.K_l*abs(a_t_l)));
            
            % Estimeted velocity and omega
            v_t_est   = (a_t_r + a_t_l)*obj.dt/2;
            omega_est = (a_t_r - a_t_l)*obj.dt/obj.d;
         

            % Change velocity and acceleration reference frame from t-n to x-y
            v_x_est = v_t_est*cos(obj.x_est(3));
            v_y_est = v_t_est*sin(obj.x_est(3));

            a_x_est = 0.5*(a_t_r + a_t_l) *cos(obj.x_est(3));
            a_y_est = 0.5*(a_t_r + a_t_l) *sin(obj.x_est(3));

          

            % State dynamics
            obj.x_est(1) = obj.x_est(1) + obj.dt*v_x_est + 0.5*a_x_est*obj.dt^2;
            obj.x_est(2) = obj.x_est(2) + obj.dt*v_y_est + 0.5*a_y_est*obj.dt^2;
            obj.x_est(3) = obj.x_est(3) + obj.dt*omega_est + 0.5*omega_est*obj.dt;
            
            % Odometry estimation
            Q = [obj.K_r*abs(a_t_r), 0; 0, obj.K_l*abs(a_t_l)];
            obj.odometry_estimation = {[v_t_est omega_est], Q};
            odometry_estimation = obj.odometry_estimation;
            

        end


        function  [a_t_r,a_t_l] = calculus_acc(obj,v_t,omega)
            
            a_t_r = (2*v_t + omega*obj.d)/(2*obj.dt);
            a_t_l = (2*v_t - omega*obj.d)/(2*obj.dt);
           
        end

        function distance = getDistance (obj,x_t,y_t)

            x_r = obj.x(1);
            y_r = obj.x(2);

            distance = sqrt((x_r - x_t)^2 + (y_r - y_t)^2);

        end

    
    end

end

