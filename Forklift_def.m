classdef Forklift_def < handle

    properties
        x; % state of the robot 3*1
        x_est; % estimated state of the robot 3*1
 
        % OTHERS
        dt; % Time integration step
        d; % axes length
        R; % radius wheel

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
        
        function obj = Forklift_def(initial_state,d,R,dt,K_r,K_l,nM,NaN)

            % State
            obj.x=zeros(3,1); % initialize to 0 all the state
            obj.x_est = zeros(3,1); % initialize to 0 all the state
            
            % Then assign to obj.x the initial state we pass to it
            for i =1:length(initial_state)
                obj.x(i) = initial_state(i);
                obj.x_est(i) = initial_state(i);
            end


            % Data 
            obj.d = d;
            obj.R = R;
            
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


            % Speed sensor measurements retrieve
            [omega_r,omega_l] = calculus_vec(obj,u_v_t,u_omega);

            delta_s_l = obj.dt*obj.R*omega_l;
            delta_s_r = obj.dt*obj.R*omega_r;

            delta_s = (delta_s_r + delta_s_l)/2;
            delta_theta = (delta_s_r - delta_s_l)/obj.d;
            
            obj.x(1) = obj.x(1) + delta_s*cos(obj.x(3) + delta_theta/2);
            obj.x(2) = obj.x(2) + delta_s*sin(obj.x(3) + delta_theta/2);
            obj.x(3) = obj.x(3) + delta_theta;

            % Update
            x_next = [obj.x(1) obj.x(2) obj.x(3)];
        
        end


        function odometry_estimation = odometry_step(obj,u_v_t,u_omega)
            
            % Speed sensor measurements retrieve
            [omega_r,omega_l] = calculus_vec(obj,u_v_t,u_omega);

            % Adding noise to speed sensor measurements
            eta_r = normrnd(0,sqrt(obj.K_r*abs(omega_r)));
            eta_l = normrnd(0,sqrt(obj.K_l*abs(omega_l)));

            % Noise measures
            omega_r = omega_r + eta_r;
            omega_l = omega_l + eta_l;

            % Traslational and rotational velocity
            vt_est = obj.R * (omega_r + omega_l)/2;
            omega_est = obj.R * (omega_r - omega_l)/obj.d;

            delta_s_l_est = obj.dt*obj.R*omega_l;
            delta_s_r_est = obj.dt*obj.R*omega_r;

            delta_s_est = (delta_s_r_est + delta_s_l_est)/2;
            delta_theta_est = (delta_s_r_est - delta_s_l_est)/obj.d;

            % Dynamics
            obj.x_est(1) = obj.x_est(1) + delta_s_est*cos(obj.x_est(3) + delta_theta_est/2);
            obj.x_est(2) = obj.x_est(2) + delta_s_est*sin(obj.x_est(3) + delta_theta_est/2);
            obj.x_est(3) = obj.x_est(3) + delta_theta_est;
                    
            % Odometry estimation
            Q = [obj.K_r*abs(omega_r)^2, 0; 0, obj.K_l*abs(omega_l)^2];
            obj.odometry_estimation = {[vt_est omega_est], Q};
            odometry_estimation = obj.odometry_estimation;
            
        end


        % Retrieve sensor velocity measurements from input
        function  [omega_r,omega_l] = calculus_vec(obj,v_t,omega)
           
           % Calculus of right and left front wheels speed 
           omega_r = (2*v_t + omega*obj.d)/(2*obj.R);
           omega_l = (2*v_t - omega*obj.d)/(2*obj.R);
           
        end

        

        % Distance of robot from target
        function distance = getDistance(obj,x_t,y_t)

            x_r = obj.x_est(1);
            y_r = obj.x_est(2);

            distance = sqrt((x_r - x_t)^2 + (y_r - y_t)^2);

        end

    
    end

end

