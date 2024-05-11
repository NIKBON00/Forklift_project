classdef Forklift_def < handle

    properties
        x; % state of the robot 3*1
        x_est; % estimated state of the robot 3*1
 
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
            obj.x_est = zeros(3,1); % initialize to 0 all the state
            
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


            % Speed sensor measurements retrieve
            [vt_r,vt_l] = calculus_vec(obj,u_v_t,u_omega);

            %{
            if (abs(vt_r - vt_l) > 0.5 )

                % To solve the forward kinematics we have to identify firstly
                % the istantaneous center of rotation

                % ICR radius:
                r = (obj.d/2)*(vt_r + vt_l)/(vt_r - vt_l);

                % ICR coordinates:
                ICR_x = obj.x(1) - r*sin(obj.x(3));
                ICR_y = obj.x(2) + r*cos(obj.x(3));

                % State kinematics:
                obj.x(1) = r*cos( ((vt_r-vt_l)/obj.d)*obj.dt )*sin(obj.x(3)) + r*cos(obj.x(3))*sin( ((vt_r-vt_l)/obj.d)*obj.dt ) + ICR_x;
                obj.x(2) = r*sin( ((vt_r-vt_l)/obj.d)*obj.dt )*sin(obj.x(3)) - r*cos(obj.x(3))*cos( ((vt_r-vt_l)/obj.d)*obj.dt ) + ICR_y;
                obj.x(3) = obj.x(3) + ((vt_r-vt_l)/obj.d)*obj.dt;

           

            else
            %}
           


                obj.x(1) = obj.x(1) + ((vt_r + vt_l)/2)*obj.dt*cos(obj.x(3));
                obj.x(2) = obj.x(2) + ((vt_r + vt_l)/2)*obj.dt*sin(obj.x(3));
                obj.x(3) = obj.x(3) + ((vt_r - vt_l)/obj.d)*obj.dt;


            %end

            % Update
            x_next = [obj.x(1) obj.x(2) obj.x(3)];
        
        end


        function odometry_estimation = odometry_step(obj,u_v_t,u_omega)
            
            % Speed sensor measurements retrieve
            [vt_r,vt_l] = calculus_vec(obj,u_v_t,u_omega);

            % Adding noise to speed sensor measurements
            vt_r = vt_r + normrnd(0,sqrt(obj.K_r*abs(vt_r)));
            vt_l = vt_l + normrnd(0,sqrt(obj.K_l*abs(vt_l)));

            %disp([vt_r,vt_l]);

            vt_est    = (vt_r + vt_l)/2;
            omega_est = (vt_r - vt_l)/obj.d;

            %{
            if (abs(vt_r - vt_l) > 0.5 )

                % To solve the forward kinematics we have to identify firstly
                % the istantaneous center of rotation
    
                % ICR radius:
                r = (obj.d/2)*(vt_r + vt_l)/(vt_r - vt_l);
    
                % ICR coordinates:
                ICR_x = obj.x_est(1) - r*sin(obj.x_est(3));
                ICR_y = obj.x_est(2) + r*cos(obj.x_est(3));
    
                % State kinematics:
                obj.x_est(1) = r*cos( ((vt_r-vt_l)/obj.d)*obj.dt )*sin(obj.x_est(3)) + r*cos(obj.x_est(3))*sin( ((vt_r-vt_l)/obj.d)*obj.dt ) + ICR_x;
                obj.x_est(2) = r*sin( ((vt_r-vt_l)/obj.d)*obj.dt )*sin(obj.x_est(3)) - r*cos(obj.x_est(3))*cos( ((vt_r-vt_l)/obj.d)*obj.dt ) + ICR_y;
                obj.x_est(3) = obj.x_est(3) + ((vt_r-vt_l)/obj.d)*obj.dt;

            else
            %}
            

                obj.x_est(1) = obj.x_est(1) + vt_est*obj.dt*cos(obj.x_est(3));
                obj.x_est(2) = obj.x_est(2) + vt_est*obj.dt*sin(obj.x_est(3));
                obj.x_est(3) = obj.x_est(3) + omega_est*obj.dt;

           %end

            
            % Odometry estimation
            Q = [obj.K_r*abs(vt_r), 0; 0, obj.K_l*abs(vt_l)];
            obj.odometry_estimation = {[vt_est omega_est], Q};
            odometry_estimation = obj.odometry_estimation;
            
        end


        % Retrieve sensor velocity measurements from input
        function  [vt_r,vt_l] = calculus_vec(obj,v_t,omega)
           
           % Calculus of right and left front wheels speed 
           vt_r = (2*v_t + obj.d*omega)/2;
           vt_l = (2*v_t - obj.d*omega)/2;
           
        end

        

        % Distance of robot from target
        function distance = getDistance(obj,x_t,y_t)

            x_r = obj.x(1);
            y_r = obj.x(2);

            distance = sqrt((x_r - x_t)^2 + (y_r - y_t)^2);

        end

    
    end

end

