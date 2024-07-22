function [v_t,omega,target_tmp,activation] = controller(Kp_v_t,Kp_omega,Kd_omega,Ki_omega, dt,x_target, y_target, odometry_state,obstacle,i_index,min_distance,activation,target_tmp, ranges)

        x_odometry = odometry_state(1);
        y_odometry = odometry_state(2);
        theta_odometry = odometry_state(3);
        L_safe = 3;
        error_heading_prec = 0;
        integral = 0;
        target = [x_target,y_target];
        L_dist = 3;
       
        
   
        beta_i = zeros(length(i_index),1);

        % Velocity controller
        dist = sqrt((target(2) - y_odometry)^2 + (target(1) - x_odometry)^2); 

        % Angular velocity controller
        theta_desired = atan2(target(2) - y_odometry, target(1) - x_odometry);

        
      
          if ((ranges(91) == 0) && (activation == 0))   
            activation  = 1;

            % If Obtacle detected
            if (obstacle == 1)     
                beta_i = theta_desired + (i_index-91)*pi/180;    
            end
            
            if (obstacle == 1 && min_distance <= L_safe)
                [beta,beta_index] = min(abs(beta_i - theta_desired));
                theta_desired = theta_desired + sign(i_index(beta_index) - 91)*(beta + 0.5);
                target_tmp = [(x_odometry)+L_dist*cos(theta_desired), (y_odometry) + L_dist*sin(theta_desired)];

            end
      
          end


            error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
            omega = Kp_omega*error_heading;
            
%{
            % If not Obstacle detected
            if (obstacle == 0 || (obstacle == 1 && min_distance > L_safe))
                error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
                omega = Kp_omega*error_heading;
            end
%}

        
        v_t = Kp_v_t*dist;

        if v_t>4
            v_t=4;
        end

         % Velocity controller
        dist = sqrt((target(2) - y_odometry)^2 + (target(1) - x_odometry)^2);

        if(dist<=0.5)
            activation = 0;
        end


end