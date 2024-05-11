function [v_t,omega] = controller(Kp_v_t,Kp_omega, x_target, y_target, odometry_state,obstacle,i_index,min_distance)

        x_odometry = odometry_state(1);
        y_odometry = odometry_state(2);
        theta_odometry = odometry_state(3);
        L_safe = 3;
   
        beta_i = zeros(length(i_index),1);

        % Velocity controller
        dist = sqrt((y_target - y_odometry)^2 + (x_target - x_odometry)^2); 
        


        % Angular velocity controller
        theta_desired = atan2(y_target - y_odometry, x_target - x_odometry);
        


        % If Obtacle detected
        if (obstacle == 1)     
            beta_i = theta_desired + (i_index-91)*pi/180;    
        end
        disp('Beta vector')
        disp(beta_i)

        if (obstacle == 1 && min_distance <= L_safe)
        [beta,beta_index] = min(abs(beta_i - theta_desired));
        disp('Beta:');
        disp(beta);
        disp('Beta_index');
        disp(beta_index);
        disp('Theta before');
        disp(theta_desired);
        theta_desired = theta_desired + sign(i_index(beta_index) - 91)*(beta + 0.1);
        disp('Theta after');
        disp(theta_desired);
        error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
        disp(error_heading);
        omega = Kp_omega*error_heading;
        Kp_v_t = 0.1;
        end

        % If not Obstacle detected
        if (obstacle == 0 || (obstacle == 1 && min_distance > L_safe))
            error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
            omega = Kp_omega*error_heading;
        end

        v_t = Kp_v_t*dist;

        if v_t>4
            v_t=4;
        end


end