function [a_t,a_w] = controller(Kp_a_t,Kp_a_w, x_target, y_target, odometry_state)

        x_odometry = odometry_state(1);
        y_odometry = odometry_state(2);
        theta_odometry = odometry_state(3);
        dist = sqrt((y_target - y_odometry)^2 + (x_target - x_odometry)^2); 
        a_t = Kp_a_t*dist;



        theta_desired = atan2(y_target - y_odometry, x_target - x_odometry); 
        error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
        a_w = Kp_a_w*error_heading;


        if a_t>4
            a_t=4;
        end


        if a_w>4
            a_w=4;
        end

       



end