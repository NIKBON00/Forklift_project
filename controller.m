function [v_t,omega] = controller(Kp_v_t,Kp_omega, x_target, y_target, odometry_state)

        x_odometry = odometry_state(1);
        y_odometry = odometry_state(2);
        theta_odometry = odometry_state(3);
        dist = sqrt((y_target - y_odometry)^2 + (x_target - x_odometry)^2); 
        v_t = Kp_v_t*dist;



        theta_desired = atan2(y_target - y_odometry, x_target - x_odometry); 
        error_heading = atan2(sin(theta_desired-theta_odometry),cos(theta_desired-theta_odometry));
        omega = Kp_omega*error_heading;


        if v_t>4
            v_t=4;
        end


end