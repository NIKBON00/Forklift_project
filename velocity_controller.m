for v=1:length(total_pose(:,1))-1
    v_x = (1/dt)* (total_pose(v+1,1)-total_pose(v,1));
    v_y = (1/dt)* (total_pose(v+1,2)-total_pose(v,2));
    dir = 0;
   
    if total_pose(v,3) >= 0 && total_pose(v,3) < pi/2
        if v_x >= 0 && v_y >=0
            dir = 1;
        else 
            dir = -1;
        end
    end

    if total_pose(v,3) >= pi/2 && total_pose(v,3) < pi
        if v_x <= 0 && v_y >=0
            dir = 1;
        else 
            dir = -1;
        end
    end

    if total_pose(v,3) >= pi && total_pose(v,3) < 3*pi/2
        if v_x <= 0 && v_y <=0
            dir = 1;
        else 
            dir = -1;
        end
    end

    if total_pose(v,3) >= 3*pi/2 && total_pose(v,3) < 2*pi
        if v_x >= 0 && v_y <=0
            dir = 1;
        else 
            dir = -1;
        end
    end



    v_t = dir*sqrt(v_x^2 + v_y^2);
    omega = (1/dt)* (total_pose(v+1,3)-total_pose(v,3));

    % Dynamics
    x_next = robots(i).dynamics(v_t,omega);
    % Odometry 
    odometry = robots(i).odometry_step(v_t,omega);


     % Save data exact position
     
     pose_real_robot{i} = [ pose_real_robot{i}; x_next];
     
     % Save data estimated position with only speed sensor
     pose_est_robot{i}  = [ pose_est_robot{i};  robots(i).x_est(1) robots(i).x_est(2) robots(i).x_est(3) ];

     % Do EKF prediction + correction
     MHEKFs(i).EKF_prediction(robots(i).odometry_estimation, dt,d);
     MHEKFs(i).EKF_correction(sigma_meas,[robots(i).x(1),robots(i).x(2),robots(i).x(3)]);
     EKF_pose{i} = [EKF_pose{i}; MHEKFs(i).x(1), MHEKFs(i).x(2) MHEKFs(i).x(3)];
 
end