classdef Forklift < handle

    properties
        x; % state of the robot 22*1
        x_est; % estimated state of the robot 22*1

        tf; % front track width
        tr; % rear track width;
        Jsx; % Moment of inertia around longitudinal axis
        ms; % sprung mass
        hs; % cog height of sprung mass
        h; % cog height
        k_phi; % roll stiffness
        beta_phi; % roll damping
        r; % wheel radius
        l; % vehicle length
        sM; % tire slip value
        sG; % tire slip value
        FM; % maximum tire force
        FG; % full sliding force
        dF0;  % horizonatl initial inclination
        ba; % distance between the accelerometer and the front axle
        h_z; % height of the acccelerometer

        % PARAMETERS
        xp; % [mass m, inertia Jz, b distance between cog position and wheels]

        % POSITION
        pos;

        
        % OTHERS
        dt; % Time integration step

        KR;
        KL;

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
        
        function obj = Forklift(initial_state,xp_init,tf,tr,Jsx,ms,hs,h,k_phi,beta_phi,r,l,sM,sG,FM,FG,dF0,ba,h_z, dt,nM,NaN,pos_in)

            % State
            obj.x=zeros(22); % initialize to 0 all the state
            
            % Then assign to obj.x the initial state we pass to it
            for i=1:length(initial_state)
               obj.x(i)=initial_state(i);
               obj.x_est(i)=initial_state(i);
            end

            % Data
            obj.tf = tf;
            obj.tr = tr;
            obj.Jsx = Jsx;
            obj.ms = ms;
            obj.hs = hs;
            obj.h = h;
            obj.k_phi = k_phi;
            obj.beta_phi = beta_phi;
            obj.r = r;
            obj.l = l;
            obj.sM = sM;
            obj.sG = sG;
            obj.FM = FM;
            obj.FG = FG;
            obj.dF0 = dF0;
            obj.ba = ba;
            obj.h_z = h_z;

            % Parameters
            obj.xp(1) = xp_init(1); % m
            obj.xp(2) = xp_init(2); % Jz
            obj.xp(3) = xp_init(3); % b

            obj.pos = pos_in; % Initial position of robot


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

            obj.odometry_estimation = {[0,0,0], diag([0,0,0])};

            obj.P_cov = zeros(22);

        end



        function state = get_state(obj)
            state = obj.x;
        end



        function param = get_parameters(obj)
            param = obj.xp;
        end



        function state_odometry = get_odometry_state(obj)
            state_odometry = obj.x_est;
        end



        function x_next = dynamics(obj,u)
    
            [F_xfl, F_yfl, F_xfr, F_yfr, F_xrl, F_yrl, F_xrr, F_yrr] = calculation_forces(obj,u);

            % Input
            delta = u(1);
            
            % Parameters
            m = obj.xp(1);
            Jz = obj.xp(2);
            b = obj.xp(3);
            
            % State rename
            v_x = obj.x(1);
            v_y = obj.x(2);
            psi_dot = obj.x(3);
            a_y = obj.x(4);
            Gamma = obj.x(5);
            a_x = obj.x(7);
            s_fl = obj.x(16);
            s_fr = obj.x(17);
            s_rl = obj.x(18);
            s_rr = obj.x(19);
            phi = obj.x(20);
            phi_dot = obj.x(21);
            

            x_next = [  v_x + Dt*(a_x + v_y*psi_dot);
                        v_y + Dt*(a_y - v_x*psi_dot);
                        psi_dot + Dt*Gamma/Jz;
                        1/m*(F_yfl*cos(delta) + F_xfl*sin(delta) + F_yfr*cos(delta) + F_xfr*sin(delta) + F_yrl + F_yrr);
                        obj.tr/2*F_xrl - obj.tr/2*F_xrr - (obj.l - b) * F_yrl - (obj.l - b)*F_yrr;
                        atan2(v_y, v_x);
                        1/m*(F_xfl*cos(delta) - F_yfl*sin(delta) + F_xfr*cos(delta) - F_yfr*sin(delta) + F_xrl + F_xrr);
                        delta - atan2((v_y + b*psi_dot), (v_x + obj.tf*psi_dot/2));
                        delta - atan2((v_y + b*psi_dot), (v_x - obj.tf*psi_dot/2));
                        atan2((-v_y + (obj.l - b)*psi_dot), (v_x + obj.tr*psi_dot/2));
                        atan2((-v_y + (obj.l - b)*psi_dot), (v_x - obj.tr*psi_dot/2));
                        (1/2*m*g + m*a_y*obj.h/obj.tf)*(obj.l - b)/obj.l - 1/2*m*a_x*obj.h/obj.l;
                        (1/2*m*g - m*a_y*obj.h/obj.tf)*(obj.l - b)/obj.l - 1/2*m*a_x*obj.h/obj.l;
                        (1/2*m*g + m*a_y*obj.h/obj.tr)*b/obj.l + 1/2*m*a_x*obj.h/obj.l;
                        (1/2*m*g - m*a_y*obj.h/obj.tr)*b/obj.l + 1/2*m*a_x*obj.h/obj.l;
                        s_fl;
                        s_fr;
                        s_rl;
                        s_rr;
                        phi + Dt*phi_dot;
                        phi_dot + Dt/obj.Jsx*(-obj.ms*obj.hs*a_y + phi*(obj.ms*g*obj.hs - obj.k_phi) - phi_dot*obj.beta_phi);
                        (a_y + Gamma/Jz*(b - obj.ba) + 1/obj.Jsx*(-obj.ms*obj.hs*a_y + phi*(obj.ms*g*obj.hs - obj.k_phi) - phi_dot*obj.beta_phi)*obj.h_z) + g*phi   ];
        
        
        end



        % Modello forze-gomme
        function Ft = calculation_forces(obj,u)

            % Pacejka model - (fl, fr, rl, rr), (x, y)
            vcog = sqrt(obj.x(1)^2 + obj.x(2)^2);
            % F_xfl, F_yfl
            vel = vcog + obj.x(3)*(obj.tf/2 + - obj.xp(3)*obj.x(6));
            w_fl = u(2);
            s_fl = obj.r*w_fl/vel - 1;
            if s_fl <= obj.sM
                sigma = s_fl/obj.sM;
                F = obj.sM*obj.dF0*sigma/(1 + sigma*(sigma + obj.dF0*(obj.sM/obj.FM) - 2));
            elseif s_fl <= obj.sG
                sigma = (s_fl - obj.sM)/(obj.sG - obj.sM);
                F = obj.FM - (obj.FM - obj.FG)*sigma^2*(3 - 2*sigma);
            else
                F = obj.FG;
            end
            F_xfl = F*cos(obj.x(8));
            F_yfl = F*sin(obj.x(8));
            % F_xfr, F_yfr
            vel = vcog + obj.x(3)*(-obj.tf/2 + -obj.xp(3)*obj.x(6));
            w_fr = u(3);
            s_fr = obj.r*w_fr/vel - 1;
            if s_fr <= obj.sM
                sigma = s_fr/obj.sM;
                F = obj.sM*obj.dF0*sigma/(1 + sigma*(sigma + obj.dF0*(obj.sM/obj.FM) - 2));
            elseif s_fr <=obj. sG
                sigma = (s_fr -obj.sM)/(obj.sG - obj.sM);
                F = obj.FM - (obj.FM - obj.FG)*sigma^2*(3 - 2*sigma);
            else
                F = obj.FG;
            end
            F_xfr = F*cos(obj.x(9));
            F_yfr = F*sin(obj.x(9));
            % F_xrl, F_yrl
            vel = vcog + obj.x(3)*(obj.tr/2 + (obj.l-obj.xp(3))*obj.x(6));
            w_rl = u(4);
            s_rl = obj.r*w_rl/vel - 1;
            if s_rl <= obj.sM
                sigma = s_rl/obj.sM;
                F = obj.sM*obj.dF0*sigma/(1 + sigma*(sigma + obj.dF0*(obj.sM/obj.FM) - 2));
            elseif s_rl <= obj.sG
                sigma = (s_rl - obj.sM)/(obj.sG - obj.sM);
                F = obj.FM - (obj.FM - obj.FG)*sigma^2*(3 - 2*sigma);
            else
                F = obj.FG;
            end
            F_xrl = F*cos(obj.x(10));
            F_yrl = F*sin(obj.x(10));
            % F_xrr, F_yrr
            vel = vcog + obj.x(3)*(-obj.tr/2 + (obj.l-obj.xp(3))*obj.x(6));
            w_rr = u(5);
            s_rr = obj.r*w_rr/vel - 1;
            if s_rr <= obj.sM
                sigma = s_rr/obj.sM;
                F = obj.sM*obj.dF0*sigma/(1 + sigma*(sigma + obj.dF0*(obj.sM/obj.FM) - 2));
            elseif s_rr <= obj.sG
                sigma = (s_rr - obj.sM)/(obj.sG - obj.sM);
                F = obj.FM - (obj.FM - obj.FG)*sigma^2*(3 - 2*sigma);
            else
                F = obj.FG;
            end
            F_xrr = F*cos(obj.x(11));
            F_yrr = F*sin(obj.x(11));
            
            % Store the tire forces
            Ft = [F_xfl, F_yfl, F_xfr, F_yfr, F_xrl, F_yrl, F_xrr, F_yrr]';

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


        % Position model
        function posForklit_next = NextForkliftPosition(obj)
            obj.pos(1) = obj.pos(1) + obj.dt*(obj.x(1)) + (1/2)*obj.x(7)*obj.dt^2;
            obj.pos(2) = obj.pos(2) + obj.dt*(obj.x(2)) + (1/2)*obj.x(4)*obj.dt^2;

            posForklit_next = obj.pos;
        end


        % Funzione calcolo distanza robot-tag
        function distance = getTagDistance(obj,tag_position)
            x_tag = tag_position(1);
            y_tag = tag_position(2);

            distance = sqrt((obj.pos(1) - x_tag)^2 + (obj.pos(2) - y_tag)^2);
            
        end
        
    
    
    end

end


