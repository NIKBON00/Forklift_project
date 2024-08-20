clear all;
clc;

%% Simulation set-up

% Sampling time
MdlInit.Ts = 0.05;  % [s]

% Length of simulation
MdlInit.T = 10;     % [s]


%% Vehicle set-up

% Vehicle data
Vehicle.q0 = zeros(3,1);

% Vehicle params
Vehicle.r = 0.2;    % [m]
Vehicle.D = 0.4;    % [m]

% Actual unicycle simulation in Matlab, with trajectory returned with a
% sampling time 'MdlInit.Ts'
[q, t, u] = UnicycleKinematicMatlab(MdlInit, Vehicle);

% Vehicle left and right angular velocities
Vehicle.A = [Vehicle.r/2, Vehicle.r/2; Vehicle.r/Vehicle.D, -Vehicle.r/Vehicle.D];
Vehicle.WheelsVel = inv(Vehicle.A)*u;

% Encoders value
Vehicle.Enc = cumsum([[0; 0], MdlInit.Ts*Vehicle.WheelsVel],2);


%% Proprioceptive sensors

% Resolution
Sensor.EncRes = 2*pi/1024;

% Encoder uncertainties (right, left)
Sensor.EncSigma = [Sensor.EncRes*2; Sensor.EncRes*3];
Sensor.EncVal = Vehicle.Enc + diag(Sensor.EncSigma)*randn(2,size(Vehicle.Enc,2));

% Rounding on the resolution
Sensor.EncVal = round(Sensor.EncVal/Sensor.EncRes)*Sensor.EncRes;


%% Indirect measurements of the forward and angular velocities

% State estimates
q_est = zeros(length(Vehicle.q0), length(t));
P = cell(1, length(t));
% Initial convariance matrix: max error on x,y = 2 m; max error for
% orientation theta = 0.1 rad
P{1} = diag([(2/3)^2, (2/3)^2, (0.1/3)^2]);

% Inputs to the system
u_dt = zeros(2,length(t));

for k=1:length(t)-1

    % Model inputs
    u_dt(:,k) = Vehicle.A*(Sensor.EncVal(:,k+1) - Sensor.EncVal(:,k));

    % F(q)
    f_q = [cos(q_est(3,k)), 0; sin(q_est(3,k)), 0; 0 1];

    % Discrete dynamic of the system (Prediction)
    q_est(:,k+1) = q_est(:,k) + f_q*u_dt(:,k);
    Q = 2*(diag(Sensor.EncSigma).^2);
    Cov_u_dt = Vehicle.A*Q*Vehicle.A';
    Ad = eye(3) + [0 0 -sin(q_est(3,k))*u_dt(1,k); 0 0 cos(q_est(3,k))*u_dt(1,k); 0 0 0];
    P{k+1} = Ad*P{k}*Ad' + f_q*Cov_u_dt*f_q';

end


%% Plot

figure(1), clf, hold on;
plot(t, q_est(1,:) - q(:,1)', 'b');
plot(t, q_est(2,:) - q(:,2)', 'k-.');
plot(t, q_est(3,:) - q(:,3)', 'g--');
legend('Error x', 'Error y', 'Error \theta', 'Location', 'best');