close all;
clc;
clear all;
%% Setup MonteCarlo Simulation
% For doing the following simulations, disactivate all the figure (plots
% and histogram) in main_core, in order to have faster times of computation !!

N_test = 5; % Number of simulations

% Vectors for storing the total errors on position, orientation trace
N_error_position = [];
N_error_orientation = [];
N_error_trace = [];
errors_total = [];
errors_orien = [];

% Run the simulations
for n=1:N_test
    main_core;

    N_error_position = [N_error_position; E_position];
    N_error_orientation = [N_error_orientation; E_orientation];
    N_error_trace = [N_error_trace; E_trace];
    errors_total = [errors_total; mean_error];
    errors_orien = [errors_orien; mean_error_theta];

end


%% Mean values

% Calculation of mean values
mean_position_value = mean(N_error_position);
mean_orientation_value = mean(N_error_orientation);
mean_trace_value = mean(N_error_trace);

%% Histogram
indexes = find(errors_total==0);
errors_total(indexes)=[];

figure();
histogram(errors_total);
grid on;
xlabel('Error [m]');
ylabel('Number of values');
title('Monte Carlo Analysis on Position Error');


indexes = find(errors_orien==0);
errors_orien(indexes)=[];

figure();
histogram(errors_orien);
grid on;
xlabel('Error [°]');
ylabel('Number of values');
title('Monte Carlo Analysis on Orientation Error');

