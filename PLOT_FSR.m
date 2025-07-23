clear all
clc

% Load multiple datasets from different CSV log files
data_all = {
    readmatrix('tracking_log_AL.csv'); ...
    readmatrix('tracking_log_FSFL.csv'); ...
    readmatrix('tracking_log_IO.csv'); ...
};

%%
% Extract relevant data from each dataset and store in the results structure
for idx = 1:length(data_all)
    dato = data_all{idx};
    results(idx).t = dato(:,1);        % Time
    results(idx).x = dato(:,2);       % Actual x position
    results(idx).y = dato(:,3);       % Actual y position
    results(idx).xd = dato(:,6);      % Desired x position
    results(idx).yd = dato(:,7);      % Desired y position
    results(idx).v = dato(:,9);        % Actual linear velocity
    results(idx).omega = dato(:,10);   % Actual angular velocity
    results(idx).vd = dato(:,11);      % Desired linear velocity
    results(idx).omegad = dato(:,12);  % Desired angular velocity
    results(idx).ex = dato(:,13);      % x error
    results(idx).ey = dato(:,14);      % y error
    results(idx).etheta = dato(:,15);  % Orientation error (theta)
    results(idx).ephi = dato(:,16);    % Steering angle error (phi)
    results(idx).theta = dato(:,4);    % Actual orientation
    results(idx).phi = dato(:,5);      % Actual steering angle
end

%%
% Call custom function to plot mobile robot tracking results
plot_project(results)

%%
% Load arm tracking data from CSV file
arm_data = readmatrix('tracking_log_arm.csv');

% Extract relevant data from arm dataset
result.t = arm_data(:,1);         % Time
result.error = arm_data(:,2);     % Tracking error
result.dq1 = arm_data(:,3);       % Joint 1 velocity
result.dq2 = arm_data(:,4);       % Joint 2 velocity
result.dq3 = arm_data(:,5);       % Joint 3 velocity
result.dq4 = arm_data(:,6);       % Joint 4 velocity
result.dq5 = arm_data(:,7);       % Joint 5 velocity
result.dq6 = arm_data(:,8);       % Joint 6 velocity

%%
% Call custom function to plot arm tracking results
plot_arm(result)  

