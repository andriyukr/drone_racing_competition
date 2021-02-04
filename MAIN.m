%% Clean the workspace

clc
clear all
close all

global dt initial_state desired_speed enable_animation;

%% Changeable parameters (can be adjusted by participants)

desired_speed = 1; % desired speed over the trajectory [in m/s]
enable_animation = true; % enable/disable animation (animation slows down the simulation)

simulation_time = 60; % [s]

%% Static parameters

dt = 0.001;
kend = simulation_time/dt;

initial_state = [0 0 0.1 0 0 0 0 0 pi 0 0 0];

%% Prealocate variables

odom = zeros(kend, 12);
odom(1,:) = initial_state;

t = dt*(1:kend)';

%% Trajectory genaration

[pose_d, velocity_d] = trajectory_generator(dt);

%% Main loop

elapsed = 1;
for k = 1:kend
    
    %% UAV controller
    tic;
    
    command = CONTROLLER(odom(k,[1:3,7:9]), pose_d(k,:), velocity_d(k,:));
    
    elapsed = elapsed + toc;
    
    %% UAV model
    
    odom(k + 1,:) = uav(command);
    
end

%% Show animation

score = environment(odom(:,[1:3,7:9]), pose_d);

%% Show results

disp('**********');
disp(['Mean speed is ', num2str(mean(sqrt(sum(odom(:,4:6).^2, 2)))), 'm/s']);
disp(['Controller runs at ', num2str(kend/elapsed), 'Hz']);
disp(['Score is ', num2str(score)]);