%% Clean the workspace

clc
clear all
close all

global dt desired_speed enable_animation;

%% Changeable parameters (can be adjusted by participants)

desired_speed = 3; % desired speed over the trajectory [in m/s]
enable_animation = true; % enable/disable animation (animation slows down the simulation)

simulation_time = 60; % [s]

%% Static parameters

dt = 0.001;
kend = simulation_time/dt;

%% Prealocate variables

odom = zeros(kend, 12);

t = dt*(1:kend)';

%% Trajectory genaration

[pose_d, velocity_d] = trajectory_generator(dt);

%% Main loop

elapsed = 1;
for k = 1:kend
    
    %% UAV controller
    tic;
    
    command(k + 1,:) = CONTROLLER(odom(k,[1:3,7:9]), pose_d(k,:), velocity_d(k,:));
    
    elapsed = elapsed + toc;
    
    %% UAV model
    
    odom(k + 1,:) = uav(command(k + 1,:));
    
end

% figure(2)
% hold on
% grid on
% plot(1:kend+1, 180/pi*command(:,1), 'r:');
% plot(1:kend+1, 180/pi*command(:,2), 'g:');
% plot(1:kend+1, 180/pi*command(:,3), 'b:');
% plot(1:kend+1, 180/pi*odom(:,7), 'r-');
% plot(1:kend+1, 180/pi*odom(:,8), 'g-');
% plot(1:kend+1, 180/pi*odom(:,9), 'b-');
% plot(1:kend+1, 10*odom(:,4), 'r--');
% plot(1:kend+1, 10*odom(:,5), 'g--');
% plot(1:kend+1, 10*odom(:,1), 'm-');
% plot(1:kend+1, 10*odom(:,2), 'c-');

%% Show animation

score = environment(odom, pose_d);

%% Show results

disp('**********');
disp(['Mean speed is ', num2str(mean(sqrt(sum(odom(:,4:6).^2, 2)))), 'm/s']);
disp(['Controller runs at ', num2str(kend/elapsed), 'Hz']);
disp(['Score is ', num2str(score)]);