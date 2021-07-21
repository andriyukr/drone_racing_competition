%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN: main file for the autonomous drone racing simulation
%
% m-files required:
%    - controller
%    - environment
%    - trajectory
%    - uav
% mat-files required: none
% other files required:
%    - gates.txt (in /gates): contains poses of the gates
%
% Author: Andriy Sarabakha
% email: andriyukr@gmail.com
% Website: http://www.sarabakha.info
% Last revision: 08/02/2021
% Environment: MATLAB R2020b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean the workspace

clc
clear all
close all

global team round dt initial_state;

%% Team number

% Default - PID: 0
% Team1 - University1: 1

round = 1;
team = 1;

%% Parameters

simulation_duration = 60; % [s]

dt = 0.001; % [s]
kend = simulation_duration/dt;

initial_state = [0 0 0.1 0 0 -pi/2 0 0 0 0 0 0];

%% Prealocate variables

pose = zeros(kend, 6);
pose(1,:) = initial_state(1:6);

t = dt*(1:kend)';

%% Read gates' poses

gates = load('gates/gates_validation.txt');
gates(:,4) = gates(:,4)/180*pi; % converts from degrees to radiants

%% Load submission from team

addpath(['submissions/team', num2str(team)]);

%% Trajectory genaration

[pose_d, velocity_d] = trajectory(gates);

%% Main loop

elapsed = 0;
for k = 1:kend
    
    %% UAV controller
    
    tic;
    
    command = controller(pose(k,:), pose_d(k,:), velocity_d(k,:));
    
    elapsed = elapsed + toc; % for computational time
    
    %% UAV model
    
    pose(k + 1,:) = uav(command);
    
    %% Show progress
    
    if rem(100*k/kend, 1) == 0
        disp(['Progress: ', num2str(100*k/kend), '%']);
    end
end

save(['submissions/team', num2str(team), '/round', num2str(round), '.mat'], 'pose', 'elapsed');

%% Show animation

score = environment(gates, pose, pose_d);

%% Show results

disp('**********');
disp(['Controller runs at ', num2str(kend/elapsed), 'Hz']);
disp(['Score is ', num2str(score)]);

%% Unload submission from team

rmpath(['submissions/team', num2str(team)]);