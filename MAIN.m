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
% Website: http://www.sarabkha.info
% Last revision: 08/02/2021
% Environment: MATLAB R2020b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean the workspace

clc
clear all
close all

global dt initial_state;

%% Changeable parameters

simulation_duration = 60; % [s]

%% Static parameters

dt = 0.001;
kend = simulation_duration/dt;

initial_state = [0 0 0.1 0 0 pi 0 0 0 0 0 0];%[0 0 0.1 0 0 pi 0 0 0 0 0 0];

%% Prealocate variables

pose = zeros(kend, 6);
pose(1,:) = initial_state(1:6);

t = dt*(1:kend)';

%% Read gates' poses

gates = load('gates/gates.txt');
gates(:,4) = gates(:,4)/180*pi; % converts from degrees to radiants

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
    
end

%% Show animation

score = environment(gates, pose, pose_d);

%% Show results

disp('**********');
disp(['Controller runs at ', num2str(kend/elapsed), 'Hz']);
disp(['Score is ', num2str(score)]);