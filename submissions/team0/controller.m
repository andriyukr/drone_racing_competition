%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% controller: PD controller for UAV position and yaw
%
% Syntax: commands = controller(pose, pose_d, velocity_d)
%
% Inputs:
%    - pose: actual pose ([x y z roll pitch yaw]) of UAV
%    - pose_d: desired pose ([x* y* z* yaw*]) of UAV
%    - velocity_d: desired velocity ([v_x* v_y* v_z* w_yaw*]) of UAV
%
% Outputs:
%    - commands: control commands ([roll* pitch* yaw* thrust*]) to UAV
%
% Example: 
%    command = controller(pose, pose_d, velocity_d);
%
% m-files required: none
% mat-files required: none
% other files required: none
%
% Author: Andriy Sarabakha
% email: andriyukr@gmail.com
% Website: http://www.sarabakha.info
% Last revision: 08/02/2021
% Environment: MATLAB R2020b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function commands = controller(pose, pose_d, velocity_d)

global dt initial_state;

persistent old_pose; % stores previous pose

if isempty(old_pose)
    old_pose = initial_state(1:6);
end

%% Initialize gains

Kp_x = 1;
Kd_x = 2;

Kp_y = 1;
Kd_y = 2;

Kp_z = 50;
Kd_z = 10;

%% Actual state

x = pose(1);
y = pose(2);
z = pose(3);
yaw = pose(6);

vx = (pose(1) - old_pose(1))/dt;
vy = (pose(2) - old_pose(2))/dt;
vz = (pose(3) - old_pose(3))/dt;

old_pose = pose;

%% Reference values

x_ref = pose_d(1);
y_ref = pose_d(2);
z_ref = pose_d(3);
yaw_ref = pose_d(4);

dx_ref = velocity_d(1);
dy_ref = velocity_d(2);
dz_ref = velocity_d(3);

%% Compute errors

e_x = cos(yaw)*(x_ref - x) - sin(yaw)*(y_ref - y);
e_dx = cos(yaw)*(dx_ref - vx) - sin(yaw)*(dy_ref - vy);

e_y = sin(yaw)*(x_ref - x) + cos(yaw)*(y_ref - y);
e_dy = sin(yaw)*(dx_ref - vx) + cos(yaw)*(dy_ref - vy);

e_z = z_ref - z;
e_dz = dz_ref - vz;

%% Position controller

roll_ref = Kp_y*e_y + Kd_y*e_dy; %
pitch_ref = -Kp_x*e_x - Kd_x*e_dx; %
thrust = 1.1*9.81 + ...     % hower thrust = mass*gravity
    Kp_z*e_z + Kd_z*e_dz;   % total thrust on the body along z-axis

roll_ref = max([min([roll_ref pi/2]) -pi/2]); %
pitch_ref = max([min([pitch_ref pi/2]) -pi/2]); %

commands = [roll_ref pitch_ref yaw_ref thrust];