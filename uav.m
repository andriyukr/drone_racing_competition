%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% uav: dynamical model of UAV and the low-level controller
%
% Syntax: pose = uav(command)
%
% Inputs:
%    - command: control command ([roll* pitch* yaw* thrust*]) to UAV
%
% Outputs:
%    - pose: actual pose ([x y z roll pitch yaw]) of UAV
%
% Example: 
%    pose = uav(command);
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

function pose = uav(command)

%% Parameters

global dt initial_state;

%% Initialize gains

Kp_roll = 30;
Kd_roll = 5;

Kp_pitch = 30;
Kd_pitch = 5;

Kp_yaw = 30;
Kd_yaw = 5;

%% Initialize constants

Ixx = 8.1*10^(-3);  % Quadrotor moment of inertia around X axis
Iyy = 8.1*10^(-3);  % Quadrotor moment of inertia around Y axis
Izz = 14.2*10^(-3);  % Quadrotor moment of inertia around Z axis
m = 1.1;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration

%% Initialize state

persistent state;

if isempty(state)
    state = initial_state;
end

%% Attitude controller

roll_ref = command(1);
pitch_ref = -command(2);
yaw_ref = command(3);
thrust = command(4);

roll_ref = max([min([roll_ref pi/2]) -pi/2]); %
pitch_ref = max([min([pitch_ref pi/2]) -pi/2]); %
thrust = max([min([thrust 10*g]) 0]); %

%% Compute attitude errors

% denormalise yaw
yaw = state(6);
if(abs(yaw_ref - yaw) > pi)
    if(yaw_ref < yaw)
        yaw = yaw - 2*pi;
    else
        yaw = yaw + 2*pi;
    end
end

e_roll = roll_ref - state(4);
e_droll = -state(10);

e_pitch = pitch_ref - state(5);
e_dpitch = -state(11);

e_yaw = yaw_ref - yaw;
e_dyaw = -state(12);

%% Attitude controller

tau_roll = Kp_roll*e_roll + Kd_roll*e_droll; % Roll rate
tau_pitch = Kp_pitch*e_pitch + Kd_pitch*e_dpitch; % Pitch rate
tau_yaw = Kp_yaw*e_yaw + Kd_yaw*e_dyaw; % Yaw rate

input = [thrust tau_roll tau_pitch tau_yaw];

%% System dynamics

dstate(1) = state(7);                                                                                   % x_dot
dstate(2) = state(8);                                                                                   % y_dot
dstate(3) = state(9);                                                                                   % z_dot
dstate(4) = state(10) + sin(state(4))*tan(state(5))*state(11) + cos(state(4))*tan(state(5))*state(12);  % roll_dot
dstate(5) = cos(state(4))*state(11) - sin(state(4))*state(12);                                          % pitch_dot
dstate(6) = sin(state(4))/cos(state(5))*state(11) + cos(state(4))/cos(state(5))*state(12);              % yaw_dot
dstate(7) = (cos(state(4))*cos(state(6))*sin(state(5)) + sin(state(4))*sin(state(6)))*(input(1)/m);     % v_dot
dstate(8) = (-cos(state(4))*sin(state(5))*sin(state(6)) + cos(state(6))*sin(state(4)))*(input(1)/m);    % u_dot
dstate(9) = (cos(state(5))*cos(state(4)))*(input(1)/m) - g;                                             % w_dot
dstate(10) = ((Iyy - Izz)/Ixx)*state(11)*state(12) + (input(2)/Ixx);                                    % p_dot
dstate(11) = ((Izz - Ixx)/Iyy)*state(10)*state(12) + (input(3)/Iyy);                                    % q_dot
dstate(12) = ((Ixx - Iyy)/Izz)*state(10)*state(11) + (input(4)/Izz);                                    % r_dot

state = state + dt*dstate;

%% Noisy localisation

pose = state(1:6); % x, y, z, roll, pitch, yaw

speed = sqrt(sum(state(7:9).^2));
rate = sqrt(sum(state(10:12).^2));
pose(1:3) = pose(1:3) + randn(1,3)/1000*speed; % add white noise to position
pose(4:6) = pose(4:6) + randn(1,3)/1000*rate; % add white noise to attitude