function pose = uav(command)

%% Parameters

global dt;

%% Initialize gains

Kp_roll = 30;
Kd_roll = 5;

Kp_pitch = 30;
Kd_pitch = 5;

Kp_yaw = 30;
Kd_yaw = 5;

%% Initial state

x_init = 0;
y_init = 0;
z_init = 0.1;
yaw_init = 0;

%% Initialize constants

Ixx = 8.1*10^(-3);  % Quadrotor moment of inertia around X axis
Iyy = 8.1*10^(-3);  % Quadrotor moment of inertia around Y axis
Izz = 14.2*10^(-3);  % Quadrotor moment of inertia around Z axis
Jtp = 104*10^(-6);  % Total rotational moment of inertia around the propeller axis
b = 54.2*10^(-5);  % Thrust factor
d = 1.1*10^(-6);  % Drag factor
l = 0.2;  % Distance to the center of the Quadrotor
m = 1;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration

%% Initialize state

persistent state;

if isempty(state)
    state = zeros(1, 12);
    state(1) = x_init;
    state(2) = y_init;
    state(3) = z_init;
    state(9) = yaw_init;
end

%% Position controller

roll_ref = command(1);
pitch_ref = -command(2);
yaw_ref = command(3);
thrust = command(4);

roll_ref = max([min([roll_ref pi/2]) -pi/2]); %
pitch_ref = max([min([pitch_ref pi/2]) -pi/2]); %
thrust = max([min([thrust 10*g]) 0]); %

%% Compute attitude errors

% denormalise yaw
yaw = state(9);
if(abs(yaw_ref - yaw) > pi)
    if(yaw_ref < yaw)
        yaw = yaw - 2*pi;
    else
        yaw = yaw + 2*pi;
    end
end

e_roll = roll_ref - state(7);
e_droll = -state(10);

e_pitch = pitch_ref - state(8);
e_dpitch = -state(11);

e_yaw = yaw_ref - yaw;
e_dyaw = -state(12);

%% Attitude controller

tau_roll = Kp_roll*e_roll + Kd_roll*e_droll; % Roll rate
tau_pitch = Kp_pitch*e_pitch + Kd_pitch*e_dpitch; % Pitch rate
tau_yaw = Kp_yaw*e_yaw + Kd_yaw*e_dyaw; % Yaw rate

input = [thrust tau_roll tau_pitch tau_yaw];

%% System dynamics

% dstate(1) = state(4);                                                                                   % x_dot
% dstate(2) = state(5);                                                                                   % y_dot
% dstate(3) = state(6);                                                                                   % z_dot
% dstate(4) = (sin(state(9))*sin(state(7)) + cos(state(9))*sin(state(8))*cos(state(7)))*(input(1)/m);     % x_dotdot
% dstate(5) = (-cos(state(9))*sin(state(7)) + sin(state(9))*sin(state(8))*cos(state(7)))*(input(1)/m);    % y_dotdot
% dstate(6) = (cos(state(8))*cos(state(7)))*(input(1)/m) - g;                                             % z_dotdot
% dstate(7) = state(10);                                                                                  % roll_dot
% dstate(8) = state(11);                                                                                  % pitch_dot
% dstate(9) = state(12);                                                                                  % yaw_dot
% dstate(10) = ((Iyy - Izz)/Ixx)*state(11)*state(12) - (Jtp/Ixx)*state(11) + (input(2)/Ixx);              % roll_dotdot
% dstate(11) = ((Izz - Ixx)/Iyy)*state(10)*state(12) + (Jtp/Iyy)*state(10) + (input(3)/Iyy);              % pitch_dotdot
% dstate(12) = ((Ixx - Iyy)/Izz)*state(10)*state(11) + (input(4)/Izz);                                    % yaw_dotdot

dstate(1) = state(4);                                                                                                 % x_dot
dstate(2) = state(5);                                                                                                 % y_dot
dstate(3) = state(6);                                                                                                 % z_dot
dstate(4) = (cos(state(7))*cos(state(9))*sin(state(8)) + sin(state(7))*sin(state(9)))*(input(1)/m);         % x_dotdot
dstate(5) = (-cos(state(7))*sin(state(8))*sin(state(9)) + cos(state(9))*sin(state(7)))*(input(1)/m);        % y_dotdot
dstate(6) = (cos(state(8))*cos(state(7)))*(input(1)/m) - g;                                                       % z_dotdot
dstate(7) = state(10) + sin(state(7))*tan(state(8))*state(11) + cos(state(7))*tan(state(8))*state(12);    % roll_dot
dstate(8) = cos(state(7))*state(11) - sin(state(7))*state(12);                                                  % pitch_dot
dstate(9) = sin(state(7))/cos(state(8))*state(11) + cos(state(7))/cos(state(8))*state(12);                  % yaw_dot
dstate(10) = ((Iyy - Izz)/Ixx)*state(11)*state(12) + (input(2)/Ixx);                                              % roll_dotdot
dstate(11) = ((Izz - Ixx)/Iyy)*state(10)*state(12) + (input(3)/Iyy);                                              % pitch_dotdot
dstate(12) = ((Ixx - Iyy)/Izz)*state(10)*state(11) + (input(4)/Izz); 

state = state + dt*dstate;

% pose = [state(1:3) state(9)]; % x, y, z, yaw
pose = state; % full state

%% Noisy localisation

speed = sqrt(sum(state(4:6).^2));
rate = sqrt(sum(state(10:12).^2));
pose(1:3) = pose(1:3) + randn(1,3)/1000*speed; % add white noise to position
pose(7:9) = pose(7:9) + randn(1,3)/1000*rate; % add white noise to attitude