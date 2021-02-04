function [pose_d, velocity_d] = trajectory_generator(dt)

global desired_speed;

%% Read gates' poses

gates = load('gates/gates.txt');
gates(:,4) = gates(:,4)/180*pi; % converts from degrees to radiants

numGates = size(gates, 1);

%% Calculate waypoints

waypoints(1,:) = [0 0 0 0];
waypoints(2,:) = [0 0 1 atan2(gates(1,2), gates(1,1))];
for i=0:50
    currentGate = rem(i, numGates) + 1;
    nextGate = rem(i + 1, numGates) + 1;
    
    % position
    waypoints(i*3 + 3,1:3) = gates(currentGate,1:3) + ...
        [0 -1 0]*[cos(gates(currentGate,4)) sin(gates(currentGate,4)) 0; -sin(gates(currentGate,4)) cos(gates(currentGate,4)) 0; 0 0 1]; % before the gate
    waypoints(i*3 + 4,1:3) = gates(currentGate,1:3); % at the gate center
    waypoints(i*3 + 5,1:3) = gates(currentGate,1:3) + ...
        [0 1 0]*[cos(gates(currentGate,4)) sin(gates(currentGate,4)) 0; -sin(gates(currentGate,4)) cos(gates(currentGate,4)) 0; 0 0 1]; % after the gate
    
    % heading
    waypoints(i*3 + 3,4) = atan2(gates(currentGate,2) - waypoints(i*3 + 3,2), gates(currentGate,1) - waypoints(i*3 + 3,1)); % before the gate
    waypoints(i*3 + 4,4) = atan2(gates(nextGate,2) - waypoints(i*3 + 4,2), gates(nextGate,1) - waypoints(i*3 + 4,1)); % at the gate center
    waypoints(i*3 + 5,4) = atan2(gates(nextGate,2) - waypoints(i*3 + 5,2), gates(nextGate,1) - waypoints(i*3 + 5,1)); % after the gate   
end

% denormalise headind (to make it continuos)
for i = 2:size(waypoints, 1)
    waypoints(i,4) = waypoints(i,4) - 2*pi*fix((waypoints(i,4) - waypoints(i - 1,4) + pi*sign(waypoints(i,4) - waypoints(i - 1,4)))/(2*pi));
end

%% Calculate distances between waypoints

distances = sqrt(sum(diff(waypoints(:,1:3)).^2, 2));

%% Calculate time to reach each waypoint

time(1) = 0;
for i=2:numel(distances) + 1
    time(i) = time(i - 1) + distances(i - 1)/desired_speed;
end

%% Interpolate waypoints

timeInterpolated = time(1):dt:time(end);

pose_d(:,1:3) = interp1(time, waypoints(:,1:3), timeInterpolated, 'spline'); % method = 'pchip' | 'makima' | 'spline'
pose_d(:,4) = interp1(time, waypoints(:,4), timeInterpolated, 'linear');

%% Calculate desred velocity

velocity_d = diff(pose_d)/dt;