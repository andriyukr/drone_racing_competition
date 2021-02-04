function score = environment(pose, pose_d)

global enable_animation;

%% Parameters

sizeDrone = 0.1; % [m]

tailLength = 1000; % [ms]

%% Flags

flagApproaching = true;

%% Read gates' poses

gates = load('gates/gates.txt');

numGates = size(gates, 1);

for i = 1:numGates
    gate(i).translation = gates(i,1:3);
    gate(i).orientation = [0 0 gates(i,4)]/180*pi;
end

%% Transform the data

for i = 1:numGates
    gate(i).corners.front.xOriginal = [-0.8 0.8; -0.8 0.8];
    gate(i).corners.front.yOriginal = [0 0; 0 0];
    gate(i).corners.front.z = [0.8 0.8; -0.8 -0.8];
    
    gate(i).corners.front.x = cos(gate(i).orientation(3)) * gate(i).corners.front.xOriginal - sin(gate(i).orientation(3)) * gate(i).corners.front.yOriginal;
    gate(i).corners.front.y = sin(gate(i).orientation(3)) * gate(i).corners.front.xOriginal + cos(gate(i).orientation(3)) * gate(i).corners.front.yOriginal;
    
    gate(i).corners.front.x = gate(i).corners.front.x + gate(i).translation(1);
    gate(i).corners.front.y = gate(i).corners.front.y + gate(i).translation(2);
    gate(i).corners.front.z = gate(i).corners.front.z + gate(i).translation(3);
    
    gate(i).corners.back.xOriginal = [-0.8 0.8; -0.8 0.8];
    gate(i).corners.back.yOriginal = [0.01 0.01; 0.01 0.01];
    gate(i).corners.back.z = [0.8 0.8; -0.8 -0.8];
    
    gate(i).corners.back.x = cos(gate(i).orientation(3)) * gate(i).corners.back.xOriginal - sin(gate(i).orientation(3)) * gate(i).corners.back.yOriginal;
    gate(i).corners.back.y = sin(gate(i).orientation(3)) * gate(i).corners.back.xOriginal + cos(gate(i).orientation(3)) * gate(i).corners.back.yOriginal;
    
    gate(i).corners.back.x = gate(i).corners.back.x + gate(i).translation(1);
    gate(i).corners.back.y = gate(i).corners.back.y + gate(i).translation(2);
    gate(i).corners.back.z = gate(i).corners.back.z + gate(i).translation(3);
    
    gate(i).legs.left.xOriginal = [0 0 0 0];
    gate(i).legs.left.yOriginal = [0 0 0.5 -0.5];
    gate(i).legs.left.z = [gate(i).corners.front.z(2, 1) 0 0 0];
    gate(i).legs.right.xOriginal = [0 0 0 0];
    gate(i).legs.right.yOriginal = [0 0 0.5 -0.5];
    gate(i).legs.right.z = [gate(i).corners.front.z(2, 1) 0 0 0];
    
    gate(i).legs.left.x = cos(gate(i).orientation(3)) * gate(i).legs.left.xOriginal - sin(gate(i).orientation(3)) * gate(i).legs.left.yOriginal;
    gate(i).legs.left.y = sin(gate(i).orientation(3)) * gate(i).legs.left.xOriginal + cos(gate(i).orientation(3)) * gate(i).legs.left.yOriginal;
    gate(i).legs.right.x = cos(gate(i).orientation(3)) * gate(i).legs.right.xOriginal - sin(gate(i).orientation(3)) * gate(i).legs.right.yOriginal;
    gate(i).legs.right.y = sin(gate(i).orientation(3)) * gate(i).legs.right.xOriginal + cos(gate(i).orientation(3)) * gate(i).legs.right.yOriginal;
    
    gate(i).legs.left.x = gate(i).legs.left.x + gate(i).corners.front.x(1, 1);
    gate(i).legs.left.y = gate(i).legs.left.y + gate(i).corners.front.y(1, 1);
    gate(i).legs.right.x = gate(i).legs.right.x + gate(i).corners.front.x(1, 2);
    gate(i).legs.right.y = gate(i).legs.right.y + gate(i).corners.front.y(1, 2);
end

%% Show 3D environment

figure(1);
hold on;
grid on;

% img_front = imread('gates/1x1_front.png');
% img_back = imread('gates/1x1_back.png');

load('gates/gates');

set(gca, 'fontsize', 15);
axis equal;
axis ([-9 9 -9 9 0 4]);
xlabel('$x$ [m]', 'interpreter', 'latex', 'fontsize', 15);
ylabel('$y$ [m]', 'interpreter', 'latex', 'fontsize', 15);
zlabel('$z$ [m]', 'interpreter', 'latex', 'fontsize', 15);
set(gca, 'TickLabelInterpreter', 'latex')
view(-40, 40);

%% Show gates

for i = 1:numGates
    s_front = surf(gate(i).corners.front.x, gate(i).corners.front.y, gate(i).corners.front.z, 'CData', img_front, 'FaceColor', 'texturemap');    % Plot the surface
    s_front.AlphaData = (img_front(:,:,1)<255 | img_front(:,:,2)>0);
    s_front.FaceAlpha = 'texturemap';
    s_back = surf(gate(i).corners.back.x, gate(i).corners.back.y, gate(i).corners.back.z, 'CData', img_back, 'FaceColor', 'texturemap');    % Plot the surface
    s_back.AlphaData = (img_back(:,:,1)<255 | img_back(:,:,2)>0);
    s_back.FaceAlpha = 'texturemap';
    plot3(gate(i).legs.left.x, gate(i).legs.left.y, gate(i).legs.left.z, 'color', [0.5, 0.5, 0.5], 'linewidth', 1);
    plot3(gate(i).legs.right.x, gate(i).legs.right.y, gate(i).legs.right.z, 'color', [0.5, 0.5, 0.5], 'linewidth', 1);
end

%% Show desired trajectory

plot3(pose_d(:,1), pose_d(:,2), pose_d(:,3), 'k:');

drawnow;

%% Generate external and internal regions of each gate

[xExternal,yExternal,zExternal] = ...
    meshgrid([-0.8 - sizeDrone 0.8 + sizeDrone], [-sizeDrone 0], [0.8 + sizeDrone -0.5 - sizeDrone]);
xExternal = [xExternal(:);0];
yExternal = [yExternal(:);0];
zExternal = [zExternal(:);0];

[xInternal,yInternal,zInternal] = ...
    meshgrid([-0.5 + sizeDrone 0.5 - sizeDrone], [-sizeDrone 0], [0.5 - sizeDrone -0.5 + sizeDrone]);
xInternal = [xInternal(:);0];
yInternal = [yInternal(:);0];
zInternal = [zInternal(:);0];

for i = 1:numGates
    gate(i).external = [xExternal yExternal zExternal];
    gate(i).external(:,1:2) = gate(i).external(:,1:2)*[cos(gates(i,4)) -sin(gates(i,4)); sin(gates(i,4)) cos(gates(i,4))]; % rotate the gate
    gate(i).external = gate(i).external + gates(i,1:3); % translate the gate
    
    gate(i).triangulizationExternal = delaunayn(gate(i).external); % Generate delaunay triangulization
    
    gate(i).internal = [xInternal yInternal zInternal];
    gate(i).internal(:,1:2) = gate(i).internal(:,1:2)*[cos(gates(i,4)) -sin(gates(i,4)); sin(gates(i,4)) cos(gates(i,4))]; % rotate the gate
    gate(i).internal = gate(i).internal + gates(i,1:3); % translate the gate
 
    gate(i).triangulizationInternal = delaunayn(gate(i).internal); % generate delaunay triangulization
end

%% Show actual trajectory

score = 0;
nextGate = 1;
lastGate = 0;
disp(['Aiming gate ', num2str(nextGate), '.']);

c = flipud(autumn(tailLength)); % vector of colors
for k = 100:100:size(pose(:,1), 1)
    
    %% Score performance

    for i = 1:numGates
        inside = tsearchn(gate(i).external, gate(i).triangulizationExternal, pose(max(1, k - tailLength + 1):k,1:3));

        if sum(~isnan(inside)) > 0
            if flagApproaching && i ~= lastGate
                disp(['Approaching gate ', num2str(i), '.']);
                flagApproaching = false;
            end
            crossed = tsearchn(gate(i).internal, gate(i).triangulizationInternal, pose(max(1, k - tailLength + 1):k,1:3));
            
            if sum(~isnan(crossed)) > 0
                if i ~= lastGate
                    disp(['Crossed gate ', num2str(i), '!']);
                    score = score + 1;
                    if i ~= nextGate
                        score = score - 0.5;
                    end
                    lastGate = i;
                    nextGate = rem(i, numGates) + 1;
                    flagApproaching = true;
                    disp(['Aiming gate ', num2str(nextGate), '.']);
                end
            else
                disp(['Crashed!!!', ' (', num2str(i), ')']);
                
                scatter3(pose(max(1, k - tailLength + 1):k,1), pose(max(1, k - tailLength + 1):k,2), pose(max(1, k - tailLength + 1):k,3), 10, c(end - (k - max(1, k - tailLength + 1)):end,:), 'filled');
                scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
                annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"));
                annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)));
                return;
            end
        end
    end
    
    if enable_animation
        hTrajectory = scatter3(pose(max(1, k - tailLength + 1):k,1), pose(max(1, k - tailLength + 1):k,2), pose(max(1, k - tailLength + 1):k,3), 10, c(end - (k - max(1, k - tailLength + 1)):end,:), 'filled');
        hPoint = scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
        hTimer = annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"));
        hScore = annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)));
            
        drawnow;
        
        delete(hTrajectory);
        delete(hScore);
        delete(hTimer);
        delete(hPoint);
    end
end

score = score + 5;

c = flipud(autumn(size(pose, 1))); % vector of colors
scatter3(pose(:,1), pose(:,2), pose(:,3), 10, c(:,:), 'filled');
scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"));
annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)));

print('result.jpg', '-djpeg', '-r600');
