%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% environment: visualisation of the racing arena and performance evaluation
%
% Syntax: score = environment(gates, pose, pose_d)
%
% Inputs:
%    - gates: positions ([x y z]) and headings (yaw) of the gates
%    - pose: actual pose ([x y z roll pitch yaw]) of UAV
%    - pose_d: desired pose ([x* y* z* yaw*]) of UAV
%
% Outputs:
%    - score: score accumulated during racing
%
% Results:
%    - result.jpg: snapshot of the racing environment and the flown path
%
% Meta parameters:
%    - enable_animation: flag to enable/disable animation (animation slows down the simulation) {true, false}
%
% Example: 
%    score = environment(gates, pose, pose_d);
%
% m-files required: none
% mat-files required:
%    - gates (in /gates): contains meshes of the gates
% other files required: none
%
% Author: Andriy Sarabakha
% email: andriyukr@gmail.com
% Website: http://www.sarabakha.info
% Last revision: 08/02/2021
% Environment: MATLAB R2020b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function score = environment(gates, pose, pose_d)

global team round

%% Parameters

enable_animation = true; % flag to enable/disable animation (animation slows down the simulation) {true, false}
save_video = false; % flag to the video with animation (can be activated only if animation is activated) {true, false}

tailLength = 1000; % [ms]

sizeDrone = 0.05; % radius [m]
sizeEnvironment = [-10 10 -10 10 0 5]; % [m]

%% Flags

flagApproaching = true;
flagLost = false;

%% Initialise video writer

if save_video
    v = VideoWriter(['submissions/team', num2str(team), '/round', num2str(round), '.avi']);
    v.Quality = 100;
    v.FrameRate = 50; % 10 [fps]
    open(v);
end

%% Read gates' poses

numGates = size(gates, 1);

for i = 1:numGates
    gate(i).translation = gates(i,1:3);
    gate(i).orientation = [0 0 gates(i,4)];
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
    
    gate(i).legs.left.x = cos(gate(i).orientation(3))*gate(i).legs.left.xOriginal - sin(gate(i).orientation(3))*gate(i).legs.left.yOriginal;
    gate(i).legs.left.y = sin(gate(i).orientation(3))*gate(i).legs.left.xOriginal + cos(gate(i).orientation(3))*gate(i).legs.left.yOriginal;
    gate(i).legs.right.x = cos(gate(i).orientation(3))*gate(i).legs.right.xOriginal - sin(gate(i).orientation(3))*gate(i).legs.right.yOriginal;
    gate(i).legs.right.y = sin(gate(i).orientation(3))*gate(i).legs.right.xOriginal + cos(gate(i).orientation(3))*gate(i).legs.right.yOriginal;
    
    gate(i).legs.left.x = gate(i).legs.left.x + gate(i).corners.front.x(1, 1);
    gate(i).legs.left.y = gate(i).legs.left.y + gate(i).corners.front.y(1, 1);
    gate(i).legs.right.x = gate(i).legs.right.x + gate(i).corners.front.x(1, 2);
    gate(i).legs.right.y = gate(i).legs.right.y + gate(i).corners.front.y(1, 2);
end

%% Show 3D environment

figure('units', 'normalized', 'outerposition', [0 0 1 1]);
hold on;
grid on;

mesh_gate_front = imread('gates/1x1_front.png');
mesh_gate_back = imread('gates/1x1_back.png');
mesh_fuzzieee = imread('gates/fuzzieee.png');
mesh_tum = imread('gates/tum.png');
mesh_au = imread('gates/au.png');
mesh_mathworks = imread('gates/mathworks.png');
mesh_fnr = imread('gates/fnr.png');
save('gates/meshes', 'mesh_gate_front', 'mesh_gate_back', 'mesh_fuzzieee', 'mesh_tum', 'mesh_au', 'mesh_mathworks', 'mesh_fnr');

load('gates/meshes');

set(gca, 'fontsize', 15);
axis equal;
axis (sizeEnvironment);
xlabel('$x$ [m]', 'interpreter', 'latex', 'fontsize', 15);
ylabel('$y$ [m]', 'interpreter', 'latex', 'fontsize', 15);
zlabel('$z$ [m]', 'interpreter', 'latex', 'fontsize', 15);
set(gca, 'TickLabelInterpreter', 'latex')
view(-40, 40);
% view(0, 90);

%% Show sponsors

s_front = surf([-1 3; -3 1], [3 -1; 1 -3], [0 0; 0 0], 'CData', mesh_fuzzieee, 'FaceColor', 'texturemap', 'EdgeColor', 'none');    % Plot the surface
s_front.AlphaData = (mesh_fuzzieee(:,:,1)<255 | mesh_fuzzieee(:,:,2)<255 | mesh_fuzzieee(:,:,3)<255);
s_front.FaceAlpha = 'texturemap';

s_front = surf([6 10; 4 8], [10 6; 8 4], [0 0; 0 0], 'CData', mesh_au, 'FaceColor', 'texturemap', 'EdgeColor', 'none');    % Plot the surface
s_front.AlphaData = (mesh_au(:,:,1)<255 | mesh_au(:,:,2)<255 | mesh_au(:,:,3)<255);
s_front.FaceAlpha = 'texturemap';

s_front = surf([10 6; 8 4], [-6 -10; -4 -8], [0 0; 0 0], 'CData', mesh_fnr, 'FaceColor', 'texturemap', 'EdgeColor', 'none');    % Plot the surface
s_front.AlphaData = (mesh_fnr(:,:,1)<255 | mesh_fnr(:,:,2)<255 | mesh_fnr(:,:,3)<255);
s_front.FaceAlpha = 'texturemap';

s_front = surf([-8 -4; -10 -6], [-4 -8; -6 -10], [0 0; 0 0], 'CData', mesh_tum, 'FaceColor', 'texturemap', 'EdgeColor', 'none');    % Plot the surface
s_front.AlphaData = (mesh_tum(:,:,1)<255 | mesh_tum(:,:,2)<255 | mesh_tum(:,:,3)<255);
s_front.FaceAlpha = 'texturemap';

s_front = surf([-10 -6; -8 -4], [6 10; 4 8], [0 0; 0 0], 'CData', mesh_mathworks, 'FaceColor', 'texturemap', 'EdgeColor', 'none');    % Plot the surface
s_front.AlphaData = (mesh_mathworks(:,:,1)<255 | mesh_mathworks(:,:,2)<255 | mesh_mathworks(:,:,3)<255);
s_front.FaceAlpha = 'texturemap';

%% Show gates

for i = 1:numGates
    s_front = surf(gate(i).corners.front.x, gate(i).corners.front.y, gate(i).corners.front.z, 'CData', mesh_gate_front, 'FaceColor', 'texturemap');    % Plot the surface
    s_front.AlphaData = (mesh_gate_front(:,:,1)<255 | mesh_gate_front(:,:,2)>0);
    s_front.FaceAlpha = 'texturemap';
    s_back = surf(gate(i).corners.back.x, gate(i).corners.back.y, gate(i).corners.back.z, 'CData', mesh_gate_back, 'FaceColor', 'texturemap');    % Plot the surface
    s_back.AlphaData = (mesh_gate_back(:,:,1)<255 | mesh_gate_back(:,:,2)>0);
    s_back.FaceAlpha = 'texturemap';
    plot3(gate(i).legs.left.x, gate(i).legs.left.y, gate(i).legs.left.z, 'color', [0.5, 0.5, 0.5], 'linewidth', 1);
    plot3(gate(i).legs.right.x, gate(i).legs.right.y, gate(i).legs.right.z, 'color', [0.5, 0.5, 0.5], 'linewidth', 1);
end

%% Show desired trajectory

plot3(pose_d(:,1), pose_d(:,2), pose_d(:,3), 'k--');

drawnow;

% score = 0;
% return;

%% Generate external and internal regions of each gate

[xExternal,yExternal,zExternal] = ...
    meshgrid([-0.8 - sizeDrone 0.8 + sizeDrone], [-sizeDrone 0], [0.8 + sizeDrone -0.8 - sizeDrone]);
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
    gate(i).external(:,1:2) = gate(i).external(:,1:2)*[cos(gate(i).orientation(3)) -sin(gate(i).orientation(3)); sin(gate(i).orientation(3)) cos(gate(i).orientation(3))]'; % rotate the gate
    gate(i).external = gate(i).external + gate(i).translation; % translate the gate
    
    gate(i).triangulizationExternal = delaunayn(gate(i).external); % Generate delaunay triangulization
    
    gate(i).internal = [xInternal yInternal zInternal];
    gate(i).internal(:,1:2) = gate(i).internal(:,1:2)*[cos(gate(i).orientation(3)) -sin(gate(i).orientation(3)); sin(gate(i).orientation(3)) cos(gate(i).orientation(3))]'; % rotate the gate
    gate(i).internal = gate(i).internal + gate(i).translation; % translate the gate
 
    gate(i).triangulizationInternal = delaunayn(gate(i).internal); % generate delaunay triangulization
    
%     fill3(...
%         [gate(i).external(1,1) gate(i).external(3,1) gate(i).external(7,1) gate(i).external(5,1)], ...
%         [gate(i).external(1,2) gate(i).external(3,2) gate(i).external(7,2) gate(i).external(5,2)], ...
%         [gate(i).external(1,3) gate(i).external(3,3) gate(i).external(7,3) gate(i).external(5,3)], [1 0 0]);
%     fill3(...
%         [gate(i).internal(1,1) gate(i).internal(3,1) gate(i).internal(7,1) gate(i).internal(5,1)], ...
%         [gate(i).internal(1,2) gate(i).internal(3,2) gate(i).internal(7,2) gate(i).internal(5,2)], ...
%         [gate(i).internal(1,3) gate(i).internal(3,3) gate(i).internal(7,3) gate(i).internal(5,3)], [0 1 0]);
end


%% Show actual trajectory

score = 0;
nextGate = 1;
lastGate = numGates;
disp(['Aiming gate ', num2str(nextGate), '.']);

c = flipud(autumn(tailLength)); % vector of colors
for k = 100:100:size(pose(:,1), 1)
    
    %% Score performance

    for i = 1:numGates       
        if any(abs(pose(k - 99:k,1)) > sizeEnvironment(2)) || any(abs(pose(k - 99:k,2)) > sizeEnvironment(4)) || any(pose(k - 99:k,3) < sizeEnvironment(5)) || any(pose(k - 99:k,3) > sizeEnvironment(6)) % crashed on the wall, floor or ceiling
            disp('Crashed!!!');
            
            c = flipud(autumn(k)); % vector of colors
            scatter3(pose(1:k,1), pose(1:k,2), pose(1:k,3), 10, c(:,:), 'filled');
            scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
            annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"), 'FontSize', 20);
            annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)), 'FontSize', 20);
            return;
        end
        
        approaching = tsearchn(gate(i).external, gate(i).triangulizationExternal, pose(k - 99:k,1:3));
        if any(~isnan(approaching))
            if flagApproaching && i ~= lastGate && ~all(gate(i).translation == gate(lastGate).translation)
                disp(['Approaching gate ', num2str(i), '.']);
                flagApproaching = false;
            end
            
            crossed = tsearchn(gate(i).internal, gate(i).triangulizationInternal, pose(k - 99:k,1:3));          
            if ~any(~isnan(approaching) & isnan(crossed))
                if i ~= lastGate && ~all(gate(i).translation == gate(lastGate).translation)
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
            else % crashed on the gate
                disp('Crashed!!!');
                
                c = flipud(autumn(k)); % vector of colors
                scatter3(pose(1:k,1), pose(1:k,2), pose(1:k,3), 10, c(:,:), 'filled');
                scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
                annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"), 'FontSize', 20);
                annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)), 'FontSize', 20);
                return;
            end
        end
    end
    heading = atan2(gates(nextGate,2) - pose(k,2), gates(nextGate,1) - pose(k,1));
    heading = heading - 2*pi*fix((heading - pose(k,6) + pi*sign(heading - pose(k,6)))/(2*pi));
    if any(heading > pose(k - 99:k,6) + 70/180*pi | heading < pose(k - 99:k,6) - 70/180*pi) % gate not in the field of view
        if ~flagLost
            disp('Gate lost!');
            flagLost = true;
        end
        score = score - sum(heading > pose(k - 99:k,6) + 70/180*pi | heading < pose(k - 99:k,6) - 70/180*pi)/1000;
    else
        if flagLost
            disp('Gate regained!');
            flagLost = false;
        end        
    end
    
    if enable_animation
        hTrajectory = scatter3(pose(max(1, k - tailLength + 1):k,1), pose(max(1, k - tailLength + 1):k,2), pose(max(1, k - tailLength + 1):k,3), 10, c(end - (k - max(1, k - tailLength + 1)):end,:), 'filled');
        cameraFOV = 10; % 10 -> 90deg, 20 -> 126deg, 30 -> 142deg
        heading1 = eul2rotm(pose(k,4:6), 'XYZ')*[10; cameraFOV; cameraFOV];
        heading2 = eul2rotm(pose(k,4:6), 'XYZ')*[10; cameraFOV; -cameraFOV];
        heading3 = eul2rotm(pose(k,4:6), 'XYZ')*[10; -cameraFOV; -cameraFOV];
        heading4 = eul2rotm(pose(k,4:6), 'XYZ')*[10; -cameraFOV; cameraFOV];
        hCamera11 = fill3([pose(k,1) pose(k,1) + heading1(1) pose(k,1) + heading2(1)], [pose(k,2) pose(k,2) + heading1(2) pose(k,2) + heading2(2)], [pose(k,3) pose(k,3) + heading1(3) pose(k,3) + heading2(3)], [0.9 0.9 1]);
        hCamera12 = fill3([pose(k,1) pose(k,1) + heading2(1) pose(k,1) + heading3(1)], [pose(k,2) pose(k,2) + heading2(2) pose(k,2) + heading3(2)], [pose(k,3) pose(k,3) + heading2(3) pose(k,3) + heading3(3)], [0.9 0.9 1]);
        hCamera13 = fill3([pose(k,1) pose(k,1) + heading3(1) pose(k,1) + heading4(1)], [pose(k,2) pose(k,2) + heading3(2) pose(k,2) + heading4(2)], [pose(k,3) pose(k,3) + heading3(3) pose(k,3) + heading4(3)], [0.9 0.9 1]);
        hCamera14 = fill3([pose(k,1) pose(k,1) + heading4(1) pose(k,1) + heading1(1)], [pose(k,2) pose(k,2) + heading4(2) pose(k,2) + heading1(2)], [pose(k,3) pose(k,3) + heading4(3) pose(k,3) + heading1(3)], [0.9 0.9 1]);
        hCamera1 = plot3([pose(k,1), pose(k,1) + heading1(1)], [pose(k,2), pose(k,2) + heading1(2)], [pose(k,3), pose(k,3) + heading1(3)], 'b:');
        hCamera2 = plot3([pose(k,1), pose(k,1) + heading2(1)], [pose(k,2), pose(k,2) + heading2(2)], [pose(k,3), pose(k,3) + heading2(3)], 'b:');
        hCamera3 = plot3([pose(k,1), pose(k,1) + heading3(1)], [pose(k,2), pose(k,2) + heading3(2)], [pose(k,3), pose(k,3) + heading3(3)], 'b:');
        hCamera4 = plot3([pose(k,1), pose(k,1) + heading4(1)], [pose(k,2), pose(k,2) + heading4(2)], [pose(k,3), pose(k,3) + heading4(3)], 'b:');
        hPoint = scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
        hTimer = annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"), 'FontSize', 20);
        hScore = annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)), 'FontSize', 20);
        
        if save_video
            frame = getframe(gcf);
            writeVideo(v, frame);
        end
        
        drawnow;
        
        delete(hTrajectory);
        delete(hCamera11);
        delete(hCamera12);
        delete(hCamera13);
        delete(hCamera14);
        delete(hCamera1);
        delete(hCamera2);
        delete(hCamera3);
        delete(hCamera4);
        delete(hScore);
        delete(hTimer);
        delete(hPoint);
    end
end

score = score + 5;

c = flipud(autumn(size(pose, 1))); % vector of colors
scatter3(pose(:,1), pose(:,2), pose(:,3), 10, c(:,:), 'filled');
scatter3(pose(k,1), pose(k,2), pose(k,3), 100, c(end,:), 'filled');
annotation('textbox', [0.1, 0.9, 0.2, 0.05], 'String', strcat("Timer: ", num2str(fix(k/1000)), ".", num2str(rem(k, 1000)/100), " s"), 'FontSize', 20);
annotation('textbox', [0.7, 0.9, 0.2, 0.05], 'String', strcat("Score: ", num2str(score)), 'FontSize', 20);

if save_video
    frame = getframe(gcf);
    for i = 1:v.FrameRate
        writeVideo(v, frame);
    end
end

print(['submissions/team', num2str(team), '/round', num2str(round), '.jpg'], '-djpeg', '-r600');
if save_video
    close(v);
end
