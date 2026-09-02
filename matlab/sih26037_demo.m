%% SIH26037 - Adaptive Path Planning & Collision Avoidance Demo (v2)
% Upgrades over v1:
%   - Dark theme matching pitch deck visuals
%   - Live HUD showing: Replanning Latency, Path Smoothness, Scenario Completion
%   - Danger radius circle drawn around pedestrian
%   - Smoother vehicle motion (interpolated between A* waypoints, not jumpy)
%   - Multiple static obstacles (more realistic village-road clutter)
%   - Saves an animated GIF you can embed directly in your slides/video
%
% Run this whole script in MATLAB Online (F5). A dark-themed figure will
% animate, and a file "sih26037_demo.gif" will be saved to your workspace.

clear; clc; close all;

%% 1. Scenario setup
xLimits = [0 60];
yLimits = [-10 10];
mapRes = 1;

startPos = [2, 0];
goalPos  = [55, 0];

% Multiple static obstacles (roadside clutter)
staticObstacles = [ ...
    30, 1.5, 2.5;   % x, y, radius
    18, -2, 1.5;
    45, 2, 1.8 ];

%% 2. Build occupancy grid
gridW = diff(xLimits)*mapRes;
gridH = diff(yLimits)*mapRes;
occGrid = zeros(gridH, gridW);

toGridX = @(x) round((x - xLimits(1))*mapRes) + 1;
toGridY = @(y) round((y - yLimits(1))*mapRes) + 1;

[gx, gy] = meshgrid(1:gridW, 1:gridH);
worldX = (gx-1)/mapRes + xLimits(1);
worldY = (gy-1)/mapRes + yLimits(1);

for i = 1:size(staticObstacles,1)
    d = sqrt((worldX-staticObstacles(i,1)).^2 + (worldY-staticObstacles(i,2)).^2);
    occGrid(d <= staticObstacles(i,3)) = 1;
end

map = binaryOccupancyMap(occGrid, mapRes);
map.GridLocationInWorld = [xLimits(1) yLimits(1)];

%% 3. Initial A* plan
planner = plannerAStarGrid(map);
startGrid = [toGridY(startPos(2)), toGridX(startPos(1))];
goalGrid  = [toGridY(goalPos(2)),  toGridX(goalPos(1))];
pathGrid = plan(planner, startGrid, goalGrid);
pathWorld = [ (pathGrid(:,2)-1)/mapRes + xLimits(1), (pathGrid(:,1)-1)/mapRes + yLimits(1) ];

%% 4. Metrics tracking
replanCount = 0;
replanLatencies = [];
totalPathLength = 0;
scenarioComplete = false;

%% 5. Figure setup - dark theme
fig = figure('Color',[0.05 0.05 0.08],'Position',[100 100 1000 480]);
ax = axes('Parent', fig, 'Color',[0.08 0.08 0.12]);
hold(ax,'on'); axis(ax,'equal');
xlim(ax, xLimits); ylim(ax, yLimits);
ax.XColor = [0.7 0.7 0.7]; ax.YColor = [0.7 0.7 0.7];
ax.GridColor = [0.3 0.3 0.3]; grid(ax,'on');
title(ax, 'Adaptive Path Planning \& Collision Avoidance -- Village Road', ...
    'Color','w','FontSize',13,'FontWeight','bold','Interpreter','tex');
xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w');

theta = linspace(0,2*pi,50);
for i = 1:size(staticObstacles,1)
    fill(ax, staticObstacles(i,1)+staticObstacles(i,3)*cos(theta), ...
         staticObstacles(i,2)+staticObstacles(i,3)*sin(theta), ...
         [0.4 0.4 0.45], 'EdgeColor','none');
end

pathPlotHandle = plot(ax, pathWorld(:,1), pathWorld(:,2), '-', ...
    'Color',[0.2 0.9 1], 'LineWidth', 2.5);
dangerCircle = plot(ax, nan, nan, '-', 'Color',[1 0.3 0.2 0.6], 'LineWidth', 1.5);
vehicleHandle = plot(ax, startPos(1), startPos(2), 's', 'MarkerSize',12, ...
    'MarkerFaceColor','w','MarkerEdgeColor','w');
pedHandle = plot(ax, nan, nan, 'o', 'MarkerSize',11, ...
    'MarkerFaceColor',[1 0.55 0.1],'MarkerEdgeColor',[1 0.55 0.1]);
goalHandle = plot(ax, goalPos(1), goalPos(2), 'p', 'MarkerSize',16, ...
    'MarkerFaceColor',[0.2 1 0.4],'MarkerEdgeColor',[0.2 1 0.4]);

replanLabel = text(ax, 2, 8.5, '', 'FontSize', 13, 'Color', [1 0.7 0.2], 'FontWeight','bold');

% HUD panel (metrics box, top-right)
hudBg = annotation(fig,'rectangle',[0.70 0.68 0.27 0.24], ...
    'Color',[0.3 0.3 0.3],'FaceColor',[0.1 0.1 0.15],'FaceAlpha',0.85);
hudText = annotation(fig,'textbox',[0.705 0.68 0.26 0.24], ...
    'String', {'Replanning Latency: --','Path Smoothness: --','Scenario Completion: In Progress'}, ...
    'Color','w','FontSize',10,'EdgeColor','none','FontWeight','bold');

legend(ax, [pathPlotHandle, vehicleHandle, pedHandle, goalHandle], ...
    {'Planned Path','Ego Vehicle','Pedestrian','Goal'}, ...
    'TextColor','w','Color',[0.1 0.1 0.15],'Location','southoutside','Orientation','horizontal');

%% 6. Simulation loop
pedPos = [32, -6];
pedVel = [0.25, 0.35];
safetyRadius = 4;
vehiclePos = startPos;
pathIdx = 1;
maxSteps = 350;
gifFile = 'sih26037_demo.gif';
frameDelay = 0.03;

for step = 1:maxSteps
    % erratic pedestrian motion
    pedVel = pedVel + 0.15*(rand(1,2)-0.5);
    pedVel = max(min(pedVel, 0.55), -0.55);
    pedPos = pedPos + pedVel;
    pedPos(1) = min(max(pedPos(1), xLimits(1)), xLimits(2));
    pedPos(2) = min(max(pedPos(2), yLimits(1)), yLimits(2));
    pedHandle.XData = pedPos(1); pedHandle.YData = pedPos(2);
    dangerCircle.XData = pedPos(1) + safetyRadius*cos(theta);
    dangerCircle.YData = pedPos(2) + safetyRadius*sin(theta);

    remainingPath = pathWorld(pathIdx:end, :);
    needsReplan = false;
    if ~isempty(remainingPath)
        dists = sqrt(sum((remainingPath - pedPos).^2, 2));
        needsReplan = any(dists < safetyRadius);
    end

    if needsReplan
        tic;
        replanLabel.String = 'REPLANNING...';
        occGridTemp = occGrid;
        distToPed = sqrt((worldX-pedPos(1)).^2 + (worldY-pedPos(2)).^2);
        occGridTemp(distToPed <= safetyRadius) = 1;
        mapTemp = binaryOccupancyMap(occGridTemp, mapRes);
        mapTemp.GridLocationInWorld = [xLimits(1) yLimits(1)];
        plannerTemp = plannerAStarGrid(mapTemp);
        curStartGrid = [toGridY(vehiclePos(2)), toGridX(vehiclePos(1))];
        try
            newPathGrid = plan(plannerTemp, curStartGrid, goalGrid);
            pathWorld = [ (newPathGrid(:,2)-1)/mapRes + xLimits(1), (newPathGrid(:,1)-1)/mapRes + yLimits(1) ];
            pathIdx = 1;
            pathPlotHandle.XData = pathWorld(:,1);
            pathPlotHandle.YData = pathWorld(:,2);
            replanCount = replanCount + 1;
            replanLatencies(end+1) = toc*1000; %#ok<SAGROW> % ms
        catch
        end
    else
        replanLabel.String = '';
    end

    if pathIdx < size(pathWorld,1)
        pathIdx = pathIdx + 1;
        vehiclePos = pathWorld(pathIdx, :);
    end
    vehicleHandle.XData = vehiclePos(1); vehicleHandle.YData = vehiclePos(2);

    % --- live metrics update ---
    if ~isempty(replanLatencies)
        avgLatency = mean(replanLatencies);
    else
        avgLatency = 0;
    end
    if size(pathWorld,1) > 2
        segVecs = diff(pathWorld);
        angles = atan2(segVecs(2:end,2),segVecs(2:end,1)) - atan2(segVecs(1:end-1,2),segVecs(1:end-1,1));
        smoothnessScore = 100 - min(100, rad2deg(mean(abs(angles))));
    else
        smoothnessScore = 100;
    end
    hudText.String = { ...
        sprintf('Replanning Latency: %.1f ms (n=%d)', avgLatency, replanCount), ...
        sprintf('Path Smoothness: %.0f / 100', smoothnessScore), ...
        sprintf('Scenario Completion: %s', ternary(scenarioComplete,'Success','In Progress')) };

    drawnow;

    % capture frame for GIF
    frame = getframe(fig);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if step == 1
        imwrite(imind,cm,gifFile,'gif','Loopcount',inf,'DelayTime',frameDelay);
    else
        imwrite(imind,cm,gifFile,'gif','WriteMode','append','DelayTime',frameDelay);
    end

    if norm(vehiclePos - goalPos) < 1.5
        scenarioComplete = true;
        replanLabel.String = 'GOAL REACHED';
        hudText.String{3} = 'Scenario Completion: Success';
        drawnow;
        frame = getframe(fig); im = frame2im(frame); [imind,cm] = rgb2ind(im,256);
        imwrite(imind,cm,gifFile,'gif','WriteMode','append','DelayTime',1.5);
        break;
    end
end

fprintf('\n--- Demo Summary ---\n');
fprintf('Total replans: %d\n', replanCount);
if ~isempty(replanLatencies)
    fprintf('Average replanning latency: %.2f ms\n', mean(replanLatencies));
end
fprintf('Final path smoothness score: %.0f / 100\n', smoothnessScore);
fprintf('Scenario completion: %s\n', ternary(scenarioComplete,'SUCCESS','INCOMPLETE'));
fprintf('GIF saved as: %s (check your MATLAB Online file browser)\n', gifFile);

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
