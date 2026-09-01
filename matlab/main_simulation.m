% MAIN_SIMULATION Closed-Loop Autonomous Pathfinder Simulation (PS 26037)
%   Featuring Smooth Curvature Splines & Dual Path Rendering (Red = Past Traveled, Green = Future Replanned)

clear; clc; close all;

fprintf('========================================================\n');
fprintf(' ADAPTIVE PATH PLANNING FOR UNSTRUCTURED INDIAN ROADS   \n');
fprintf(' MathWorks Smart Vehicles - Problem Statement 26037     \n');
fprintf('========================================================\n\n');

scenario_id = 1; 
scenario = generate_scenarios(scenario_id);

dt = 0.05;           
T_max = 20.0;        
time_vec = 0:dt:T_max;
num_steps = length(time_vec);

ego_state = [scenario.start_pose(1); scenario.start_pose(2); scenario.start_pose(3); 3.0];
target_v = 4.5;

state_hist    = zeros(num_steps, 4);
control_hist  = zeros(num_steps, 2);
latency_hist  = zeros(num_steps, 1);
obstacles_hist = cell(num_steps, 1);

fig = figure('Name', ['PS 26037 Simulation - ' scenario.title], 'Color', [0.1 0.12 0.15]);
ax = axes('Parent', fig, 'Color', [0.15 0.17 0.2]);
hold(ax, 'on'); grid(ax, 'on');

current_obs = scenario.obstacles;

for step = 1:num_steps
    t = time_vec(step);
    
    % Update dynamic obstacles
    for i = 1:length(current_obs)
        current_obs(i).position = current_obs(i).position + current_obs(i).velocity * dt;
    end
    obstacles_hist{step} = current_obs;
    
    % Prediction & Smooth Spline Replanning
    pred_trajectories = dynamic_obstacle_predictor(current_obs, dt, 15);
    [current_path, costmap, latency_ms] = adaptive_path_planner(...
        ego_state(1:3)', scenario.goal_pose, scenario.static_map, pred_trajectories, scenario.grid_res);
    latency_hist(step) = latency_ms;
    
    % Control & Kinematics
    [control, e_y, e_theta] = pure_pursuit_controller(ego_state, current_path, target_v);
    stateDot = vehicle_kinematics(ego_state, control);
    ego_state = ego_state + stateDot * dt;
    
    state_hist(step, :) = ego_state';
    control_hist(step, :) = control';
    
    % Render Frame
    cla(ax);
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('Scenario %d: %s | t = %.2fs', scenario.id, scenario.title, t), 'Color', 'w', 'FontSize', 12);
    xlabel(ax, 'X Position (m)', 'Color', 'w'); ylabel(ax, 'Y Position (m)', 'Color', 'w');
    set(ax, 'XColor', 'w', 'YColor', 'w', 'XLim', [0 60], 'YLim', [-10 10]);
    
    % Road edge bounds
    plot(ax, [0 60], [3.5 3.5], 'w--', 'LineWidth', 1.5);
    plot(ax, [0 60], [-3.5 -3.5], 'w--', 'LineWidth', 1.5);
    
    % Goal
    plot(ax, scenario.goal_pose(1), scenario.goal_pose(2), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
    
    % 1. PAST PATH TRAVELED (RED TRAIL)
    plot(ax, state_hist(1:step, 1), state_hist(1:step, 2), 'r-', 'LineWidth', 3.0, 'DisplayName', 'Past Path Traveled');
    
    % 2. FUTURE REPLANNED PATH (GREEN SPLINE)
    plot(ax, current_path(:,1), current_path(:,2), 'g-', 'LineWidth', 3.0, 'DisplayName', 'Live Replanned Path');
    
    % Obstacles
    for k = 1:length(current_obs)
        pos = current_obs(k).position;
        obs_type = current_obs(k).type;
        if strcmpi(obs_type, 'cattle')
            plot(ax, pos(1), pos(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [1 0.6 0], 'MarkerEdgeColor', 'w');
            text(ax, pos(1)+0.5, pos(2), 'CATTLE', 'Color', [1 0.6 0], 'FontSize', 9, 'FontWeight', 'bold');
        else
            plot(ax, pos(1), pos(2), 's', 'MarkerSize', 11, 'MarkerFaceColor', [0.8 0.2 1], 'MarkerEdgeColor', 'w');
            text(ax, pos(1)+0.5, pos(2), upper(obs_type), 'Color', [0.8 0.2 1], 'FontSize', 9, 'FontWeight', 'bold');
        end
    end
    
    % Ego Vehicle Body
    plot(ax, ego_state(1), ego_state(2), 'd', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 1], 'MarkerEdgeColor', 'w');
    quiver(ax, ego_state(1), ego_state(2), cos(ego_state(3))*2, sin(ego_state(3))*2, 'Color', 'y', 'LineWidth', 2, 'MaxHeadSize', 0.8);
    
    legend(ax, 'TextColor', 'w', 'Location', 'northwest');
    drawnow limitrate;
    
    if norm(ego_state(1:2) - scenario.goal_pose(1:2)') < 2.0
        fprintf('Goal reached successfully at t = %.2f seconds!\n', t);
        state_hist = state_hist(1:step, :);
        control_hist = control_hist(1:step, :);
        latency_hist = latency_hist(1:step);
        obstacles_hist = obstacles_hist(1:step);
        time_vec = time_vec(1:step);
        break;
    end
end

metrics = evaluate_metrics(time_vec, state_hist, control_hist, latency_hist, obstacles_hist);
