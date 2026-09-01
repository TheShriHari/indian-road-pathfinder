% MAIN_SIMULATION Closed-Loop Autonomous Pathfinder Simulation (PS 26037)
%   Runs adaptive perception, trajectory prediction, path planning, and
%   kinematic vehicle control across 5 unstructured Indian road scenarios.

clear; clc; close all;

fprintf('========================================================\n');
fprintf(' ADAPTIVE PATH PLANNING FOR UNSTRUCTURED INDIAN ROADS   \n');
fprintf(' MathWorks Smart Vehicles - Problem Statement 26037     \n');
fprintf('========================================================\n\n');

% Select scenario (1 to 5)
% 1: Village Road | 2: Intersection | 3: Highway Merge | 4: Market | 5: Cattle Crossing
scenario_id = 1; 

scenario = generate_scenarios(scenario_id);
fprintf('Loaded Scenario [%d]: %s\n', scenario.id, scenario.title);
fprintf('Description: %s\n\n', scenario.description);

% Simulation Parameters
dt = 0.05;           % 20 Hz simulation loop
T_max = 20.0;        % Max simulation time (s)
time_vec = 0:dt:T_max;
num_steps = length(time_vec);

% Initial Ego Vehicle State [x, y, theta, v]
ego_state = [scenario.start_pose(1); scenario.start_pose(2); scenario.start_pose(3); 3.0]; % 3 m/s (~11 km/h)
target_v = 4.5; % Target cruise speed (m/s)

% History Loggers
state_hist    = zeros(num_steps, 4);
control_hist  = zeros(num_steps, 2);
latency_hist  = zeros(num_steps, 1);
obstacles_hist = cell(num_steps, 1);

% Prepare Visualization Figure
fig = figure('Name', ['PS 26037 Simulation - ' scenario.title], 'Color', [0.1 0.12 0.15]);
ax = axes('Parent', fig, 'Color', [0.15 0.17 0.2]);
hold(ax, 'on'); grid(ax, 'on');
title(ax, ['Scenario ' num2str(scenario.id) ': ' scenario.title], 'Color', 'w', 'FontSize', 14);
xlabel(ax, 'X Position (m)', 'Color', 'w'); ylabel(ax, 'Y Position (m)', 'Color', 'w');
set(ax, 'XColor', 'w', 'YColor', 'w', 'XLim', [0 60], 'YLim', [-10 10]);

% Main Simulation Loop
current_obs = scenario.obstacles;

for step = 1:num_steps
    t = time_vec(step);
    
    % 1. Perception & Motion Update of Dynamic Obstacles
    for i = 1:length(current_obs)
        current_obs(i).position = current_obs(i).position + current_obs(i).velocity * dt;
    end
    obstacles_hist{step} = current_obs;
    
    % 2. Dynamic Trajectory Prediction
    pred_trajectories = dynamic_obstacle_predictor(current_obs, dt, 15);
    
    % 3. Adaptive Path Replanning
    [current_path, costmap, latency_ms] = adaptive_path_planner(...
        ego_state(1:3)', scenario.goal_pose, scenario.static_map, pred_trajectories, scenario.grid_res);
    latency_hist(step) = latency_ms;
    
    % 4. Steering & Speed Control
    [control, e_y, e_theta] = pure_pursuit_controller(ego_state, current_path, target_v);
    
    % 5. Vehicle Kinematics Integration
    stateDot = vehicle_kinematics(ego_state, control);
    ego_state = ego_state + stateDot * dt;
    
    % Log States
    state_hist(step, :) = ego_state';
    control_hist(step, :) = control';
    
    % Check Goal Reached
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

% Evaluate Metrics
metrics = evaluate_metrics(time_vec, state_hist, control_hist, latency_hist, obstacles_hist);

% Plot Trajectory
plot(ax, state_hist(:,1), state_hist(:,2), 'c-', 'LineWidth', 2.5, 'DisplayName', 'Ego Path');
plot(ax, scenario.goal_pose(1), scenario.goal_pose(2), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Goal');
legend(ax, 'TextColor', 'w', 'Location', 'northwest');
