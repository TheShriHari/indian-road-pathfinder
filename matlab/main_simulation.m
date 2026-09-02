% MAIN_SIMULATION Closed-Loop Autonomous Pathfinder Simulation (PS 26037)
%   Integrates Kinematic Bicycle Model, Steering Rate Limits, Pure Pursuit Controller,
%   Catmull-Rom Spline Path Smoothing, and Dual Trail Rendering.

clear; clc; close all;

fprintf('========================================================\n');
fprintf(' ADAPTIVE PATH PLANNING FOR UNSTRUCTURED INDIAN ROADS   \n');
fprintf(' MathWorks Smart Vehicles - Problem Statement 26037     \n');
fprintf('========================================================\n\n');

scenario_id = 1; % 1: Village Road | 2: Intersection | 3: Merge | 4: Market | 5: Cattle Crossing
scenario = generate_scenarios(scenario_id);

dt = 0.05;           
T_max = 20.0;        
time_vec = 0:dt:T_max;
num_steps = length(time_vec);

% Controller & Vehicle Parameters
params.L = 2.7;                  % Wheelbase (meters)
params.k_lookahead = 0.6;        % Lookahead gain (seconds)
params.min_lookahead = 2.5;      % Minimum lookahead distance (meters)
params.Kp_v = 1.0;               % Speed P gain
params.max_steer = deg2rad(35);   % Max steering angle (35 degrees)
params.max_steer_rate = deg2rad(60); % Max steering rate (60 deg/s)
params.max_accel = 2.5;          % Max acceleration (m/s^2)
params.min_accel = -4.5;         % Max braking deceleration (m/s^2)

% Initial Ego Vehicle State [x; y; theta; v]
ego_state = [scenario.start_pose(1); scenario.start_pose(2); scenario.start_pose(3); 3.0];
target_v = 4.5; % Target cruise velocity (m/s)
last_steer = 0.0; % Tracking steering rate

state_hist     = zeros(num_steps, 4);
control_hist   = zeros(num_steps, 2);
latency_hist   = zeros(num_steps, 1);
obstacles_hist = cell(num_steps, 1);

fig = figure('Name', ['PS 26037 Simulation - ' scenario.title], 'Color', [0.1 0.12 0.15]);
ax = axes('Parent', fig, 'Color', [0.15 0.17 0.2]);
hold(ax, 'on'); grid(ax, 'on');

current_obs = scenario.obstacles;

for step = 1:num_steps
    t = time_vec(step);
    
    % 1. Update Dynamic Obstacle Positions
    for i = 1:length(current_obs)
        current_obs(i).position = current_obs(i).position + current_obs(i).velocity * dt;
    end
    obstacles_hist{step} = current_obs;
    
    % 2. Dynamic Prediction & Catmull-Rom Spline Replanning Pipeline
    pred_trajectories = dynamic_obstacle_predictor(current_obs, dt, 15);
    [current_path, costmap, latency_ms] = adaptive_path_planner(...
        ego_state(1:3)', scenario.goal_pose, scenario.static_map, pred_trajectories, scenario.grid_res);
    latency_hist(step) = latency_ms;
    
    % 3. Pure Pursuit Controller Steering & Acceleration Command Calculation
    [raw_control, e_y, e_theta] = pure_pursuit_controller(ego_state, current_path, target_v, params);
    
    % Enforce Max Steering Angle & Steering Rate Constraints
    raw_steer = raw_control(1);
    max_steer_delta = params.max_steer_rate * dt;
    steer_change = min(max(raw_steer - last_steer, -max_steer_delta), max_steer_delta);
    clamped_steer = min(max(last_steer + steer_change, -params.max_steer), params.max_steer);
    last_steer = clamped_steer;
    
    clamped_accel = min(max(raw_control(2), params.min_accel), params.max_accel);
    control = [clamped_steer; clamped_accel];
    
    % 4. Kinematic Bicycle Model Integration (State Equation Integration)
    stateDot = vehicle_kinematics(ego_state, control, params);
    ego_state = ego_state + stateDot * dt;
    ego_state(3) = atan2(sin(ego_state(3)), cos(ego_state(3))); % Normalize yaw theta [-pi, pi]
    ego_state(4) = max(0, ego_state(4)); % Velocity >= 0
    
    state_hist(step, :) = ego_state';
    control_hist(step, :) = control';
    
    % 5. Real-Time Canvas Rendering
    cla(ax);
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('Scenario %d: %s | t = %.2fs', scenario.id, scenario.title, t), 'Color', 'w', 'FontSize', 12);
    xlabel(ax, 'X Position (m)', 'Color', 'w'); ylabel(ax, 'Y Position (m)', 'Color', 'w');
    set(ax, 'XColor', 'w', 'YColor', 'w', 'XLim', [0 60], 'YLim', [-10 10]);
    
    % Road edge boundaries
    plot(ax, [0 60], [3.5 3.5], 'w--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, [0 60], [-3.5 -3.5], 'w--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Goal Marker
    plot(ax, scenario.goal_pose(1), scenario.goal_pose(2), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Goal');
    
    % Trail Line 1: PAST PATH TRAVELED (SOLID RED LINE)
    plot(ax, state_hist(1:step, 1), state_hist(1:step, 2), 'r-', 'LineWidth', 3.0, 'DisplayName', 'Past Path Traveled');
    
    % Trail Line 2: LIVE REPLANNED PATH HORIZON (LIGHTER DASHED GREEN LINE)
    plot(ax, current_path(:, 1), current_path(:, 2), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Live Replanned Path');
    
    % Obstacle Markers & Non-Overlapping Labels
    for k = 1:length(current_obs)
        pos = current_obs(k).position;
        obs_type = current_obs(k).type;
        if strcmpi(obs_type, 'cattle')
            plot(ax, pos(1), pos(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [1 0.6 0], 'MarkerEdgeColor', 'w', 'HandleVisibility', 'off');
            text(ax, pos(1), pos(2) + 1.2, 'CATTLE', 'Color', [1 0.6 0], 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        else
            plot(ax, pos(1), pos(2), 's', 'MarkerSize', 11, 'MarkerFaceColor', [0.8 0.2 1], 'MarkerEdgeColor', 'w', 'HandleVisibility', 'off');
            text(ax, pos(1), pos(2) + 1.2, upper(obs_type), 'Color', [0.8 0.2 1], 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        end
    end
    
    % Ego Vehicle Body - Oriented Rotated Rectangle matching Heading theta
    L_v = 3.8; W_v = 1.8;
    corners = [-L_v/2 -W_v/2; L_v/2 -W_v/2; L_v/2 W_v/2; -L_v/2 W_v/2]';
    R_rot = [cos(ego_state(3)) -sin(ego_state(3)); sin(ego_state(3)) cos(ego_state(3))];
    rot_corners = R_rot * corners + [ego_state(1); ego_state(2)];
    fill(ax, rot_corners(1, :), rot_corners(2, :), [0 0.8 1], 'EdgeColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'Ego Vehicle');
    
    % Heading Vector Arrow
    quiver(ax, ego_state(1), ego_state(2), cos(ego_state(3))*2.5, sin(ego_state(3))*2.5, 0, 'Color', 'y', 'LineWidth', 2.0, 'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
    
    legend(ax, 'TextColor', 'w', 'Location', 'northwest');
    drawnow limitrate;
    
    % Termination Check
    if norm(ego_state(1:2) - scenario.goal_pose(1:2)') < 2.5 || ego_state(1) >= 50.0
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
