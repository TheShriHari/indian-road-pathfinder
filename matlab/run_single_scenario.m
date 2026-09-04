function result = run_single_scenario(obstacle_config, seed, sim_opts)
%% RUN_SINGLE_SCENARIO  Executes one closed-loop autonomous driving simulation in MOCK mode.
%
% Syntax:
%   result = run_single_scenario(obstacle_config, seed, sim_opts)
%
% Inputs:
%   obstacle_config - struct with fields:
%                       .potholes: array of struct('x', x, 'y', y, 'radius', r)
%                       .dynamic_agents: array of struct('id', id, 'type', str, ...
%                           'position', [x,y], 'velocity', [vx,vy], 'behavior_profile', str)
%                       .road_boundaries: (optional) Nx2 matrix of road boundaries
%                       .start_pose: (optional) [x, y, yaw] (default: [2.0, 0.0, 0.0])
%                       .goal_pose: (optional) [x, y, yaw] (default: [70.0, 0.0, 0.0])
%                     If empty or not provided, uses default mock scenario from carla_simulation_bridge.
%   seed            - (optional) integer seed for reproducibility (rng(seed))
%   sim_opts        - (optional) struct with options:
%                       .max_steps: maximum simulation steps (default: 400 = 40s)
%                       .dt: timestep in seconds (default: 0.1)
%                       .collision_thresh: distance threshold for agent collision (default: 1.0 m)
%                       .goal_dist_tol: distance to goal tolerance (default: 2.5 m)
%                       .verbose: boolean, print step telemetry to console (default: false)
%
% Outputs:
%   result          - struct with fields:
%                       .outcome: 'SUCCESS' | 'TIMEOUT' | 'COLLISION' | 'STALLED' | 'ERROR'
%                       .time_to_goal: seconds to reach goal (if SUCCESS, else NaN)
%                       .replan_count: total Hybrid A* replans triggered
%                       .min_clearance_achieved: minimum obstacle clearance in meters
%                       .max_lateral_position: maximum absolute y position of ego vehicle
%                       .seed: seed used
%                       .steps_taken: total steps executed
%                       .final_pos: [x, y] final ego vehicle position
%                       .collision_details: details of colliding obstacle if COLLISION
%                       .num_potholes: number of potholes in scenario
%                       .num_agents: number of dynamic agents in scenario
%                       .error_msg: error message string if ERROR
%                       .error_stack: call stack string if ERROR
%                       .obstacle_config: the exact obstacle configuration tested

if nargin < 1, obstacle_config = []; end
if nargin < 2, seed = []; end
if nargin < 3, sim_opts = struct(); end

% Apply seed if supplied
if ~isempty(seed)
    rng(seed);
end

% Set simulation options & defaults
dt = 0.1;
if isfield(sim_opts, 'dt'), dt = sim_opts.dt; end

max_steps = 500;
if isfield(sim_opts, 'max_steps'), max_steps = sim_opts.max_steps; end

collision_thresh = 1.0;
if isfield(sim_opts, 'collision_thresh'), collision_thresh = sim_opts.collision_thresh; end

goal_dist_tol = 2.5;
if isfield(sim_opts, 'goal_dist_tol'), goal_dist_tol = sim_opts.goal_dist_tol; end

verbose = false;
if isfield(sim_opts, 'verbose'), verbose = sim_opts.verbose; end

% Default scenario configuration if none provided
if isempty(obstacle_config)
    obstacle_config.potholes = [ ...
        struct('x', 20.0, 'y',  1.0, 'radius', 0.8), ...
        struct('x', 35.0, 'y', -0.8, 'radius', 1.0) ...
    ];
    obstacle_config.dynamic_agents = [ ...
        struct('id', 1, 'type', 'cattle',       'position', [30.0, -3.5], 'velocity', [ 0.0,  0.7], 'behavior_profile', 'erratic'), ...
        struct('id', 2, 'type', 'auto_rickshaw', 'position', [45.0,  1.2], 'velocity', [-3.2,  0.0], 'behavior_profile', 'weaving') ...
    ];
end

if ~isfield(obstacle_config, 'potholes')
    obstacle_config.potholes = struct([]);
end
if ~isfield(obstacle_config, 'dynamic_agents')
    obstacle_config.dynamic_agents = struct([]);
end
if ~isfield(obstacle_config, 'start_pose') || isempty(obstacle_config.start_pose)
    obstacle_config.start_pose = [2.0, 0.0, 0.0];
end
if ~isfield(obstacle_config, 'goal_pose') || isempty(obstacle_config.goal_pose)
    obstacle_config.goal_pose = [70.0, 0.0, 0.0];
end
if ~isfield(obstacle_config, 'road_boundaries') || isempty(obstacle_config.road_boundaries)
    xs = (-5:120)';
    obstacle_config.road_boundaries = [xs, repmat(-2.5, length(xs), 1); xs, repmat( 2.5, length(xs), 1)];
end

% Execute simulation with error handling
try
    if verbose
        result = run_simulation_core(obstacle_config, seed, dt, max_steps, collision_thresh, goal_dist_tol, verbose);
    else
        % Suppress internal diagnostic prints using evalc
        evalc('result = run_simulation_core(obstacle_config, seed, dt, max_steps, collision_thresh, goal_dist_tol, verbose);');
    end
catch ME
    result.outcome = 'ERROR';
    result.time_to_goal = NaN;
    result.replan_count = 0;
    result.min_clearance_achieved = NaN;
    result.max_lateral_position = NaN;
    result.seed = seed;
    result.steps_taken = 0;
    result.final_pos = [NaN, NaN];
    result.collision_details = struct([]);
    result.num_potholes = length(obstacle_config.potholes);
    result.num_agents = length(obstacle_config.dynamic_agents);
    result.error_msg = ME.message;
    result.error_stack = getReport(ME, 'extended', 'hyperlinks', 'off');
    result.obstacle_config = obstacle_config;
end

end

%% ── Core Simulation Function ─────────────────────────────────────────────
function result = run_simulation_core(obstacle_config, seed, dt, max_steps, collision_thresh, goal_dist_tol, verbose)

% Reset persistent EKF state and sensor simulation state
clear dynamic_obstacle_predictor;
clear simulate_sensor_detection;

% Vehicle parameters (matching CARLA specification)
L_WB          = 2.7;
MAX_STEER_RAD = pi/6;  % 30 deg
LOCAL_GOAL_HORIZON = 28.0;
N_HORIZON     = 35;     % 3.5s prediction horizon (increased from 20 steps)
REPLAN_EVERY  = 8;

% Rolling costmap config
map_cfg.grid_res  = 0.2;
map_cfg.range_fwd = 50.0;
map_cfg.range_bwd = 10.0;
map_cfg.range_lat = 15.0;

% Sensor simulation config (realistic perception limitations layer)
sensor_cfg.max_detection_range = 35.0;   % m
sensor_cfg.field_of_view_deg   = 140.0;  % ±70° from ego heading
sensor_cfg.base_dropout_prob   = 0.05;   % 5% base dropout rate
sensor_cfg.std_pos_base        = 0.3;    % position noise std at close range (m)
sensor_cfg.std_pos_max         = 0.5;    % position noise std at max range (m)
sensor_cfg.std_vel             = 0.2;    % velocity noise std (m/s)
sensor_cfg.misclass_prob       = 0.03;   % 3% misclassification chance
sensor_cfg.latency_ticks       = 2;      % 200ms processing delay
sensor_cfg.verbose             = verbose; % match scenario verbosity

% Initial vehicle state
start_pose = obstacle_config.start_pose;
goal_pose  = obstacle_config.goal_pose;
ego_state  = [start_pose(1); start_pose(2); start_pose(3); 0.0]; % [x, y, theta, v]
mock_x     = ego_state(1);
mock_y     = ego_state(2);
mock_theta = ego_state(3);
mock_v     = 0.0;

% Persistent algorithm state
bsm_state   = 'CRUISE';
path        = zeros(0, 2);
replan_cnt  = 0;
force_replan = false;
prev_target  = [NaN, NaN];

min_clearance           = Inf;
max_lateral             = abs(ego_state(2));
stall_consecutive_steps = 0;
outcome                 = '';
time_to_goal            = NaN;
collision_details       = struct([]);
last_replan_step        = -100;
safe_stop_active        = false;
stopped_wait_steps      = 0;
MAX_STOP_WAIT_STEPS     = round(6.0 / dt); % 60 steps = 6.0 seconds of sustained blockage

% EKF innovation logging — accumulated over the run for post-hoc statistics
innov_log        = [];   % Nx1 vector of |innov| norms (measurement updates only)
total_dropouts   = 0;    % sensor dropout event counter
total_misclasses = 0;    % sensor misclassification event counter

step = 0;

potholes       = obstacle_config.potholes;
dynamic_agents = obstacle_config.dynamic_agents;

% Static perception struct
sensor_det.road_boundaries = obstacle_config.road_boundaries;
sensor_det.potholes        = potholes;
sensor_det.static_boxes    = struct([]);
sensor_det.static_points   = [];

while step < max_steps
    step = step + 1;
    t    = step * dt;

    % ── A. Update dynamic agents linearly in MOCK mode ───────────────────
    raw_obstacles = struct([]);
    for k = 1:length(dynamic_agents)
        pos = dynamic_agents(k).position + dynamic_agents(k).velocity * t;
        raw_obstacles(k).id               = dynamic_agents(k).id;
        raw_obstacles(k).type             = dynamic_agents(k).type;
        raw_obstacles(k).position         = pos;
        raw_obstacles(k).velocity         = dynamic_agents(k).velocity;
        raw_obstacles(k).behavior_profile = dynamic_agents(k).behavior_profile;
    end

    % ── B. Rolling costmap builder ───────────────────────────────────────
    [rolling_costmap, grid_meta] = local_occupancy_grid_builder( ...
        ego_state(1:3)', sensor_det, map_cfg);

    % ── C. Sensor simulation layer — noisy, range-limited, dropout-prone ───
    % Replaces oracle feed: EKF now receives realistic perception output.
    [detected_obs, s_log] = simulate_sensor_detection(raw_obstacles, ego_state, sensor_cfg);
    total_dropouts   = total_dropouts   + s_log.n_dropped;
    total_misclasses = total_misclasses + s_log.n_misclassed;

    % ── D. EKF dynamic obstacle predictor ────────────────────────────────
    [predictions, innov_tick] = dynamic_obstacle_predictor(detected_obs, dt, N_HORIZON);

    % Accumulate innovation norms (measurement-update ticks only, not predict-only)
    for ii = 1:length(innov_tick)
        if ~innov_tick(ii).predict_only
            innov_log(end+1) = innov_tick(ii).innov_norm; %#ok<AGROW>
        end
    end

    % ── E. Hybrid A* replanning ──────────────────────────────────────────
    can_replan = force_replan || (size(path, 1) < 2) || ((step - last_replan_step) >= 3);
    needs_replan = force_replan || (size(path, 1) < 2) || (mod(step, REPLAN_EVERY) == 0);
    if safe_stop_active && (ego_state(4) < 0.2) && (mod(step, 4) == 0)
        needs_replan = true; % Periodically check if road has cleared while waiting
    end
    force_replan = false;

    % Fix 2: Proximity threat check for static potholes
    if can_replan && ~needs_replan && ~isempty(potholes) && size(path, 1) >= 2
        for j = 1:length(potholes)
            d_pot_path = hypot(path(:, 1) - potholes(j).x, path(:, 2) - potholes(j).y) - potholes(j).radius;
            if min(d_pot_path) < 1.2
                needs_replan = true;
                break;
            end
        end
    end

    % Fix 3 & 4: Full horizon scan for dynamic agents with adaptive trigger distance
    if can_replan && ~needs_replan && ~isempty(predictions) && size(path, 1) >= 2
        vx_ego = ego_state(4) * cos(ego_state(3));
        vy_ego = ego_state(4) * sin(ego_state(3));
        for k = 1:length(predictions)
            wp = predictions(k).waypoints;
            if isempty(wp), continue; end
            
            if k <= length(raw_obstacles)
                vx_ag = raw_obstacles(k).velocity(1);
                vy_ag = raw_obstacles(k).velocity(2);
            else
                vx_ag = 0.0; vy_ag = 0.0;
            end
            rel_closing_speed = hypot(vx_ego - vx_ag, vy_ego - vy_ag);
            trigger_dist = max(3.0, min(8.0, rel_closing_speed * 0.8));
            
            for h = 1:size(wp, 1)
                d = min(hypot(path(:,1) - wp(h,1), path(:,2) - wp(h,2)));
                if d < trigger_dist
                    needs_replan = true;
                    break;
                end
            end
            if needs_replan, break; end
        end
    end

    if needs_replan
        last_replan_step = step;
        cur_pose   = [ego_state(1), ego_state(2), ego_state(3)];
        local_goal = compute_local_goal(ego_state(1:2)', path, LOCAL_GOAL_HORIZON, goal_pose);
        grid_org   = [grid_meta.x_min, grid_meta.y_min];
        [path_new, ~, ~, plan_ok] = adaptive_path_planner( ...
            cur_pose, local_goal, rolling_costmap, predictions, ...
            map_cfg.grid_res, grid_org);
        if plan_ok && size(path_new, 1) > 2
            path = path_new;
            replan_cnt = replan_cnt + 1;
            safe_stop_active = false;
            stopped_wait_steps = 0;
        elseif ~plan_ok
            % Fix 1: Validate whether the previous path is genuinely collision-free ahead
            path_is_valid = false;
            if size(path, 1) >= 2 && max(path(:,1)) > cur_pose(1) + 4.0
                ahead_mask = (path(:,1) >= cur_pose(1));
                path_ahead = path(ahead_mask, :);
                path_is_valid = true;
                
                % Check rolling costmap along path_ahead to catch any lethal obstacle
                for pt_i = 1:size(path_ahead, 1)
                    c_pt = min(max(round((path_ahead(pt_i, 1) - grid_meta.x_min) / map_cfg.grid_res) + 1, 1), grid_meta.nX);
                    r_pt = min(max(round((path_ahead(pt_i, 2) - grid_meta.y_min) / map_cfg.grid_res) + 1, 1), grid_meta.nY);
                    if rolling_costmap(r_pt, c_pt) >= 200
                        path_is_valid = false;
                        break;
                    end
                end
                
                % Check against potholes ahead with safety margin
                if path_is_valid
                    for j = 1:length(potholes)
                        d_p = hypot(path_ahead(:,1) - potholes(j).x, path_ahead(:,2) - potholes(j).y) - potholes(j).radius;
                        if min(d_p) < 0.40
                            path_is_valid = false;
                            break;
                        end
                    end
                end
                
                % Check against dynamic agent predictions ahead
                if path_is_valid && ~isempty(predictions)
                    for k = 1:length(predictions)
                        wp = predictions(k).waypoints;
                        if isempty(wp), continue; end
                        for h = 1:min(10, size(wp, 1))
                            d_a = hypot(path_ahead(:,1) - wp(h,1), path_ahead(:,2) - wp(h,2));
                            if min(d_a) < 1.25
                                path_is_valid = false;
                                break;
                            end
                        end
                        if ~path_is_valid, break; end
                    end
                end
            end
            
            if path_is_valid
                % Retain previous valid path
                safe_stop_active = false;
                stopped_wait_steps = 0;
            else
                % No valid collision-free path exists ahead.
                % Engage controlled SAFE STOP before reaching the obstacle.
                safe_stop_active = true;
                path = zeros(0, 2);
            end
        end
    end

    % ── F. Universal Bottleneck Decider ───────────────────────────────────
    ubd_params = struct('sim_t', t);
    [vstop_active, ~, bottleneck] = universal_bottleneck_decider( ...
        ego_state', path, rolling_costmap, grid_meta, predictions, ubd_params);
    if isfield(bottleneck, 'path_invalid') && bottleneck.path_invalid
        force_replan = true;
    end

    bsm_params = struct();
    bsm_params.virtual_stop_active = vstop_active;
    if vstop_active
        bsm_params.stop_line_dist = max(0.0, bottleneck.station_s - 3.5);
    end

    % ── G. Behavior State Machine ─────────────────────────────────────────
    [bsm_state, v_ref, ~] = behavior_state_machine( ...
        bsm_state, ego_state', predictions, dt, bsm_params);

    % ── H. Pure Pursuit Controller ────────────────────────────────────────
    if safe_stop_active
        steer_rad = 0.0;
        if ego_state(4) > 0.05
            accel = -3.5; % controlled braking deceleration to stop before obstacle
        else
            accel  = 0.0;
            mock_v = 0.0;
        end
        prev_target = [NaN, NaN];
    elseif size(path, 1) >= 2
        pp_params.L             = L_WB;
        pp_params.k_lookahead   = 0.8;
        pp_params.min_lookahead = 3.5;
        pp_params.Kp_v          = 1.0;
        pp_params.max_steer     = MAX_STEER_RAD;
        pp_params.prev_target   = prev_target;
        pp_params.dt            = dt;
        [ctrl_out, ~, ~, target_pt] = pure_pursuit_controller(ego_state', path, v_ref, pp_params);
        steer_rad = ctrl_out(1);
        accel     = ctrl_out(2);
        prev_target = target_pt;
    else
        steer_rad = 0.0;
        accel     = -2.0;
        prev_target = [NaN, NaN];
    end

    if ~safe_stop_active && v_ref <= 0.0
        accel = -3.5;
    end

    % Off-road corridor guard
    if abs(ego_state(2)) > 4.0
        steer_rad = 0.0;
        accel     = min(accel, -2.5);
    end

    % ── I. Kinematic Integration ──────────────────────────────────────────
    mock_v     = max(0.0, min(8.0, mock_v + accel * dt));
    mock_x     = mock_x + mock_v * cos(mock_theta) * dt;
    mock_y     = mock_y + mock_v * sin(mock_theta) * dt;
    mock_theta = mock_theta + (mock_v / L_WB) * tan(steer_rad) * dt;
    ego_state  = [mock_x; mock_y; mock_theta; mock_v];

    % Track maximum lateral excursion
    max_lateral = max(max_lateral, abs(ego_state(2)));

    % ── I. Clearance and Collision Check ──────────────────────────────────
    is_collided = false;
    % 1. Dynamic agents: distance to center point
    for k = 1:length(raw_obstacles)
        d_agent = norm(ego_state(1:2)' - raw_obstacles(k).position);
        if d_agent < min_clearance
            min_clearance = d_agent;
        end
        if d_agent < collision_thresh
            is_collided = true;
            collision_details = struct('type', 'agent', ...
                'id', raw_obstacles(k).id, ...
                'agent_type', raw_obstacles(k).type, ...
                'position', raw_obstacles(k).position, ...
                'distance', d_agent, 't', t);
            break;
        end
    end

    % 2. Potholes: distance to perimeter (rim)
    if ~is_collided
        for j = 1:length(potholes)
            d_center = norm(ego_state(1:2)' - [potholes(j).x, potholes(j).y]);
            d_edge   = d_center - potholes(j).radius;
            if d_edge < min_clearance
                min_clearance = d_edge;
            end
            % Wheel into pothole: vehicle half-width is ~0.9m, if ego center is
            % within radius + 0.2m, the wheel is definitively inside the pothole
            if d_center < (potholes(j).radius + 0.2)
                is_collided = true;
                collision_details = struct('type', 'pothole', ...
                    'id', j, ...
                    'position', [potholes(j).x, potholes(j).y], ...
                    'radius', potholes(j).radius, ...
                    'distance', d_edge, 't', t);
                break;
            end
        end
    end

    if is_collided
        outcome = 'COLLISION';
        if verbose
            fprintf('[SIM] COLLISION at t=%.1fs with %s (dist=%.2fm)\n', ...
                t, collision_details.type, collision_details.distance);
        end
        break;
    end

    % ── J. Safe Stop & Stall Checks ───────────────────────────────────────
    if safe_stop_active && ego_state(4) < 0.1
        stopped_wait_steps = stopped_wait_steps + 1;
        % If road has been permanently blocked for > MAX_STOP_WAIT_STEPS (6.0s),
        % conclude as a verified permanent SAFE_STOP
        if stopped_wait_steps >= MAX_STOP_WAIT_STEPS
            outcome = 'SAFE_STOP';
            if verbose
                fprintf('[SIM] SAFE_STOP at t=%.1fs: Controlled stop sustained for %.1fs before permanent blockage.\n', ...
                    t, stopped_wait_steps * dt);
            end
            break;
        end
    else
        stopped_wait_steps = 0;
    end

    % Stalled if velocity < 0.1 m/s for > 3 consecutive seconds without being in YIELD_WAIT or SAFE_STOP
    if ego_state(4) < 0.1 && ~strcmp(bsm_state, 'YIELD_WAIT') && ~safe_stop_active
        stall_consecutive_steps = stall_consecutive_steps + 1;
    else
        stall_consecutive_steps = 0;
    end

    if stall_consecutive_steps > round(3.0 / dt)
        outcome = 'STALLED';
        if verbose
            fprintf('[SIM] STALLED at t=%.1fs (v=%.2fm/s in state %s for >3.0s)\n', ...
                t, ego_state(4), bsm_state);
        end
        break;
    end

    % ── K. Goal Check ─────────────────────────────────────────────────────
    dist_goal = norm(ego_state(1:2) - goal_pose(1:2)');
    if dist_goal < goal_dist_tol
        outcome = 'SUCCESS';
        time_to_goal = t;
        if verbose
            fprintf('[SIM] SUCCESS! Reached goal at t=%.1fs with %d replans\n', ...
                time_to_goal, replan_cnt);
        end
        break;
    end

    if verbose && mod(step, 10) == 0
        fprintf('[t=%5.1fs #%3d] Pos:[%5.1f, %5.1f] v=%4.1fm/s | %-11s | min_clr=%.2fm\n', ...
            t, step, ego_state(1), ego_state(2), ego_state(4), bsm_state, min_clearance);
    end
end

if isempty(outcome)
    if safe_stop_active && ego_state(4) < 0.2
        outcome = 'SAFE_STOP';
    else
        outcome = 'TIMEOUT';
    end
    if verbose
        fprintf('[SIM] %s after %d steps (dist_goal=%.2fm)\n', outcome, step, dist_goal);
    end
end

% ── Print EKF innovation statistics for this run ──────────────────────────
if verbose
    fprintf('\n=========================================================\n');
    fprintf('  SENSOR LAYER + EKF INNOVATION STATISTICS\n');
    fprintf('=========================================================\n');
    fprintf('  Total sensor dropout  events : %d\n', total_dropouts);
    fprintf('  Total misclassification events: %d\n', total_misclasses);
    if ~isempty(innov_log)
        fprintf('  EKF innovation |innov| stats  :\n');
        fprintf('    N measurements updated : %d\n',  length(innov_log));
        fprintf('    Mean |innov|           : %.4f m\n', mean(innov_log));
        fprintf('    Std  |innov|           : %.4f m\n', std(innov_log));
        fprintf('    Max  |innov|           : %.4f m\n', max(innov_log));
        fprintf('    Min  |innov|           : %.4f m\n', min(innov_log));
    else
        fprintf('  No EKF measurement updates recorded (all detections dropped?)\n');
    end
    fprintf('=========================================================\n\n');
end

% Pack result struct
result.outcome                = outcome;
result.time_to_goal           = time_to_goal;
result.replan_count           = replan_cnt;
result.min_clearance_achieved = min_clearance;
result.max_lateral_position   = max_lateral;
result.seed                   = seed;
result.steps_taken            = step;
result.final_pos              = ego_state(1:2)';
result.collision_details      = collision_details;
result.num_potholes           = length(potholes);
result.num_agents             = length(dynamic_agents);
result.error_msg              = '';
result.error_stack            = '';
result.obstacle_config        = obstacle_config;
% Sensor layer stats
result.innov_log              = innov_log;
result.total_dropouts         = total_dropouts;
result.total_misclasses       = total_misclasses;
if ~isempty(innov_log)
    result.innov_mean = mean(innov_log);
    result.innov_max  = max(innov_log);
else
    result.innov_mean = NaN;
    result.innov_max  = NaN;
end

end

%% ── Local Helper Functions ───────────────────────────────────────────────
function lg = compute_local_goal(ego_xy, planned_path, horizon_m, goal_pose)
    target_x = min(goal_pose(1), ego_xy(1) + horizon_m);
    lg = [target_x, goal_pose(2), goal_pose(3)];
end
