%% SUMO_SIMULATION_BRIDGE — MATLAB <-> Eclipse SUMO Co-Simulation Driver
% SIH 2026 PS-26037: Adaptive Path Planning for Autonomous Vehicles
% =========================================================================
%
% ARCHITECTURE
% ─────────────────────────────────────────────────────────────────────────
%  [Eclipse SUMO (sumo / sumo-gui)]
%       ↕  (TraCI API)
%  [sumo_sim.py --bridge]  ←→  TCP JSON on port 20000
%       ↕
%  [THIS FILE — sumo_simulation_bridge.m]
%       │  ego pose / obstacle list / road boundaries / camera stats
%       │  received every tick from Python SUMO bridge
%       │
%       ├─ local_occupancy_grid_builder.m  → rolling 0-255 costmap
%       ├─ dynamic_obstacle_predictor.m    → EKF per-agent prediction
%       ├─ universal_bottleneck_decider.m  → dynamic virtual stop line
%       ├─ adaptive_path_planner.m         → Hybrid A* on rolling costmap
%       ├─ behavior_state_machine.m        → CRUISE/NUDGE/YIELD/RESUME
%       └─ pure_pursuit_controller.m       → steer + accel commands
%       │
%       └→ {"steer":…, "throttle":…, "brake":…} sent back to SUMO
%
% HOW TO RUN
% ─────────────────────────────────────────────────────────────────────────
%   1. In terminal: python sumo_sim.py --gui --bridge
%   2. In MATLAB:   cd('matlab'); sumo_simulation_bridge
% =========================================================================

clear; clc;

fprintf('=========================================================\n');
fprintf('  Eclipse SUMO <-> MATLAB Co-Simulation (SIH PS-26037)    \n');
fprintf('=========================================================\n\n');

%% ── Config ───────────────────────────────────────────────────────────────
BRIDGE_HOST         = '127.0.0.1';
BRIDGE_PORT         = 20000;
DT                  = 0.1;    % SUMO synchronous step size (seconds)
N_HORIZON           = 35;     % EKF prediction steps (3.5s lead time)
GOAL_POSE           = [75.0, -1.75, 0.0];  % [x, y, theta]
LOCAL_GOAL_HORIZON  = 28.0;   % metres ahead along path for Hybrid A*
GOAL_DIST_TOL       = 2.5;    % metres
MAX_STEPS           = 600;    % safety limit (60s)

% Vehicle parameters
L_WB          = 2.8;    % wheelbase (m)
MAX_STEER_RAD = pi/6;   % 30 deg max physical steer

map_cfg.resolution = 0.2;
map_cfg.length_x   = 50.0;
map_cfg.width_y    = 14.0;
map_cfg.offset_x   = 10.0;

%% ── Establish TCP connection to SUMO Python bridge ───────────────────────
fprintf('[NET] Connecting to SUMO bridge at %s:%d ...\n', BRIDGE_HOST, BRIDGE_PORT);
try
    tcp = tcpclient(BRIDGE_HOST, BRIDGE_PORT, 'Timeout', 15);
    configureTerminator(tcp, 'LF');
    fprintf('[NET] Connected successfully to SUMO bridge.\n\n');
catch ME
    error('[NET] Connection failed: %s\nPlease ensure "python sumo_sim.py --bridge" is running first.', ME.message);
end

%% ── Persistent algorithm state ───────────────────────────────────────────
clear dynamic_obstacle_predictor;
bsm_state    = 'CRUISE';
v_ref        = 4.5;
path         = zeros(0, 2);
force_replan = true;
step         = 0;
replan_cnt   = 0;
REPLAN_EVERY = 5;

fprintf('[SIM] Starting co-simulation loop...\n');

%% ── Main Control Loop ────────────────────────────────────────────────────
while step < MAX_STEPS
    step = step + 1;
    t = step * DT;

    % Read telemetry from SUMO
    try
        raw_line = readline(tcp);
    catch ME
        fprintf('[NET] Connection closed by SUMO: %s\n', ME.message);
        break;
    end
    if strlength(raw_line) == 0, break; end

    sensor_pkg = jsondecode(raw_line);
    ego_state = sensor_pkg.ego_state(:);   % [x; y; theta; v]

    if sensor_pkg.collision
        fprintf('\n[!!!] COLLISION detected in SUMO at t=%.1fs!\n', t);
        break;
    end

    % Build sensor detections struct
    sensor_det.road_boundaries = pkg_road_boundaries(sensor_pkg);
    if isfield(sensor_pkg, 'potholes') && ~isempty(sensor_pkg.potholes)
        sensor_det.potholes = sensor_pkg.potholes;
    else
        sensor_det.potholes = struct([]);
    end
    sensor_det.static_boxes    = struct([]);
    sensor_det.static_points   = [];

    % Rolling occupancy costmap
    [rolling_costmap, grid_meta] = local_occupancy_grid_builder( ...
        ego_state(1:3)', sensor_det, map_cfg);

    % Dynamic obstacle EKF prediction
    raw_obstacles = pkg_obstacles(sensor_pkg);
    predictions   = dynamic_obstacle_predictor(raw_obstacles, DT, N_HORIZON);

    % Path Re-planning
    needs_replan = force_replan || (size(path, 1) < 2) || (mod(step, REPLAN_EVERY) == 0);
    force_replan = false;
    if ~needs_replan && ~isempty(predictions)
        for k = 1:length(predictions)
            wp = predictions(k).waypoints;
            if isempty(wp), continue; end
            d = min(hypot(path(:,1) - wp(1,1), path(:,2) - wp(1,2)));
            if d < 3.0
                needs_replan = true;
                break;
            end
        end
    end

    if needs_replan
        cur_pose = [ego_state(1), ego_state(2), ego_state(3)];
        target_x = min(GOAL_POSE(1), ego_state(1) + LOCAL_GOAL_HORIZON);
        local_goal = [target_x, GOAL_POSE(2), GOAL_POSE(3)];
        grid_org = [grid_meta.x_min, grid_meta.y_min];

        [path_new, ~, ~, plan_ok] = adaptive_path_planner( ...
            cur_pose, local_goal, rolling_costmap, ...
            map_cfg.resolution, grid_org, predictions, bsm_state);

        if plan_ok && size(path_new, 1) >= 2
            path = path_new;
            replan_cnt = replan_cnt + 1;
        end
    end

    if isempty(path)
        path = [ego_state(1), ego_state(2); ego_state(1)+10, ego_state(2)];
    end

    % Bottleneck decider
    road_w = 7.0;
    bottleneck = universal_bottleneck_decider( ...
        ego_state(1:2)', ego_state(4), raw_obstacles, ...
        predictions, road_w, path, bsm_state);
    vstop_active = bottleneck.virtual_stop_active;

    % Behavior State Machine
    road_env.has_blind_spot = false;
    road_env.road_condition = 'NORMAL';
    road_env.road_width     = road_w;

    [bsm_state, v_ref, ~] = behavior_state_machine( ...
        bsm_state, ego_state(4), raw_obstacles, ...
        predictions, road_env, vstop_active);

    % Pure pursuit controller
    dist_goal = norm(ego_state(1:2) - GOAL_POSE(1:2)');
    if dist_goal < GOAL_DIST_TOL
        v_ref = 0.0;
    end

    cur_pose = [ego_state(1), ego_state(2), ego_state(3)];
    [steer_rad, accel, ~] = pure_pursuit_controller( ...
        cur_pose, ego_state(4), path, v_ref, DT);

    % Normalize controls
    norm_steer = max(-1.0, min(1.0, steer_rad / MAX_STEER_RAD));
    if v_ref <= 0.0 && ego_state(4) < 0.1
        throttle = 0.0;
        brake    = 1.0;
    elseif accel >= 0
        throttle = min(max(accel / 3.0, 0.0), 1.0);
        brake    = 0.0;
    else
        throttle = 0.0;
        brake    = min(max(-accel / 5.0, 0.0), 1.0);
    end

    % Send control back to SUMO
    ctrl.steer    = norm_steer;
    ctrl.throttle = throttle;
    ctrl.brake    = brake;
    writeline(tcp, jsonencode(ctrl));

    % Console output
    if mod(step, 10) == 0 || vstop_active
        vstop_tag = '';
        if vstop_active, vstop_tag = ' [VSTOP ACTIVE]'; end
        fprintf('[t=%5.1fs #%03d] Pos:[%5.1f,%5.2f] %4.1fm/s | %-12s | steer=%+.2f T=%.2f B=%.2f%s\n', ...
            t, step, ego_state(1), ego_state(2), ego_state(4), ...
            bsm_state, norm_steer, throttle, brake, vstop_tag);
    end

    if dist_goal < GOAL_DIST_TOL
        fprintf('\n=========================================================\n');
        fprintf('  [SUCCESS] Goal reached at t = %.1f s!\n', t);
        fprintf('  Final pos : [%.2f, %.2f]  |  Replans : %d\n', ...
                ego_state(1), ego_state(2), replan_cnt);
        fprintf('=========================================================\n\n');
        break;
    end
end

if ~isempty(tcp)
    clear tcp;
end
fprintf('[SUMO] Bridge closed.\n');

%% ── Helpers ──────────────────────────────────────────────────────────────
function obs_arr = pkg_obstacles(pkg)
    obs_arr = struct([]);
    if ~isfield(pkg, 'obstacles') || isempty(pkg.obstacles), return; end
    raw = pkg.obstacles;
    for k = 1:length(raw)
        entry = raw(k);
        obs_arr(k).id               = entry.id;
        obs_arr(k).type             = entry.type;
        pos                         = entry.position;
        obs_arr(k).position         = [pos(1), pos(2)];
        vel                         = entry.velocity;
        obs_arr(k).velocity         = [vel(1), vel(2)];
        obs_arr(k).behavior_profile = entry.behavior_profile;
    end
end

function rb = pkg_road_boundaries(pkg)
    rb = [];
    if ~isfield(pkg, 'road_boundaries') || isempty(pkg.road_boundaries), return; end
    raw = pkg.road_boundaries;
    if iscell(raw), rb = cell2mat(raw);
    elseif isnumeric(raw), rb = raw; end
end
