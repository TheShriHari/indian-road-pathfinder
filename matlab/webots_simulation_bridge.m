%% WEBOTS_SIMULATION_BRIDGE — MATLAB <-> Webots Co-Simulation Driver
% SIH 2026 PS-26037: Adaptive Path Planning for Autonomous Vehicles
% =========================================================================
%
% ARCHITECTURE
% ─────────────────────────────────────────────────────────────────────────
%  [Webots 3D Simulator (webots.exe)]
%       ↕  (Webots Supervisor API)
%  [indian_road_supervisor.py]  ←→  TCP JSON on port 20000
%       ↕
%  [THIS FILE — webots_simulation_bridge.m]
%       │  ego pose / obstacle list / road boundaries / pothole specs
%       │  received every tick from Webots Supervisor
%       │
%       ├─ local_occupancy_grid_builder.m  → rolling 0-255 costmap
%       ├─ dynamic_obstacle_predictor.m    → EKF per-agent prediction
%       ├─ universal_bottleneck_decider.m  → dynamic virtual stop line
%       ├─ adaptive_path_planner.m         → Hybrid A* on rolling costmap
%       ├─ behavior_state_machine.m        → CRUISE/NUDGE/YIELD/RESUME
%       └─ pure_pursuit_controller.m       → steer + accel commands
%       │
%       └→ {"steer":…, "throttle":…, "brake":…} sent back to Webots
%
% HOW TO RUN
% ─────────────────────────────────────────────────────────────────────────
%   1. In terminal: python webots_sim.py --bridge
%   2. In MATLAB:   cd('matlab'); webots_simulation_bridge
% =========================================================================

clear; clc;

fprintf('=========================================================\n');
fprintf('  Webots 3D <-> MATLAB Co-Simulation (SIH PS-26037)      \n');
fprintf('=========================================================\n\n');

%% ── Config ───────────────────────────────────────────────────────────────
BRIDGE_HOST         = '127.0.0.1';
BRIDGE_PORT         = 20000;
DT                  = 0.02;   % Webots synchronous step size (20ms)
N_HORIZON           = 35;     % EKF prediction steps (3.5s lead time)
GOAL_POSE           = [75.0, -1.75, 0.0];  % [x, y, theta]
LOCAL_GOAL_HORIZON  = 28.0;   % metres ahead along path for Hybrid A*
GOAL_DIST_TOL       = 2.5;    % metres
MAX_STEPS           = 2500;   % safety limit (50s at 20ms)

% Vehicle parameters
L_WB          = 2.8;    % wheelbase (m)
MAX_STEER_RAD = pi/6;   % 30 deg max physical steer

map_cfg.resolution = 0.2;
map_cfg.length_x   = 50.0;
map_cfg.width_y    = 14.0;
map_cfg.offset_x   = 10.0;

%% ── Establish TCP connection to Webots Python bridge ─────────────────────
fprintf('[NET] Connecting to Webots bridge at %s:%d ...\n', BRIDGE_HOST, BRIDGE_PORT);
try
    tcp = tcpclient(BRIDGE_HOST, BRIDGE_PORT, 'Timeout', 15);
    configureTerminator(tcp, 'LF');
    fprintf('[NET] Connected successfully to Webots bridge.\n\n');
catch ME
    error('[NET] Connection failed: %s\nPlease ensure "python webots_sim.py --bridge" is running first.', ME.message);
end

%% ── Persistent algorithm state ───────────────────────────────────────────
clear dynamic_obstacle_predictor;
bsm_state    = 'CRUISE';
v_ref        = 5.0;
path         = zeros(0, 2);
force_replan = true;
step         = 0;
replan_cnt   = 0;
REPLAN_EVERY = 10;      % Replan every 200ms at 20ms step

fprintf('[SIM] Starting co-simulation loop...\n');

%% ── Main Control Loop ────────────────────────────────────────────────────
while step < MAX_STEPS
    step = step + 1;
    t = step * DT;

    % Read telemetry from Webots
    try
        raw_line = readline(tcp);
    catch ME
        fprintf('[NET] Connection closed by Webots: %s\n', ME.message);
        break;
    end
    if strlength(raw_line) == 0, break; end

    sensor_pkg = jsondecode(raw_line);
    ego_state = sensor_pkg.ego_state(:);   % [x; y; theta; v]

    if sensor_pkg.collision
        fprintf('\n[!!!] COLLISION / HAZARD detected in Webots at t=%.2fs!\n', t);
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
    predictions   = dynamic_obstacle_predictor(raw_obstacles, DT * 5, N_HORIZON);

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
        ctrl.steer    = 0.0;
        ctrl.throttle = 0.0;
        ctrl.brake    = 1.0;
        send_control(tcp, ctrl);
        fprintf('\n[SUCCESS] Goal arrived at X=%.1fm in t=%.1fs!\n', ego_state(1), t);
        break;
    end

    cur_pose_4d = [ego_state(1); ego_state(2); ego_state(3); ego_state(4)];
    [delta_cmd, a_cmd] = pure_pursuit_controller(cur_pose_4d, path, v_ref, L_WB);

    steer_norm = delta_cmd / MAX_STEER_RAD;
    steer_norm = max(-1.0, min(1.0, steer_norm));

    if a_cmd >= 0
        throttle = min(1.0, a_cmd / 3.0);
        brake    = 0.0;
    else
        throttle = 0.0;
        brake    = min(1.0, -a_cmd / 5.0);
    end

    ctrl.steer    = round(steer_norm, 4);
    ctrl.throttle = round(throttle, 4);
    ctrl.brake    = round(brake, 4);
    send_control(tcp, ctrl);

    if mod(step, 25) == 0
        fprintf('[t=%5.2fs] X=%5.1f Y=%+4.2f V=%4.1f | BSM=%-14s | steer=%+5.1f deg thr=%.2f brk=%.2f\n', ...
            t, ego_state(1), ego_state(2), ego_state(4), bsm_state, ...
            rad2deg(delta_cmd), throttle, brake);
    end
end

clear tcp;
fprintf('[SIM] Co-simulation finished.\n');


%% ── Helper functions ─────────────────────────────────────────────────────
function obs_list = pkg_obstacles(pkg)
    if ~isfield(pkg, 'obstacles') || isempty(pkg.obstacles)
        obs_list = struct([]);
        return;
    end
    raw = pkg.obstacles;
    N = length(raw);
    obs_list = repmat(struct('id', 0, 'type', '', 'pos', [0 0], ...
                             'vel', [0 0], 'heading', 0, 'length', 0, 'width', 0), N, 1);
    for k = 1:N
        if iscell(raw), item = raw{k}; else, item = raw(k); end
        obs_list(k).id      = item.id;
        obs_list(k).type    = item.type;
        obs_list(k).pos     = item.pos(:)';
        obs_list(k).vel     = item.vel(:)';
        obs_list(k).heading = item.heading;
        obs_list(k).length  = item.length;
        obs_list(k).width   = item.width;
    end
end

function bounds = pkg_road_boundaries(pkg)
    if isfield(pkg, 'road_boundaries') && ~isempty(pkg.road_boundaries)
        bounds = pkg.road_boundaries(:)';
    else
        bounds = [3.5, -3.5];
    end
end

function send_control(tcp, ctrl)
    msg = sprintf('{"steer":%.4f,"throttle":%.4f,"brake":%.4f}\n', ...
        ctrl.steer, ctrl.throttle, ctrl.brake);
    write(tcp, uint8(msg));
end
