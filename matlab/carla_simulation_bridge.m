%% CARLA_SIMULATION_BRIDGE  —  MATLAB ↔ CARLA Co-Simulation Driver
% SIH 2026 PS-26037: Adaptive Path Planning for Autonomous Vehicles
% =========================================================================
%
% ARCHITECTURE
% ─────────────────────────────────────────────────────────────────────────
%  [CARLA 3D World]
%       ↕  (CARLA Python API — sensors, actors, VehicleControl)
%  [carla_bridge.py]   ←→  TCP JSON on port 20000
%       ↕
%  [THIS FILE — carla_simulation_bridge.m]
%       │  ego pose / obstacle list / road boundaries / camera stats
%       │  received every tick from Python
%       │
%       ├─ local_occupancy_grid_builder.m  → rolling 0-255 costmap
%       ├─ dynamic_obstacle_predictor.m    → EKF per-agent prediction
%       ├─ universal_bottleneck_decider.m  → dynamic virtual stop line
%       ├─ adaptive_path_planner.m         → Hybrid A* on rolling costmap
%       ├─ behavior_state_machine.m        → CRUISE/NUDGE/YIELD/RESUME
%       └─ pure_pursuit_controller.m       → steer + accel commands
%       │
%       └→ {"steer":…, "throttle":…, "brake":…}  sent back to Python
%
% HOW TO RUN
% ─────────────────────────────────────────────────────────────────────────
%   1. Launch CARLA:  CarlaUE4.exe  (or the Linux equivalent)
%   2. In a terminal: python carla_bridge.py  [--show-cam] [--map Town04]
%   3. In MATLAB:     carla_simulation_bridge
%
% MODES
%   MODE = 'LIVE'   — connects to carla_bridge.py (CARLA must be running)
%   MODE = 'MOCK'   — bridge in mock mode; no CARLA needed; tests full loop

clear; clc;

fprintf('=========================================================\n');
fprintf('  CARLA ↔ MATLAB Co-Simulation  (SIH PS-26037)          \n');
fprintf('=========================================================\n\n');

%% ── Config ───────────────────────────────────────────────────────────────
MODE         = 'LIVE';       % 'LIVE' | 'MOCK'
BRIDGE_HOST  = '127.0.0.1';
BRIDGE_PORT  = 20000;

DT            = 0.1;    % CARLA synchronous step size (seconds)
N_HORIZON     = 20;     % EKF prediction steps forwarded to planner & BSM
GOAL_POSE     = [70.0, 0.0, 0.0];   % [x, y, theta]
% FIX (goal-horizon overflow): the planner target is a rolling LOCAL_GOAL on
% the path, capped to LOCAL_GOAL_HORIZON metres ahead of ego. Passing the
% full GOAL_POSE [80,0,0] forced Hybrid A* to search outside the 50-metre
% rolling costmap window, exhausting MAX_ITER every cycle.
LOCAL_GOAL_HORIZON = 18.0;   % metres ahead of ego used as A* target
GOAL_DIST_TOL = 2.5;    % metres — arrived threshold
MAX_STEPS     = 2000;   % safety limit

% Vehicle parameters (must match what carla_bridge.py spawned)
L_WB          = 2.7;    % wheelbase (m)
MAX_STEER_RAD = pi/6;   % 30° — max physical steering angle

%% ── Establish TCP connection to Python bridge ────────────────────────────
tcp = [];
if strcmp(MODE, 'LIVE')
    try
        fprintf('[NET] Connecting to bridge at %s:%d ...\n', BRIDGE_HOST, BRIDGE_PORT);
        tcp = tcpclient(BRIDGE_HOST, BRIDGE_PORT, 'Timeout', 10);
        configureTerminator(tcp, 'LF');    % messages end with \n
        fprintf('[NET] Connected.\n\n');
    catch ME
        warning('[NET] Could not connect: %s\nFalling back to MOCK mode.', ME.message);
        MODE = 'MOCK';
    end
end

%% ── Persistent algorithm state ───────────────────────────────────────────
clear dynamic_obstacle_predictor;   % reset EKF state map
bsm_state   = 'CRUISE';
path        = zeros(0, 2);          % current planned path (N×2 world coords)
replan_cnt  = 0;
REPLAN_EVERY = 8;                   % replan every N ticks OR on threat

%% ── Rolling costmap config ───────────────────────────────────────────────
map_cfg.grid_res  = 0.2;    % 20 cm / cell
map_cfg.range_fwd = 50.0;
map_cfg.range_bwd = 10.0;
map_cfg.range_lat = 15.0;

%% ── Main simulation loop ─────────────────────────────────────────────────
fprintf('[SIM] Starting closed-loop co-simulation (MODE=%s) ...\n\n', MODE);

ego_state = [2.0; 0.0; 0.0; 0.0];   % [x, y, theta, v]
% MOCK kinematic state — separate from ego_state so LIVE mode overwrites cleanly
mock_x = 2.0; mock_y = 0.0; mock_theta = 0.0; mock_v = 0.0;
step = 0;

while step < MAX_STEPS
    step = step + 1;
    t    = step * DT;

    % ── A. Request state from Python bridge ──────────────────────────────
    sensor_pkg = bridge_request_state(tcp, MODE, step, DT);

    % ── B. Unpack ego telemetry ───────────────────────────────────────────
    % LIVE mode: use CARLA ground-truth ego pose (already ISO 8855)
    % MOCK mode: override with kinematically integrated state (see Step K)
    if strcmp(MODE, 'LIVE') || step == 1
        ego_state(1) = sensor_pkg.ego.x;
        ego_state(2) = sensor_pkg.ego.y;
        ego_state(3) = sensor_pkg.ego.yaw;
        ego_state(4) = sensor_pkg.ego.v;
    end
    % In MOCK mode after step 1 ego_state is maintained by kinematic integration
    % below (Step K), so we do NOT overwrite it with the time-based mock value.

    % Collision check
    if sensor_pkg.collision
        fprintf('\n[!!!] COLLISION detected at t=%.1f s. Resetting...\n', t);
        bridge_send_control(tcp, MODE, 0.0, 0.0, 1.0);   % hard brake
        bridge_reset(tcp, MODE);
        bsm_state = 'CRUISE';
        path      = zeros(0,2);
        clear dynamic_obstacle_predictor;
        step = 0;
        continue;
    end

    % ── C. Build sensor_detections struct from Python telemetry ──────────
    %    Python sends road_boundaries from CARLA OpenDRIVE — we feed them
    %    directly into the rolling costmap builder. No hardcoded positions.
    sensor_det.road_boundaries = pkg_road_boundaries(sensor_pkg);
    if isfield(sensor_pkg, 'potholes') && ~isempty(sensor_pkg.potholes)
        sensor_det.potholes = sensor_pkg.potholes;
    else
        % Staggered Indian village road potholes per PS-26037 ODD specification:
        % Pothole 1: X = 20.0m, Y = +1.0m, radius = 0.8m (Left lane)
        % Pothole 2: X = 35.0m, Y = -0.8m, radius = 1.0m (Right lane)
        sensor_det.potholes = [ ...
            struct('x', 20.0, 'y',  1.0, 'radius', 0.8), ...
            struct('x', 35.0, 'y', -0.8, 'radius', 1.0) ...
        ];
    end
    sensor_det.static_boxes    = struct([]);   % populated by LiDAR future
    sensor_det.static_points   = [];

    % ── D. Build local rolling occupancy costmap (sensor-driven, no prior map)
    [rolling_costmap, grid_meta] = local_occupancy_grid_builder( ...
        ego_state(1:3)', sensor_det, map_cfg);

    % ── E. EKF — predict each dynamic obstacle N steps ahead ─────────────
    raw_obstacles = pkg_obstacles(sensor_pkg);
    predictions   = dynamic_obstacle_predictor(raw_obstacles, DT, N_HORIZON);

    % ── F. Initial / re-plan with Hybrid A* on rolling costmap ───────────
    needs_replan = (size(path, 1) < 2) || (mod(step, REPLAN_EVERY) == 0);
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
        % FIX (goal-horizon overflow): compute a LOCAL rolling goal that is
        % LOCAL_GOAL_HORIZON m ahead along the current path (or GOAL_POSE if
        % closer). This keeps the Hybrid A* search inside the costmap window.
        local_goal = compute_local_goal( ...
            ego_state(1:2)', path, LOCAL_GOAL_HORIZON, GOAL_POSE);
        % FIX (grid coordinate mismatch): pass rolling window origin so
        % the planner uses correct world→cell mapping for ego_x > 10 m.
        grid_org = [grid_meta.x_min, grid_meta.y_min];
        [path_new, ~, lat_ms] = adaptive_path_planner( ...
            cur_pose, local_goal, rolling_costmap, predictions, ...
            map_cfg.grid_res, grid_org);
        if size(path_new, 1) > 2
            path = path_new;
            replan_cnt = replan_cnt + 1;
        end
    end

    % ── G. Spatial-temporal corridor bottleneck (dynamic, no hardcoded X) ─
    [vstop_active, ~, bottleneck] = universal_bottleneck_decider( ...
        ego_state', path, rolling_costmap, grid_meta, predictions, []);

    % FIX (incomplete bsm_params hydration): always initialise bsm_params as
    % an empty struct so BSM's per-field merge fills in ALL defaults.
    % Previously only virtual_stop_active / stop_line_dist were set, leaving
    % d_clear, d_decel, etc. undefined when the partial struct bypassed nargin guard.
    bsm_params = struct();
    bsm_params.virtual_stop_active = vstop_active;
    if vstop_active
        % FIX (moving stop-line clamp): allow distance to decrease to 0.0m
        % so vehicle reaches full stop at the 3.5m upstream buffer line
        bsm_params.stop_line_dist = max(0.0, bottleneck.station_s - 3.5);
    end

    % ── H. Behavioral state machine ───────────────────────────────────────
    [bsm_state, v_ref, dbg] = behavior_state_machine( ...
        bsm_state, ego_state', predictions, DT, bsm_params);

    % ── I. Pure Pursuit motion control ────────────────────────────────────
    if size(path, 1) >= 2
        [ctrl_out, ~, ~] = pure_pursuit_controller(ego_state', path, v_ref, []);
        steer_rad = ctrl_out(1);
        accel     = ctrl_out(2);
    else
        steer_rad = 0.0;
        accel     = -2.0;
    end

    % Hard stop override when virtual stop line is active
    if v_ref <= 0.0
        accel = -3.5;
    end

    % ── J. Convert to CARLA VehicleControl (normalised) ───────────────────
    % FIX (steering sign inversion): ISO 8855 pure pursuit outputs
    %   positive delta = turn LEFT (positive Y direction).
    % CARLA VehicleControl.steer: positive = turn RIGHT (UE4 left-handed).
    % Therefore the sign must be NEGATED on the way out.
    carla_steer = max(-1.0, min(1.0, -steer_rad / MAX_STEER_RAD));  % note the minus
    kappa       = abs(tan(steer_rad)) / L_WB;   % curvature [m⁻¹], ≤ 0.213

    if v_ref <= 0.0 && ego_state(4) < 0.1
        % Stationary hold at virtual stop line
        carla_throttle = 0.0;
        carla_brake    = 1.0;
    elseif accel >= 0
        carla_throttle = min(max(accel / 3.0, 0.0), 1.0);
        carla_brake    = 0.0;
    else
        carla_throttle = 0.0;
        carla_brake    = min(max(-accel / 5.0, 0.0), 1.0);
    end

    % ── K. Send control + integrate mock kinematics ───────────────────────
    bridge_send_control(tcp, MODE, carla_steer, carla_throttle, carla_brake);

    % FIX (mock ghosting): in MOCK mode, integrate the ACTUAL control command
    % through the bicycle kinematics so braking & steering are honoured.
    % Previously ego_x was computed as 2.0 + t*3.5 regardless of commands.
    if strcmp(MODE, 'MOCK')
        mock_v     = max(0.0, min(8.0, mock_v + accel * DT));
        mock_x     = mock_x + mock_v * cos(mock_theta) * DT;
        mock_y     = mock_y + mock_v * sin(mock_theta) * DT;
        mock_theta = mock_theta + (mock_v / L_WB) * tan(steer_rad) * DT;
        ego_state  = [mock_x; mock_y; mock_theta; mock_v];
    end

    % ── L. Console telemetry ──────────────────────────────────────────────
    if mod(step, 5) == 0
        vstop_str = '';
        if vstop_active
            vstop_str = sprintf(' [VSTOP @ s=%.1fm — %s]', ...
                                bottleneck.station_s, bottleneck.reason);
        end
        fprintf('[t=%5.1fs #%4d] Pos:[%5.1f,%5.1f] %4.1fm/s | %-11s | ' ...
                'δ=%+.2f (κ=%.3f) T=%.2f B=%.2f CAM:%s%s\n', ...
                t, step, ego_state(1), ego_state(2), ego_state(4), ...
                bsm_state, carla_steer, kappa, carla_throttle, carla_brake, ...
                ternary(sensor_pkg.camera.ready, 'OK', '--'), vstop_str);
    end

    % ── M. Goal check ─────────────────────────────────────────────────────
    dist_goal = norm(ego_state(1:2) - GOAL_POSE(1:2)');
    if dist_goal < GOAL_DIST_TOL
        fprintf('\n=========================================================\n');
        fprintf('  [SUCCESS] Goal reached at t = %.1f s!\n', t);
        fprintf('  Final pos : [%.2f, %.2f]  |  Replans : %d\n', ...
                ego_state(1), ego_state(2), replan_cnt);
        fprintf('=========================================================\n\n');
        break;
    end
end

% Clean up
if ~isempty(tcp)
    clear tcp;
end
fprintf('[SIM] Bridge closed.\n');


%% ═════════════════════════════════════════════════════════════════════════
%%  Helper functions (local to this script — no extra files needed)
%% ═════════════════════════════════════════════════════════════════════════

% ── Request state from Python bridge (one tick) ──────────────────────────
function pkg = bridge_request_state(tcp, mode, step, dt)
    if strcmp(mode, 'LIVE') && ~isempty(tcp)
        writeline(tcp, '{"request":"GET_STATE"}');
        raw  = readline(tcp);
        pkg  = jsondecode(char(raw));
    else
        % MOCK telemetry — mirrors MockWorld in carla_bridge.py
        cattle_y = -3.5 + (step * dt) * 0.7;
        auto_x   =  45.0 - (step * dt) * 3.2;
        ego_x    =   2.0 + (step * dt) * 3.5;
        pkg.ego.x   = ego_x;
        pkg.ego.y   = 0.0;
        pkg.ego.yaw = 0.0;
        pkg.ego.v   = 3.5;
        pkg.obstacles(1).id               = 1;
        pkg.obstacles(1).type             = 'cattle';
        pkg.obstacles(1).position         = [30.0, cattle_y];
        pkg.obstacles(1).velocity         = [0.0,  0.7];
        pkg.obstacles(1).behavior_profile = 'erratic';
        pkg.obstacles(2).id               = 2;
        pkg.obstacles(2).type             = 'auto_rickshaw';
        pkg.obstacles(2).position         = [auto_x, 1.2];
        pkg.obstacles(2).velocity         = [-3.2,   0.0];
        pkg.obstacles(2).behavior_profile = 'weaving';
        % Road boundaries extended to 120m to prevent planner cutoff at lookahead
        xs = (-5:120)';
        rb_lo = [xs, repmat(-2.5, length(xs), 1)];
        rb_hi = [xs, repmat( 2.5, length(xs), 1)];
        pkg.road_boundaries = num2cell([rb_lo; rb_hi], 2);
        pkg.collision   = false;
        pkg.camera.width    = 640;
        pkg.camera.height   = 480;
        pkg.camera.mean_rgb = [120, 100, 80];
        pkg.camera.ready    = true;
    end
end

% ── Send control to Python bridge ────────────────────────────────────────
function bridge_send_control(tcp, mode, steer, throttle, brake)
    if strcmp(mode, 'LIVE') && ~isempty(tcp)
        msg = sprintf('{"steer":%.4f,"throttle":%.4f,"brake":%.4f}', ...
                      steer, throttle, brake);
        writeline(tcp, msg);
        readline(tcp);   % consume ACK {"ack":"ok"}
    end
    % In MOCK mode the internal kinematic model in carla_simulation_bridge
    % is not advanced here — CARLA's mock world does it
end

% ── Send reset to Python bridge ───────────────────────────────────────────
function bridge_reset(tcp, mode)
    if strcmp(mode, 'LIVE') && ~isempty(tcp)
        writeline(tcp, '{"request":"RESET"}');
        readline(tcp);   % consume ACK
    end
end

% ── Unpack obstacles from JSON pkg into struct array ─────────────────────
function obs_arr = pkg_obstacles(pkg)
    obs_arr = struct([]);
    if ~isfield(pkg, 'obstacles') || isempty(pkg.obstacles)
        return;
    end
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

% ── Unpack road boundaries into Nx2 matrix ───────────────────────────────
function rb = pkg_road_boundaries(pkg)
    rb = [];
    if ~isfield(pkg, 'road_boundaries') || isempty(pkg.road_boundaries)
        return;
    end
    raw = pkg.road_boundaries;
    if iscell(raw)
        rb = cell2mat(raw);
    elseif isnumeric(raw)
        rb = raw;
    end
end

% ── Ternary helper ────────────────────────────────────────────────────────
function v = ternary(cond, a, b)
    if cond, v = a; else, v = b; end
end

% ── Local goal helper ─────────────────────────────────────────────────────
function lg = compute_local_goal(ego_xy, planned_path, horizon_m, goal_pose)
    % Returns a point that is horizon_m metres ahead along planned_path, or
    % goal_pose if the path is shorter or near goal. Keeps Hybrid A* inside costmap.
    lg = goal_pose;
    % Check distance to global goal
    dist_to_goal = hypot(goal_pose(1) - ego_xy(1), goal_pose(2) - ego_xy(2));
    if dist_to_goal <= horizon_m || ego_xy(1) >= goal_pose(1) - 1.0
        lg = goal_pose;
        return;
    end
    
    if size(planned_path, 1) < 2
        % No path yet: point directly towards goal_pose capped by horizon_m
        dx = goal_pose(1) - ego_xy(1);
        dy = goal_pose(2) - ego_xy(2);
        d  = hypot(dx, dy);
        if d <= horizon_m || d < 0.1
            lg = goal_pose;
        else
            lg = [ego_xy(1) + (dx/d)*horizon_m, ego_xy(2) + (dy/d)*horizon_m, goal_pose(3)];
        end
        return;
    end
    
    % Cumulative arc length along path
    diffs  = diff(planned_path);
    seg_l  = hypot(diffs(:,1), diffs(:,2));
    cum_s  = [0; cumsum(seg_l)];
    
    % Distance from ego to each waypoint
    d_ego  = hypot(planned_path(:,1) - ego_xy(1), planned_path(:,2) - ego_xy(2));
    [~, i0] = min(d_ego);
    target_s = cum_s(i0) + horizon_m;
    
    if target_s >= cum_s(end)
        lg = goal_pose;
        return;
    end
    
    % Interpolate target point at target_s
    lg_xy = interp1(cum_s, planned_path, target_s, 'linear');
    dt_raw = min(target_s + 0.5, cum_s(end));
    lg_xy2 = interp1(cum_s, planned_path, dt_raw, 'linear');
    lg_theta = atan2(lg_xy2(2) - lg_xy(2), lg_xy2(1) - lg_xy(1));
    lg = [lg_xy(1), lg_xy(2), lg_theta];
end
