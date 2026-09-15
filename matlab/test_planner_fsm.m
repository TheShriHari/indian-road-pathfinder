%% TEST_PLANNER_FSM  Rigorous verification of Phase 4 Planning, FSM, and Control.
%
% Tests:
%   1. Kinematic Feasibility & Curvature Bound (|kappa(s)| <= 0.2222 m^-1)
%   2. Replanning Latency Benchmark across 50 iterations (Mean < 30ms, P99 < 50ms)
%   3. FSM Spatial Hysteresis Verification (+0.5m unlatch at 3.05m)
%   4. Dynamic Squeeze Full Cycle (CRUISE -> YIELD_DECEL -> YIELD_WAIT -> RESUME -> CRUISE)
%   5. Longitudinal Jerk Clamping (|j(t)| < 1.0 m/s^3)
%   6. Steering Slew Rate Limiting (|d_delta/dt| <= 15 deg/s)
%
% NOTE: All assertions throw terminating error() calls for headless batch execution.

clear; clc;
fprintf('=================================================================\n');
fprintf('  PHASE 4: KINEMATIC PLANNING & BEHAVIORAL ARBITRATION TEST SUITE\n');
fprintf('=================================================================\n\n');

% Compute repo root and ensure .tmp exists
script_path = mfilename('fullpath');
if isempty(script_path)
    repo_root = pwd;
else
    repo_root = fileparts(fileparts(script_path));
end
addpath(fullfile(repo_root, 'matlab'));
addpath(pwd);

tmp_dir = fullfile(repo_root, '.tmp');
if ~exist(tmp_dir, 'dir')
    mkdir(tmp_dir);
end

metrics = struct();

%% ── Test 1: Kinematic Curvature Bounds (|kappa| <= 0.2222 m^-1) ───────────
fprintf('--- [TEST 1] Kinematic Feasibility & Curvature Bound ---\n');

% Construct a realistic costmap with an offset obstacle requiring detour
ego_pose = [0.0, 0.0, 0.0];
goal_pose = [35.0, -1.2, 0.0];

sensor_det = struct();
sensor_det.road_boundaries = [
    [(-10:50)', repmat(-2.5, 61, 1)];
    [(-10:50)', repmat( 2.5, 61, 1)]
];
sensor_det.potholes = [struct('x', 15.0, 'y', 0.3, 'radius', 0.8)];
sensor_det.static_boxes = [];
sensor_det.static_points = [];

map_cfg.grid_res  = 0.2;
map_cfg.range_fwd = 50.0;
map_cfg.range_bwd = 10.0;
map_cfg.range_lat = 15.0;

[test_cmap, gmeta] = local_occupancy_grid_builder(ego_pose, sensor_det, map_cfg);
grid_org = [gmeta.x_min, gmeta.y_min];

[path1, ~, lat1, ok1] = adaptive_path_planner(ego_pose, goal_pose, test_cmap, [], 0.2, grid_org);
assert(ok1, 'TEST 1 FAILED: Planner failed to find path around obstacle.');
assert(size(path1, 1) >= 20, 'TEST 1 FAILED: Path has insufficient waypoints.');

% Compute analytical curvature along the smoothed trajectory
dx  = gradient(path1(:, 1));
ddx = gradient(dx);
dy  = gradient(path1(:, 2));
ddy = gradient(dy);

kappa1 = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(1.5) + 1e-6);
max_kappa = max(abs(kappa1));

fprintf('  Max Curvature Observed : %.4f m^-1 (Limit: <= 0.2222 m^-1)\n', max_kappa);
fprintf('  Planner Latency        : %.2f ms\n', lat1);
assert(max_kappa <= 0.2222 + 1e-4, ...
       sprintf('TEST 1 FAILED: Curvature exceeds 0.2222 m^-1 (got %.4f)', max_kappa));

metrics.kinematic_curvature_valid = true;
metrics.max_curvature_inv_m       = round(max_kappa, 4);
fprintf('  [PASS] Trajectory strictly satisfies Ackerman curvature limits.\n\n');


%% ── Test 2: Replanning Latency Benchmark across 50 Iterations ──────────────
fprintf('--- [TEST 2] Replanning Latency Benchmark (50 Iterations) ---\n');

% JIT Warmup (2 iterations) to prevent cold invocation artifacts
for w = 1:2
    adaptive_path_planner(ego_pose, goal_pose, test_cmap, [], 0.2, grid_org);
end

N_runs = 50;
latencies = zeros(N_runs, 1);

for r = 1:N_runs
    % Shift start pose and obstacle slightly to simulate rolling online replanning
    s_x = mod(r * 0.4, 8.0);
    s_pose = [s_x, 0.0, 0.0];
    g_pose = [s_x + 30.0, -1.0, 0.0];
    
    [~, ~, lat_r, plan_ok_r] = adaptive_path_planner(s_pose, g_pose, test_cmap, [], 0.2, grid_org);
    assert(plan_ok_r, sprintf('TEST 2 FAILED: Search failed on iteration %d', r));
    latencies(r) = lat_r;
end

mean_lat = mean(latencies);
p99_lat  = prctile(latencies, 99);

fprintf('  Mean Replanning Latency: %.2f ms (Target: < 30.0 ms)\n', mean_lat);
fprintf('  P99 Replanning Latency : %.2f ms (Target: < 50.0 ms)\n', p99_lat);

assert(mean_lat < 30.0, sprintf('TEST 2 FAILED: Mean latency %.2f ms exceeds 30ms', mean_lat));
assert(p99_lat < 50.0,  sprintf('TEST 2 FAILED: P99 latency %.2f ms exceeds 50ms', p99_lat));

metrics.mean_replanning_latency_ms = round(mean_lat, 2);
metrics.p99_replanning_latency_ms  = round(p99_lat, 2);
fprintf('  [PASS] Replanning latency satisfies real-time < 50ms budget.\n\n');


%% ── Test 3: FSM Spatial Hysteresis (+0.5m unlatch at 3.05m) ───────────────
fprintf('--- [TEST 3] FSM Spatial Hysteresis Verification ---\n');

% Reset FSM state
behavior_state_machine([], [], [], 0.1, struct('reset', true));

ego_st = [10.0, 0.0, 0.0, 6.0];
fsm_state = 'CRUISE';

% Step 1: Road width narrows to 2.40m (< 2.55m critical) -> Enter NUDGE
p1 = struct('W_free', 2.40, 'virtual_stop_active', false);
[fsm_state, v1, ~] = behavior_state_machine(fsm_state, ego_st, [], 0.1, p1);
fprintf('  W_free = 2.40 m -> State: %s (v_ref = %.1f m/s)\n', fsm_state, v1);
assert(strcmp(fsm_state, 'NUDGE'), 'TEST 3 FAILED: Expected transition to NUDGE at W_free = 2.40m');
assert(v1 == 4.0, 'TEST 3 FAILED: Expected v_ref = 4.0 m/s in NUDGE');

% Step 2: Road width widens to 2.70m in [2.55, 3.05) -> Must STAY in NUDGE
p2 = struct('W_free', 2.70, 'virtual_stop_active', false);
[fsm_state, v2, ~] = behavior_state_machine(fsm_state, ego_st, [], 0.1, p2);
fprintf('  W_free = 2.70 m -> State: %s (Hysteresis Latched)\n', fsm_state);
assert(strcmp(fsm_state, 'NUDGE'), 'TEST 3 FAILED: FSM chatter! Must stay in NUDGE at W_free = 2.70m');

% Step 3: Road width widens to 2.95m in [2.55, 3.05) -> Must STILL stay in NUDGE
p3 = struct('W_free', 2.95, 'virtual_stop_active', false);
[fsm_state, v3, ~] = behavior_state_machine(fsm_state, ego_st, [], 0.1, p3);
fprintf('  W_free = 2.95 m -> State: %s (Hysteresis Latched)\n', fsm_state);
assert(strcmp(fsm_state, 'NUDGE'), 'TEST 3 FAILED: FSM chatter! Must stay in NUDGE at W_free = 2.95m');

% Step 4: Road width reaches 3.10m (>= 3.05m unlatch) -> Return to CRUISE
p4 = struct('W_free', 3.10, 'virtual_stop_active', false);
[fsm_state, v4, ~] = behavior_state_machine(fsm_state, ego_st, [], 0.1, p4);
fprintf('  W_free = 3.10 m -> State: %s (v_ref = %.1f m/s)\n', fsm_state, v4);
assert(strcmp(fsm_state, 'CRUISE'), 'TEST 3 FAILED: Expected unlatch to CRUISE at W_free >= 3.05m');
assert(v4 == 8.0, 'TEST 3 FAILED: Expected v_ref = 8.0 m/s in CRUISE');

metrics.fsm_hysteresis_verified = true;
fprintf('  [PASS] Spatial hysteresis (+0.50m margin) strictly verified.\n\n');


%% ── Test 4: Dynamic Squeeze Full Cycle (CRUISE -> YIELD -> RESUME) ────────
fprintf('--- [TEST 4] Dynamic Squeeze Full Cycle Verification ---\n');

% Reset FSM state
behavior_state_machine([], [], [], 0.1, struct('reset', true));

st_test = 'CRUISE';
v_curr  = 6.0;

% Step 1: Oncoming vehicle creates dynamic bottleneck (VSL Active at 15m)
p_dyn1 = struct('virtual_stop_active', true, 'stop_line_dist', 15.0, 'W_free', 2.0);
[st_test, v_ref_d1, ~] = behavior_state_machine(st_test, [0, 0, 0, v_curr], [], 0.1, p_dyn1);
fprintf('  1. Bottleneck Detected -> State: %s (Approach v_ref = %.2f m/s)\n', st_test, v_ref_d1);
assert(strcmp(st_test, 'YIELD_DECEL'), 'TEST 4 FAILED: Must enter YIELD_DECEL on dynamic VSL.');

% Step 2: Vehicle decelerates to standstill near VSL (dist = 0.5m, v = 0.05 m/s)
p_dyn2 = struct('virtual_stop_active', true, 'stop_line_dist', 0.5, 'W_free', 2.0);
[st_test, v_ref_d2, ~] = behavior_state_machine(st_test, [14.5, 0, 0, 0.05], [], 0.1, p_dyn2);
fprintf('  2. Vehicle at VSL      -> State: %s (v_ref = %.2f m/s)\n', st_test, v_ref_d2);
assert(strcmp(st_test, 'YIELD_WAIT'), 'TEST 4 FAILED: Must enter YIELD_WAIT at standstill.');
assert(v_ref_d2 == 0.0, 'TEST 4 FAILED: Reference speed in YIELD_WAIT must be 0.0 m/s.');

% Step 3: Oncoming vehicle clears bottleneck -> VSL dropped
p_dyn3 = struct('virtual_stop_active', false, 'stop_line_dist', Inf, 'W_free', 2.2);
[st_test, v_ref_d3, ~] = behavior_state_machine(st_test, [14.5, 0, 0, 0.0], [], 0.1, p_dyn3);
fprintf('  3. Oncoming Cleared    -> State: %s (Launch v_ref = %.2f m/s)\n', st_test, v_ref_d3);
assert(strcmp(st_test, 'RESUME'), 'TEST 4 FAILED: Must enter RESUME when VSL clears.');

% Step 4: Pinch zone passed, clearance restored (W_free = 3.20m)
p_dyn4 = struct('virtual_stop_active', false, 'stop_line_dist', Inf, 'W_free', 3.20);
[st_test, v_ref_d4, ~] = behavior_state_machine(st_test, [25.0, 0, 0, 3.5], [], 0.1, p_dyn4);
fprintf('  4. Clearance Restored  -> State: %s (Cruise v_ref = %.2f m/s)\n', st_test, v_ref_d4);
assert(strcmp(st_test, 'CRUISE'), 'TEST 4 FAILED: Must return to CRUISE when clearance restored.');
assert(v_ref_d4 == 8.0, 'TEST 4 FAILED: Must restore nominal cruise speed.');

fprintf('  [PASS] Full yield-wait-resume cycle executed with zero chattering.\n\n');


%% ── Test 5: Longitudinal Jerk Clamping (|j(t)| < 1.0 m/s^3) ───────────────
fprintf('--- [TEST 5] Longitudinal Jerk Clamping (|j(t)| < 1.0 m/s^3) ---\n');

% Reset controller
pure_pursuit_controller([], [], [], struct('reset', true));

pp_path = [(0:0.5:50)', zeros(101, 1)];
ctrl_dt = 0.1;
ctrl_params = struct('dt', ctrl_dt, 'max_jerk', 0.90, 'reset', true);

% Step response from cruise speed (8.0 m/s) to full emergency braking (0.0 m/s)
v_sim = 8.0;
x_sim = 0.0;
N_steps = 40;
accel_hist = zeros(N_steps, 1);

for k = 1:N_steps
    sim_st = [x_sim, 0.0, 0.0, v_sim];
    ctrl_out = pure_pursuit_controller(sim_st, pp_path, 0.0, ctrl_params);
    accel_hist(k) = ctrl_out(2);
    
    % Update simulation
    v_sim = max(0.0, v_sim + ctrl_out(2) * ctrl_dt);
    x_sim = x_sim + v_sim * ctrl_dt;
end

% Compute discrete jerk: j = da / dt
jerks = abs(diff(accel_hist)) / ctrl_dt;
max_jerk = max(jerks);

fprintf('  Max Discrete Jerk: %.4f m/s^3 (Strict Bound: < 1.0 m/s^3)\n', max_jerk);
assert(max_jerk < 1.0, sprintf('TEST 5 FAILED: Jerk %.4f m/s^3 violates 1.0 m/s^3 bound.', max_jerk));

metrics.max_longitudinal_jerk_mps3 = round(max_jerk, 4);
fprintf('  [PASS] Longitudinal jerk strictly clamped below 1.0 m/s^3.\n\n');


%% ── Test 6: Steering Slew Rate Limiting (|d_delta/dt| <= 15 deg/s) ─────────
fprintf('--- [TEST 6] Steering Slew Rate Limiting (<= 15 deg/s) ---\n');

% Reset controller
pure_pursuit_controller([], [], [], struct('reset', true));

% Path sharply offset laterally by 2.0m to provoke max steering deflection
sharp_path = [(0:0.5:40)', [zeros(20, 1); repmat(2.0, 61, 1)]];
steer_hist = zeros(30, 1);
st_veh = [8.0, 0.0, 0.0, 5.0];

pp_cfg = struct('dt', 0.1, 'max_slew_rate', 15.0 * (pi / 180), 'reset', true);

for step_k = 1:30
    ctrl_k = pure_pursuit_controller(st_veh, sharp_path, 5.0, pp_cfg);
    steer_hist(step_k) = ctrl_k(1);
    
    % Advance vehicle kinematics
    st_veh(4) = 5.0;
    st_veh(1) = st_veh(1) + st_veh(4) * cos(st_veh(3)) * 0.1;
    st_veh(2) = st_veh(2) + st_veh(4) * sin(st_veh(3)) * 0.1;
    st_veh(3) = st_veh(3) + (st_veh(4) / 2.7) * tan(ctrl_k(1)) * 0.1;
end

steer_rates_degps = abs(diff(steer_hist)) / 0.1 * (180 / pi);
max_steer_rate = max(steer_rates_degps);

fprintf('  Max Steering Slew Rate: %.2f deg/s (Limit: <= 15.0 deg/s)\n', max_steer_rate);
assert(max_steer_rate <= 15.0 + 1e-3, ...
       sprintf('TEST 6 FAILED: Steering rate %.2f deg/s exceeds 15 deg/s', max_steer_rate));

metrics.max_steering_rate_degps = round(max_steer_rate, 2);
metrics.status                  = 'PASS';
fprintf('  [PASS] Steering slew rate strictly bounded to <= 15 deg/s.\n\n');


%% ── Export Structured Metrics to .tmp/planner_metrics.json ────────────────
json_str = sprintf(['{\n', ...
    '  "kinematic_curvature_valid": %s,\n', ...
    '  "max_curvature_inv_m": %.4f,\n', ...
    '  "mean_replanning_latency_ms": %.2f,\n', ...
    '  "p99_replanning_latency_ms": %.2f,\n', ...
    '  "fsm_hysteresis_verified": %s,\n', ...
    '  "max_longitudinal_jerk_mps3": %.4f,\n', ...
    '  "max_steering_rate_degps": %.2f,\n', ...
    '  "status": "PASS"\n', ...
    '}\n'], ...
    tf2str(metrics.kinematic_curvature_valid), ...
    metrics.max_curvature_inv_m, ...
    metrics.mean_replanning_latency_ms, ...
    metrics.p99_replanning_latency_ms, ...
    tf2str(metrics.fsm_hysteresis_verified), ...
    metrics.max_longitudinal_jerk_mps3, ...
    metrics.max_steering_rate_degps);

fid = fopen(fullfile(tmp_dir, 'planner_metrics.json'), 'w');
if fid ~= -1
    fwrite(fid, json_str);
    fclose(fid);
end

if ~strcmp(tmp_dir, fullfile(pwd, '.tmp'))
    local_tmp = fullfile(pwd, '.tmp');
    if ~exist(local_tmp, 'dir'), mkdir(local_tmp); end
    fid_local = fopen(fullfile(local_tmp, 'planner_metrics.json'), 'w');
    if fid_local ~= -1
        fwrite(fid_local, json_str);
        fclose(fid_local);
    end
end

fprintf('=================================================================\n');
fprintf('  ALL PHASE 4 PLANNER & FSM TESTS PASSED SUCCESSFULLY!\n');
fprintf('=================================================================\n');

function s = tf2str(b)
    if b
        s = 'true';
    else
        s = 'false';
    end
end
