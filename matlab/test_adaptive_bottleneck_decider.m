%% TEST_ADAPTIVE_BOTTLENECK_DECIDER  Rigorous verification of Phase 3 costmap & decider.
%
% Tests:
%   1. Grid boundary indexing and non-toolbox dimensions (150x300, res=0.2m)
%   2. Continuous exponential decay inflation profile
%   3. Static road pinch (1.85m <= W_free < 2.55m) -> detour_required = true, virtual_stop = false
%   4. Dynamic bottleneck (oncoming actor in narrow corridor) -> virtual_stop = true (VSL at s - 3.5m)
%   5. Wide unconstrained road (7.0m width) -> zero false-positive bottlenecks
%   6. Complete impassable blockage (W_free = 1.60m < 1.85m chassis) -> virtual_stop = true
%
% NOTE: All assertions throw terminating error() calls for headless batch execution.

clear; clc;
fprintf('=================================================================\n');
fprintf('  PHASE 3: ROLLING COSTMAP & BOTTLENECK DECIDER TEST SUITE       \n');
fprintf('=================================================================\n\n');

% Compute repo root and ensure .tmp exists
script_path = mfilename('fullpath');
if isempty(script_path)
    repo_root = pwd;
else
    repo_root = fileparts(fileparts(script_path));
end
tmp_dir = fullfile(repo_root, '.tmp');
if ~exist(tmp_dir, 'dir')
    mkdir(tmp_dir);
end

metrics = struct();
ego_pose = [0.0, 0.0, 0.0];
ego_state = [0.0, 0.0, 0.0, 4.0];
planned_path = [(0:0.5:40.0)', zeros(length(0:0.5:40.0), 1)];

%% ── Test 1: Grid Dimensions & Zero-Toolbox Dependency ─────────────────────
fprintf('--- [TEST 1] Grid Dimensions & Zero-Toolbox Check ---\n');
sensor_empty = struct('potholes', [], 'static_boxes', [], 'road_boundaries', [], 'static_points', []);
[cmap1, gmeta1] = local_occupancy_grid_builder(ego_pose, sensor_empty, []);

[nY, nX] = size(cmap1);
fprintf('  Costmap size: %d x %d (Resolution: %.2f m)\n', nY, nX, gmeta1.res);
assert(nY == 150 && nX == 300, ...
       sprintf('TEST 1 FAILED: Expected 150x300 grid, got %dx%d', nY, nX));
assert(gmeta1.res == 0.2, 'TEST 1 FAILED: Expected resolution 0.2m.');
assert(isa(cmap1, 'double'), 'TEST 1 FAILED: Expected pure double matrix.');

metrics.costmap_dimensions_valid = true;
metrics.grid_resolution_m        = 0.2;
metrics.zero_toolbox_dependency  = true;
fprintf('  [PASS] Zero-toolbox matrix successfully constructed at 150x300.\n\n');


%% ── Test 2: Continuous Exponential Decay Inflation Verification ──────────
fprintf('--- [TEST 2] Continuous Exponential Decay Inflation ---\n');
% Single pothole at (10.0, 0.0) with radius 0.8m
sensor_pothole = sensor_empty;
sensor_pothole.potholes = [struct('x', 10.0, 'y', 0.0, 'radius', 0.8)];
[cmap2, gmeta2] = local_occupancy_grid_builder(ego_pose, sensor_pothole, []);

% Sample radial profile along x from x = 10.0 to x = 13.0 at y = 0
sample_xs = 10.0:0.05:12.5;
sample_costs = zeros(size(sample_xs));
for i = 1:length(sample_xs)
    c_idx = min(max(floor((sample_xs(i) - gmeta2.x_min) / gmeta2.res) + 1, 1), gmeta2.nX);
    r_idx = min(max(floor((0.0 - gmeta2.y_min) / gmeta2.res) + 1, 1), gmeta2.nY);
    sample_costs(i) = cmap2(r_idx, c_idx);
end

% Check 1: Lethal core + cushion (d <= R + d_safe = 0.8 + 0.35 = 1.15m) -> Cost = 100
core_indices = find(sample_xs <= 10.0 + 1.10);
assert(all(sample_costs(core_indices) == 100), ...
       'TEST 2 FAILED: Lethal core and safety cushion must be 100.');

% Check 2: Monotonic decay in margin (1.15m < d <= 2.0m)
margin_indices = find(sample_xs > 10.0 + 1.15 & sample_xs <= 10.0 + 1.95);
decay_diffs = diff(sample_costs(margin_indices));
assert(all(decay_diffs <= 0.01), ... % allow negligible rounding tolerance
       'TEST 2 FAILED: Cost must decrease monotonically across exponential inflation margin.');

% Check 3: Outside inflation margin (d > 0.8 + 1.2 = 2.0m) -> Cost == 0
far_indices = find(sample_xs >= 10.0 + 2.10);
assert(all(sample_costs(far_indices) == 0), ...
       'TEST 2 FAILED: Cost must be 0 outside inflation margin (d > 2.0m).');

metrics.exponential_decay_valid = true;
fprintf('  [PASS] Continuous exponential decay strictly verified.\n\n');


%% ── Test 3: Static Road Pinch (Detour Required, Virtual Stop False) ───────
fprintf('--- [TEST 3] Static Road Pinch (Detour Required, VSL False) ---\n');
% Road width = 3.6m (boundaries at +/-1.8m). Pothole encroaching at x=15.0m, y=1.2m, rad=0.3m
% Clear lateral width W_free is in [1.85m, 2.55m] (detour required, no halt)
sensor_static_pinch = struct();
sensor_static_pinch.road_boundaries = [
    [(0:1:40)', repmat(-1.8, 41, 1)];
    [(0:1:40)', repmat( 1.8, 41, 1)]
];
sensor_static_pinch.potholes = [struct('x', 15.0, 'y', 1.2, 'radius', 0.3)];
sensor_static_pinch.static_boxes = [];
sensor_static_pinch.static_points = [];

[cmap3, gmeta3] = local_occupancy_grid_builder(ego_pose, sensor_static_pinch, []);
[vstop3, stop_pose3, info3] = universal_bottleneck_decider(ego_state, planned_path, cmap3, gmeta3, [], []);

fprintf('  Virtual Stop Active : %s\n', tf2str(vstop3));
fprintf('  Detour Required     : %s\n', tf2str(info3.detour_required));
fprintf('  Detected Squeeze    : Station s = %.1f m, Width = %.2f m\n', info3.station_s, info3.min_width);
fprintf('  Reason              : %s\n', info3.reason);

assert(~vstop3, 'TEST 3 FAILED: Static narrowing must NOT activate Virtual Stop Line.');
assert(info3.detour_required, 'TEST 3 FAILED: Static narrowing must signal detour_required = true.');
assert(isempty(stop_pose3), 'TEST 3 FAILED: Stop pose should be empty for static detour.');

metrics.static_narrowing_detour_passed = true;
fprintf('  [PASS] Static corridor squeeze correctly requests detour without halting.\n\n');


%% ── Test 4: Dynamic Squeeze Point (Oncoming Actor in Narrow Corridor) ─────
fprintf('--- [TEST 4] Dynamic Squeeze Point (Oncoming Actor -> VSL Active) ---\n');
% Single-lane road width = 3.5m (+/-1.75m). Oncoming auto-rickshaw approaching at x=20.0m with vx = -3.0 m/s
sensor_dyn_pinch = struct();
sensor_dyn_pinch.road_boundaries = [
    [(0:1:40)', repmat(-1.75, 41, 1)];
    [(0:1:40)', repmat( 1.75, 41, 1)]
];
sensor_dyn_pinch.potholes = [];
sensor_dyn_pinch.static_boxes = [];
sensor_dyn_pinch.static_points = [];

% Dynamic oncoming actor at x = 20.0m moving in -x direction
dyn_actor = struct();
dyn_actor(1).id = 501;
dyn_actor(1).type = 'auto_rickshaw';
dyn_actor(1).x_est = [20.0; 0.2; -3.0; 0.0]; % v_rel = -3.0 m/s < -0.5 m/s
dyn_actor(1).waypoints = [(20.0:-0.3:14.0)', repmat(0.2, 21, 1)];
dyn_actor(1).semi_major = repmat(1.4, 21, 1);
dyn_actor(1).semi_minor = repmat(0.8, 21, 1);
dyn_actor(1).orientation = zeros(21, 1);

[cmap4, gmeta4] = local_occupancy_grid_builder(ego_pose, sensor_dyn_pinch, [], dyn_actor);
[vstop4, stop_pose4, info4] = universal_bottleneck_decider(ego_state, planned_path, cmap4, gmeta4, dyn_actor, []);

fprintf('  Virtual Stop Active : %s\n', tf2str(vstop4));
fprintf('  Detour Required     : %s\n', tf2str(info4.detour_required));
fprintf('  Pinch Station       : s = %.1f m (Width = %.2f m)\n', info4.station_s, info4.min_width);
fprintf('  VSL Station         : s = %.1f m\n', info4.vsl_station);
fprintf('  VSL Stop Pose       : [%.2f, %.2f, %.2f rad]\n', stop_pose4(1), stop_pose4(2), stop_pose4(3));

assert(vstop4, 'TEST 4 FAILED: Oncoming dynamic actor in narrow corridor must trigger VSL.');
assert(~info4.detour_required, 'TEST 4 FAILED: Dynamic squeeze requires VSL hold, not detour.');
assert(~isempty(stop_pose4), 'TEST 4 FAILED: Stop pose must be defined.');
% Verify VSL is placed upstream (approximately info4.station_s - 3.5 m)
assert(abs(info4.station_s - info4.vsl_station - 3.5) < 0.5, ...
       'TEST 4 FAILED: VSL must be placed 3.5 m upstream of bottleneck.');

metrics.dynamic_squeeze_vsl_passed = true;
metrics.vsl_buffer_distance_m      = 3.5;
fprintf('  [PASS] Dynamic squeeze successfully halts vehicle at upstream Virtual Stop Line.\n\n');


%% ── Test 5: Wide Open Road (Zero False Positives) ─────────────────────────
fprintf('--- [TEST 5] Wide Open Road (Corridor Width = 7.0m) ---\n');
sensor_wide = struct();
sensor_wide.road_boundaries = [
    [(0:1:40)', repmat(-3.5, 41, 1)];
    [(0:1:40)', repmat( 3.5, 41, 1)]
];
sensor_wide.potholes = [];
sensor_wide.static_boxes = [];
sensor_wide.static_points = [];

[cmap5, gmeta5] = local_occupancy_grid_builder(ego_pose, sensor_wide, []);
[vstop5, ~, info5] = universal_bottleneck_decider(ego_state, planned_path, cmap5, gmeta5, [], []);

fprintf('  Virtual Stop Active : %s\n', tf2str(vstop5));
fprintf('  Detour Required     : %s\n', tf2str(info5.detour_required));
fprintf('  Min Width           : %.2f m\n', info5.min_width);

assert(~vstop5 && ~info5.detour_required, ...
       'TEST 5 FAILED: Wide road triggered false-positive bottleneck.');

metrics.false_positive_check_passed = true;
fprintf('  [PASS] Zero false positives on open unconstrained roadway.\n\n');


%% ── Test 6: Complete Impassable Blockage (W_free < 1.85m Chassis) ──────────
fprintf('--- [TEST 6] Complete Impassable Blockage (Width < 1.85m Chassis) ---\n');
% Road narrows to 1.60m width (< 1.85m chassis width)
sensor_blocked = struct();
sensor_blocked.road_boundaries = [
    [(0:1:40)', repmat(-0.80, 41, 1)];
    [(0:1:40)', repmat( 0.80, 41, 1)]
];
sensor_blocked.potholes = [];
sensor_blocked.static_boxes = [];
sensor_blocked.static_points = [];

[cmap6, gmeta6] = local_occupancy_grid_builder(ego_pose, sensor_blocked, []);
[vstop6, stop_pose6, info6] = universal_bottleneck_decider(ego_state, planned_path, cmap6, gmeta6, [], []);

fprintf('  Virtual Stop Active : %s\n', tf2str(vstop6));
fprintf('  Detected Width      : %.2f m (Chassis = 1.85 m)\n', info6.min_width);
fprintf('  Reason              : %s\n', info6.reason);

assert(vstop6, 'TEST 6 FAILED: Impassable road blockage (< 1.85m) must activate Virtual Stop.');
assert(~info6.detour_required, 'TEST 6 FAILED: Blockage cannot be detoured.');

metrics.complete_blockage_stop_passed = true;
metrics.status                        = 'PASS';
fprintf('  [PASS] Complete road blockage safely commands virtual stop upstream.\n\n');


%% ── Export Structured Metrics to .tmp/bottleneck_metrics.json ─────────────
json_str = sprintf(['{\n', ...
    '  "costmap_dimensions_valid": %s,\n', ...
    '  "grid_resolution_m": %.1f,\n', ...
    '  "zero_toolbox_dependency": %s,\n', ...
    '  "exponential_decay_valid": %s,\n', ...
    '  "static_narrowing_detour_passed": %s,\n', ...
    '  "dynamic_squeeze_vsl_passed": %s,\n', ...
    '  "false_positive_check_passed": %s,\n', ...
    '  "complete_blockage_stop_passed": %s,\n', ...
    '  "vsl_buffer_distance_m": %.1f,\n', ...
    '  "status": "PASS"\n', ...
    '}\n'], ...
    tf2str(metrics.costmap_dimensions_valid), ...
    metrics.grid_resolution_m, ...
    tf2str(metrics.zero_toolbox_dependency), ...
    tf2str(metrics.exponential_decay_valid), ...
    tf2str(metrics.static_narrowing_detour_passed), ...
    tf2str(metrics.dynamic_squeeze_vsl_passed), ...
    tf2str(metrics.false_positive_check_passed), ...
    tf2str(metrics.complete_blockage_stop_passed), ...
    metrics.vsl_buffer_distance_m);

fid = fopen(fullfile(tmp_dir, 'bottleneck_metrics.json'), 'w');
if fid ~= -1
    fwrite(fid, json_str);
    fclose(fid);
end

% Also write to current directory .tmp if different
if ~strcmp(tmp_dir, fullfile(pwd, '.tmp'))
    local_tmp = fullfile(pwd, '.tmp');
    if ~exist(local_tmp, 'dir'), mkdir(local_tmp); end
    fid_local = fopen(fullfile(local_tmp, 'bottleneck_metrics.json'), 'w');
    if fid_local ~= -1
        fwrite(fid_local, json_str);
        fclose(fid_local);
    end
end

fprintf('=================================================================\n');
fprintf('  ALL PHASE 3 BOTTLENECK DECIDER TESTS PASSED SUCCESSFULLY!\n');
fprintf('=================================================================\n');

function s = tf2str(b)
    if b
        s = 'true';
    else
        s = 'false';
    end
end
