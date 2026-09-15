%% test_hybrid_astar_detour.m
% Explicit Hybrid A* obstacle-detour test (Component 3 verification).
%
% Setup: start=[2,0,0], goal=[55,0,0].
%        A large WALL obstacle (width=4m, height=14m) is placed DIRECTLY on the
%        straight line between start and goal at x=30, centred on y=0.
%        The straight line is completely blocked — any path must detour
%        to y > 7 m or y < -7 m to get around the wall.
%
% Test: Run adaptive_path_planner. Print all returned waypoints.
%       Pass criterion: at least one waypoint has abs(y) > 3.5 m,
%       proving the planner found a real detour rather than clipping through.
%
% Run: test_hybrid_astar_detour

clear; clc;

% Reset EKF state (not needed but good hygiene)
clear dynamic_obstacle_predictor;

fprintf('==========================================================\n');
fprintf('  HYBRID A* DETOUR TEST — Large Blocking Wall Obstacle   \n');
fprintf('==========================================================\n');
fprintf('  Start: [2, 0, 0 rad]   Goal: [55, 0, 0 rad]           \n');
fprintf('  Wall : x=28..32, y=-7..+7  (fully blocks direct path)  \n\n');

GRID_RES = 0.2;  % m per cell
X_MIN = 0; X_MAX = 60;
Y_MIN = -10; Y_MAX = 10;

nX = round((X_MAX - X_MIN) / GRID_RES);  % 300
nY = round((Y_MAX - Y_MIN) / GRID_RES);  % 100

% Build a static map with a large rectangular wall obstacle
static_map = zeros(nY, nX);

% Road edges (standard scenario bounds)
static_map(1:20,  :) = 1;   % y < -8
static_map(81:nY, :) = 1;   % y >  8

% WALL: x in [28, 32], y in [-7, +7]  — completely blocks direct straight line
wall_x_lo = 28; wall_x_hi = 32;
wall_y_lo = -7; wall_y_hi =  7;
col_lo = max(1,  round((wall_x_lo - X_MIN) / GRID_RES) + 1);
col_hi = min(nX, round((wall_x_hi - X_MIN) / GRID_RES) + 1);
row_lo = max(1,  round((wall_y_lo - Y_MIN) / GRID_RES) + 1);
row_hi = min(nY, round((wall_y_hi - Y_MIN) / GRID_RES) + 1);
static_map(row_lo:row_hi, col_lo:col_hi) = 1;

fprintf('  Wall spans: rows %d-%d  cols %d-%d  (at %.1f m resolution)\n', ...
        row_lo, row_hi, col_lo, col_hi, GRID_RES);

% Verify start and goal are not in obstacle
s_col = round((2.0 - X_MIN)/GRID_RES) + 1;
s_row = round((0.0 - Y_MIN)/GRID_RES) + 1;
g_col = round((55.0 - X_MIN)/GRID_RES) + 1;
g_row = round((0.0  - Y_MIN)/GRID_RES) + 1;
assert(~static_map(s_row, s_col), 'Start overlaps obstacle — fix test setup!');
assert(~static_map(g_row, g_col), 'Goal overlaps obstacle — fix test setup!');
fprintf('  Start cell free: YES  |  Goal cell free: YES\n');

% Run planner (no dynamic predictions)
start_pose = [2.0, 0.0, 0.0];
goal_pose  = [55.0, 0.0, 0.0];

fprintf('\n  Running adaptive_path_planner (Hybrid A*)...\n');
[path, ~, latency_ms] = adaptive_path_planner(start_pose, goal_pose, static_map, [], GRID_RES);

fprintf('\n  Planning latency: %.2f ms\n', latency_ms);
fprintf('  Path has %d waypoints.\n\n', size(path,1));

% Print all waypoints
fprintf('  %-5s  %-10s  %-10s\n', 'idx', 'x (m)', 'y (m)');
fprintf('  %s\n', repmat('-', 1, 28));
for k = 1:size(path,1)
    in_wall = (path(k,1) >= wall_x_lo && path(k,1) <= wall_x_hi && ...
               path(k,2) >= wall_y_lo && path(k,2) <= wall_y_hi);
    flag = '';
    if in_wall, flag = '  *** INSIDE WALL ***'; end
    fprintf('  %-5d  %-10.4f  %-10.4f%s\n', k, path(k,1), path(k,2), flag);
end

% ---- Evaluation ----
max_lat_deviation = max(abs(path(:,2)));
any_in_wall       = any(path(:,1) >= wall_x_lo & path(:,1) <= wall_x_hi & ...
                        path(:,2) >= wall_y_lo & path(:,2) <= wall_y_hi);
detour_happened   = max_lat_deviation > 3.5;  % lateral deviation > 3.5 m from centreline

fprintf('\n==========================================================\n');
fprintf('  DETOUR TEST RESULTS\n');
fprintf('==========================================================\n');
fprintf('  Max lateral deviation from y=0   : %.4f m\n', max_lat_deviation);
fprintf('  Any waypoint inside wall?         : %s\n', tf_str(any_in_wall));
fprintf('  Detour exceeds 3.5 m laterally?  : %s\n', tf_str(detour_happened));

if detour_happened && ~any_in_wall
    fprintf('\n  PASS: Planner found a real detour around the blocking wall.\n');
elseif any_in_wall
    fprintf('\n  FAIL: Path clips through the wall — search did not work correctly.\n');
else
    fprintf('\n  PARTIAL: Path avoided wall but detour smaller than expected.\n');
end
fprintf('==========================================================\n\n');

function s = tf_str(b)
    if b, s = 'YES'; else, s = 'NO'; end
end
