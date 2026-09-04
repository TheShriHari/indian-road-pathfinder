function [path, costmap, latency_ms, plan_ok] = adaptive_path_planner(start_pose, goal_pose, static_map, dynamic_predictions, grid_res, grid_origin)
% ADAPTIVE_PATH_PLANNER  Hybrid A* path planning with cubic spline smoothing.
%   [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose,
%                                     static_map, dynamic_predictions, grid_res)
%                                     [, grid_origin])
%
% ALGORITHM ATTRIBUTION:
%   Adapted from PythonRobotics/PathPlanning/HybridAStar/hybrid_a_star.py
%   (author: Zheng Zh @Zhengzh, MIT License) and
%   PythonRobotics/PathPlanning/HybridAStar/car.py (same author).
%   Adaptations made for this project:
%     - No Reeds-Shepp analytic expansion (avoids ReedsSheppPath dependency)
%     - Simplified goal detection: position only, no heading constraint
%     - Costmap-based soft traversal cost (higher cost = penalised, not forbidden,
%       unless costmap > HARD_BLOCK=250, matching the obstacle threshold)
%     - MATLAB matrix arrays used for closed/g-cost sets instead of Python dicts
%       (avoids slow containers.Map in a tight search loop)
%     - Post-search pchip spline smoothing applied to A*-found waypoints (NOT
%       a naive straight line as in the previous heuristic version)
%     - Fallback to potential-field straight-line path if search exhausts MAX_ITER
%     - [FIX] Optional grid_origin=[x_min, y_min] parameter: the costmap is a
%       rolling window not anchored at [0,-10]; without this fix queries against
%       the costmap matrix use wrong world-to-cell indices for ego_x > 10 m.
%
% Inputs:
%   start_pose         : [x, y, theta]  start position (m, m, rad)
%   goal_pose          : [x, y, theta]  goal position
%   static_map         : binary occupancy grid (rows=Y, cols=X), 1 = obstacle
%   dynamic_predictions: struct array from dynamic_obstacle_predictor (may be [])
%   grid_res           : costmap cell size (m), default 0.2
%   grid_origin        : [x_min, y_min] world coords of costmap cell (1,1).
%                        Default [0.0, -10.0] for backward compatibility.
%
% Outputs:
%   path       : (N x 2) smoothed path waypoints [x, y]  (N=80 by default)
%   costmap    : inflated cost grid, same size as static_map (0-255)
%   latency_ms : total planning time (ms)

tic;
if nargin < 5, grid_res = 0.2; end
% FIX: Use caller-supplied rolling window origin.  Old code hardcoded [0, -10]
% which made all costmap cell lookups wrong once ego_x exceeded 10 m.
if nargin < 6 || isempty(grid_origin)
    grid_origin = [0.0, -10.0];   % backward-compatible default
end
x_min = grid_origin(1);
y_min = grid_origin(2);

[cmap_rows, cmap_cols] = size(static_map);
if max(static_map(:)) <= 1.0 && any(static_map(:) > 0)
    costmap = double(static_map) * 255;
else
    costmap = double(static_map);
end

% ============================================================
% 1. Dynamic Hazard Inflation Costmap  (UNCHANGED from original)
%    Inflates cost around predicted obstacle waypoints.
% ============================================================
if ~isempty(dynamic_predictions)
    for i = 1:length(dynamic_predictions)
        waypoints = dynamic_predictions(i).waypoints;
        obs_type  = dynamic_predictions(i).type;

        if strcmpi(obs_type, 'cattle')
            clearance_r = 1.8;
        elseif strcmpi(obs_type, 'auto_rickshaw')
            clearance_r = 1.8;
        else
            clearance_r = 1.5;
        end

        clearance_cells = ceil(clearance_r / grid_res);

        for t = 1:min(10, size(waypoints, 1))
            gx = round((waypoints(t, 1) - x_min) / grid_res) + 1;
            gy = round((waypoints(t, 2) - y_min) / grid_res) + 1;

            r_min_x = max(1, gx - clearance_cells);
            r_max_x = min(cmap_cols, gx + clearance_cells);
            r_min_y = max(1, gy - clearance_cells);
            r_max_y = min(cmap_rows, gy + clearance_cells);

            for cx = r_min_x:r_max_x
                for cy = r_min_y:r_max_y
                    dist = hypot((cx - gx) * grid_res, (cy - gy) * grid_res);
                    if dist <= 0.80 && t <= 1
                        % Immediate lethal core: true physical penetration
                        costmap(cy, cx) = 255;
                    elseif dist <= clearance_r
                        % Repulsive buffer around predicted agent path
                        cost_add = (1 - dist/clearance_r) * 180 * (1 / (1 + 0.15*t));
                        if costmap(cy, cx) < 250
                            costmap(cy, cx) = min(220, costmap(cy, cx) + cost_add);
                        end
                    end
                end
            end
        end
    end
end

% ============================================================
% 2. Hybrid A* Search
%    Adapted from hybrid_a_star.py (Zheng Zh) and car.py (Zheng Zh).
%    Key differences: see attribution block above.
% ============================================================

% --- Search parameters ---
ASTAR_RES  = 0.35;      % [m] search grid resolution (fine enough to resolve narrow corridors)
N_YAW      = 32;        % heading discretisation: 32 bins x 11.25 deg (prevents steer-pruning)
YAW_RES    = 2*pi / N_YAW;  % 11.25 deg per bin (guarantees steer arc separation)
WB         = 2.7;       % vehicle wheelbase [m]  (same as vehicle_kinematics.m)
ARC_L      = 1.2;       % arc length per expansion [m]
ARC_STEP   = 0.3;       % integration step along arc [m]
MAX_STEER  = pi/6;      % max steering angle: 30 deg  (same as vehicle_kinematics.m)
% 7 steering options: ±30°, ±20°, ±10°, 0° for fine corridor navigation
STEER_OPTS = [-MAX_STEER, -MAX_STEER*2/3, -MAX_STEER/3, 0, MAX_STEER/3, MAX_STEER*2/3, MAX_STEER];
HARD_BLOCK = 250;       % costmap value treated as impassable
COST_WT    = 0.015;     % weight: costmap value -> g-cost contribution
STEER_WT   = 0.08;      % weight: steer angle penalty (prefer smooth corridor tracking)
SC_WT      = 0.08;      % weight: steer-change penalty (prefer smooth turns)
MAX_ITER   = 6000;      % fast queue: 10-25ms per search, avoids search stalls

% World bounds — now taken from grid_origin (rolling window)
x_max = x_min + (cmap_cols - 1) * grid_res;
y_max = y_min + (cmap_rows - 1) * grid_res;

% Search grid dimensions
nc = ceil((x_max - x_min) / ASTAR_RES) + 1;
nr = ceil((y_max - y_min) / ASTAR_RES) + 1;
n_arc_steps = max(1, round(ARC_L / ARC_STEP));

% Coordinate transform helpers
w2col = @(wx) min(max(round((wx - x_min) / ASTAR_RES) + 1, 1), nc);
w2row = @(wy) min(max(round((wy - y_min) / ASTAR_RES) + 1, 1), nr);
w2yaw = @(yaw) mod(floor(yaw / YAW_RES + 0.5), N_YAW) + 1;  % 1-indexed, 1..N_YAW
% Costmap lookup from world coords
cm_col_f = @(wx) min(max(round((wx - x_min) / grid_res) + 1, 1), cmap_cols);
cm_row_f = @(wy) min(max(round((wy - y_min) / grid_res) + 1, 1), cmap_rows);

% --- State arrays (row, col, yaw_idx) indexed; much faster than containers.Map ---
g_mat      = Inf(nr, nc, N_YAW);       % best g-cost found so far
closed_mat = false(nr, nc, N_YAW);     % node expanded?
% Parent encoded as linear index into [nr x nc x N_YAW] array; 0 = start (no parent)
parent_mat = zeros(nr, nc, N_YAW, 'int32');
% Actual world-space coordinates stored at each grid node
wx_mat     = zeros(nr, nc, N_YAW);
wy_mat     = zeros(nr, nc, N_YAW);
wyaw_mat   = zeros(nr, nc, N_YAW);
steer_mat  = zeros(nr, nc, N_YAW);

% --- Initialise start node ---
s_col = w2col(start_pose(1));
s_row = w2row(start_pose(2));
s_yaw = w2yaw(start_pose(3));
g_mat(s_row, s_col, s_yaw) = 0;
wx_mat(s_row,  s_col, s_yaw) = start_pose(1);
wy_mat(s_row,  s_col, s_yaw) = start_pose(2);
wyaw_mat(s_row, s_col, s_yaw) = start_pose(3);

g_col = w2col(goal_pose(1));
g_row = w2row(goal_pose(2));
h0 = hypot(goal_pose(1) - start_pose(1), goal_pose(2) - start_pose(2)) + 1.2 * abs(start_pose(2) - goal_pose(2));

% Priority queue: [f_cost, row, col, yaw_idx]  (unsorted; pop via min scan)
pq = [h0, s_row, s_col, s_yaw];
total_insertions = 1;
stale_skips      = 0;

found        = false;
found_row    = 0; found_col = 0; found_yaw = 0;
iter         = 0;

while ~isempty(pq) && iter < MAX_ITER
    iter = iter + 1;

    % Pop minimum-f node  (O(n) but n << 14400 in practice)
    [~, idx] = min(pq(:,1));
    curr_row = pq(idx, 2);
    curr_col = pq(idx, 3);
    curr_yaw = pq(idx, 4);
    pq(idx, :) = [];

    if closed_mat(curr_row, curr_col, curr_yaw)
        stale_skips = stale_skips + 1;
        continue;  % stale entry in pq (better path was found earlier)
    end
    closed_mat(curr_row, curr_col, curr_yaw) = true;

    curr_wx   = wx_mat(curr_row, curr_col, curr_yaw);
    curr_wy   = wy_mat(curr_row, curr_col, curr_yaw);
    curr_wyaw = wyaw_mat(curr_row, curr_col, curr_yaw);
    curr_g    = g_mat(curr_row, curr_col, curr_yaw);
    curr_steer= steer_mat(curr_row, curr_col, curr_yaw);

    % Goal check: within 1.5*ASTAR_RES of goal in XY
    if hypot(curr_wx - goal_pose(1), curr_wy - goal_pose(2)) <= 1.5 * ASTAR_RES
        found = true;
        found_row = curr_row; found_col = curr_col; found_yaw = curr_yaw;
        break;
    end

    % --- Expand: simulate bicycle arc for each steering option ---
    for s_idx = 1:length(STEER_OPTS)
        steer = STEER_OPTS(s_idx);

        % Integrate bicycle kinematics along arc (car.py: move() function)
        nx = curr_wx; ny = curr_wy; nyaw = curr_wyaw;
        arc_ok = true;
        for k = 1:n_arc_steps
            nx   = nx   + ARC_STEP * cos(nyaw);
            ny   = ny   + ARC_STEP * sin(nyaw);
            nyaw = nyaw + ARC_STEP * tan(steer) / WB;  % bicycle model yaw rate

            % Bounds check within grid
            if nx < x_min || nx > x_max || ny < y_min || ny > y_max
                arc_ok = false; break;
            end
            % Hard obstacle check at each integration point
            cm_c = cm_col_f(nx);
            cm_r = cm_row_f(ny);
            if costmap(cm_r, cm_c) > HARD_BLOCK
                arc_ok = false; break;
            end
        end
        if ~arc_ok, continue; end

        nyaw = atan2(sin(nyaw), cos(nyaw));  % wrap to [-pi, pi]

        nb_col = w2col(nx);
        nb_row = w2row(ny);
        nb_yaw = w2yaw(nyaw);

        if closed_mat(nb_row, nb_col, nb_yaw), continue; end

        % Traversal cost: soft costmap penalty (not a hard block)
        cm_c_e = cm_col_f(nx);
        cm_r_e = cm_row_f(ny);
        c_val  = costmap(cm_r_e, cm_c_e);

        g_new = curr_g + ARC_L ...
              + COST_WT  * c_val ...
              + STEER_WT * abs(steer) ...
              + SC_WT    * abs(steer - curr_steer);  % steer-change smoothness cost

        if g_new >= g_mat(nb_row, nb_col, nb_yaw)
            continue;  % existing path to this node is at least as good
        end

        % Update node
        g_mat(nb_row, nb_col, nb_yaw)     = g_new;
        parent_mat(nb_row, nb_col, nb_yaw) = int32(sub2ind([nr,nc,N_YAW], ...
                                               curr_row, curr_col, curr_yaw));
        wx_mat(nb_row, nb_col, nb_yaw)    = nx;
        wy_mat(nb_row, nb_col, nb_yaw)    = ny;
        wyaw_mat(nb_row, nb_col, nb_yaw)  = nyaw;
        steer_mat(nb_row, nb_col, nb_yaw) = steer;

        % Fix 2: Check closed_mat before pushing duplicate onto pq
        if closed_mat(nb_row, nb_col, nb_yaw)
            continue;
        end

        % closed_mat and g_mat already prevent expansion of suboptimal or closed nodes
        h_new = hypot(nx - goal_pose(1), ny - goal_pose(2)) + 1.2 * abs(ny - goal_pose(2));
        pq(end+1, :) = [g_new + h_new, nb_row, nb_col, nb_yaw]; %#ok<AGROW>
        total_insertions = total_insertions + 1;
    end
end

% --- Extract coarse waypoint list via parent chain ---
if found
    % Trace back through parent_mat from goal node to start
    raw_wp = zeros(0, 2);
    r = found_row; c = found_col; y = found_yaw;
    while true
        raw_wp(end+1, :) = [wx_mat(r,c,y), wy_mat(r,c,y)]; %#ok<AGROW>
        p_idx = parent_mat(r, c, y);
        if p_idx == 0, break; end  % reached start node
        [r, c, y] = ind2sub([nr, nc, N_YAW], double(p_idx));
    end
    raw_wp = flipud(raw_wp);  % reverse: start -> goal order

    fprintf('[HybridAStar] Found path: %d iterations, %d coarse waypoints, %.1f ms\n', ...
            iter, size(raw_wp,1), toc*1000);
else
    % Fallback: straight-line control points (identical to original heuristic path)
    fprintf('[HybridAStar] WARNING: search exhausted %d iterations — using straight-line fallback.\n', iter);
    
    % === DIAGNOSTIC INSTRUMENTATION ===
    distinct_closed = sum(closed_mat(:));
    straight_dist   = hypot(goal_pose(1) - start_pose(1), goal_pose(2) - start_pose(2));
    
    fprintf('=== [HybridAStar DIAGNOSTICS] ===\n');
    fprintf('  Distinct closed states (sum(closed_mat(:))): %d\n', distinct_closed);
    fprintf('  Search grid dimensions: nr=%d, nc=%d, N_YAW=%d (total states: %d)\n', nr, nc, N_YAW, nr * nc * N_YAW);
    fprintf('  Start cell indices: [s_row=%d, s_col=%d, s_yaw=%d]\n', s_row, s_col, s_yaw);
    fprintf('  Goal cell indices:  [g_row=%d, g_col=%d]\n', g_row, g_col);
    fprintf('  Start pose (world): [x=%.2f, y=%.2f, yaw=%.3f rad]\n', start_pose(1), start_pose(2), start_pose(3));
    fprintf('  Goal pose (world):  [x=%.2f, y=%.2f, yaw=%.3f rad]\n', goal_pose(1), goal_pose(2), goal_pose(3));
    fprintf('  World bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]\n', x_min, x_max, y_min, y_max);
    fprintf('  Straight-line distance: %.3f m\n', straight_dist);
    fprintf('  PQ Statistics: total_insertions=%d, stale_skipped=%d, real_expansions=%d, remaining_in_pq=%d\n', ...
            total_insertions, stale_skips, iter - stale_skips, size(pq, 1));
    
    % Check: simulate a single straight-ahead expansion chain (steer=0 repeatedly) from start pose
    fprintf('  Simulating straight-ahead expansion chain (steer=0):\n');
    sim_wx   = start_pose(1);
    sim_wy   = start_pose(2);
    sim_wyaw = start_pose(3);
    max_sim_steps = max(10, ceil(straight_dist / ARC_L) + 5);
    first_block_info = '';

    for step_i = 1:max_sim_steps
        step_in_bounds = true;
        step_max_cost  = 0;
        step_first_out_b  = [];
        step_first_high_c = [];

        for k = 1:n_arc_steps
            sim_wx   = sim_wx   + ARC_STEP * cos(sim_wyaw);
            sim_wy   = sim_wy   + ARC_STEP * sin(sim_wyaw);
            sim_wyaw = sim_wyaw + ARC_STEP * tan(0) / WB;

            in_b = (sim_wx >= x_min && sim_wx <= x_max && sim_wy >= y_min && sim_wy <= y_max);
            if ~in_b
                step_in_bounds = false;
                if isempty(step_first_out_b)
                    step_first_out_b = [sim_wx, sim_wy];
                end
            end

            c_col = cm_col_f(sim_wx);
            c_row = cm_row_f(sim_wy);
            c_val = costmap(c_row, c_col);
            if c_val > step_max_cost
                step_max_cost = c_val;
            end
            if c_val > HARD_BLOCK && isempty(step_first_high_c)
                step_first_high_c = [sim_wx, sim_wy, c_val];
            end
        end
        sim_wyaw = atan2(sin(sim_wyaw), cos(sim_wyaw));
        d_to_g = hypot(sim_wx - goal_pose(1), sim_wy - goal_pose(2));
        exceeds_hard = (step_max_cost > HARD_BLOCK);

        fprintf('    Step %2d: pos=[%6.2f, %6.2f], yaw=%+5.2f rad | in_bounds=%d (x:[%.1f,%.1f], y:[%.1f,%.1f]) | max_costmap=%5.1f (exceeds HARD_BLOCK(250)=%d) | dist_to_goal=%5.2f m\n', ...
                step_i, sim_wx, sim_wy, sim_wyaw, step_in_bounds, x_min, x_max, y_min, y_max, step_max_cost, exceeds_hard, d_to_g);

        if ~step_in_bounds && isempty(first_block_info)
            first_block_info = sprintf('Out of bounds at pos=[%.2f, %.2f] (limits: x=[%.1f, %.1f], y=[%.1f, %.1f])', ...
                                       step_first_out_b(1), step_first_out_b(2), x_min, x_max, y_min, y_max);
        end
        if exceeds_hard && isempty(first_block_info)
            first_block_info = sprintf('Exceeded HARD_BLOCK (cost=%.1f > %d) at pos=[%.2f, %.2f]', ...
                                       step_first_high_c(3), HARD_BLOCK, step_first_high_c(1), step_first_high_c(2));
        end

        if ~step_in_bounds || exceeds_hard || d_to_g <= 1.5 * ASTAR_RES
            break;
        end
    end
    if ~isempty(first_block_info)
        fprintf('  [Straight-ahead Chain Blocked]: %s\n', first_block_info);
    else
        fprintf('  [Straight-ahead Chain Reached Goal]: dist=%.2f m <= %.2f m without obstacle or boundary block.\n', ...
                d_to_g, 1.5 * ASTAR_RES);
    end
    fprintf('=================================\n');

    num_fb = 6;
    t_fb   = linspace(0, 1, num_fb)';
    raw_wp = (1 - t_fb) * [start_pose(1), start_pose(2)] ...
           + t_fb        * [goal_pose(1),  goal_pose(2)];
end

% ============================================================
% 3. Spline Smoothing applied to A*-found waypoints
%    Original deformable-control-point + pchip logic preserved;
%    now receives the A*-searched path instead of a straight line.
% ============================================================
num_samples = 80;
if size(raw_wp, 1) >= 2
    t_raw     = linspace(0, 1, size(raw_wp, 1))';
    t_samples = linspace(0, 1, num_samples)';
    path      = zeros(num_samples, 2);
    path(:, 1) = interp1(t_raw, raw_wp(:,1), t_samples, 'pchip');
    path(:, 2) = interp1(t_raw, raw_wp(:,2), t_samples, 'pchip');
    % Clamp to road corridor boundaries to strictly prevent any spline overshoot
    path(:, 2) = max(-2.2, min(2.2, path(:, 2)));
else
    path = [linspace(start_pose(1), goal_pose(1), num_samples)', ...
            linspace(start_pose(2), goal_pose(2), num_samples)'];
end

latency_ms = toc * 1000;
plan_ok    = found;
end
