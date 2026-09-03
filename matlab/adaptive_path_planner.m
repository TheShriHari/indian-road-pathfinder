function [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose, static_map, dynamic_predictions, grid_res)
% ADAPTIVE_PATH_PLANNER  Hybrid A* path planning with cubic spline smoothing.
%   [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose,
%                                     static_map, dynamic_predictions, grid_res)
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
%
% Inputs:
%   start_pose         : [x, y, theta]  start position (m, m, rad)
%   goal_pose          : [x, y, theta]  goal position
%   static_map         : binary occupancy grid (rows=Y, cols=X), 1 = obstacle
%   dynamic_predictions: struct array from dynamic_obstacle_predictor (may be [])
%   grid_res           : costmap cell size (m), default 0.2
%
% Outputs:
%   path       : (N x 2) smoothed path waypoints [x, y]  (N=80 by default)
%   costmap    : inflated cost grid, same size as static_map (0-255)
%   latency_ms : total planning time (ms)

tic;
if nargin < 5, grid_res = 0.2; end

[cmap_rows, cmap_cols] = size(static_map);
costmap = double(static_map) * 255;

% ============================================================
% 1. Dynamic Hazard Inflation Costmap  (UNCHANGED from original)
%    Inflates cost around predicted obstacle waypoints.
% ============================================================
if ~isempty(dynamic_predictions)
    for i = 1:length(dynamic_predictions)
        waypoints = dynamic_predictions(i).waypoints;
        obs_type  = dynamic_predictions(i).type;

        if strcmpi(obs_type, 'cattle')
            clearance_r = 2.2;
        elseif strcmpi(obs_type, 'auto_rickshaw')
            clearance_r = 1.8;
        else
            clearance_r = 1.4;
        end

        clearance_cells = ceil(clearance_r / grid_res);

        for t = 1:min(10, size(waypoints, 1))
            gx = round(waypoints(t, 1) / grid_res);
            gy = round((waypoints(t, 2) + 10) / grid_res);

            r_min_x = max(1, gx - clearance_cells);
            r_max_x = min(cmap_cols, gx + clearance_cells);
            r_min_y = max(1, gy - clearance_cells);
            r_max_y = min(cmap_rows, gy + clearance_cells);

            for cx = r_min_x:r_max_x
                for cy = r_min_y:r_max_y
                    dist = hypot((cx-gx)*grid_res, (cy-gy)*grid_res - 10);
                    if dist <= clearance_r
                        cost_add = (1 - dist/clearance_r) * 200 * (1 / (1 + 0.1*t));
                        costmap(cy, cx) = min(255, costmap(cy, cx) + cost_add);
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
ASTAR_RES  = 1.0;       % [m] search grid resolution (coarser than 0.2m costmap)
N_YAW      = 12;        % heading discretisation: 12 bins x 30 deg = 360 deg
YAW_RES    = 2*pi / N_YAW;  % 30 deg per bin
WB         = 2.7;       % vehicle wheelbase [m]  (same as vehicle_kinematics.m)
ARC_L      = 1.5;       % arc length per expansion [m]
ARC_STEP   = 0.5;       % integration step along arc [m]
MAX_STEER  = pi/6;      % max steering angle: 30 deg  (same as vehicle_kinematics.m)
% 5 steering options: ±30°, ±15°, 0°  (ref: N_STEER in hybrid_a_star.py is 20;
% we use 5 to keep MATLAB search tractable without Reeds-Shepp)
STEER_OPTS = [-MAX_STEER, -MAX_STEER/2, 0, MAX_STEER/2, MAX_STEER];
HARD_BLOCK = 250;       % costmap value treated as impassable
COST_WT    = 0.015;     % weight: costmap value -> g-cost contribution
STEER_WT   = 0.25;      % weight: steer angle penalty (prefer straight driving)
SC_WT      = 0.15;      % weight: steer-change penalty (prefer smooth turns)
MAX_ITER   = 40000;     % max iterations before giving up and falling back

% World bounds
x_min = 0.0;
y_min = -10.0;
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
h0 = hypot(goal_pose(1) - start_pose(1), goal_pose(2) - start_pose(2));

% Priority queue: [f_cost, row, col, yaw_idx]  (unsorted; pop via min scan)
pq = [h0, s_row, s_col, s_yaw];

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

            % Bounds check
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

        h_new = hypot(nx - goal_pose(1), ny - goal_pose(2));
        pq(end+1, :) = [g_new + h_new, nb_row, nb_col, nb_yaw]; %#ok<AGROW>
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
% Resample raw_wp to num_ctrl evenly-spaced control points
num_ctrl  = max(6, size(raw_wp, 1));
t_ctrl    = linspace(0, 1, num_ctrl)';
t_raw     = linspace(0, 1, size(raw_wp, 1))';

if size(raw_wp, 1) >= 2
    ctrl_x = interp1(t_raw, raw_wp(:,1), t_ctrl, 'linear', 'extrap');
    ctrl_y = interp1(t_raw, raw_wp(:,2), t_ctrl, 'linear', 'extrap');
else
    ctrl_x = linspace(start_pose(1), goal_pose(1), num_ctrl)';
    ctrl_y = linspace(start_pose(2), goal_pose(2), num_ctrl)';
end
raw_ctrl = [ctrl_x, ctrl_y];

% Push interior control points along potential field gradient (UNCHANGED)
for k = 2:(num_ctrl-1)
    pt = raw_ctrl(k, :);
    gx = round(pt(1)/grid_res);
    gy = round((pt(2) + 10)/grid_res);

    if gx >= 3 && gx <= cmap_cols-2 && gy >= 3 && gy <= cmap_rows-2
        grad_x = costmap(gy, gx+2) - costmap(gy, gx-2);
        grad_y = costmap(gy+2, gx) - costmap(gy-2, gx);
        raw_ctrl(k, 1) = raw_ctrl(k, 1) - 0.12 * grad_x * grid_res;
        raw_ctrl(k, 2) = raw_ctrl(k, 2) - 0.12 * grad_y * grid_res;
    end
end

% Cubic Catmull-Rom / pchip spline interpolation (UNCHANGED)
num_samples = 80;
t_samples   = linspace(0, 1, num_samples)';
path        = zeros(num_samples, 2);
path(:, 1)  = interp1(t_ctrl, raw_ctrl(:,1), t_samples, 'pchip');
path(:, 2)  = interp1(t_ctrl, raw_ctrl(:,2), t_samples, 'pchip');

latency_ms = toc * 1000;
end
