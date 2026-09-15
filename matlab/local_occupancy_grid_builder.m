function [local_costmap, grid_meta] = local_occupancy_grid_builder(ego_pose, sensor_detections, map_config, dynamic_predictions)
%% LOCAL_OCCUPANCY_GRID_BUILDER Builds a rolling local hazard costmap from real-time perception (Phase 3).
%   [local_costmap, grid_meta] = local_occupancy_grid_builder(ego_pose, sensor_detections, map_config, dynamic_predictions)
%
% Pure MATLAB matrix implementation (Zero-Toolbox dependency: no binaryOccupancyMap).
% Constructs a 150 x 300 grid (0.2 m resolution) covering [-10, +50] m forward and
% [-15, +15] m lateral. Superimposes road boundaries, continuous exponential inflation
% fields around static hazards, and dynamic confidence ellipses.
%
% Inputs:
%   ego_pose            : [x, y, theta] in world coordinates
%   sensor_detections   : struct containing perception streams:
%                           .potholes       : struct array with .x, .y, .radius
%                           .static_boxes   : struct array with .x, .y, .radius
%                           .road_boundaries: [N x 2] [x, y] coordinates of boundary markers
%                           .static_points  : [M x 2] raw LiDAR hits
%   map_config          : (optional) struct overriding defaults:
%                           .grid_res  : resolution (default: 0.2 m)
%                           .range_fwd : forward window (default: 50.0 m)
%                           .range_bwd : backward window (default: 10.0 m)
%                           .range_lat : lateral half-width (default: 15.0 m)
%   dynamic_predictions : (optional) struct array from dynamic_obstacle_predictor
%
% Outputs:
%   local_costmap       : [150 x 300] matrix with costs in [0, 100] (0 = free, 100 = lethal)
%   grid_meta           : struct with coordinate transform metadata

if nargin < 3 || isempty(map_config)
    map_config = struct();
end
if ~isfield(map_config, 'grid_res'),  map_config.grid_res  = 0.2;  end % 0.2 m/cell
if ~isfield(map_config, 'range_fwd'), map_config.range_fwd = 50.0; end % +50 m
if ~isfield(map_config, 'range_bwd'), map_config.range_bwd = 10.0; end % -10 m
if ~isfield(map_config, 'range_lat'), map_config.range_lat = 15.0; end % +/-15 m

if nargin < 4, dynamic_predictions = []; end
if isempty(dynamic_predictions) && isfield(sensor_detections, 'dynamic_predictions')
    dynamic_predictions = sensor_detections.dynamic_predictions;
end

res = map_config.grid_res;
x_min = ego_pose(1) - map_config.range_bwd;
x_max = ego_pose(1) + map_config.range_fwd;
y_min = ego_pose(2) - map_config.range_lat;
y_max = ego_pose(2) + map_config.range_lat;

nX = round((x_max - x_min) / res); % 300 columns
nY = round((y_max - y_min) / res); % 150 rows

local_costmap = zeros(nY, nX);

% Coordinate conversion closures (1-indexed matrix mapping)
world2col = @(wx) min(max(floor((wx - x_min) / res) + 1, 1), nX);
world2row = @(wy) min(max(floor((wy - y_min) / res) + 1, 1), nY);

grid_meta.x_min = x_min; grid_meta.x_max = x_max;
grid_meta.y_min = y_min; grid_meta.y_max = y_max;
grid_meta.res   = res;
grid_meta.nX    = nX;
grid_meta.nY    = nY;
grid_meta.world2grid = @(wx, wy) [world2row(wy), world2col(wx)];
grid_meta.grid2world = @(r, c) [x_min + (c - 0.5) * res, y_min + (r - 0.5) * res];

%% 1. Ingest Road Boundaries
if isfield(sensor_detections, 'road_boundaries') && ~isempty(sensor_detections.road_boundaries)
    rb = sensor_detections.road_boundaries;
    % Check if road boundaries are provided as left/right points
    rb_right = rb(rb(:, 2) < ego_pose(2), :);
    rb_left  = rb(rb(:, 2) >= ego_pose(2), :);
    
    col_xs = x_min + ((1:nX) - 0.5) * res;
    
    if ~isempty(rb_right) && size(rb_right, 1) >= 2
        y_r_interp = interp1(rb_right(:, 1), rb_right(:, 2), col_xs, 'linear', 'extrap');
    else
        y_r_interp = repmat(-2.35, 1, nX);
    end
    
    if ~isempty(rb_left) && size(rb_left, 1) >= 2
        y_l_interp = interp1(rb_left(:, 1), rb_left(:, 2), col_xs, 'linear', 'extrap');
    else
        y_l_interp = repmat(2.35, 1, nX);
    end
    
    row_ys = (y_min + ((1:nY)' - 0.5) * res); % [nY x 1]
    
    for c = 1:nX
        out_mask = (row_ys < y_r_interp(c)) | (row_ys > y_l_interp(c));
        local_costmap(out_mask, c) = 100;
    end
end

%% 2. Ingest Static Potholes with Continuous Exponential Inflation
if isfield(sensor_detections, 'potholes') && ~isempty(sensor_detections.potholes)
    for k = 1:length(sensor_detections.potholes)
        p = sensor_detections.potholes(k);
        apply_exponential_inflation(p.x, p.y, p.radius);
    end
end

%% 3. Ingest Static Obstacles (Debris, crates, barriers)
if isfield(sensor_detections, 'static_boxes') && ~isempty(sensor_detections.static_boxes)
    for k = 1:length(sensor_detections.static_boxes)
        box = sensor_detections.static_boxes(k);
        rad = 1.0;
        if isfield(box, 'radius'), rad = box.radius; end
        apply_exponential_inflation(box.x, box.y, rad);
    end
end

%% 4. Ingest Raw Point Cloud / LiDAR Hits
if isfield(sensor_detections, 'static_points') && ~isempty(sensor_detections.static_points)
    pts = sensor_detections.static_points;
    for k = 1:size(pts, 1)
        px = pts(k, 1);
        py = pts(k, 2);
        if px >= x_min && px <= x_max && py >= y_min && py <= y_max
            c = world2col(px);
            r = world2row(py);
            local_costmap(r, c) = 100;
        end
    end
end

%% 5. Ingest Dynamic Obstacle Confidence Ellipses (from Phase 2)
if ~isempty(dynamic_predictions)
    for k = 1:length(dynamic_predictions)
        dp = dynamic_predictions(k);
        % Current track position
        if isfield(dp, 'x_est') && ~isempty(dp.x_est)
            px = dp.x_est(1);
            py = dp.x_est(2);
            psi = atan2(dp.x_est(4), dp.x_est(3));
        elseif isfield(dp, 'waypoints') && ~isempty(dp.waypoints)
            px = dp.waypoints(1, 1);
            py = dp.waypoints(1, 2);
            psi = 0.0;
        else
            continue;
        end
        
        if isfield(dp, 'semi_major') && ~isempty(dp.semi_major) && dp.semi_major(1) > 0
            a = dp.semi_major(1);
            b = max(dp.semi_minor(1), 0.3);
            if isfield(dp, 'orientation') && ~isempty(dp.orientation)
                psi = dp.orientation(1);
            end
        else
            a = 1.2;
            b = 0.8;
        end
        
        apply_dynamic_ellipse_cost(px, py, a, b, psi);
    end
end

%% ── Nested Function: Continuous Exponential Inflation Field ───────────────
% Avoids 45,000-cell full scan via local bounding box cropping:
% r in [row(yh - R - 1.2), row(yh + R + 1.2)]
function apply_exponential_inflation(cx, cy, radius)
    d_safe   = 0.35; % safe vehicle cushion (m)
    d_margin = 1.20; % inflation field extent (m)
    alpha    = 2.50; % exponential decay rate (m^-1)
    if isfield(map_config, 'alpha_cost') && ~isempty(map_config.alpha_cost)
        alpha = map_config.alpha_cost;
    elseif isfield(map_config, 'alpha') && ~isempty(map_config.alpha)
        alpha = map_config.alpha;
    end
    
    r_total = radius + d_margin;
    
    c_start = world2col(cx - r_total);
    c_end   = world2col(cx + r_total);
    r_start = world2row(cy - r_total);
    r_end   = world2row(cy + r_total);
    
    % Generate subgrid coordinates
    sub_cols = c_start:c_end;
    sub_rows = r_start:r_end;
    [C_grid, R_grid] = meshgrid(sub_cols, sub_rows);
    
    sub_x = x_min + (C_grid - 0.5) * res;
    sub_y = y_min + (R_grid - 0.5) * res;
    
    d_E = hypot(sub_x - cx, sub_y - cy);
    
    % Lethal core + cushion (d_E <= radius + d_safe)
    cost_sub = zeros(size(d_E));
    lethal_mask = (d_E <= radius + d_safe);
    cost_sub(lethal_mask) = 100;
    
    % Exponential decay in margin: (radius + d_safe < d_E <= radius + d_margin)
    decay_mask = (d_E > radius + d_safe) & (d_E <= r_total);
    cost_sub(decay_mask) = 100.0 * exp(-alpha * (d_E(decay_mask) - radius - d_safe));
    
    % Clamp to [0, 100]
    cost_sub = min(max(cost_sub, 0.0), 100.0);
    
    % Update local costmap via maximum operator
    local_costmap(sub_rows, sub_cols) = max(local_costmap(sub_rows, sub_cols), cost_sub);
end

%% ── Nested Function: Dynamic Obstacle Confidence Ellipse Inflation ────────
function apply_dynamic_ellipse_cost(px, py, a, b, psi)
    r_box = max(a, b) * 1.5;
    c_start = world2col(px - r_box);
    c_end   = world2col(px + r_box);
    r_start = world2row(py - r_box);
    r_end   = world2row(py + r_box);
    
    sub_cols = c_start:c_end;
    sub_rows = r_start:r_end;
    [C_grid, R_grid] = meshgrid(sub_cols, sub_rows);
    
    sub_x = x_min + (C_grid - 0.5) * res;
    sub_y = y_min + (R_grid - 0.5) * res;
    
    dx = sub_x - px;
    dy = sub_y - py;
    
    % Transformed local ellipse coordinates
    x_rot =  dx * cos(psi) + dy * sin(psi);
    y_rot = -dx * sin(psi) + dy * cos(psi);
    
    d_ell = (x_rot.^2) / (a^2) + (y_rot.^2) / (b^2);
    
    cost_dyn = zeros(size(d_ell));
    % Lethal ellipse core (d_ell <= 1.0)
    cost_dyn(d_ell <= 1.0) = 100.0;
    
    % Exponential safety field: 1.0 < d_ell <= 2.25
    field_mask = (d_ell > 1.0) & (d_ell <= 2.25);
    cost_dyn(field_mask) = 100.0 * exp(-2.0 * (sqrt(d_ell(field_mask)) - 1.0));
    
    cost_dyn = min(max(cost_dyn, 0.0), 100.0);
    local_costmap(sub_rows, sub_cols) = max(local_costmap(sub_rows, sub_cols), cost_dyn);
end

end
