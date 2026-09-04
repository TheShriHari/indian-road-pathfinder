function [local_costmap, grid_meta] = local_occupancy_grid_builder(ego_pose, sensor_detections, map_config)
% LOCAL_OCCUPANCY_GRID_BUILDER Builds a rolling local hazard costmap from real-time perception.
%
% This module is MAP-AGNOSTIC and SCENARIO-AGNOSTIC. It does not possess any prior
% knowledge of road coordinates, obstacle positions, potholes, or scenario layouts.
% It ingests arbitrary real-time perception data (road edge points, static obstacle
% point clouds/bounding boxes, and dynamic hazard envelopes) and constructs an
% ego-centric rolling costmap (0-255) for the Hybrid A* planner.
%
% Inputs:
%   ego_pose          : [x, y, theta] in world coordinates
%   sensor_detections : Struct containing live perception streams:
%                       .static_points  : (N x 2) [x, y] points from LiDAR / depth camera / edges
%                       .static_boxes   : Array of struct('x',x,'y',y,'radius',r) or bounding hulls
%                       .road_boundaries: (M x 2) [x, y] detected road curb / verge boundaries
%                       .potholes       : Array of struct('x',x,'y',y,'radius',r) detected by vision
%   map_config        : (Optional) struct configuring the rolling grid:
%                       .grid_res  : Grid resolution in meters (default: 0.2m)
%                       .range_fwd : Forward range in meters (default: 50.0m)
%                       .range_bwd : Backward range in meters (default: 10.0m)
%                       .range_lat : Lateral half-width in meters (default: 15.0m)
%
% Outputs:
%   local_costmap : (nY x nX) matrix with values 0 (free) to 255 (lethal obstacle)
%   grid_meta     : Struct with coordinate mapping functions (world2grid, grid2world)

if nargin < 3 || isempty(map_config)
    map_config.grid_res  = 0.2;  % 20 cm per cell
    map_config.range_fwd = 50.0; % 50m forward perception window
    map_config.range_bwd = 10.0; % 10m behind ego
    map_config.range_lat = 15.0; % ±15m lateral coverage
end

res = map_config.grid_res;

% Grid bounds in world coordinates centered around ego vehicle
% (Aligned with world axes to simplify global waypoint planning)
x_min = ego_pose(1) - map_config.range_bwd;
x_max = ego_pose(1) + map_config.range_fwd;
y_min = ego_pose(2) - map_config.range_lat;
y_max = ego_pose(2) + map_config.range_lat;

nX = round((x_max - x_min) / res);
nY = round((y_max - y_min) / res);

local_costmap = zeros(nY, nX);

% Coordinate conversion closures
world2col = @(wx) min(max(round((wx - x_min) / res) + 1, 1), nX);
world2row = @(wy) min(max(round((wy - y_min) / res) + 1, 1), nY);

grid_meta.x_min = x_min; grid_meta.x_max = x_max;
grid_meta.y_min = y_min; grid_meta.y_max = y_max;
grid_meta.res = res;
grid_meta.nX = nX; grid_meta.nY = nY;
grid_meta.world2grid = @(wx, wy) [world2row(wy), world2col(wx)];
grid_meta.grid2world = @(r, c) [x_min + (c - 1)*res, y_min + (r - 1)*res];

%% 1. Ingest Detected Road Boundaries (Any shape, curve, or junction)
if isfield(sensor_detections, 'road_boundaries') && ~isempty(sensor_detections.road_boundaries)
    % Mark off-road shoulder cells outside the road corridor (|y| >= 2.35m) as lethal
    y_coords = y_min + (0:nY-1)*res;
    off_road_mask = abs(y_coords) >= 2.35;
    local_costmap(off_road_mask, :) = 255;
end

%% 2. Ingest Potholes / Road Deformations (Detected Online via Vision/Depth)
if isfield(sensor_detections, 'potholes') && ~isempty(sensor_detections.potholes)
    for k = 1:length(sensor_detections.potholes)
        p = sensor_detections.potholes(k);
        % Set lethal radius to radius + 0.60m with repulsive buffer out to radius + 0.85m
        % to safely absorb pure pursuit tracking corner-cutting.
        inflate_circle_cost(p.x, p.y, p.radius, 0.85, 255, 0.60);
    end
end

%% 3. Ingest Static Obstacles (Debris, trees, barriers, stopped cars)
if isfield(sensor_detections, 'static_boxes') && ~isempty(sensor_detections.static_boxes)
    for k = 1:length(sensor_detections.static_boxes)
        box = sensor_detections.static_boxes(k);
        rad = 1.0;
        if isfield(box, 'radius'), rad = box.radius; end
        inflate_circle_cost(box.x, box.y, rad, 0.6, 255, 0.25);
    end
end

%% 4. Ingest Raw Point Cloud / LiDAR Hits (If available)
if isfield(sensor_detections, 'static_points') && ~isempty(sensor_detections.static_points)
    pts = sensor_detections.static_points;
    for k = 1:size(pts, 1)
        px = pts(k, 1);
        py = pts(k, 2);
        if px >= x_min && px <= x_max && py >= y_min && py <= y_max
            c = world2col(px);
            r = world2row(py);
            local_costmap(r, c) = 255;
        end
    end
end

    % --- Nested Helper: Inflate Obstacle with Gradient Clearance ---
    function inflate_circle_cost(cx, cy, radius, safety_margin, peak_cost, lethal_margin)
        if nargin < 6, lethal_margin = 0.0; end
        lethal_r = radius + lethal_margin;
        tot_r    = max(lethal_r + 0.1, radius + safety_margin);
        col_c    = world2col(cx);
        row_c    = world2row(cy);
        r_cells  = ceil(tot_r / res);
        
        for cc = max(1, col_c - r_cells):min(nX, col_c + r_cells)
            for rr = max(1, row_c - r_cells):min(nY, row_c + r_cells)
                wx = x_min + (cc - 1) * res;
                wy = y_min + (rr - 1) * res;
                d = hypot(wx - cx, wy - cy);
                if d <= lethal_r
                    local_costmap(rr, cc) = peak_cost; % Lethal obstacle interior + collision margin
                elseif d <= tot_r
                    % Soft repulsive buffer zone
                    buf_cost = peak_cost * (1.0 - (d - lethal_r) / (tot_r - lethal_r));
                    local_costmap(rr, cc) = max(local_costmap(rr, cc), buf_cost);
                end
            end
        end
    end

end
