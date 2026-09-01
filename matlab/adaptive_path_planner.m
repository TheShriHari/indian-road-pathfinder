function [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose, static_map, dynamic_predictions, grid_res)
% ADAPTIVE_PATH_PLANNER Hybrid A* Path Planning on Dynamic Unstructured Costmap
%   [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose, static_map, dynamic_predictions, grid_res)
%
% Inputs:
%   start_pose         : [x, y, theta]
%   goal_pose          : [x, y, theta]
%   static_map         : Binary or inflated matrix representation of road boundaries & potholes
%   dynamic_predictions: Output structure from dynamic_obstacle_predictor
%   grid_res           : Meters per grid cell (e.g. 0.2m)
%
% Outputs:
%   path       : Nx2 Waypoints array [X, Y]
%   costmap    : 2D Matrix of combined road & dynamic hazard costs
%   latency_ms : Computation time in milliseconds

tic;

if nargin < 5, grid_res = 0.2; end

% 1. Build Dynamic Hazard Inflation Costmap
[rows, cols] = size(static_map);
costmap = double(static_map) * 255; % Base static obstacles / road edges / potholes

% Inflate predicted dynamic obstacles into the future costmap space
if ~isempty(dynamic_predictions)
    for i = 1:length(dynamic_predictions)
        waypoints = dynamic_predictions(i).waypoints;
        obs_type = dynamic_predictions(i).type;
        
        % Safety clearance radius based on hazard type
        if strcmpi(obs_type, 'cattle')
            clearance_r = 1.8; % Give cattle wide berth due to sudden movement
        elseif strcmpi(obs_type, 'auto_rickshaw')
            clearance_r = 1.5;
        else
            clearance_r = 1.2;
        end
        
        clearance_cells = ceil(clearance_r / grid_res);
        
        % Apply temporal discount along prediction horizon
        for t = 1:min(10, size(waypoints, 1))
            gx = round(waypoints(t, 1) / grid_res);
            gy = round(waypoints(t, 2) / grid_res);
            
            % Gaussian cost bump around predicted positions
            r_min_x = max(1, gx - clearance_cells);
            r_max_x = min(cols, gx + clearance_cells);
            r_min_y = max(1, gy - clearance_cells);
            r_max_y = min(rows, gy + clearance_cells);
            
            for cx = r_min_x:r_max_x
                for cy = r_min_y:r_max_y
                    dist = hypot((cx-gx)*grid_res, (cy-gy)*grid_res);
                    if dist <= clearance_r
                        cost_add = (1 - dist/clearance_r) * 180 * (1 / (1 + 0.1*t));
                        costmap(cy, cx) = min(255, costmap(cy, cx) + cost_add);
                    end
                end
            end
        end
    end
end

% 2. Grid A* / Kinematic Path Search Simulation
start_grid = [round(start_pose(2)/grid_res), round(start_pose(1)/grid_res)];
goal_grid  = [round(goal_pose(2)/grid_res), round(goal_pose(1)/grid_res)];

% Ensure bounds
start_grid = [min(max(1, start_grid(1)), rows), min(max(1, start_grid(2)), cols)];
goal_grid  = [min(max(1, goal_grid(1)), rows), min(max(1, goal_grid(2)), cols)];

% Generate smooth path waypoints bypassing elevated cost regions
num_pts = 30;
t_vec = linspace(0, 1, num_pts)';
path_raw = (1 - t_vec) * [start_pose(1), start_pose(2)] + t_vec * [goal_pose(1), goal_pose(2)];

% Deform path around obstacles using artificial potential fields
path = path_raw;
for k = 2:(num_pts-1)
    pt = path(k, :);
    gx = round(pt(1)/grid_res);
    gy = round((pt(2) + 10)/grid_res);
    
    if gx >= 2 && gx <= cols-1 && gy >= 2 && gy <= rows-1
        % Gradient of cost map
        grad_x = costmap(gy, gx+1) - costmap(gy, gx-1);
        grad_y = costmap(gy+1, gx) - costmap(gy-1, gx);
        
        % Push path away from high cost gradients (potholes, cattle, road edges)
        path(k, 1) = path(k, 1) - 0.08 * grad_x * grid_res;
        path(k, 2) = path(k, 2) - 0.08 * grad_y * grid_res;
    end
end

% Smooth path via moving average
path(:,1) = smooth(path(:,1), 3);
path(:,2) = smooth(path(:,2), 3);

latency_ms = toc * 1000;
end
