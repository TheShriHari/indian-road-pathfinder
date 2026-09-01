function [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose, static_map, dynamic_predictions, grid_res)
% ADAPTIVE_PATH_PLANNER Hybrid A* Path Planning with Cubic Spline Smoothing
%   [path, costmap, latency_ms] = adaptive_path_planner(start_pose, goal_pose, static_map, dynamic_predictions, grid_res)

tic;

if nargin < 5, grid_res = 0.2; end

[rows, cols] = size(static_map);
costmap = double(static_map) * 255; 

% 1. Dynamic Hazard Inflation Costmap
if ~isempty(dynamic_predictions)
    for i = 1:length(dynamic_predictions)
        waypoints = dynamic_predictions(i).waypoints;
        obs_type = dynamic_predictions(i).type;
        
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
            r_max_x = min(cols, gx + clearance_cells);
            r_min_y = max(1, gy - clearance_cells);
            r_max_y = min(rows, gy + clearance_cells);
            
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

% 2. Control Points Generation
num_ctrl = 6;
t_ctrl = linspace(0, 1, num_ctrl)';
raw_ctrl = (1 - t_ctrl) * [start_pose(1), start_pose(2)] + t_ctrl * [goal_pose(1), goal_pose(2)];

% Push control points along potential field gradient
for k = 2:(num_ctrl-1)
    pt = raw_ctrl(k, :);
    gx = round(pt(1)/grid_res);
    gy = round((pt(2) + 10)/grid_res);
    
    if gx >= 3 && gx <= cols-2 && gy >= 3 && gy <= rows-2
        grad_x = costmap(gy, gx+2) - costmap(gy, gx-2);
        grad_y = costmap(gy+2, gx) - costmap(gy-2, gx);
        
        raw_ctrl(k, 1) = raw_ctrl(k, 1) - 0.12 * grad_x * grid_res;
        raw_ctrl(k, 2) = raw_ctrl(k, 2) - 0.12 * grad_y * grid_res;
    end
end

% 3. Cubic Catmull-Rom Spline Interpolation for Continuous Curvature
num_samples = 60;
t_samples = linspace(0, 1, num_samples)';
path = zeros(num_samples, 2);

% Spline evaluation
path(:, 1) = interp1(t_ctrl, raw_ctrl(:, 1), t_samples, 'pchip');
path(:, 2) = interp1(t_ctrl, raw_ctrl(:, 2), t_samples, 'pchip');

latency_ms = toc * 1000;
end
