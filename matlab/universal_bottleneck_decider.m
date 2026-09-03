function [virtual_stop_active, stop_pose, bottleneck_info] = universal_bottleneck_decider(ego_state, planned_path, costmap, grid_meta, dynamic_predictions, params)
% UNIVERSAL_BOTTLENECK_DECIDER Evaluates spatio-temporal corridor squeeze without hardcoded map positions.
%
% This algorithm computes the continuous traversable width W_free(s) along the ego
% path horizon. If at any longitudinal station s ahead, the navigable corridor narrows
% below (vehicle_width + 2*safety_margin) due to ANY combination of static clutter
% and dynamic predicted agents, it establishes a Virtual Stop Line upstream of the pinch.
%
% Inputs:
%   ego_state           : [x, y, theta, v]
%   planned_path        : (N x 2) waypoints [x, y] ahead of ego
%   costmap             : (nY x nX) matrix of lethal/soft costs from local_occupancy_grid_builder
%   grid_meta           : Coordinate transform metadata
%   dynamic_predictions : Struct array from dynamic_obstacle_predictor (EKF)
%   params              : (Optional) tuning parameters:
%                           .vehicle_width : Vehicle body width (default: 1.85m)
%                           .min_clearance : Minimum lateral margin required (default: 0.35m)
%                           .scan_horizon  : Maximum distance ahead to evaluate (default: 30.0m)
%                           .stop_buffer   : Standstill buffer before bottleneck (default: 3.5m)
%
% Outputs:
%   virtual_stop_active : Boolean flag indicating that ego must halt
%   stop_pose           : [x_stop, y_stop, theta_stop] Virtual Stop Line coordinates
%   bottleneck_info     : Struct with bottleneck station s, bottleneck width, and reason

if nargin < 6 || isempty(params)
    params.vehicle_width = 1.85; % Standard sedan width (m)
    params.min_clearance = 0.35; % Margin on each side (m)
    params.scan_horizon  = 30.0; % Forward scan horizon (m)
    params.stop_buffer   = 3.5;  % Halt 3.5m upstream of pinch point
end

min_traversable_width = params.vehicle_width + 2 * params.min_clearance; % ~2.55m

virtual_stop_active = false;
stop_pose = [];
bottleneck_info = struct('station_s', Inf, 'min_width', Inf, 'reason', 'Clear');

if isempty(planned_path) || size(planned_path, 1) < 2
    return;
end

ego_x = ego_state(1);
ego_y = ego_state(2);
ego_theta = ego_state(3);

% 1. Sample trajectory along path up to scan_horizon
cum_s = 0;
path_stations = zeros(size(planned_path, 1), 1);
for k = 2:size(planned_path, 1)
    cum_s = cum_s + hypot(planned_path(k,1) - planned_path(k-1,1), planned_path(k,2) - planned_path(k-1,2));
    path_stations(k) = cum_s;
end

% 2. Evaluate lateral width at regular station intervals
ds = 1.0; % 1-meter evaluation resolution
s_eval = 2.0:ds:min(params.scan_horizon, cum_s);

for idx = 1:length(s_eval)
    s = s_eval(idx);
    
    % Interpolate path point and tangent direction at station s
    pt = interp1(path_stations, planned_path, s, 'linear');
    pt_next = interp1(path_stations, planned_path, min(cum_s, s + 0.5), 'linear');
    tangent_yaw = atan2(pt_next(2) - pt(2), pt_next(1) - pt(1));
    
    % Normal vector to path (pointing Left: +90 deg, Right: -90 deg)
    n_left  = [-sin(tangent_yaw),  cos(tangent_yaw)];
    n_right = [ sin(tangent_yaw), -cos(tangent_yaw)];
    
    % Scan laterally to find closest obstacle boundary on Left and Right
    left_dist  = scan_lateral_clearance(pt, n_left,  costmap, grid_meta, dynamic_predictions, s, ego_state(4));
    right_dist = scan_lateral_clearance(pt, n_right, costmap, grid_meta, dynamic_predictions, s, ego_state(4));
    
    corridor_width = left_dist + right_dist;
    
    % Check if corridor is squeezed below vehicle passage threshold
    if corridor_width < min_traversable_width
        virtual_stop_active = true;
        
        % Place stop pose upstream by params.stop_buffer
        s_stop = max(0.5, s - params.stop_buffer);
        stop_xy = interp1(path_stations, planned_path, s_stop, 'linear');
        stop_pose = [stop_xy(1), stop_xy(2), tangent_yaw];
        
        bottleneck_info.station_s = s;
        bottleneck_info.min_width = corridor_width;
        bottleneck_info.reason = sprintf('Corridor Squeeze (Width=%.2fm < %.2fm required at s=%.1fm)', ...
                                         corridor_width, min_traversable_width, s);
        return;
    end
end

    % --- Nested Function: Scan Lateral Clearance from Centerline ---
    function clearance = scan_lateral_clearance(center_pt, normal_dir, cmap, g_meta, dyn_preds, s_dist, ego_speed)
        MAX_LAT_SCAN = 6.0; % Scan up to 6m left/right
        dl = 0.2;           % 20cm search step
        clearance = MAX_LAT_SCAN;
        
        % Estimated arrival time at station s
        if ego_speed > 0.5
            t_arrival = s_dist / ego_speed;
        else
            t_arrival = s_dist / 3.0; % Nominal assumption if currently slow/stopped
        end
        horizon_step = max(1, min(20, round(t_arrival / 0.1)));
        
        for lat_dist = 0:dl:MAX_LAT_SCAN
            test_x = center_pt(1) + lat_dist * normal_dir(1);
            test_y = center_pt(2) + lat_dist * normal_dir(2);
            
            % Check static/rolling costmap cell
            if test_x >= g_meta.x_min && test_x <= g_meta.x_max && ...
               test_y >= g_meta.y_min && test_y <= g_meta.y_max
                c = min(max(round((test_x - g_meta.x_min) / g_meta.res) + 1, 1), g_meta.nX);
                r = min(max(round((test_y - g_meta.y_min) / g_meta.res) + 1, 1), g_meta.nY);
                if cmap(r, c) >= 200
                    clearance = lat_dist;
                    return;
                end
            end
            
            % Check dynamic agents' predicted positions at arrival time
            for a_idx = 1:length(dyn_preds)
                wp = dyn_preds(a_idx).waypoints;
                if isempty(wp), continue; end
                h_idx = min(horizon_step, size(wp, 1));
                ag_x = wp(h_idx, 1);
                ag_y = wp(h_idx, 2);
                
                % Agent radius buffer (~1.2m)
                if hypot(test_x - ag_x, test_y - ag_y) <= 1.2
                    clearance = lat_dist;
                    return;
                end
            end
        end
    end

end
