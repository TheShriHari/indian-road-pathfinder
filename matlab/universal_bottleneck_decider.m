function [virtual_stop_active, stop_pose, bottleneck_info] = universal_bottleneck_decider(ego_state, planned_path, costmap, grid_meta, dynamic_predictions, params)
%% UNIVERSAL_BOTTLENECK_DECIDER Evaluates forward corridor squeeze and arbitrates VSL vs detour (Phase 3).
%   [virtual_stop_active, stop_pose, bottleneck_info] = universal_bottleneck_decider(...)
%
% Algorithm:
%   1. Profiles continuous traversable width W_free(s) along forward path stations s in [0, 30] m.
%   2. Enforces Disjoint Free Opening Rule: extracts the single contiguous opening containing
%      or closest to the reference path (never sums disjoint clear gaps).
%   3. Evaluates against W_crit = W_veh + 2*clearance = 1.85 + 2*0.35 = 2.55 m.
%   4. Arbitrates among:
%      - Static Narrowing (1.85 <= W_free < 2.55 m, no oncoming actor) -> detour_required = true, virtual_stop = false
%      - Dynamic Squeeze (W_free < 2.55 m + oncoming actor v_rel < -0.5 m/s) -> virtual_stop = true (VSL at s - 3.5 m)
%      - Complete Blockage (W_free < 1.85 m static) -> virtual_stop = true (VSL at s - 3.5 m)
%      - Clear (W_free >= 2.55 m) -> virtual_stop = false, detour_required = false

% ── Configuration Defaults ─────────────────────────────────────────────────
defaults_ubd.vehicle_width   = 1.85;  % standard chassis width (m)
defaults_ubd.min_clearance   = 0.35;  % lateral clearance per side (m)
defaults_ubd.scan_horizon    = 30.0;  % scan ahead horizon (m)
defaults_ubd.stop_buffer     = 3.5;   % VSL offset upstream of bottleneck (m)
defaults_ubd.cost_threshold  = 90.0;  % cost < 90 is traversable corridor (excludes lethal core)

if nargin < 6 || isempty(params), params = defaults_ubd; end
fnames = fieldnames(defaults_ubd);
for fi = 1:length(fnames)
    if ~isfield(params, fnames{fi})
        params.(fnames{fi}) = defaults_ubd.(fnames{fi});
    end
end

if nargin < 5, dynamic_predictions = []; end

W_crit = params.vehicle_width + 2.0 * params.min_clearance; % 2.55 m
W_chassis = params.vehicle_width;                           % 1.85 m

virtual_stop_active = false;
stop_pose           = [];
bottleneck_info     = struct('station_s', Inf, 'min_width', Inf, ...
                             'detour_required', false, 'vsl_station', NaN, ...
                             'reason', 'Clear Open Road', 'path_invalid', false, ...
                             'immediate_standstill', false);

if isempty(planned_path) || size(planned_path, 1) < 2
    return;
end

ego_x = ego_state(1);
ego_y = ego_state(2);
if length(ego_state) >= 3
    ego_theta = ego_state(3);
else
    ego_theta = 0.0;
end

% ── Step 1: Compute Path Cumulative Stations ───────────────────────────────
N_pts = size(planned_path, 1);
cum_s = zeros(N_pts, 1);
for k = 2:N_pts
    cum_s(k) = cum_s(k-1) + hypot(planned_path(k,1) - planned_path(k-1,1), ...
                                  planned_path(k,2) - planned_path(k-1,2));
end

% Find ego station along path
d_ego = hypot(planned_path(:,1) - ego_x, planned_path(:,2) - ego_y);
[~, nearest_idx] = min(d_ego);
s_ego = cum_s(nearest_idx);

% ── Step 2: Forward Corridor Width Profiling (s in [0, 30] m) ──────────────
ds = 1.0; % 1-meter evaluation step
s_start = s_ego + 1.0;
s_end   = min(cum_s(end), s_ego + params.scan_horizon);

if s_start >= s_end
    return;
end

s_eval = s_start:ds:s_end;
res = grid_meta.res;
y_min = grid_meta.y_min;
y_max = grid_meta.y_max;
nY = grid_meta.nY;
nX = grid_meta.nX;
row_ys = y_min + ((1:nY) - 0.5) * res;

found_pinch       = false;
pinch_station_rel = Inf;
pinch_width       = Inf;
pinch_pt          = [0, 0];
pinch_yaw         = ego_theta;

for idx = 1:length(s_eval)
    s_curr = s_eval(idx);
    dist_ahead = s_curr - s_ego;
    
    pt = interp1(cum_s, planned_path, s_curr, 'linear');
    pt_next = interp1(cum_s, planned_path, min(cum_s(end), s_curr + 0.5), 'linear');
    yaw = atan2(pt_next(2) - pt(2), pt_next(1) - pt(1));
    
    % Find costmap column index for pt(1)
    col_idx = min(max(floor((pt(1) - grid_meta.x_min) / res) + 1, 1), nX);
    col_costs = costmap(:, col_idx);
    
    % Contiguous free intervals where cost < cost_threshold (40)
    is_clear = (col_costs < params.cost_threshold);
    
    % Find contiguous blocks of 1s in is_clear
    d_clear = diff([0; is_clear; 0]);
    run_starts = find(d_clear == 1);
    run_ends   = find(d_clear == -1) - 1;
    
    if isempty(run_starts)
        % Completely blocked column
        W_free = 0.0;
    else
        % Compute width and y-bounds of each contiguous opening
        num_runs = length(run_starts);
        run_widths = zeros(num_runs, 1);
        dist_to_path = zeros(num_runs, 1);
        
        for ri = 1:num_runs
            y_lower = row_ys(run_starts(ri)) - 0.5 * res;
            y_upper = row_ys(run_ends(ri))   + 0.5 * res;
            run_widths(ri) = y_upper - y_lower;
            
            % Distance from reference path y to opening interval
            if pt(2) >= y_lower && pt(2) <= y_upper
                dist_to_path(ri) = 0.0; % reference path inside this opening
            else
                dist_to_path(ri) = min(abs(pt(2) - y_lower), abs(pt(2) - y_upper));
            end
        end
        
        % Disjoint Free Opening Selection Rule:
        % Select the single contiguous opening containing the reference path,
        % or closest to it if none contains it. NEVER sum multiple disjoint gaps!
        [min_dist, ~] = min(dist_to_path);
        candidate_indices = find(dist_to_path == min_dist);
        if length(candidate_indices) == 1
            best_ri = candidate_indices;
        else
            % If equidistant, choose the larger opening
            [~, best_sub] = max(run_widths(candidate_indices));
            best_ri = candidate_indices(best_sub);
        end
        
        W_free = run_widths(best_ri);
    end
    
    % Check bottleneck threshold (W_free < W_crit = 2.55 m)
    if W_free < W_crit
        found_pinch       = true;
        pinch_station_rel = dist_ahead;
        pinch_width       = W_free;
        pinch_pt          = pt;
        pinch_yaw         = yaw;
        break; % Evaluate earliest squeeze point encountered
    end
end

if ~found_pinch
    % Corridor is fully clear across all 30m
    bottleneck_info.min_width = 7.0; % nominal open width
    return;
end

% ── Step 3: Check for Dynamic Actors in Pinch Zone ────────────────────────
% Pinch zone: [s_pinch - 2.0 m, s_pinch + 4.0 m]
has_oncoming_dynamic = false;

if ~isempty(dynamic_predictions)
    for ai = 1:length(dynamic_predictions)
        dp = dynamic_predictions(ai);
        
        % Check velocity: approaching relative velocity (v_rel < -0.5 m/s)
        is_approaching = false;
        if isfield(dp, 'x_est') && ~isempty(dp.x_est)
            v_x = dp.x_est(3);
            if v_x < -0.5
                is_approaching = true;
            end
        end
        
        % Check if waypoints intersect the pinch zone
        intersects_pinch = false;
        if isfield(dp, 'waypoints') && ~isempty(dp.waypoints)
            wps = dp.waypoints;
            for h = 1:min(20, size(wps, 1))
                d_to_pinch = hypot(wps(h, 1) - pinch_pt(1), wps(h, 2) - pinch_pt(2));
                if d_to_pinch < 3.5
                    intersects_pinch = true;
                    break;
                end
            end
        end
        
        if is_approaching && intersects_pinch
            has_oncoming_dynamic = true;
            break;
        end
    end
end

% ── Step 4: Three-Way Bottleneck State Arbitration ────────────────────────
bottleneck_info.station_s = pinch_station_rel;
bottleneck_info.min_width = pinch_width;

if pinch_width < W_chassis
    % Condition 3: Complete Road Blockage (< 1.85 m static) -> Must Halt
    virtual_stop_active          = true;
    bottleneck_info.detour_required = false;
    
    % Upstream VSL Clamping: s_vsl = max(0.5, s_pinch - 3.5)
    if pinch_station_rel <= 1.0
        bottleneck_info.immediate_standstill = true;
        s_vsl = 0.0;
        stop_pose = [ego_x, ego_y, ego_theta];
    else
        bottleneck_info.immediate_standstill = false;
        s_vsl = max(0.5, pinch_station_rel - params.stop_buffer);
        vsl_xy = interp1(cum_s, planned_path, s_ego + s_vsl, 'linear');
        stop_pose = [vsl_xy(1), vsl_xy(2), pinch_yaw];
    end
    bottleneck_info.vsl_station = s_vsl;
    bottleneck_info.reason = sprintf('Complete Road Blockage: Width=%.2fm < %.2fm chassis width at +%.1fm', ...
                                     pinch_width, W_chassis, pinch_station_rel);

elseif has_oncoming_dynamic
    % Condition 2: Dynamic Squeeze Point -> Halt at Upstream Virtual Stop Line
    virtual_stop_active          = true;
    bottleneck_info.detour_required = false;
    
    if pinch_station_rel <= 1.0
        bottleneck_info.immediate_standstill = true;
        s_vsl = 0.0;
        stop_pose = [ego_x, ego_y, ego_theta];
    else
        bottleneck_info.immediate_standstill = false;
        s_vsl = max(0.5, pinch_station_rel - params.stop_buffer);
        vsl_xy = interp1(cum_s, planned_path, s_ego + s_vsl, 'linear');
        stop_pose = [vsl_xy(1), vsl_xy(2), pinch_yaw];
    end
    bottleneck_info.vsl_station = s_vsl;
    bottleneck_info.reason = sprintf('Dynamic Squeeze: Oncoming actor in narrow corridor (Width=%.2fm at +%.1fm)', ...
                                     pinch_width, pinch_station_rel);

else
    % Condition 1: Static Narrowing Only (1.85 m <= W_free < 2.55 m) -> Local Detour
    virtual_stop_active          = false;
    bottleneck_info.detour_required = true;
    bottleneck_info.vsl_station  = NaN;
    stop_pose                    = [];
    bottleneck_info.reason       = sprintf('Static Road Narrowing: Detour required (Width=%.2fm at +%.1fm)', ...
                                     pinch_width, pinch_station_rel);
end

end
