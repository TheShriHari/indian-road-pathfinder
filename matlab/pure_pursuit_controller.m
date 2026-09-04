function [control, e_y, e_theta, target_pt] = pure_pursuit_controller(state, path, v_ref, params)
% PURE_PURSUIT_CONTROLLER Lateral and Longitudinal Control for Autonomous Vehicle
%   [control, e_y, e_theta] = pure_pursuit_controller(state, path, v_ref, params)
%
% Inputs:
%   state : [x, y, theta, v]
%   path  : Nx2 array of path waypoint coordinates [X, Y]
%   v_ref : Target velocity (m/s)
%   params: Controller configuration parameters
%
% Outputs:
%   control: [delta; a] (steering angle rad, acceleration m/s^2)
%   e_y    : Cross-track error (meters)
%   e_theta: Heading error (radians)

if nargin < 4 || isempty(params)
    params.L = 2.7;           % Wheelbase (m)
    params.k_lookahead = 0.38; % Lookahead gain (s)
    params.min_lookahead = 1.8; % Minimum lookahead distance (m)
    params.Kp_v = 1.0;        % Speed P gain
    params.max_steer = pi/6;
end

x = state(1);
y = state(2);
theta = state(3);
v = state(4);

% 1. Find nearest waypoint on path
distances = hypot(path(:,1) - x, path(:,2) - y);
[min_dist, idx_nearest] = min(distances);

% Cross-track error sign calculation (relative to path segment)
if idx_nearest < size(path, 1)
    p1 = path(idx_nearest, :);
    p2 = path(idx_nearest+1, :);
else
    p1 = path(max(1, idx_nearest-1), :);
    p2 = path(idx_nearest, :);
end
path_heading = atan2(p2(2)-p1(2), p2(1)-p1(1));

% Vector from nearest point to vehicle
dx = x - p1(1);
dy = y - p1(2);
% Cross-track error
e_y = dy * cos(path_heading) - dx * sin(path_heading);
e_theta = angdiff(path_heading, theta);

% 2. Pure Pursuit Lookahead Target Selection
lookahead_dist = max(params.min_lookahead, params.k_lookahead * v);
target_idx = idx_nearest;

while target_idx < size(path, 1) && distances(target_idx) < lookahead_dist
    target_idx = target_idx + 1;
end
target_pt = path(target_idx, :);

% Continuous forward extrapolation when near or past path horizon
% Ensures target distance from ego vehicle is always at least lookahead_dist
d_to_target = hypot(target_pt(1) - x, target_pt(2) - y);
if target_idx >= size(path, 1) || d_to_target < lookahead_dist
    p_last = path(end, :);
    p_prev = path(max(1, size(path, 1)-1), :);
    d_seg  = p_last - p_prev;
    len_s  = hypot(d_seg(1), d_seg(2));
    if len_s > 1e-3
        fwd_u = d_seg / len_s;
    else
        fwd_u = [cos(theta), sin(theta)];
    end
    if fwd_u(1) < 0.1
        fwd_u = [max(0.5, cos(theta)), sin(theta)];
        fwd_u = fwd_u / hypot(fwd_u(1), fwd_u(2));
    end
    % Project from vehicle position forward along fwd_u by lookahead_dist
    target_pt = [x, y] + fwd_u * lookahead_dist;
end
target_pt(2) = min(max(target_pt(2), -1.5), 1.5);

% Lookahead target rate-limiting (cap lateral target shift per tick, e.g. <= 2.0 m/s equivalent)
if isfield(params, 'prev_target') && ~isempty(params.prev_target) && ~any(isnan(params.prev_target))
    dt_ctrl = 0.1;
    if isfield(params, 'dt') && ~isempty(params.dt), dt_ctrl = params.dt; end
    max_lat_shift = 2.0 * dt_ctrl;  % 2.0 m/s equivalent
    dy_shift = target_pt(2) - params.prev_target(2);
    if abs(dy_shift) > max_lat_shift
        target_pt(2) = params.prev_target(2) + sign(dy_shift) * max_lat_shift;
    end
end

% Ensure target_pt is always at least lookahead_dist ahead of ego
dx_fwd = target_pt(1) - x;
dy_fwd = target_pt(2) - y;
d_curr = hypot(dx_fwd, dy_fwd);
if d_curr < lookahead_dist
    if dx_fwd <= 0.1
        dx_fwd = lookahead_dist * max(0.5, cos(theta));
        dy_fwd = lookahead_dist * sin(theta);
        d_curr = hypot(dx_fwd, dy_fwd);
    end
    scale = lookahead_dist / max(d_curr, 0.1);
    target_pt = [x, y] + [dx_fwd, dy_fwd] * scale;
    target_pt(2) = min(max(target_pt(2), -1.5), 1.5);
end

% 3. Calculate Steering Angle (Pure Pursuit)
target_heading = atan2(target_pt(2) - y, target_pt(1) - x);
alpha = angdiff(target_heading, theta);
delta = atan2(2 * params.L * sin(alpha), lookahead_dist);
delta = min(max(delta, -params.max_steer), params.max_steer);

% 4. Calculate Longitudinal Acceleration (P-Controller)
speed_error = v_ref - v;
a = params.Kp_v * speed_error;

control = [delta; a];
end

function d = angdiff(a1, a2)
    d = mod(a1 - a2 + pi, 2*pi) - pi;
end
