function [control, e_y, e_theta] = pure_pursuit_controller(state, path, v_ref, params)
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
    params.k_lookahead = 0.5; % Lookahead gain (s)
    params.min_lookahead = 2.5; % Minimum lookahead distance (m)
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

% 3. Calculate Steering Angle (Pure Pursuit)
alpha = atan2(target_pt(2) - y, target_pt(1) - x) - theta;
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
