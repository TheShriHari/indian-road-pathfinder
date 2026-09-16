function [control, e_y, e_theta, target_pt] = pure_pursuit_controller(state, path, v_ref, params)
%% PURE_PURSUIT_CONTROLLER Speed-Adaptive Tracking Controller with Slew & Jerk Clamping (Phase 4).
%   [control, e_y, e_theta, target_pt] = pure_pursuit_controller(state, path, v_ref, params)
%
% Algorithm Formulation:
%   1. Speed-Adaptive Lookahead Distance:
%      L_d(v) = clip(K_v * v_ego, L_min, L_max)
%      K_v = 1.2 s, L_min = 3.0 m, L_max = 12.0 m
%   2. Standstill boundary condition at Virtual Stop Line:
%      When v -> 0, L_d = L_min = 3.0m, target projection strictly enforced
%      along or forward of ego heading axis (prevents +-pi alpha flip).
%   3. Steering actuation saturation & slew rate limiting:
%      delta_cmd in [-30, +30] deg, |d_delta / dt| <= 15 deg/s (0.2618 rad/s).
%   4. Longitudinal jerk limiting:
%      |j(t)| = |da / dt| <= 0.9 m/s^3 (< 1.0 m/s^3 limit).
%
% Inputs:
%   state   : [x, y, theta, v]
%   path    : [N x 2] array of waypoints [x, y]
%   v_ref   : target velocity (m/s)
%   params  : optional configuration struct:
%               .L             - wheelbase (m, default 2.7)
%               .k_lookahead   - speed gain K_v (s, default 1.2)
%               .min_lookahead - minimum lookahead (m, default 3.0)
%               .max_lookahead - maximum lookahead (m, default 12.0)
%               .max_steer     - steer limit (rad, default pi/6 = 30 deg)
%               .max_slew_rate - steer slew limit (rad/s, default 0.2618 = 15 deg/s)
%               .max_jerk      - longitudinal jerk limit (m/s^3, default 0.9)
%               .dt            - controller timestep (s, default 0.1)
%               .prev_steer    - prior steering angle (rad)
%               .prev_accel    - prior acceleration (m/s^2)
%               .reset         - reset persistent state if true
%
% Outputs:
%   control   : [delta; a] (steering angle rad, acceleration m/s^2)
%   e_y       : cross-track error (meters)
%   e_theta   : heading error (radians)
%   target_pt : [x, y] lookahead target point coordinates

persistent p_last_steer p_last_accel;

if nargin < 4 || isempty(params), params = struct(); end

% Explicit persistent state reset
if isfield(params, 'reset') && params.reset
    p_last_steer = [];
    p_last_accel = [];
    if isempty(state)
        control = [0.0; 0.0];
        e_y = 0.0;
        e_theta = 0.0;
        target_pt = [0.0, 0.0];
        return;
    end
end

% ── Configuration Defaults ─────────────────────────────────────────────────
L          = 2.7;               % Wheelbase (m)
K_v        = 1.2;               % Speed-adaptive lookahead gain (s)
L_min      = 3.0;               % Minimum lookahead distance (m)
L_max      = 12.0;              % Maximum lookahead distance (m)
max_steer  = pi / 6.0;          % 30 deg (0.5236 rad)
slew_rate  = 15.0 * (pi / 180); % 15 deg/s = 0.2618 rad/s
jerk_max   = 0.90;              % 0.9 m/s^3 (< 1.0 m/s^3)
Kp_v       = 1.0;               % Speed P-gain
dt         = 0.1;               % Controller tick (s)

if isfield(params, 'L')             && ~isempty(params.L),             L          = params.L;             end
if isfield(params, 'k_lookahead')   && ~isempty(params.k_lookahead),   K_v        = params.k_lookahead;   end
if isfield(params, 'K_v')           && ~isempty(params.K_v),           K_v        = params.K_v;           end
if isfield(params, 'min_lookahead') && ~isempty(params.min_lookahead), L_min      = params.min_lookahead; end
if isfield(params, 'L_min')         && ~isempty(params.L_min),         L_min      = params.L_min;         end
if isfield(params, 'max_lookahead') && ~isempty(params.max_lookahead), L_max      = params.max_lookahead; end
if isfield(params, 'max_steer')     && ~isempty(params.max_steer),     max_steer  = params.max_steer;     end
if isfield(params, 'max_slew_rate') && ~isempty(params.max_slew_rate), slew_rate  = params.max_slew_rate; end
if isfield(params, 'max_jerk')      && ~isempty(params.max_jerk),      jerk_max   = params.max_jerk;      end
if isfield(params, 'Kp_v')          && ~isempty(params.Kp_v),          Kp_v       = params.Kp_v;          end
if isfield(params, 'dt')            && ~isempty(params.dt),            dt         = params.dt;            end

if isfield(params, 'prev_steer') && ~isempty(params.prev_steer)
    last_steer = params.prev_steer;
elseif ~isempty(p_last_steer)
    last_steer = p_last_steer;
else
    last_steer = 0.0;
end

if isfield(params, 'prev_accel') && ~isempty(params.prev_accel)
    last_accel = params.prev_accel;
elseif ~isempty(p_last_accel)
    last_accel = p_last_accel;
else
    last_accel = 0.0;
end

x     = state(1);
y     = state(2);
theta = state(3);
v     = state(4);

% ── 1. Nearest Waypoint & Tracking Errors ──────────────────────────────────
distances = hypot(path(:, 1) - x, path(:, 2) - y);
[~, idx_nearest] = min(distances);

if idx_nearest < size(path, 1)
    p1 = path(idx_nearest, :);
    p2 = path(idx_nearest + 1, :);
else
    p1 = path(max(1, idx_nearest - 1), :);
    p2 = path(idx_nearest, :);
end
path_heading = atan2(p2(2) - p1(2), p2(1) - p1(1));

dx_near = x - p1(1);
dy_near = y - p1(2);
e_y = dy_near * cos(path_heading) - dx_near * sin(path_heading);
e_theta = angdiff(path_heading, theta);

% ── 2. Speed-Adaptive Lookahead Distance ───────────────────────────────────
lookahead_dist = min(max(K_v * max(0.0, v), L_min), L_max);

% Search forward along path for target point at distance lookahead_dist
target_idx = idx_nearest;
while target_idx < size(path, 1) && distances(target_idx) < lookahead_dist
    target_idx = target_idx + 1;
end
target_pt = path(target_idx, :);

% ── 3. Standstill & Near-Goal Boundary Projection ───────────────────────────
% When approaching end of path or stopped at VSL, project strictly forward
% along ego heading axis to ensure alpha never flips across +-pi
d_to_target = hypot(target_pt(1) - x, target_pt(2) - y);

if target_idx >= size(path, 1) || d_to_target < (lookahead_dist * 0.8)
    p_last = path(end, :);
    p_prev = path(max(1, size(path, 1) - 1), :);
    d_seg  = p_last - p_prev;
    len_s  = hypot(d_seg(1), d_seg(2));
    if len_s > 1e-3
        fwd_dir = d_seg / len_s;
    else
        fwd_dir = [cos(theta), sin(theta)];
    end
    
    % Ensure forward projection remains aligned with vehicle heading (dot > 0)
    if (fwd_dir(1) * cos(theta) + fwd_dir(2) * sin(theta)) < 0.2
        fwd_dir = [cos(theta), sin(theta)];
    end
    target_pt = [x, y] + fwd_dir * lookahead_dist;
end

% Check forward heading alignment
dx_tgt = target_pt(1) - x;
dy_tgt = target_pt(2) - y;
lon_proj = dx_tgt * cos(theta) + dy_tgt * sin(theta);
if lon_proj <= 0.2
    % Standstill / VSL clamp: enforce forward target along ego heading
    target_pt = [x + lookahead_dist * cos(theta), y + lookahead_dist * sin(theta)];
    dx_tgt = target_pt(1) - x;
    dy_tgt = target_pt(2) - y;
end

% ── 4. Pure Pursuit Steering Calculation & Slew Rate Clamping ──────────────
target_heading = atan2(dy_tgt, dx_tgt);
alpha = angdiff(target_heading, theta);

% Ackerman steering curvature command
raw_steer = atan2(2.0 * L * sin(alpha), lookahead_dist);

% Saturation clamp: [-max_steer, +max_steer]
clamped_steer = min(max(raw_steer, -max_steer), max_steer);

% Steering slew rate limit: |delta - last_steer| <= slew_rate * dt
max_delta_steer = slew_rate * dt;
delta_cmd = min(max(clamped_steer, last_steer - max_delta_steer), last_steer + max_delta_steer);

% ── 5. Longitudinal Acceleration & Jerk Limiting ───────────────────────────
raw_accel = Kp_v * (v_ref - v);

% Clamp raw acceleration to vehicle physical envelope [-3.5, +2.5] m/s^2
raw_accel = min(max(raw_accel, -3.5), 2.5);

% Longitudinal jerk limit: comfort for acceleration and braking (|da/dt| <= jerk_max)
max_delta_a = jerk_max * dt;
a_cmd = min(max(raw_accel, last_accel - max_delta_a), last_accel + max_delta_a);

% Update persistent state
p_last_steer = delta_cmd;
p_last_accel = a_cmd;

control = [delta_cmd; a_cmd];

end

function d = angdiff(a1, a2)
    d = mod(a1 - a2 + pi, 2 * pi) - pi;
end
