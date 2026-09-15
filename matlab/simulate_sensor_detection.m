function [detections, sensor_log] = simulate_sensor_detection(ground_truth_obs, ego_state, sensor_cfg)
%% SIMULATE_SENSOR_DETECTION  Realistic perception simulation layer (Phase 1).
%   [detections, sensor_log] = simulate_sensor_detection(ground_truth_obs, ego_state, sensor_cfg)
%
% Implements the 3-Layer Architecture synthetic perception and sensor degradation
% model for autonomous navigation on unstructured Indian roads.
%
% SENSOR PIPELINE STAGES:
%   1. Spatial Euclidean Range Gate (R_max = 35.0 m)
%   2. Azimuth Horizontal FoV Gate (Phi = 140 deg, +/- 70 deg with 4-quadrant atan2 wrap)
%   3. Quadratic Bernoulli Dropout:
%        P_drop(d) = P_base + (P_max - P_base)*(d/R_max)^2 + delta_class
%        (delta_class = +0.02 for 'pedestrian' and 'cattle')
%   4. Heteroscedastic Range-Dependent Measurement Noise:
%        sigma_p(d) = sigma_min + (sigma_max - sigma_min)*(d/R_max)
%        (sigma_min = 0.15 m, sigma_max = 0.30 m)
%        Doppler velocity noise: sigma_v = 0.20 m/s
%   5. Semantic Categorical Confusion Matrix:
%        3% uniform mutation into Indian ODD set:
%        {'cattle', 'auto_rickshaw', 'pedestrian', 'pushcart'} \ {gt_type}
%   6. Transport Delay (FIFO Buffer):
%        Z_out(t) = Z_in(t - 2*dt) for latency_ticks = 2 (200 ms latency)
%
% Inputs:
%   ground_truth_obs : struct array with fields:
%                        .id               — unique integer agent id
%                        .type             — ground-truth class string
%                        .position         — [x, y] (m)
%                        .velocity         — [vx, vy] (m/s)
%                        .behavior_profile — string
%   ego_state        : [x, y, theta] or [x, y, theta, v]
%   sensor_cfg       : struct with optional fields:
%                        .max_detection_range  (default 35.0 m)
%                        .field_of_view_deg    (default 140.0 deg -> +/-70 deg)
%                        .p_base / .base_dropout_prob (default 0.05)
%                        .p_max                (default 0.10)
%                        .std_pos_min / .std_pos_base (default 0.15 m)
%                        .std_pos_max          (default 0.30 m)
%                        .std_vel              (default 0.20 m/s)
%                        .misclass_prob        (default 0.03)
%                        .latency_ticks        (default 2)
%                        .verbose              (default false)
%                        .reset                (default false; if true, clears FIFO)
%
% Outputs:
%   detections : struct array matching downstream EKF tracker interfaces
%   sensor_log : struct with counts (.n_in_range, .n_dropped, .n_misclassed) and .events

% ── Persistent latency FIFO queue ──────────────────────────────────────────
persistent fifo_queue tick_counter;

% ── Explicit State Reset ───────────────────────────────────────────────────
if (nargin >= 3 && isstruct(sensor_cfg) && isfield(sensor_cfg, 'reset') && sensor_cfg.reset) ...
   || (nargin >= 1 && isempty(ground_truth_obs) && nargin >= 2 && isempty(ego_state))
    fifo_queue   = {};
    tick_counter = 0;
    if isempty(ground_truth_obs)
        detections = struct([]);
        sensor_log = struct('n_in_range', 0, 'n_dropped', 0, 'n_misclassed', 0, 'events', {{}});
        return;
    end
end

if isempty(fifo_queue)
    fifo_queue = {};
    tick_counter = 0;
end
tick_counter = tick_counter + 1;

% ── Parse Configuration Defaults ───────────────────────────────────────────
if nargin < 3 || isempty(sensor_cfg), sensor_cfg = struct(); end

cfg.max_detection_range = 35.0;    % m — max sensor range
cfg.field_of_view_deg   = 140.0;   % total FoV width (+/-70 deg)
cfg.p_base              = 0.05;    % baseline near-field dropout (5%)
cfg.p_max               = 0.10;    % max far-field baseline dropout (10%)
cfg.std_pos_min         = 0.15;    % position noise std at close range (0.15 m)
cfg.std_pos_max         = 0.30;    % position noise std at max range (0.30 m)
cfg.std_vel             = 0.20;    % velocity noise std (0.20 m/s)
cfg.misclass_prob       = 0.03;    % probability of wrong type label (3%)
cfg.latency_ticks       = 2;       % 2 ticks (200 ms latency at dt=0.1s)
cfg.verbose             = false;   % print dropout/misclass events
cfg.reset               = false;

% Backward compatibility mapping for legacy config names
if isfield(sensor_cfg, 'base_dropout_prob') && ~isfield(sensor_cfg, 'p_base')
    sensor_cfg.p_base = sensor_cfg.base_dropout_prob;
end
if isfield(sensor_cfg, 'std_pos_base') && ~isfield(sensor_cfg, 'std_pos_min')
    sensor_cfg.std_pos_min = sensor_cfg.std_pos_base;
end

% Merge caller-supplied fields
fnames = fieldnames(sensor_cfg);
for fi = 1:length(fnames)
    cfg.(fnames{fi}) = sensor_cfg.(fnames{fi});
end

fov_half_rad = (cfg.field_of_view_deg / 2.0) * (pi / 180.0);

% Core Indian ODD class dictionary (grounded in IDD dataset)
indian_odd_classes = {'cattle', 'auto_rickshaw', 'pedestrian', 'pushcart'};

% ── Sensor log initialisation ──────────────────────────────────────────────
sensor_log.n_in_range   = 0;
sensor_log.n_dropped    = 0;
sensor_log.n_misclassed = 0;
sensor_log.events       = {};

% ── Unpack ego pose ────────────────────────────────────────────────────────
ego_x     = ego_state(1);
ego_y     = ego_state(2);
ego_theta = ego_state(3);

% ── Process ground-truth observations through sensor model ─────────────────
current_frame = struct([]);   % observations captured at current tick
out_idx = 0;

for k = 1:length(ground_truth_obs)
    obs = ground_truth_obs(k);
    obs_x = obs.position(1);
    obs_y = obs.position(2);

    % ── 1. Spatial Euclidean Range Gate ───────────────────────────────────
    dx = obs_x - ego_x;
    dy = obs_y - ego_y;
    dist = hypot(dx, dy);

    if dist > cfg.max_detection_range
        % Beyond sensor range — pruned
        continue;
    end

    % ── 2. Azimuth Horizontal FoV Gate (with branch-cut wrapping) ──────────
    bearing = atan2(dy, dx);
    % Wrap bearing difference to [-pi, pi] robustly across +/-pi
    angle_diff = atan2(sin(bearing - ego_theta), cos(bearing - ego_theta));
    if abs(angle_diff) > fov_half_rad
        % Outside FoV angular cone — pruned
        continue;
    end

    sensor_log.n_in_range = sensor_log.n_in_range + 1;

    % ── 3. Non-Linear Stochastic Dropout (Bernoulli Sampling) ──────────────
    % Quadratic range scaling: P_drop(d) = P_base + (P_max - P_base)*(d/R_max)^2 + delta_class
    norm_dist = min(dist / cfg.max_detection_range, 1.0);
    p_dropout = cfg.p_base + (cfg.p_max - cfg.p_base) * (norm_dist ^ 2);

    % Class cross-section penalty (+2% for pedestrian and cattle)
    if any(strcmpi(obs.type, {'pedestrian', 'cattle'}))
        p_dropout = p_dropout + 0.02;
    end
    p_dropout = min(p_dropout, 1.0);

    if rand() < p_dropout
        % Obstacle DROPPED this frame
        sensor_log.n_dropped = sensor_log.n_dropped + 1;
        event_str = sprintf('[SENSOR] DROPOUT  tick=%d  id=%d  type=%-14s  dist=%.2fm  p_drop=%.1f%%', ...
            tick_counter, obs.id, obs.type, dist, p_dropout * 100);
        sensor_log.events{end+1} = event_str;
        if cfg.verbose
            fprintf('%s\n', event_str);
        end
        continue;   % excluded from current frame
    end

    % ── 4. Heteroscedastic Range-Dependent Measurement Noise ───────────────
    % Position std scales linearly with range:
    % sigma_p(d) = sigma_min + (sigma_max - sigma_min)*(d/R_max)
    std_pos = cfg.std_pos_min + (cfg.std_pos_max - cfg.std_pos_min) * norm_dist;
    noisy_x = obs_x + randn() * std_pos;
    noisy_y = obs_y + randn() * std_pos;

    % Doppler velocity measurement noise: sigma_v = 0.20 m/s
    noisy_vx = obs.velocity(1) + randn() * cfg.std_vel;
    noisy_vy = obs.velocity(2) + randn() * cfg.std_vel;

    % ── 5. Semantic Label Confusion Matrix (Indian ODD) ────────────────────
    reported_type = obs.type;
    if rand() < cfg.misclass_prob
        % Uniform sample from Indian ODD classes excluding ground truth
        candidates = indian_odd_classes(~strcmpi(indian_odd_classes, obs.type));
        if isempty(candidates)
            candidates = indian_odd_classes;
        end
        reported_type = candidates{randi(length(candidates))};
        sensor_log.n_misclassed = sensor_log.n_misclassed + 1;
        event_str = sprintf('[SENSOR] MISCLASS tick=%d  id=%d  GT_type=%-14s  RPT_type=%-14s  dist=%.2fm', ...
            tick_counter, obs.id, obs.type, reported_type, dist);
        sensor_log.events{end+1} = event_str;
        if cfg.verbose
            fprintf('%s\n', event_str);
        end
    end

    % ── Assemble detection struct ──────────────────────────────────────────
    out_idx = out_idx + 1;
    current_frame(out_idx).id               = obs.id;
    current_frame(out_idx).type             = reported_type;
    current_frame(out_idx).position         = [noisy_x, noisy_y];
    current_frame(out_idx).velocity         = [noisy_vx, noisy_vy];
    if isfield(obs, 'behavior_profile')
        current_frame(out_idx).behavior_profile = obs.behavior_profile;
    else
        current_frame(out_idx).behavior_profile = 'nominal';
    end
    current_frame(out_idx).gt_type          = obs.type;
    current_frame(out_idx).noise_std_pos    = std_pos;
    current_frame(out_idx).dist_to_ego      = dist;
end

% ── 6. Transport Delay (FIFO Buffer) ───────────────────────────────────────
fifo_queue{end+1} = current_frame;

if length(fifo_queue) > cfg.latency_ticks
    delayed = fifo_queue{1};
    fifo_queue(1) = [];
    if isempty(delayed)
        detections = struct([]);
    else
        detections = delayed;
    end
else
    detections = struct([]);
end

end
