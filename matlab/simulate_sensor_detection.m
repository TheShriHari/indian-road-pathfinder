function [detections, sensor_log] = simulate_sensor_detection(ground_truth_obs, ego_state, sensor_cfg)
%% SIMULATE_SENSOR_DETECTION  Realistic perception simulation layer.
%   [detections, sensor_log] = simulate_sensor_detection(ground_truth_obs, ego_state, sensor_cfg)
%
% Inserts between the ground-truth obstacle list and the EKF predictor to
% simulate the output of a real camera/LiDAR perception pipeline WITHOUT
% building actual computer vision. Replaces the oracle-feed pattern (where
% perfect position/velocity/type flows directly into the planner) with
% something that forces the EKF to do real filtering work.
%
% SENSOR LIMITATIONS SIMULATED (in order of application):
%   1. Range / FoV gate   — obstacles beyond max_detection_range or outside
%                           field_of_view are not detected at all.
%   2. Detection dropout  — each in-range obstacle has a per-tick probability
%                           of complete non-detection (simulates occlusion /
%                           missed frame). Probability scales with distance.
%   3. Position noise     — Gaussian noise on (x, y), std scaling with range.
%   4. Velocity noise     — Gaussian noise on (vx, vy).
%   5. Classification     — 3% chance per detected obstacle to assign a wrong
%                           type label from the Indian ODD agent set.
%   6. Detection latency  — fixed delay of latency_ticks before the report
%                           is released to the EKF (circular buffer, persistent).
%
% When an obstacle is dropped by the sensor this tick, it is ABSENT from
% `detections`. The EKF's predict-only pass (implemented in
% dynamic_obstacle_predictor.m) handles the gap by propagating the last
% estimate forward with the motion model.
%
% Inputs:
%   ground_truth_obs : struct array with fields:
%                        .id               — unique integer agent id
%                        .type             — ground-truth class string
%                        .position         — [x, y] (m)
%                        .velocity         — [vx, vy] (m/s)
%                        .behavior_profile — string
%   ego_state        : [x, y, theta, v]  (column or row, 4 elements)
%   sensor_cfg       : struct with optional fields:
%                        .max_detection_range  (default 35.0 m)
%                        .field_of_view_deg    (default 140.0 — i.e. ±70°)
%                        .base_dropout_prob    (default 0.05 — 5%)
%                        .std_pos_base         (default 0.3 m)
%                        .std_pos_max          (default 0.5 m)
%                        .std_vel              (default 0.2 m/s)
%                        .misclass_prob        (default 0.03 — 3%)
%                        .latency_ticks        (default 2)
%                        .verbose              (default true — prints events)
%
% Outputs:
%   detections : struct array in same format as ground_truth_obs but with
%                noisy position/velocity, possibly wrong type, possibly fewer
%                entries (dropped obstacles absent). These are the delayed,
%                noisy observations delivered to the EKF.
%   sensor_log : struct with fields:
%                  .n_in_range    — how many obstacles passed range/FoV gate
%                  .n_dropped     — how many were dropped by dropout this tick
%                  .n_misclassed  — how many had wrong type label
%                  .events        — cell array of event description strings

% ── Persistent latency buffer ──────────────────────────────────────────────
% Circular buffer: each slot holds the detections snapshot for that tick.
% Released detections = what was enqueued `latency_ticks` ticks ago.
% Reset externally via:  clear simulate_sensor_detection
persistent lat_buf lat_head lat_tick;

% ── Defaults ──────────────────────────────────────────────────────────────
if nargin < 3 || isempty(sensor_cfg), sensor_cfg = struct(); end

cfg.max_detection_range = 35.0;    % m — max sensor range ahead of ego
cfg.field_of_view_deg   = 140.0;   % total FoV width (±70° from heading)
cfg.base_dropout_prob   = 0.05;    % base probability of missed detection
cfg.std_pos_base        = 0.3;     % position noise std at close range (m)
cfg.std_pos_max         = 0.5;     % position noise std at max range (m)
cfg.std_vel             = 0.2;     % velocity noise std (m/s)
cfg.misclass_prob       = 0.03;    % probability of wrong type label
cfg.latency_ticks       = 2;       % ticks of processing delay
cfg.verbose             = true;    % print dropout/misclass events

% Merge caller-supplied fields on top of defaults
fnames = fieldnames(cfg);
for fi = 1:length(fnames)
    if isfield(sensor_cfg, fnames{fi})
        cfg.(fnames{fi}) = sensor_cfg.(fnames{fi});
    end
end

fov_half_rad = (cfg.field_of_view_deg / 2.0) * pi / 180.0;

% Possible wrong type labels for misclassification (Indian ODD set)
wrong_types = {'cattle', 'auto_rickshaw', 'pedestrian', 'pushcart', 'bicycle', 'dog'};

% ── Initialise latency buffer on first call ────────────────────────────────
buf_size = cfg.latency_ticks + 1;   % need latency_ticks+1 slots
if isempty(lat_buf) || length(lat_buf) ~= buf_size
    lat_buf  = cell(buf_size, 1);   % each cell = struct array or empty
    for bi = 1:buf_size
        lat_buf{bi} = struct([]);
    end
    lat_head = 1;   % next write slot (1-indexed, wraps mod buf_size)
    lat_tick = 0;
end
lat_tick = lat_tick + 1;

% ── Unpack ego pose ────────────────────────────────────────────────────────
ego_x     = ego_state(1);
ego_y     = ego_state(2);
ego_theta = ego_state(3);

% ── Sensor log initialisation ──────────────────────────────────────────────
sensor_log.n_in_range   = 0;
sensor_log.n_dropped    = 0;
sensor_log.n_misclassed = 0;
sensor_log.events       = {};

% ── Process ground-truth observations through sensor model ─────────────────
current_frame = struct([]);   % what the sensor "sees" this tick (before delay)
out_idx = 0;

for k = 1:length(ground_truth_obs)
    obs = ground_truth_obs(k);
    obs_x = obs.position(1);
    obs_y = obs.position(2);

    % ── 1. Range check ────────────────────────────────────────────────────
    dx = obs_x - ego_x;
    dy = obs_y - ego_y;
    dist = hypot(dx, dy);

    if dist > cfg.max_detection_range
        % Beyond sensor range — invisible
        continue;
    end

    % ── 2. Field-of-view check ────────────────────────────────────────────
    % Bearing from ego to obstacle in ego-frame heading coordinates
    bearing = atan2(dy, dx);
    angle_diff = mod(bearing - ego_theta + pi, 2*pi) - pi;   % wrap to [-π, π]
    if abs(angle_diff) > fov_half_rad
        % Outside FoV cone — invisible
        continue;
    end

    sensor_log.n_in_range = sensor_log.n_in_range + 1;

    % ── 3. Detection dropout ──────────────────────────────────────────────
    % Dropout probability scales with distance: p = base + (dist/max_range)*base
    % so at max range the chance is 2x base. Smaller objects (pedestrians,
    % dogs) get an additional +2% (simulated by type check).
    p_dropout = cfg.base_dropout_prob * (1.0 + dist / cfg.max_detection_range);
    small_types = {'pedestrian', 'dog', 'bicycle'};
    if any(strcmpi(obs.type, small_types))
        p_dropout = p_dropout + 0.02;
    end
    p_dropout = min(p_dropout, 0.25);   % cap at 25%

    if rand() < p_dropout
        % Obstacle DROPPED this frame — not included in detections
        sensor_log.n_dropped = sensor_log.n_dropped + 1;
        event_str = sprintf('[SENSOR] DROPOUT  tick=%d  id=%d  type=%-14s  dist=%.1fm  p_drop=%.1f%%', ...
            lat_tick, obs.id, obs.type, dist, p_dropout * 100);
        sensor_log.events{end+1} = event_str;
        if cfg.verbose
            fprintf('%s\n', event_str);
        end
        continue;   % absent from current_frame
    end

    % ── 4. Position noise (scales with distance) ───────────────────────────
    % std_pos interpolates linearly from std_pos_base (at 0 m) to std_pos_max
    % (at max_detection_range). Real sensors are less precise at range.
    t_dist = min(dist / cfg.max_detection_range, 1.0);
    std_pos = cfg.std_pos_base + t_dist * (cfg.std_pos_max - cfg.std_pos_base);
    noisy_x = obs_x + randn() * std_pos;
    noisy_y = obs_y + randn() * std_pos;

    % ── 5. Velocity noise ──────────────────────────────────────────────────
    noisy_vx = obs.velocity(1) + randn() * cfg.std_vel;
    noisy_vy = obs.velocity(2) + randn() * cfg.std_vel;

    % ── 6. Classification uncertainty ─────────────────────────────────────
    reported_type = obs.type;
    is_misclassed = false;
    if rand() < cfg.misclass_prob
        % Pick a random wrong type that differs from ground truth
        candidates = wrong_types(~strcmpi(wrong_types, obs.type));
        if ~isempty(candidates)
            reported_type = candidates{randi(length(candidates))};
            is_misclassed = true;
            sensor_log.n_misclassed = sensor_log.n_misclassed + 1;
            event_str = sprintf('[SENSOR] MISCLASS tick=%d  id=%d  GT_type=%-14s  RPT_type=%-14s  dist=%.1fm', ...
                lat_tick, obs.id, obs.type, reported_type, dist);
            sensor_log.events{end+1} = event_str;
            if cfg.verbose
                fprintf('%s\n', event_str);
            end
        end
    end

    % ── Assemble noisy detection ──────────────────────────────────────────
    out_idx = out_idx + 1;
    current_frame(out_idx).id               = obs.id;
    current_frame(out_idx).type             = reported_type;
    current_frame(out_idx).position         = [noisy_x, noisy_y];
    current_frame(out_idx).velocity         = [noisy_vx, noisy_vy];
    current_frame(out_idx).behavior_profile = obs.behavior_profile;
    % Store ground-truth type for diagnostic access (not used by EKF)
    current_frame(out_idx).gt_type          = obs.type;
    current_frame(out_idx).noise_std_pos    = std_pos;
    current_frame(out_idx).dist_to_ego      = dist;
end

% ── Write current_frame into latency buffer ────────────────────────────────
write_slot = lat_head;
if isempty(current_frame)
    lat_buf{write_slot} = struct([]);
else
    lat_buf{write_slot} = current_frame;
end

% Advance write head (circular, 1-indexed)
lat_head = mod(lat_head, buf_size) + 1;

% ── Read the delayed output (latency_ticks ago) ────────────────────────────
% After latency_ticks ticks the buffer is full; before that, return empty.
if lat_tick <= cfg.latency_ticks
    % Buffer not yet warmed up — deliver empty (no detections yet)
    detections = struct([]);
else
    % Read slot = current write_slot (which now points to oldest entry)
    read_slot = lat_head;   % lat_head was just advanced past write_slot
    delayed = lat_buf{read_slot};
    if isempty(delayed)
        detections = struct([]);
    else
        detections = delayed;
    end
end

end
