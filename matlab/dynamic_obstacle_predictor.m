function [predicted_trajectories, innov_stats] = dynamic_obstacle_predictor(observations, dt, N_horizon, cfg)
%% DYNAMIC_OBSTACLE_PREDICTOR  Multi-Class EKF Tracking & Trajectory Rollout (Phase 2).
%   [predicted_trajectories, innov_stats] = dynamic_obstacle_predictor(observations, dt, N_horizon, cfg)
%
% Implements the 3-Layer Architecture multi-target Extended Kalman Filter (EKF)
% with class-conditioned Continuous White Noise Acceleration (CWNA) process noise,
% range-dependent measurement covariance injection, Joseph-form covariance updates,
% Mahalanobis gating, 15-tick occlusion coasting, and 2.0s uncertainty rollout.
%
% Theoretical Grounding:
%   - Darms, Rybski & Urmson (IEEE IVS 2008, Boss / DARPA Urban Challenge)
%   - Varma et al. (IDD 2018, India Driving Dataset ODD classes & mobility models)
%   - Argoverse Motion Forecasting Dataset (2.0s rollout standard)
%
% Inputs:
%   observations : struct array of detections from simulate_sensor_detection.m:
%                    .id               - integer tracking ID
%                    .type             - 'cattle','auto_rickshaw','pedestrian','pushcart'
%                    .position         - [x, y] position in meters
%                    .velocity         - [vx, vy] Doppler velocity in m/s
%                    .behavior_profile - string descriptor
%                    .noise_std_pos    - (optional) range-dependent position std (m)
%                    .dist_to_ego      - (optional) distance to ego vehicle (m)
%   dt           : timestep size in seconds (default: 0.1 s)
%   N_horizon    : forward prediction steps (default: 20 steps = 2.0 s at dt=0.1s)
%   cfg          : optional configuration struct:
%                    .reset            - clear persistent tracks when true
%                    .gating_thresh    - chi-square gate (default: 9.488 for chi2_4, 0.05)
%                    .max_coast_ticks  - max ticks to coast missed tracks (default: 15)
%
% Outputs:
%   predicted_trajectories : struct array per active track:
%     .id          - track integer ID
%     .type        - semantic class string
%     .waypoints   - (N_horizon x 2) predicted [x, y] coordinates
%     .covariance  - (N_horizon x 1) cell array of 4x4 covariance matrices
%     .semi_major  - (N_horizon x 1) 2-sigma ellipse semi-major axis (m)
%     .semi_minor  - (N_horizon x 1) 2-sigma ellipse semi-minor axis (m)
%     .orientation - (N_horizon x 1) ellipse orientation angle (rad)
%     .x_est       - (4 x 1) filtered state vector [px, py, vx, vy]'
%     .P_est       - (4 x 4) filtered state covariance matrix
%     .is_coasting - boolean flag (true if track coasted without observation)
%   innov_stats : struct array per track with innovation diagnostics

% ── Persistent Track Registry ──────────────────────────────────────────────
persistent ekf_map tick_count;

% ── Defaults & Config Parsing ──────────────────────────────────────────────
if nargin < 2 || isempty(dt), dt = 0.1; end
if nargin < 3 || isempty(N_horizon), N_horizon = 20; end
if nargin < 4 || isempty(cfg), cfg = struct(); end

if ~isfield(cfg, 'reset'), cfg.reset = false; end
if ~isfield(cfg, 'gating_thresh'), cfg.gating_thresh = 9.488; end % chi2(4, 0.05)
if ~isfield(cfg, 'max_coast_ticks'), cfg.max_coast_ticks = 15; end

% ── Handle Explicit Reset ──────────────────────────────────────────────────
if cfg.reset || isempty(ekf_map)
    ekf_map    = containers.Map('KeyType','char','ValueType','any');
    tick_count = 0;
    if isempty(observations)
        predicted_trajectories = empty_predictions();
        innov_stats            = empty_innov();
        return;
    end
end
tick_count = tick_count + 1;

% ── 4D Kinematic Constant Velocity Model ───────────────────────────────────
% State: x = [px, py, vx, vy]'
F = [1, 0, dt, 0;
     0, 1, 0,  dt;
     0, 0, 1,  0;
     0, 0, 0,  1];

H = eye(4);  % 4D measurement [zx, zy, zvx, zvy]'

% Base discrete Continuous White Noise Acceleration (CWNA) block:
q_pos   = (dt^4) / 4.0;
q_cross = (dt^3) / 2.0;
q_vel   = dt^2;
Q_base  = [q_pos,     0,       q_cross,   0; ...
           0,         q_pos,   0,         q_cross; ...
           q_cross,   0,       q_vel,     0; ...
           0,         q_cross, 0,         q_vel];

% Class acceleration spectral densities (sigma_a in m/s^2)
sigma_a_map = struct();
sigma_a_map.cattle        = 2.2; % erratic lateral wander / sudden darting
sigma_a_map.auto_rickshaw = 1.8; % aggressive weaving & sudden braking
sigma_a_map.pedestrian    = 0.8; % high turning agility, moderate velocity
sigma_a_map.pushcart      = 0.3; % high rolling inertia, straight motion
sigma_a_map.default_      = 1.0;

% Initial state covariance for newly born tracks
P0 = diag([0.25, 0.25, 4.0, 4.0]); % [0.5m pos std, 2.0m/s vel std]

% ── Step 1: Predict Phase for All Existing Tracks ──────────────────────────
existing_keys = keys(ekf_map);
predicted_tracks = containers.Map('KeyType','char','ValueType','any');

for ki = 1:length(existing_keys)
    k_key = existing_keys{ki};
    tr = ekf_map(k_key);
    
    Q_k = get_Q_matrix(tr.type, Q_base, sigma_a_map);
    tr.x_pred = F * tr.x;
    tr.P_pred = F * tr.P * F' + Q_k;
    tr.P_pred = 0.5 * (tr.P_pred + tr.P_pred'); % enforce symmetry
    
    predicted_tracks(k_key) = tr;
end

% ── Step 2: Data Association & Measurement Update ──────────────────────────
num_obs = length(observations);
assigned_track_keys = {};
assigned_obs_indices = [];
innov_stats_list = [];

% Pre-extract measurement vectors and dynamic R matrices
obs_data = struct([]);
for j = 1:num_obs
    ob = observations(j);
    % Position
    px = ob.position(1);
    py = ob.position(2);
    % Velocity
    if isfield(ob, 'velocity') && length(ob.velocity) >= 2
        vx = ob.velocity(1);
        vy = ob.velocity(2);
    else
        vx = 0.0;
        vy = 0.0;
    end
    z = [px; py; vx; vy];
    
    % Range-dependent noise std
    if isfield(ob, 'noise_std_pos') && ~isempty(ob.noise_std_pos) && ob.noise_std_pos > 0
        sigma_p = ob.noise_std_pos;
    elseif isfield(ob, 'dist_to_ego') && ~isempty(ob.dist_to_ego)
        sigma_p = 0.15 + 0.15 * min(ob.dist_to_ego / 35.0, 1.0);
    else
        sigma_p = 0.25;
    end
    sigma_v = 0.20; % Doppler radar velocity noise std
    
    R = diag([sigma_p^2, sigma_p^2, sigma_v^2, sigma_v^2]);
    
    obs_data(j).z  = z;
    obs_data(j).R  = R;
    obs_data(j).id = ob.id;
    obs_data(j).ob = ob;
end

% Direct match by ID first (with Mahalanobis validation)
for j = 1:num_obs
    id_key = num2str(obs_data(j).id);
    if isKey(predicted_tracks, id_key) && ~any(strcmp(assigned_track_keys, id_key))
        tr = predicted_tracks(id_key);
        y_innov = obs_data(j).z - H * tr.x_pred;
        S = H * tr.P_pred * H' + obs_data(j).R;
        d_M2 = y_innov' * (S \ y_innov);
        
        if d_M2 <= cfg.gating_thresh * 2.0 % allow slightly wider gate for known ID
            % Valid match
            [tr_up, inn] = update_track(tr, obs_data(j).z, obs_data(j).R, H, obs_data(j).ob.type);
            ekf_map(id_key) = tr_up;
            assigned_track_keys{end+1} = id_key; %#ok<AGROW>
            assigned_obs_indices(end+1) = j;      %#ok<AGROW>
            innov_stats_list = [innov_stats_list, inn]; %#ok<AGROW>
        end
    end
end

% Greedy Global Nearest Neighbor (GNN) for remaining unassigned detections/tracks
unassigned_obs = setdiff(1:num_obs, assigned_obs_indices);
unassigned_tracks = setdiff(existing_keys, assigned_track_keys);

if ~isempty(unassigned_obs) && ~isempty(unassigned_tracks)
    cost_matrix = Inf(length(unassigned_tracks), length(unassigned_obs));
    for ti = 1:length(unassigned_tracks)
        t_key = unassigned_tracks{ti};
        tr = predicted_tracks(t_key);
        for oi = 1:length(unassigned_obs)
            j = unassigned_obs(oi);
            y_innov = obs_data(j).z - H * tr.x_pred;
            S = H * tr.P_pred * H' + obs_data(j).R;
            d_M2 = y_innov' * (S \ y_innov);
            if d_M2 <= cfg.gating_thresh
                cost_matrix(ti, oi) = d_M2;
            end
        end
    end
    
    % Greedy matching on lowest d_M2
    while true
        [min_val, min_idx] = min(cost_matrix(:));
        if isinf(min_val), break; end
        [t_best, o_best] = ind2sub(size(cost_matrix), min_idx);
        
        t_key = unassigned_tracks{t_best};
        j_obs = unassigned_obs(o_best);
        
        tr = predicted_tracks(t_key);
        [tr_up, inn] = update_track(tr, obs_data(j_obs).z, obs_data(j_obs).R, H, obs_data(j_obs).ob.type);
        ekf_map(t_key) = tr_up;
        assigned_track_keys{end+1} = t_key; %#ok<AGROW>
        assigned_obs_indices(end+1) = j_obs; %#ok<AGROW>
        innov_stats_list = [innov_stats_list, inn]; %#ok<AGROW>
        
        % Invalidate matched row and column
        cost_matrix(t_best, :) = Inf;
        cost_matrix(:, o_best) = Inf;
    end
end

% ── Step 3: Initialize New Tracks (Track Birth) ────────────────────────────
remaining_obs = setdiff(1:num_obs, assigned_obs_indices);
for ri = 1:length(remaining_obs)
    j = remaining_obs(ri);
    new_key = num2str(obs_data(j).id);
    
    new_tr = struct();
    new_tr.id           = obs_data(j).id;
    new_tr.type         = obs_data(j).ob.type;
    new_tr.x            = obs_data(j).z;
    new_tr.P            = P0;
    new_tr.miss_count   = 0;
    new_tr.hit_count    = 1;
    new_tr.is_coasting  = false;
    new_tr.x_pred       = obs_data(j).z;
    new_tr.P_pred       = P0;
    
    ekf_map(new_key) = new_tr;
    assigned_track_keys{end+1} = new_key; %#ok<AGROW>
    
    inn_entry = struct('id', obs_data(j).id, 'innov', zeros(4, 1), ...
                       'innov_norm', 0.0, 'predict_only', false);
    innov_stats_list = [innov_stats_list, inn_entry]; %#ok<AGROW>
end

% ── Step 4: Coasting & Track Pruning ───────────────────────────────────────
% Any track in existing_keys that received NO observation this tick
all_keys_now = keys(ekf_map);
for ki = 1:length(all_keys_now)
    k_key = all_keys_now{ki};
    if ~any(strcmp(assigned_track_keys, k_key))
        tr = predicted_tracks(k_key);
        tr.miss_count = tr.miss_count + 1;
        
        if tr.miss_count <= cfg.max_coast_ticks
            % Coast forward on motion model: estimate = prior prediction
            tr.x = tr.x_pred;
            tr.P = tr.P_pred;
            tr.is_coasting = true;
            ekf_map(k_key) = tr;
            
            inn_entry = struct('id', tr.id, 'innov', nan(4, 1), ...
                               'innov_norm', NaN, 'predict_only', true);
            innov_stats_list = [innov_stats_list, inn_entry]; %#ok<AGROW>
        else
            % Miss count exceeded 15 ticks (pruned on tick 16) -> delete track
            remove(ekf_map, k_key);
        end
    end
end

% ── Step 5: Forward Horizon Rollout & 2-Sigma Confidence Ellipses ─────────
active_keys = keys(ekf_map);
predicted_trajectories = empty_predictions();

for ki = 1:length(active_keys)
    k_key = active_keys{ki};
    tr = ekf_map(k_key);
    Q_k = get_Q_matrix(tr.type, Q_base, sigma_a_map);
    
    waypoints   = zeros(N_horizon, 2);
    cov_cell    = cell(N_horizon, 1);
    semi_major  = zeros(N_horizon, 1);
    semi_minor  = zeros(N_horizon, 1);
    orientation = zeros(N_horizon, 1);
    
    x_roll = tr.x;
    P_roll = tr.P;
    
    for h = 1:N_horizon
        x_roll = F * x_roll;
        P_roll = F * P_roll * F' + Q_k;
        P_roll = 0.5 * (P_roll + P_roll'); % enforce symmetry
        
        waypoints(h, :) = x_roll(1:2)';
        cov_cell{h}     = P_roll;
        
        % Extract 2D positional sub-covariance
        P_pos = P_roll(1:2, 1:2);
        [V, D] = eig(0.5 * (P_pos + P_pos'));
        ev = max(diag(D), 0);
        [max_ev, max_i] = max(ev);
        min_ev = min(ev);
        
        % 2-sigma confidence bounds (2 * sqrt(eigenvalue))
        semi_major(h)  = 2.0 * sqrt(max_ev);
        semi_minor(h)  = 2.0 * sqrt(min_ev);
        orientation(h) = atan2(V(2, max_i), V(1, max_i));
    end
    
    idx = length(predicted_trajectories) + 1;
    predicted_trajectories(idx).id          = tr.id;
    predicted_trajectories(idx).type        = tr.type;
    predicted_trajectories(idx).waypoints   = waypoints;
    predicted_trajectories(idx).covariance  = cov_cell;
    predicted_trajectories(idx).semi_major  = semi_major;
    predicted_trajectories(idx).semi_minor  = semi_minor;
    predicted_trajectories(idx).orientation = orientation;
    predicted_trajectories(idx).x_est       = tr.x;
    predicted_trajectories(idx).P_est       = tr.P;
    predicted_trajectories(idx).is_coasting = tr.is_coasting;
end

if isempty(innov_stats_list)
    innov_stats = empty_innov();
else
    innov_stats = innov_stats_list;
end

end

% ── Helper: Joseph-Form Measurement Update ─────────────────────────────────
function [tr, inn] = update_track(tr, z, R, H, detected_type)
    y_innov = z - H * tr.x_pred;
    S = H * tr.P_pred * H' + R;
    K = tr.P_pred * H' / S;
    
    tr.x = tr.x_pred + K * y_innov;
    
    % Joseph-Form Covariance Update (guarantees positive semi-definiteness)
    I_KH = eye(4) - K * H;
    tr.P = I_KH * tr.P_pred * I_KH' + K * R * K';
    tr.P = 0.5 * (tr.P + tr.P'); % explicit symmetrization
    
    tr.miss_count  = 0;
    tr.hit_count   = tr.hit_count + 1;
    tr.is_coasting = false;
    tr.type        = detected_type; % update type if sensor reclassified
    
    inn = struct('id', tr.id, 'innov', y_innov, ...
                 'innov_norm', norm(y_innov(1:2)), 'predict_only', false);
end

% ── Helper: Get Q Matrix for Class ─────────────────────────────────────────
function Q = get_Q_matrix(type_str, Q_base, sigma_a_map)
    k = lower(type_str);
    k(k == '-') = '_';
    if isfield(sigma_a_map, k)
        sa = sigma_a_map.(k);
    else
        sa = sigma_a_map.default_;
    end
    Q = (sa^2) * Q_base;
end

% ── Helper: Empty Structures ───────────────────────────────────────────────
function p = empty_predictions()
    p = struct('id',{},'type',{},'waypoints',{},'covariance',{}, ...
               'semi_major',{},'semi_minor',{},'orientation',{}, ...
               'x_est',{},'P_est',{},'is_coasting',{});
end

function inn = empty_innov()
    inn = struct('id',{},'innov',{},'innov_norm',{},'predict_only',{});
end
