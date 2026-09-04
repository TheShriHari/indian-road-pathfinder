function [predicted_trajectories, innov_stats] = dynamic_obstacle_predictor(observations, dt, N_horizon)
% DYNAMIC_OBSTACLE_PREDICTOR  Per-agent Extended Kalman Filter trajectory predictor.
%   [predicted_trajectories, innov_stats] = dynamic_obstacle_predictor(observations, dt, N_horizon)
%
% ALGORITHM ATTRIBUTION:
%   Adapted from PythonRobotics/Localization/extended_kalman_filter/extended_kalman_filter.py
%   (author: Atsushi Sakai, @Atsushi_twi, MIT License).
%   Adaptations made for this project:
%     - State vector changed from [x, y, yaw, v] to [x, y, vx, vy] (constant-velocity
%       model; no control input needed since we observe obstacles, not control them)
%     - Linear F/H matrices used (no Jacobians needed for linear CV model)
%     - Per-agent persistent state maintained via containers.Map keyed by agent id
%     - Per-agent-type process noise Q tuning added for Indian road agent classes
%     - N_horizon forward propagation without measurement update for trajectory output
%     - Predict-only pass for known agents absent from current observations (dropout
%       handling): EKF state coasts forward on the motion model without a measurement
%       update, simulating the filter's behaviour during sensor dropout/occlusion gaps.
%     - Optional second output innov_stats for external innovation logging.
%
% Inputs:
%   observations : struct array with fields:
%                    .id       - unique integer agent id
%                    .type     - 'cattle','auto_rickshaw','pedestrian','pushcart'
%                    .position - [x, y]   observed position (m)
%                    .velocity - [vx, vy] initial/observed velocity (m/s)
%   dt           : timestep size (s)
%   N_horizon    : number of steps to predict ahead
%
% Outputs:
%   predicted_trajectories : struct array, one per agent:
%     .id         - agent id
%     .type       - agent type string
%     .waypoints  - (N_horizon x 2) predicted [x, y] positions
%     .covariance - (N_horizon x 1) cell array of 4x4 state covariance matrices
%                   (previously declared but unpopulated; now fully computed)
%   innov_stats  : struct array, one per agent that received a MEASUREMENT UPDATE
%     .id         - agent id
%     .innov      - 2x1 innovation vector [innov_x; innov_y]
%     .innov_norm - scalar norm of innovation
%     .predict_only - false (measurement update ran this tick)
%   (agents that were in predict-only mode due to dropout have .predict_only = true
%   and .innov = [NaN; NaN])

if nargin < 2, dt = 0.1; end
if nargin < 3, N_horizon = 35; end

% -----------------------------------------------------------------------
% Persistent EKF state registry.
% ekf_map: containers.Map  (string key = agent id) -> struct(.x 4x1, .P 4x4, .type str, .last_seen int)
% Persists across calls so the filter accumulates information over time.
% Reset externally by calling:  clear dynamic_obstacle_predictor
% -----------------------------------------------------------------------
persistent ekf_map tick_count;
if isempty(ekf_map)
    ekf_map    = containers.Map('KeyType','char','ValueType','any');
    tick_count = 0;
end
tick_count = tick_count + 1;

% -----------------------------------------------------------------------
% EKF matrices — constant-velocity model
% State: x = [x_pos; y_pos; vx; vy]
%
% State transition (predict step):
%   x_{k+1} = F * x_k
% F = [1 0 dt 0 ]    (x_new  = x + vx*dt)
%     [0 1 0  dt]    (y_new  = y + vy*dt)
%     [0 0 1  0 ]    (vx_new = vx       )
%     [0 0 0  1 ]    (vy_new = vy       )
% -----------------------------------------------------------------------
F = [1, 0, dt, 0;
     0, 1, 0,  dt;
     0, 0, 1,  0;
     0, 0, 0,  1];

% Measurement matrix — we observe position only
% z = H * x  =>  z = [x_pos; y_pos]
H = [1, 0, 0, 0;
     0, 1, 0, 0];

% Measurement noise covariance (position sensor std ~ 0.5 m)
% NOTE: With the sensor simulation layer active, actual measurement noise
% ranges from 0.3m (close) to 0.5m (far). R is tuned to match this range.
R = diag([0.25, 0.25]);

% Initial state covariance used on first encounter of a new agent
P0 = diag([1.0, 1.0, 4.0, 4.0]);

% -----------------------------------------------------------------------
% Process noise Q — tuned per agent class
% Higher Q  =>  filter assumes more erratic, less predictable motion
%               (wider predicted distribution, more weight to measurements)
% Lower  Q  =>  filter trusts the constant-velocity model more
%               (tighter, smoother predictions)
% -----------------------------------------------------------------------
Q_table = struct();
Q_table.cattle       = diag([0.04, 0.04, 0.36, 0.36]); % erratic lateral wander
Q_table.auto_rickshaw= diag([0.02, 0.02, 0.25, 0.25]); % aggressive weaving
Q_table.pedestrian   = diag([0.01, 0.01, 0.04, 0.04]); % slow, relatively predictable
Q_table.pushcart     = diag([0.01, 0.01, 0.02, 0.02]); % slow, stable
Q_table.default_     = diag([0.02, 0.02, 0.09, 0.09]); % unknown agents

% -----------------------------------------------------------------------
% Build a set of observed IDs for quick lookup (for dropout handling)
% -----------------------------------------------------------------------
num_obs = length(observations);
observed_ids = cell(num_obs, 1);
for i = 1:num_obs
    observed_ids{i} = num2str(observations(i).id);
end

% -----------------------------------------------------------------------
% PREDICT-ONLY PASS for known agents not in current observations (dropout)
% The EKF's predict step is run for any agent that:
%   - exists in ekf_map (was seen previously)
%   - is NOT in the current observation list (dropped by sensor this tick)
% This keeps the state estimate coasting forward via the motion model,
% which is exactly what a real EKF does during sensor gaps.
% -----------------------------------------------------------------------
DROPOUT_COAST_TICKS = 15;   % after this many consecutive missed ticks, forget the agent
all_known_keys = keys(ekf_map);
for ki = 1:length(all_known_keys)
    k_key = all_known_keys{ki};
    if ~any(strcmp(observed_ids, k_key))
        % This agent was not detected this tick — run predict-only
        s = ekf_map(k_key);
        s.missed_ticks = s.missed_ticks + 1;
        if s.missed_ticks <= DROPOUT_COAST_TICKS
            % Select Q from stored type
            type_key = lower(s.type);
            type_key(type_key == '-') = '_';
            if isfield(Q_table, type_key)
                Q = Q_table.(type_key);
            else
                Q = Q_table.default_;
            end
            % Predict step only — no measurement update
            s.x = F * s.x;
            s.P = F * s.P * F' + Q;
            ekf_map(k_key) = s;
        else
            % Too many consecutive missed ticks — remove agent from map
            remove(ekf_map, k_key);
        end
    end
end

% -----------------------------------------------------------------------
% MEASUREMENT UPDATE PASS for observed agents
% -----------------------------------------------------------------------
predicted_trajectories = struct('id',{},'type',{},'waypoints',{},'covariance',{});
innov_stats_arr = struct('id',{},'innov',{},'innov_norm',{},'predict_only',{});

for i = 1:num_obs
    obs = observations(i);
    key = num2str(obs.id);          % string key for containers.Map
    z   = obs.position(:);         % 2x1 measurement vector

    % Select Q for this agent type
    type_key = lower(obs.type);
    type_key(type_key == '-') = '_';   % 'auto_rickshaw' -> already safe
    if isfield(Q_table, type_key)
        Q = Q_table.(type_key);
    else
        Q = Q_table.default_;
    end

    % ----------------------------------------------------------------
    % Initialise EKF state on FIRST encounter of this agent id
    % ----------------------------------------------------------------
    if ~isKey(ekf_map, key)
        s.x = [obs.position(1); obs.position(2);
               obs.velocity(1); obs.velocity(2)];
        s.P           = P0;
        s.type        = obs.type;   % store type for dropout predict-only pass
        s.missed_ticks = 0;
        ekf_map(key) = s;
    end

    s = ekf_map(key);   % retrieve persistent state
    s.missed_ticks = 0; % reset consecutive miss counter since detected this tick

    % ==== EKF PREDICT STEP ==========================================
    % Project state and covariance forward one timestep
    x_pred = F * s.x;              % a priori state estimate
    P_pred = F * s.P * F' + Q;     % a priori covariance estimate

    % ==== EKF UPDATE STEP ===========================================
    % Correct the prediction using the new position measurement
    z_pred = H * x_pred;           % predicted measurement (2x1)
    innov  = z - z_pred;           % innovation (measurement residual, 2x1)
    S      = H * P_pred * H' + R;  % innovation covariance (2x2)
    K      = P_pred * H' / S;      % Kalman gain (4x2)
    x_est  = x_pred + K * innov;   % a posteriori state estimate
    % Joseph-form covariance update (numerically stable):
    IKH    = eye(4) - K * H;
    P_est  = IKH * P_pred * IKH' + K * R * K';  % a posteriori covariance

    % Persist updated state
    s.x = x_est;
    s.P = P_est;
    s.type = obs.type;   % update type (may have changed due to misclassification)
    ekf_map(key) = s;

    % Print diagnostics for FIRST agent each call (console verification)
    if i == 1
        fprintf('[EKF] id=%-3d  type=%-14s  innov=[%+.4f, %+.4f]  |innov|=%.4f  P_trace=%.5f\n', ...
                obs.id, obs.type, innov(1), innov(2), norm(innov), trace(P_est));
    end

    % Record innovation statistics for this agent (measurement update ran)
    innov_stats_arr(end+1).id           = obs.id;
    innov_stats_arr(end).innov          = innov;
    innov_stats_arr(end).innov_norm     = norm(innov);
    innov_stats_arr(end).predict_only   = false;

    % ==== HORIZON PROPAGATION =======================================
    % Roll the filter's state forward N_horizon steps using the motion model.
    % No measurement updates beyond step 1 — pure model propagation.
    % Covariance grows with each step, reflecting increasing uncertainty.
    waypoints = zeros(N_horizon, 2);
    covs      = cell(N_horizon, 1);
    x_h = x_est;
    P_h = P_est;
    for t = 1:N_horizon
        x_h        = F * x_h;           % constant-velocity propagation
        P_h        = F * P_h * F' + Q;  % uncertainty grows
        waypoints(t, :) = x_h(1:2)';
        covs{t}    = P_h;
    end

    predicted_trajectories(end+1).id         = obs.id;
    predicted_trajectories(end).type         = obs.type;
    predicted_trajectories(end).waypoints    = waypoints;
    predicted_trajectories(end).covariance   = covs;  % now fully populated
end

% -----------------------------------------------------------------------
% Also emit horizon trajectories for coasting (dropout) agents.
% These are included in predictions so BSM/planner can still react to them
% via the EKF's last-known estimate, not "forget" them entirely.
% -----------------------------------------------------------------------
for ki = 1:length(all_known_keys)
    k_key = all_known_keys{ki};
    if ~any(strcmp(observed_ids, k_key)) && isKey(ekf_map, k_key)
        s = ekf_map(k_key);
        % Select Q
        type_key = lower(s.type);
        type_key(type_key == '-') = '_';
        if isfield(Q_table, type_key)
            Q = Q_table.(type_key);
        else
            Q = Q_table.default_;
        end
        % Propagate from current (already-predicted) state
        waypoints = zeros(N_horizon, 2);
        covs      = cell(N_horizon, 1);
        x_h = s.x;
        P_h = s.P;
        for t = 1:N_horizon
            x_h = F * x_h;
            P_h = F * P_h * F' + Q;
            waypoints(t, :) = x_h(1:2)';
            covs{t} = P_h;
        end
        predicted_trajectories(end+1).id       = str2double(k_key);
        predicted_trajectories(end).type       = s.type;
        predicted_trajectories(end).waypoints  = waypoints;
        predicted_trajectories(end).covariance = covs;

        % Record in innov_stats as predict-only (no measurement this tick)
        innov_stats_arr(end+1).id           = str2double(k_key);
        innov_stats_arr(end).innov          = [NaN; NaN];
        innov_stats_arr(end).innov_norm     = NaN;
        innov_stats_arr(end).predict_only   = true;
    end
end

% Return innov_stats (empty struct array if no agents seen)
if isempty(innov_stats_arr)
    innov_stats = struct('id',{},'innov',{},'innov_norm',{},'predict_only',{});
else
    innov_stats = innov_stats_arr;
end

end
