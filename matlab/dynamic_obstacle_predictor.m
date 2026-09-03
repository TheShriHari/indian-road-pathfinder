function predicted_trajectories = dynamic_obstacle_predictor(observations, dt, N_horizon)
% DYNAMIC_OBSTACLE_PREDICTOR  Per-agent Extended Kalman Filter trajectory predictor.
%   predicted_trajectories = dynamic_obstacle_predictor(observations, dt, N_horizon)
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

if nargin < 2, dt = 0.1; end
if nargin < 3, N_horizon = 20; end

% -----------------------------------------------------------------------
% Persistent EKF state registry.
% ekf_map: containers.Map  (string key = agent id) -> struct(.x 4x1, .P 4x4)
% Persists across calls so the filter accumulates information over time.
% Reset externally by calling:  clear dynamic_obstacle_predictor
% -----------------------------------------------------------------------
persistent ekf_map;
if isempty(ekf_map)
    ekf_map = containers.Map('KeyType','char','ValueType','any');
end

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

num_obs = length(observations);
predicted_trajectories = struct('id',{},'type',{},'waypoints',{},'covariance',{});

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
        s.P = P0;
        ekf_map(key) = s;
    end

    s = ekf_map(key);   % retrieve persistent state

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
    ekf_map(key) = s;

    % Print diagnostics for FIRST agent each call (console verification)
    if i == 1
        fprintf('[EKF] id=%-3d  type=%-14s  innov=[%+.3f, %+.3f]  |innov|=%.4f  P_trace=%.5f\n', ...
                obs.id, obs.type, innov(1), innov(2), norm(innov), trace(P_est));
    end

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

    predicted_trajectories(i).id         = obs.id;
    predicted_trajectories(i).type       = obs.type;
    predicted_trajectories(i).waypoints  = waypoints;
    predicted_trajectories(i).covariance = covs;  % now fully populated
end
end
