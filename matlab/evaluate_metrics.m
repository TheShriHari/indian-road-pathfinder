function metrics = evaluate_metrics(time_history, state_history, control_history, latency_history, obstacles_history, goal_pose)
% EVALUATE_METRICS  Computes quantitative evaluation metrics for the autonomous pathfinder.
%   metrics = evaluate_metrics(time_history, state_history, control_history,
%                              latency_history, obstacles_history, goal_pose)
%
% IMPORTANT: goal_pose is now a required parameter (not a hardcoded constant).
%   Completion is assessed against the scenario's ACTUAL goal position, not
%   a fixed magic number like 50.0 that was present in the previous version.
%
% Inputs:
%   time_history      : (1 x N)  time vector [s]
%   state_history     : (N x 4)  [x, y, theta, v] at each step
%   control_history   : (N x 2)  [delta, a] at each step
%   latency_history   : (1 x M)  replanning latencies [ms] (M <= N, nonzero entries only)
%   obstacles_history : (1 x N)  cell array; each cell = struct array of obstacle positions
%   goal_pose         : [x, y, theta]  actual scenario goal (from generate_scenarios)
%
% Outputs:
%   metrics : struct with fields:
%     .mean_latency_ms          (ms)
%     .max_latency_ms           (ms)
%     .avg_steer_rate_deg_s     (deg/s)
%     .avg_jerk_m_s3            (m/s^3)
%     .max_jerk_m_s3            (m/s^3)
%     .min_safety_clearance_m   (m)
%     .final_goal_dist_m        (m)    NEW: actual distance from final position to goal
%     .completion_status        (string)
%     .completion_rate          (%)

if isempty(time_history) || numel(time_history) < 2
    metrics.mean_latency_ms        = NaN;
    metrics.max_latency_ms         = NaN;
    metrics.avg_steer_rate_deg_s   = NaN;
    metrics.avg_jerk_m_s3          = NaN;
    metrics.max_jerk_m_s3          = NaN;
    metrics.min_safety_clearance_m = NaN;
    metrics.final_goal_dist_m      = NaN;
    metrics.completion_status      = 'NO DATA';
    metrics.completion_rate        = 0.0;
    return;
end

dt = mean(diff(time_history));
if dt <= 0, dt = 0.1; end  % safety guard

% 1. Replanning Latency
if ~isempty(latency_history)
    metrics.mean_latency_ms = mean(latency_history);
    metrics.max_latency_ms  = max(latency_history);
else
    metrics.mean_latency_ms = 0.0;
    metrics.max_latency_ms  = 0.0;
end

% 2. Path Smoothness — steering rate and longitudinal jerk
if size(control_history, 1) >= 2
    steer_angles = control_history(:, 1);
    steer_rate   = diff(steer_angles) / dt;
    metrics.avg_steer_rate_deg_s = mean(abs(steer_rate)) * (180/pi);

    accelerations = control_history(:, 2);
    jerk          = diff(accelerations) / dt;
    metrics.avg_jerk_m_s3 = mean(abs(jerk));
    metrics.max_jerk_m_s3 = max(abs(jerk));
else
    metrics.avg_steer_rate_deg_s = 0.0;
    metrics.avg_jerk_m_s3        = 0.0;
    metrics.max_jerk_m_s3        = 0.0;
end

% 3. Minimum Safety Clearance to any obstacle across all timesteps
min_clearance = Inf;
num_steps = size(state_history, 1);
for t = 1:num_steps
    ego_pos = state_history(t, 1:2);
    if t > length(obstacles_history), break; end
    obs_at_t = obstacles_history{t};
    if isempty(obs_at_t), continue; end
    for k = 1:length(obs_at_t)
        d = norm(ego_pos - obs_at_t(k).position);
        if d < min_clearance
            min_clearance = d;
        end
    end
end
if isinf(min_clearance)
    min_clearance = Inf;  % no obstacles present
end
metrics.min_safety_clearance_m = min_clearance;

% 4. Final distance to actual goal (not a hardcoded 50.0)
final_pos = state_history(end, 1:2);
metrics.final_goal_dist_m = norm(final_pos - goal_pose(1:2));

% 5. Completion status using actual goal position
GOAL_REACH_THRESH = 3.0;  % m — within 3 m of goal counts as reached
SAFETY_THRESH     = 0.8;  % m — minimum clearance for "clean" completion

if metrics.final_goal_dist_m < GOAL_REACH_THRESH && min_clearance > SAFETY_THRESH
    metrics.completion_status = 'SUCCESS (100%)';
    metrics.completion_rate   = 100.0;
elseif metrics.final_goal_dist_m < GOAL_REACH_THRESH
    metrics.completion_status = 'SUCCESS — COLLISION/CLOSE CALL';
    metrics.completion_rate   = 90.0;
else
    % Partial: fraction of X-axis distance covered
    start_x = state_history(1, 1);
    prog = (final_pos(1) - start_x) / max(1, goal_pose(1) - start_x);
    metrics.completion_status = 'PARTIAL / DID NOT REACH GOAL';
    metrics.completion_rate   = min(99.0, max(0.0, prog * 100));
end

% 6. Console report
fprintf('\n-------------------------------------------------------\n');
fprintf('  SIH PS-26037 — EVALUATION METRICS\n');
fprintf('-------------------------------------------------------\n');
fprintf('  Mean replanning latency   : %.2f ms\n',  metrics.mean_latency_ms);
fprintf('  Max replanning latency    : %.2f ms\n',  metrics.max_latency_ms);
fprintf('  Avg steering rate         : %.2f deg/s\n', metrics.avg_steer_rate_deg_s);
fprintf('  Avg longitudinal jerk     : %.4f m/s^3\n', metrics.avg_jerk_m_s3);
fprintf('  Max longitudinal jerk     : %.4f m/s^3\n', metrics.max_jerk_m_s3);
fprintf('  Min safety clearance      : %.3f m\n',   metrics.min_safety_clearance_m);
fprintf('  Final distance to goal    : %.3f m\n',   metrics.final_goal_dist_m);
fprintf('  Scenario completion rate  : %.1f%%\n',   metrics.completion_rate);
fprintf('  Status                    : %s\n',       metrics.completion_status);
fprintf('-------------------------------------------------------\n\n');
end
