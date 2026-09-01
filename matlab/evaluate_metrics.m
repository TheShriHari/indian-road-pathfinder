function metrics = evaluate_metrics(time_history, state_history, control_history, latency_history, obstacles_history)
% EVALUATE_METRICS Calculates quantitative evaluation metrics for autonomous pathfinder
%   metrics = evaluate_metrics(time_history, state_history, control_history, latency_history, obstacles_history)
%
% Metrics evaluated:
%   1. Mean & Max Replanning Latency (ms)
%   2. Path Smoothness (Average Absolute Steering Rate / Jerk)
%   3. Minimum Safety Clearance (m)
%   4. Scenario Completion Rate (%)

dt = mean(diff(time_history));

% 1. Replanning Latency
metrics.mean_latency_ms = mean(latency_history);
metrics.max_latency_ms  = max(latency_history);

% 2. Path Smoothness (Steering Rate & Accelerational Jerk)
steer_angles = control_history(:, 1);
steer_rate   = diff(steer_angles) / dt;
metrics.avg_steer_rate_deg_s = mean(abs(steer_rate)) * (180/pi);

accelerations = control_history(:, 2);
jerk          = diff(accelerations) / dt;
metrics.avg_jerk_m_s3        = mean(abs(jerk));
metrics.max_jerk_m_s3        = max(abs(jerk));

% 3. Minimum Safety Clearance
min_clearance = Inf;
num_steps = size(state_history, 1);

for t = 1:num_steps
    ego_pos = state_history(t, 1:2);
    obs_at_t = obstacles_history{t};
    for k = 1:length(obs_at_t)
        dist = norm(ego_pos - obs_at_t(k).position);
        if dist < min_clearance
            min_clearance = dist;
        end
    end
end
metrics.min_safety_clearance_m = min_clearance;

% 4. Completion Status
final_x = state_history(end, 1);
if final_x >= 50.0 && min_clearance > 0.8
    metrics.completion_status = 'SUCCESS (100%)';
    metrics.completion_rate   = 100.0;
else
    metrics.completion_status = 'PARTIAL / COLLISION';
    metrics.completion_rate   = (final_x / 55.0) * 100;
end

% Display report in console
fprintf('\n=======================================================\n');
fprintf('        SIH 2026 PS 26037 - EVALUATION METRICS REPORT   \n');
fprintf('=======================================================\n');
fprintf(' Mean Replanning Latency  : %.2f ms\n', metrics.mean_latency_ms);
fprintf(' Max Replanning Latency   : %.2f ms\n', metrics.max_latency_ms);
fprintf(' Path Smoothness (Steer)  : %.2f deg/s\n', metrics.avg_steer_rate_deg_s);
fprintf(' Avg Longitudinal Jerk    : %.2f m/s^3\n', metrics.avg_jerk_m_s3);
fprintf(' Min Safety Clearance     : %.2f meters\n', metrics.min_safety_clearance_m);
fprintf(' Scenario Completion Rate : %.1f%%\n', metrics.completion_rate);
fprintf(' Status                   : %s\n', metrics.completion_status);
fprintf('=======================================================\n\n');
end
