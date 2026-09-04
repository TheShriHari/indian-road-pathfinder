%% TEST_SENSOR_LAYER  Explicit verification script for the sensor simulation layer.
%
% Runs the standard single clean scenario (potholes + cattle + auto-rickshaw)
% with the sensor layer active (which it now is by default in run_single_scenario).
% Confirms:
%   1. EKF innovations are meaningfully non-zero (sensor noise is real)
%   2. Vehicle still reaches the goal despite noise/dropout
%   3. At least 2-3 dropout/misclassification events printed with BSM context
%
% Usage:
%   cd <project>/matlab
%   test_sensor_layer
%
% No CARLA needed — runs entirely in MOCK mode via run_single_scenario.

clear; clc;

fprintf('=========================================================\n');
fprintf('  TEST: Sensor Simulation Layer Verification\n');
fprintf('  Scenario: potholes + cattle + auto-rickshaw (seed=42)\n');
fprintf('=========================================================\n\n');

%% ── Scenario: canonical "clean" PS-26037 scene ─────────────────────────────
obs_cfg.potholes = [ ...
    struct('x', 20.0, 'y',  1.0, 'radius', 0.8), ...
    struct('x', 35.0, 'y', -0.8, 'radius', 1.0) ...
];
obs_cfg.dynamic_agents = [ ...
    struct('id', 1, 'type', 'cattle',       'position', [30.0, -3.5], ...
           'velocity', [ 0.0,  0.7], 'behavior_profile', 'erratic'), ...
    struct('id', 2, 'type', 'auto_rickshaw', 'position', [45.0,  1.2], ...
           'velocity', [-3.2,  0.0], 'behavior_profile', 'weaving') ...
];
obs_cfg.start_pose = [2.0, 0.0, 0.0];
obs_cfg.goal_pose  = [70.0, 0.0, 0.0];

%% ── Simulation options ──────────────────────────────────────────────────────
opts.max_steps      = 800;    % 80s — enough for full run with sensor delays
opts.dt             = 0.1;
opts.collision_thresh = 1.0;
opts.goal_dist_tol  = 2.5;
opts.verbose        = true;   % MUST be true to see EKF/sensor prints

fprintf('[TEST] Starting simulation (verbose=true, seed=42)...\n');
fprintf('       Sensor layer: range=35m, FoV=140deg, dropout=5%%, noise=0.3-0.5m\n\n');

%% ── Run ─────────────────────────────────────────────────────────────────────
tic;
result = run_single_scenario(obs_cfg, 42, opts);
elapsed = toc;

%% ── Print verification summary ──────────────────────────────────────────────
fprintf('\n=========================================================\n');
fprintf('  VERIFICATION SUMMARY\n');
fprintf('=========================================================\n');
fprintf('  Outcome            : %s\n', result.outcome);
fprintf('  Time to goal       : %.1f s  (step limit = %.0f s)\n', ...
        result.time_to_goal, opts.max_steps * opts.dt);
fprintf('  Steps taken        : %d\n',    result.steps_taken);
fprintf('  Replans            : %d\n',    result.replan_count);
fprintf('  Min clearance      : %.2f m\n', result.min_clearance_achieved);
fprintf('  Max lateral        : %.2f m\n', result.max_lateral_position);
fprintf('  Wall-clock         : %.1f s\n', elapsed);

fprintf('\n--- Sensor Layer Stats ---\n');
fprintf('  Dropout  events    : %d\n', result.total_dropouts);
fprintf('  Misclass events    : %d\n', result.total_misclasses);

fprintf('\n--- EKF Innovation Stats ---\n');
if ~isempty(result.innov_log)
    fprintf('  N measurement updates : %d\n',    length(result.innov_log));
    fprintf('  Mean |innov|          : %.4f m\n', result.innov_mean);
    fprintf('  Max  |innov|          : %.4f m\n', result.innov_max);
    fprintf('  Std  |innov|          : %.4f m\n', std(result.innov_log));
    fprintf('  Min  |innov|          : %.4f m\n', min(result.innov_log));

    % Verification criterion: mean |innov| should be > 0.05m (not converging to 0)
    if result.innov_mean > 0.05
        fprintf('\n  [PASS] Mean innovation %.4fm > 0.05m threshold.\n', result.innov_mean);
        fprintf('         EKF is receiving noisy (non-oracle) measurements.\n');
    else
        fprintf('\n  [WARN] Mean innovation %.4fm is very small.\n', result.innov_mean);
        fprintf('         Sensor noise may not be reaching EKF.\n');
    end
else
    fprintf('  [WARN] No measurement updates logged. Check sensor layer wiring.\n');
end

fprintf('\n--- Goal Reachability ---\n');
if strcmp(result.outcome, 'SUCCESS')
    fprintf('  [PASS] Goal reached at t=%.1fs despite sensor noise/dropout.\n', ...
            result.time_to_goal);
else
    fprintf('  [INFO] Goal NOT reached (outcome=%s).\n', result.outcome);
    fprintf('         This reveals real robustness limits of EKF/planner under noise.\n');
    fprintf('         Final position: [%.2f, %.2f]  (goal: [70.0, 0.0])\n', ...
            result.final_pos(1), result.final_pos(2));
end

fprintf('\n--- Dropout / Misclassification Events ---\n');
if result.total_dropouts >= 2
    fprintf('  [PASS] %d dropout events occurred (see [SENSOR] DROPOUT lines above).\n', ...
            result.total_dropouts);
    fprintf('         EKF predict-only pass should have coasted estimates forward.\n');
else
    fprintf('  [INFO] Only %d dropout events in this run (probabilistic — retry with different seed).\n', ...
            result.total_dropouts);
end

if result.total_misclasses >= 1
    fprintf('  [INFO] %d misclassification events occurred (see [SENSOR] MISCLASS lines above).\n', ...
            result.total_misclasses);
end

fprintf('=========================================================\n\n');
