function [best_params, theta_best] = tune_hyperparameters()
%% TUNE_HYPERPARAMETERS  Stage 3: Coordinate Descent Hyperparameter Optimization.
%
% Optimizes parameter vector theta = [K_v, L_min, w_steer, delta_hyst, alpha_cost]
% minimizing composite penalty:
%   J(theta) = w1 * (1 - CompletionRate) + w2 * (jerk / 1.0) + w3 * (replan / 50.0) + w4 * max(0, 0.8 - min_clr)
%
% Constraints (Directive SIH PS-26037 Section 5):
%   - K_v in [1.0, 1.5] s
%   - L_min in [3.0, 4.0] m
%   - w_steer in [0.02, 0.10]
%   - delta_hyst in [0.35, 0.70] m
%   - alpha_cost in [2.20, 3.10] m^-1
%
% Saves optimal configuration to matlab/best_params.mat.

% ── Constraint 2: Thread pool throttling ──────────────────────────────────
try
    maxNumCompThreads(6);
catch
end

set(0, 'DefaultFigureVisible', 'off');

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
addpath(script_dir);
addpath(fullfile(script_dir, '..'));

fprintf('=========================================================================\n');
fprintf('  STAGE 3: COORDINATE DESCENT HYPERPARAMETER AUTO-TUNING ENGINE           \n');
fprintf('=========================================================================\n\n');

% Objective weights
w1 = 100.0; % Completion penalty
w2 = 15.0;  % Jerk penalty
w3 = 10.0;  % Replanning latency penalty
w4 = 50.0;  % Proximity clearance penalty

% Evaluation batch: 100 representative seeds across all 5 domains (20 per domain)
eval_seeds = [ ...
    1:20, ...       % Domain 1: Rural (20 seeds)
    201:220, ...   % Domain 2: Intersection (20 seeds)
    401:420, ...   % Domain 3: Merge (20 seeds)
    601:620, ...   % Domain 4: Market (20 seeds)
    801:820 ...    % Domain 5: Cattle (20 seeds)
];
N_eval = length(eval_seeds);

% Initial parameter vector [K_v, L_min, w_steer, delta_hyst, alpha_cost]
theta_curr = [1.2, 3.2, 0.06, 0.50, 2.50];
param_names = {'K_v (Lookahead Speed Gain)', ...
               'L_min (Min Lookahead Dist)', ...
               'w_steer (Planner Steer Weight)', ...
               'delta_hyst (FSM Unlatch Margin)', ...
               'alpha_cost (Costmap Decay Rate)'};

param_grids = { ...
    [1.0, 1.1, 1.2, 1.3, 1.4, 1.5], ...       % K_v in [1.0, 1.5]
    [3.0, 3.2, 3.5, 3.8, 4.0], ...             % L_min in [3.0, 4.0]
    [0.02, 0.04, 0.06, 0.08, 0.10], ...       % w_steer in [0.02, 0.10]
    [0.35, 0.45, 0.50, 0.60, 0.70], ...       % delta_hyst in [0.35, 0.70]
    [2.20, 2.50, 2.70, 2.90, 3.10]            % alpha_cost in [2.20, 3.10]
};

fprintf('[INIT] Evaluating baseline candidate: [%.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
    theta_curr(1), theta_curr(2), theta_curr(3), theta_curr(4), theta_curr(5));

[J_curr, stats_curr] = evaluate_candidate(theta_curr, eval_seeds, w1, w2, w3, w4);
fprintf('       Baseline J(theta) = %.3f | Completion: %.1f%% | Jerk: %.2f | Latency: %.1fms | MinClr: %.2fm\n\n', ...
    J_curr, stats_curr.completion_rate*100, stats_curr.mean_jerk, stats_curr.mean_latency, stats_curr.min_clr);

tuning_history = [];
hist_entry.step = 0;
hist_entry.coord = 'BASELINE';
hist_entry.theta = theta_curr;
hist_entry.cost = J_curr;
hist_entry.stats = stats_curr;
tuning_history = [tuning_history; hist_entry];

% ── Coordinate Descent Sweeps ──────────────────────────────────────────────
n_coords = length(theta_curr);

for d = 1:n_coords
    candidates = param_grids{d};
    best_cand = theta_curr(d);
    best_J    = J_curr;
    best_stats = stats_curr;
    
    fprintf('>>> Sweeping Coordinate %d/5: %s <<<\n', d, param_names{d});
    
    for c_idx = 1:length(candidates)
        cand_val = candidates(c_idx);
        if abs(cand_val - theta_curr(d)) < 1e-4
            continue; % Already evaluated
        end
        
        theta_test = theta_curr;
        theta_test(d) = cand_val;
        
        [J_test, stats_test] = evaluate_candidate(theta_test, eval_seeds, w1, w2, w3, w4);
        fprintf('    Val: %5.2f -> J = %6.3f | Comp: %5.1f%% | Jerk: %4.2f | Lat: %4.1fms | Clr: %4.2fm', ...
            cand_val, J_test, stats_test.completion_rate*100, stats_test.mean_jerk, stats_test.mean_latency, stats_test.min_clr);
        
        if J_test < best_J
            best_J = J_test;
            best_cand = cand_val;
            best_stats = stats_test;
            fprintf('  <-- [NEW BEST]');
        end
        fprintf('\n');
    end
    
    % Update coordinate
    if best_J < J_curr
        fprintf('  --> Coordinate %d updated: %.2f -> %.2f (Cost: %.3f -> %.3f)\n\n', ...
            d, theta_curr(d), best_cand, J_curr, best_J);
        theta_curr(d) = best_cand;
        J_curr = best_J;
        stats_curr = best_stats;
    else
        fprintf('  --> Coordinate %d unchanged at %.2f (Cost: %.3f)\n\n', ...
            d, theta_curr(d), J_curr);
    end
    
    hist_entry.step = d;
    hist_entry.coord = param_names{d};
    hist_entry.theta = theta_curr;
    hist_entry.cost = J_curr;
    hist_entry.stats = stats_curr;
    tuning_history = [tuning_history; hist_entry];
end

theta_best = theta_curr;
best_params = struct( ...
    'K_v', theta_best(1), ...
    'L_min', theta_best(2), ...
    'w_steer', theta_best(3), ...
    'delta_hyst', theta_best(4), ...
    'alpha_cost', theta_best(5) ...
);

fprintf('=========================================================================\n');
fprintf('  COORDINATE DESCENT OPTIMIZATION CONVERGED!\n');
fprintf('=========================================================================\n');
fprintf('  Optimal Parameters (theta*):\n');
fprintf('    Pure Pursuit Speed Gain  (K_v)        : %6.2f s\n', best_params.K_v);
fprintf('    Minimum Lookahead        (L_min)      : %6.2f m\n', best_params.L_min);
fprintf('    Planner Steering Penalty (w_steer)    : %6.2f\n',   best_params.w_steer);
fprintf('    FSM Spatial Hysteresis   (delta_hyst) : %6.2f m\n', best_params.delta_hyst);
fprintf('    Costmap Decay Rate       (alpha_cost) : %6.2f m^-1\n', best_params.alpha_cost);
fprintf('  Optimized Penalty J(theta*) : %6.3f\n', J_curr);
fprintf('=========================================================================\n\n');

% Save best_params.mat
best_params_path1 = fullfile(script_dir, 'best_params.mat');
best_params_path2 = fullfile(script_dir, '..', 'matlab', 'best_params.mat');
save(best_params_path1, 'best_params', 'theta_best');
try, save(best_params_path2, 'best_params', 'theta_best'); catch, end
fprintf('[SAVED] Optimal parameters saved to %s\n', best_params_path1);

% Save tuning history JSON
tmp_dir = fullfile(script_dir, '..', '.tmp');
if ~exist(tmp_dir, 'dir'), mkdir(tmp_dir); end
json_hist_path = fullfile(tmp_dir, 'tuning_history.json');
fid_th = fopen(json_hist_path, 'w');
if fid_th ~= -1
    fwrite(fid_th, jsonencode(tuning_history));
    fclose(fid_th);
    fprintf('[SAVED] Tuning history exported to %s\n', json_hist_path);
end

end

%% ── Helper: Evaluate Candidate Parameter Vector ──────────────────────────
function [J, stats] = evaluate_candidate(theta, seeds, w1, w2, w3, w4)
    N = length(seeds);
    
    p_struct = struct( ...
        'K_v', theta(1), ...
        'L_min', theta(2), ...
        'w_steer', theta(3), ...
        'delta_hyst', theta(4), ...
        'alpha_cost', theta(5) ...
    );
    
    sim_opts.max_steps = 350;
    sim_opts.verbose   = false;
    sim_opts.params    = p_struct;
    sim_opts.record_telemetry = false;
    
    success_cnt = 0;
    safe_stop_cnt = 0;
    jerks = [];
    latencies = [];
    clearances = [];
    
    for s_idx = 1:N
        seed = seeds(s_idx);
        scenario = generate_random_scenario(seed);
        clear dynamic_obstacle_predictor simulate_sensor_detection;
        
        try
            res = run_single_scenario(scenario, seed, sim_opts);
        catch
            res.outcome = 'ERROR';
            res.mean_jerk = 1.5;
            res.mean_latency_ms = 50.0;
            res.min_clearance_achieved = 0.0;
        end
        
        if strcmp(res.outcome, 'SUCCESS')
            success_cnt = success_cnt + 1;
        elseif strcmp(res.outcome, 'SAFE_STOP')
            safe_stop_cnt = safe_stop_cnt + 1;
        end
        
        if ~isnan(res.mean_jerk) && res.mean_jerk > 0
            jerks(end+1) = res.mean_jerk; %#ok<AGROW>
        end
        if ~isnan(res.mean_latency_ms) && res.mean_latency_ms > 0
            latencies(end+1) = res.mean_latency_ms; %#ok<AGROW>
        end
        if ~isnan(res.min_clearance_achieved) && ~isinf(res.min_clearance_achieved)
            clearances(end+1) = res.min_clearance_achieved; %#ok<AGROW>
        end
    end
    
    comp_rate = (success_cnt + safe_stop_cnt) / N;
    if isempty(jerks), mean_jerk = 0.5; else, mean_jerk = mean(jerks); end
    if isempty(latencies), mean_lat = 15.0; else, mean_lat = mean(latencies); end
    if isempty(clearances), min_clr = 0.85; else, min_clr = min(clearances); end
    
    % Penalty function formulation:
    % J = w1 * (1 - CompletionRate) + w2 * (jerk / 1.0) + w3 * (replan / 50.0) + w4 * max(0, 0.8 - min_clr)
    J = w1 * (1.0 - comp_rate) ...
      + w2 * (mean_jerk / 1.0) ...
      + w3 * (mean_lat / 50.0) ...
      + w4 * max(0.0, 0.80 - min_clr);
    
    stats.completion_rate = comp_rate;
    stats.mean_jerk       = mean_jerk;
    stats.mean_latency    = mean_lat;
    stats.min_clr         = min_clr;
end
