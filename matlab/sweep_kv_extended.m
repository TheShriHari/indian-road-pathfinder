function sweep_kv_extended()
%% SWEEP_KV_EXTENDED  Evaluates Kv in [1.50, 1.55, 1.60, 1.65, 1.70, 1.75]
% Uses exact evaluation setup, seeds, and cost function from tune_hyperparameters.m

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
addpath(script_dir);
addpath(fullfile(script_dir, '..'));

% Objective weights (from tune_hyperparameters.m)
w1 = 100.0; % Completion penalty
w2 = 15.0;  % Jerk penalty
w3 = 10.0;  % Replanning latency penalty
w4 = 50.0;  % Proximity clearance penalty

eval_seeds = [ ...
    1:20, ...       % Domain 1: Rural (20 seeds)
    201:220, ...   % Domain 2: Intersection (20 seeds)
    401:420, ...   % Domain 3: Merge (20 seeds)
    601:620, ...   % Domain 4: Market (20 seeds)
    801:820 ...    % Domain 5: Cattle (20 seeds)
];

base_theta = [1.50, 3.20, 0.07, 0.55, 2.90];
kv_candidates = 1.50 : 0.05 : 1.75;

fprintf('=========================================================================\n');
fprintf('  EXTENDED Kv SWEEP: [1.50 to 1.75] on 100 Representative Seeds\n');
fprintf('  Fixed: L_min=%.2f, w_steer=%.2f, delta_hyst=%.2f, alpha_cost=%.2f\n', ...
    base_theta(2), base_theta(3), base_theta(4), base_theta(5));
fprintf('=========================================================================\n');
fprintf('  Kv   | CompRate | Succ | Safe | Coll | MeanJerk | MeanLat | MinClr |   Cost J(theta)\n');
fprintf('-------------------------------------------------------------------------\n');

results = [];

for i = 1:length(kv_candidates)
    kv_val = kv_candidates(i);
    theta = base_theta;
    theta(1) = kv_val;
    
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
    collision_cnt = 0;
    jerks = [];
    latencies = [];
    clearances = [];

    for s_idx = 1:length(eval_seeds)
        seed = eval_seeds(s_idx);
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
        elseif strcmp(res.outcome, 'COLLISION')
            collision_cnt = collision_cnt + 1;
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

    comp_rate = (success_cnt + safe_stop_cnt) / length(eval_seeds);
    if isempty(jerks), mean_jerk = 0.5; else, mean_jerk = mean(jerks); end
    if isempty(latencies), mean_lat = 15.0; else, mean_lat = mean(latencies); end
    if isempty(clearances), min_clr = 0.85; else, min_clr = min(clearances); end

    J = w1 * (1.0 - comp_rate) ...
      + w2 * (mean_jerk / 0.95) ...
      + w3 * (mean_lat / 35.0) ...
      + w4 * max(0.0, 0.80 - min_clr);

    fprintf(' %5.2f |  %5.1f%%  | %4d | %4d | %4d |  %6.3f  | %5.2fms | %5.3fm |  %9.4f\n', ...
        kv_val, comp_rate*100, success_cnt, safe_stop_cnt, collision_cnt, mean_jerk, mean_lat, min_clr, J);
    
    entry.kv = kv_val;
    entry.comp_rate = comp_rate;
    entry.success = success_cnt;
    entry.safe_stop = safe_stop_cnt;
    entry.collision = collision_cnt;
    entry.mean_jerk = mean_jerk;
    entry.mean_latency = mean_lat;
    entry.min_clr = min_clr;
    entry.cost = J;
    results = [results; entry]; %#ok<AGROW>

    % Write intermediate progress to .tmp/kv_extended_progress.txt
    prog_dir = fullfile(script_dir, '..', '.tmp');
    if ~exist(prog_dir, 'dir'), mkdir(prog_dir); end
    fid_prog = fopen(fullfile(prog_dir, 'kv_extended_progress.txt'), 'a');
    if fid_prog ~= -1
        fprintf(fid_prog, 'Kv=%.2f: Comp=%.1f%% (Succ=%d, Safe=%d, Coll=%d), Jerk=%.3f, Lat=%.2fms, MinClr=%.3fm, Cost=%.4f\n', ...
            kv_val, comp_rate*100, success_cnt, safe_stop_cnt, collision_cnt, mean_jerk, mean_lat, min_clr, J);
        fclose(fid_prog);
    end
end

fprintf('=========================================================================\n');
tmp_dir = fullfile(script_dir, '..', '.tmp');
if ~exist(tmp_dir, 'dir'), mkdir(tmp_dir); end
json_path = fullfile(tmp_dir, 'kv_extended_sweep.json');
fid = fopen(json_path, 'w');
if fid ~= -1
    fwrite(fid, jsonencode(results));
    fclose(fid);
    fprintf('[SAVED] Extended sweep results saved to %s\n', json_path);
end
end
