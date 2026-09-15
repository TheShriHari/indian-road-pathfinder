function results = run_batch_tests(num_trials, csv_out_name)
%% RUN_BATCH_TESTS  Runs closed-loop simulation across N randomized ODD scenarios.
%
% Syntax:
%   results = run_batch_tests(num_trials)
%   results = run_batch_tests(num_trials, csv_out_name)
%
% Parameters:
%   num_trials   : number of trials (default: 1000)
%   csv_out_name : base name for output CSV (default: 'batch_test_results').
%                  Pass 'batch_test_results_sensor_layer' to avoid overwriting
%                  the oracle-feed baseline.
%
% Metrics & Outcomes recorded per trial:
%   - Outcome: 'SUCCESS' | 'TIMEOUT' | 'COLLISION' | 'STALLED' | 'ERROR'
%   - time_to_goal (s) if SUCCESS
%   - replan_count
%   - min_clearance_achieved (m)
%   - max_lateral_position (m)
%   - seed used for full reproducibility
%   - innov_mean, innov_max (sensor layer: EKF innovation stats, NaN if oracle)
%   - total_dropouts (sensor layer: dropped detection events, 0 if oracle)
%
% Produces:
%   - Console summary report with breakdown, error traces, collision configurations,
%     time-to-goal distribution, and failure correlation analysis.
%   - CSV file: <csv_out_name>.csv (in matlab/ and parent dir)

if nargin < 1 || isempty(num_trials)
    num_trials = 1000;
end
if nargin < 2 || isempty(csv_out_name)
    csv_out_name = 'batch_test_results';
end

fprintf('=========================================================================\n');
fprintf('  SIH PS-26037: Autonomous Pathfinder Batch Testing Harness              \n');
fprintf('  Total Randomized Trials: %d  (Closed-loop MOCK Mode)                  \n', num_trials);
fprintf('=========================================================================\n\n');

results = repmat(struct( ...
    'trial_id', 0, ...
    'seed', 0, ...
    'outcome', '', ...
    'time_to_goal', NaN, ...
    'replan_count', 0, ...
    'min_clearance_achieved', NaN, ...
    'max_lateral_position', NaN, ...
    'num_potholes', 0, ...
    'num_agents', 0, ...
    'agent_types', '', ...
    'steps_taken', 0, ...
    'error_msg', '', ...
    'error_stack', '', ...
    'obstacle_config', struct([]), ...
    'innov_mean', NaN, ...
    'innov_max', NaN, ...
    'total_dropouts', 0 ...
), num_trials, 1);

t_batch_start = tic;

success_cnt   = 0;
collision_cnt = 0;
timeout_cnt   = 0;
stalled_cnt   = 0;
error_cnt     = 0;
safe_stop_cnt = 0;

for i = 1:num_trials
    seed = i;
    scenario = generate_random_scenario(seed);

    % Reset persistent EKF filter and sensor detection states across trials
    clear dynamic_obstacle_predictor simulate_sensor_detection;

    % Run single scenario in silent mode
    sim_opts.max_steps = 400; % 40.0s hard cap
    sim_opts.verbose   = false;
    res = run_single_scenario(scenario, seed, sim_opts);

    % Determine agent types string
    atypes = '';
    if isfield(scenario, 'dynamic_agents') && ~isempty(scenario.dynamic_agents)
        types_cell = {scenario.dynamic_agents.type};
        atypes = strjoin(types_cell, ';');
    end

    results(i).trial_id               = i;
    results(i).seed                   = seed;
    results(i).outcome                = res.outcome;
    results(i).time_to_goal           = res.time_to_goal;
    results(i).replan_count           = res.replan_count;
    results(i).min_clearance_achieved = res.min_clearance_achieved;
    results(i).max_lateral_position   = res.max_lateral_position;
    results(i).num_potholes           = res.num_potholes;
    results(i).num_agents             = res.num_agents;
    results(i).agent_types            = atypes;
    results(i).steps_taken            = res.steps_taken;
    results(i).error_msg              = res.error_msg;
    results(i).error_stack            = res.error_stack;
    results(i).obstacle_config        = res.obstacle_config;
    % Sensor layer stats (NaN / 0 for oracle-feed baseline runs)
    if isfield(res, 'innov_mean'),     results(i).innov_mean     = res.innov_mean;     end
    if isfield(res, 'innov_max'),      results(i).innov_max      = res.innov_max;      end
    if isfield(res, 'total_dropouts'), results(i).total_dropouts = res.total_dropouts; end

    switch res.outcome
        case 'SUCCESS'
            success_cnt = success_cnt + 1;
        case 'COLLISION'
            collision_cnt = collision_cnt + 1;
        case 'SAFE_STOP'
            safe_stop_cnt = safe_stop_cnt + 1;
        case 'TIMEOUT'
            timeout_cnt = timeout_cnt + 1;
        case 'STALLED'
            stalled_cnt = stalled_cnt + 1;
        case 'ERROR'
            error_cnt = error_cnt + 1;
    end

    % Periodic progress logging
    log_freq = 25;
    if num_trials <= 50, log_freq = 5; end
    if mod(i, log_freq) == 0 || i == num_trials
        fprintf('[PROGRESS] %4d / %4d complete | SUCCESS: %3d | COLLISION: %3d | SAFE_STOP: %3d | STALLED: %2d | TIMEOUT: %2d | ERROR: %2d (%.1fs)\n', ...
            i, num_trials, success_cnt, collision_cnt, safe_stop_cnt, stalled_cnt, timeout_cnt, error_cnt, toc(t_batch_start));
    end
end

total_time = toc(t_batch_start);
fprintf('\n=========================================================================\n');
fprintf('  BATCH TESTING COMPLETED in %.2f seconds (Avg %.3f s / trial)           \n', ...
    total_time, total_time / num_trials);
fprintf('=========================================================================\n\n');

%% ── 1. Outcome Summary ───────────────────────────────────────────────────
fprintf('-------------------------------------------------------------------------\n');
fprintf('  1. OUTCOME DISTRIBUTION (Total: %d)\n', num_trials);
fprintf('-------------------------------------------------------------------------\n');
fprintf('  SUCCESS   : %4d  (%6.2f%%)\n', success_cnt,   (success_cnt   / num_trials) * 100);
fprintf('  COLLISION : %4d  (%6.2f%%)\n', collision_cnt, (collision_cnt / num_trials) * 100);
fprintf('  SAFE_STOP : %4d  (%6.2f%%)\n', safe_stop_cnt, (safe_stop_cnt / num_trials) * 100);
fprintf('  TIMEOUT   : %4d  (%6.2f%%)\n', timeout_cnt,   (timeout_cnt   / num_trials) * 100);
fprintf('  STALLED   : %4d  (%6.2f%%)\n', stalled_cnt,   (stalled_cnt   / num_trials) * 100);
fprintf('  ERROR     : %4d  (%6.2f%%)\n', error_cnt,     (error_cnt     / num_trials) * 100);
fprintf('-------------------------------------------------------------------------\n\n');

%% ── 2. Time-to-Goal Distribution for SUCCESS trials ──────────────────────
success_idx = strcmp({results.outcome}, 'SUCCESS');
success_times = [results(success_idx).time_to_goal];

fprintf('-------------------------------------------------------------------------\n');
fprintf('  2. TIME-TO-GOAL DISTRIBUTION (Successful Runs: %d)\n', length(success_times));
fprintf('-------------------------------------------------------------------------\n');
if ~isempty(success_times)
    fprintf('  Min    : %5.2f s\n', min(success_times));
    fprintf('  Mean   : %5.2f s\n', mean(success_times));
    fprintf('  Median : %5.2f s\n', median(success_times));
    fprintf('  Max    : %5.2f s\n', max(success_times));
    fprintf('  StdDev : %5.2f s\n\n', std(success_times));

    % ASCII Histogram
    bins = [0, 15, 20, 25, 30, 35, 40];
    bin_labels = {'< 15s ', '15-20s', '20-25s', '25-30s', '30-35s', '35-40s'};
    counts = histcounts(success_times, bins);
    max_c = max([1, counts]);
    fprintf('  Distribution Histogram:\n');
    for b = 1:length(counts)
        pct = (counts(b) / length(success_times)) * 100;
        bar_len = round((counts(b) / max_c) * 30);
        bar_str = repmat('#', 1, bar_len);
        fprintf('    [%s] : %4d (%5.1f%%) | %s\n', bin_labels{b}, counts(b), pct, bar_str);
    end
else
    fprintf('  No successful runs to report.\n');
end
fprintf('-------------------------------------------------------------------------\n\n');

%% ── 3. Error Analysis ────────────────────────────────────────────────────
error_idx = find(strcmp({results.outcome}, 'ERROR'));
fprintf('-------------------------------------------------------------------------\n');
fprintf('  3. ERROR TRIALS (Total: %d)\n', length(error_idx));
fprintf('-------------------------------------------------------------------------\n');
if isempty(error_idx)
    fprintf('  [OK] Zero MATLAB runtime exceptions occurred across all %d trials.\n', num_trials);
else
    fprintf('  [CRITICAL] Found %d MATLAB exception(s):\n\n', length(error_idx));
    for e = 1:length(error_idx)
        idx = error_idx(e);
        fprintf('  Trial #%d (Seed: %d):\n', idx, results(idx).seed);
        fprintf('    Message: %s\n', results(idx).error_msg);
        fprintf('    Call Stack snippet:\n%s\n', results(idx).error_stack);
    end
end
fprintf('-------------------------------------------------------------------------\n\n');

%% ── 4. Collision Configuration Breakdown ─────────────────────────────────
collision_idx = find(strcmp({results.outcome}, 'COLLISION'));
fprintf('-------------------------------------------------------------------------\n');
fprintf('  4. COLLISION TRIALS (Total: %d)\n', length(collision_idx));
fprintf('-------------------------------------------------------------------------\n');
if isempty(collision_idx)
    fprintf('  [OK] Zero collisions occurred across all %d trials.\n', num_trials);
else
    % Show detailed collision configs (up to first 15)
    num_to_show = min(15, length(collision_idx));
    fprintf('  Showing first %d collision scenarios:\n', num_to_show);
    for c = 1:num_to_show
        idx = collision_idx(c);
        cfg = results(idx).obstacle_config;
        fprintf('  Collision Trial #%d (Seed: %d, Clearance: %.2fm):\n', ...
            idx, results(idx).seed, results(idx).min_clearance_achieved);
        % Potholes
        if isempty(cfg.potholes)
            fprintf('    Potholes: none\n');
        else
            pot_strs = cell(1, length(cfg.potholes));
            for p = 1:length(cfg.potholes)
                pot_strs{p} = sprintf('[x=%.1f,y=%.1f,r=%.2f]', cfg.potholes(p).x, cfg.potholes(p).y, cfg.potholes(p).radius);
            end
            fprintf('    Potholes (%d): %s\n', length(cfg.potholes), strjoin(pot_strs, ', '));
        end
        % Agents
        if isempty(cfg.dynamic_agents)
            fprintf('    Agents: none\n');
        else
            ag_strs = cell(1, length(cfg.dynamic_agents));
            for a = 1:length(cfg.dynamic_agents)
                ag = cfg.dynamic_agents(a);
                ag_strs{a} = sprintf('%s @ [%.1f,%.1f] v=[%.1f,%.1f]', ag.type, ag.position(1), ag.position(2), ag.velocity(1), ag.velocity(2));
            end
            fprintf('    Agents (%d): %s\n', length(cfg.dynamic_agents), strjoin(ag_strs, '; '));
        end
    end
    if length(collision_idx) > num_to_show
        fprintf('  ... and %d more collisions (see batch_test_results.csv for full list).\n', ...
            length(collision_idx) - num_to_show);
    end
end
fprintf('-------------------------------------------------------------------------\n\n');

%% ── 5. Disproportionate Failure Correlation Analysis ─────────────────────
fprintf('-------------------------------------------------------------------------\n');
fprintf('  5. RISK FACTOR & COLLISION RATE ANALYSIS                              \n');
fprintf('-------------------------------------------------------------------------\n');

is_collision = strcmp({results.outcome}, 'COLLISION');
is_safe_stop = strcmp({results.outcome}, 'SAFE_STOP');
overall_col_rate = (sum(is_collision) / num_trials) * 100;
fprintf('  Baseline Overall Collision Rate: %.2f%% (%d / %d)\n', ...
    overall_col_rate, sum(is_collision), num_trials);
fprintf('  Baseline Overall Safe-Stop Rate: %.2f%% (%d / %d)\n\n', ...
    (sum(is_safe_stop) / num_trials) * 100, sum(is_safe_stop), num_trials);

% A. By Dynamic Agent Count (0, 1, 2, 3)
fprintf('  [A] Breakdown by Dynamic Agent Count:\n');
agent_counts = [results.num_agents];
for ac = 0:3
    mask = (agent_counts == ac);
    n_sub = sum(mask);
    if n_sub > 0
        n_col = sum(is_collision(mask));
        n_stop = sum(is_safe_stop(mask));
        col_rate = (n_col / n_sub) * 100;
        flag = '';
        if col_rate > overall_col_rate * 1.3 && n_col >= 3
            flag = '  <-- [FLAGGED HIGH RISK]';
        end
        fprintf('    Agents = %d : %3d / %3d collisions (%5.1f%%) | %3d safe stops (%5.1f%%)%s\n', ...
            ac, n_col, n_sub, col_rate, n_stop, (n_stop/n_sub)*100, flag);
    end
end
fprintf('\n');

% B. By Pothole Count (0, 1, 2, 3, 4)
fprintf('  [B] Breakdown by Pothole Count:\n');
pothole_counts = [results.num_potholes];
for pc = 0:4
    mask = (pothole_counts == pc);
    n_sub = sum(mask);
    if n_sub > 0
        n_col = sum(is_collision(mask));
        n_stop = sum(is_safe_stop(mask));
        col_rate = (n_col / n_sub) * 100;
        flag = '';
        if col_rate > overall_col_rate * 1.3 && n_col >= 3
            flag = '  <-- [FLAGGED HIGH RISK]';
        end
        fprintf('    Potholes = %d : %3d / %3d collisions (%5.1f%%) | %3d safe stops (%5.1f%%)%s\n', ...
            pc, n_col, n_sub, col_rate, n_stop, (n_stop/n_sub)*100, flag);
    end
end
fprintf('\n');

% C. By Dynamic Agent Type Presence (specifically Collision Rate)
fprintf('  [C] Collision Rate by Agent Type Presence:\n');
types_to_check = {'cattle', 'auto_rickshaw', 'pedestrian'};
for t_idx = 1:length(types_to_check)
    atype = types_to_check{t_idx};
    has_type = contains({results.agent_types}, atype);
    n_with = sum(has_type);
    n_without = sum(~has_type);
    col_with = sum(is_collision(has_type));
    col_without = sum(is_collision(~has_type));
    col_rate_with = (col_with / max(1, n_with)) * 100;
    col_rate_without = (col_without / max(1, n_without)) * 100;
    flag = '';
    if col_rate_with > overall_col_rate * 1.3 && col_with >= 3
        flag = '  <-- [FLAGGED HIGH RISK]';
    end
    fprintf('    %-14s Present: %3d / %3d collisions (%5.1f%%) | Absent: %3d / %3d (%5.1f%%)%s\n', ...
        atype, col_with, n_with, col_rate_with, ...
        col_without, n_without, col_rate_without, flag);
end
fprintf('-------------------------------------------------------------------------\n\n');

%% ── 6. Save Results to CSV ───────────────────────────────────────────────
csv_filename1 = [csv_out_name, '.csv'];
csv_filename2 = fullfile('..', [csv_out_name, '.csv']);

fid1 = fopen(csv_filename1, 'w');
if fid1 ~= -1
    write_csv_content(fid1, results);
    fclose(fid1);
    fprintf('[CSV] Saved %d trial results to %s\n', num_trials, csv_filename1);
end

fid2 = fopen(csv_filename2, 'w');
if fid2 ~= -1
    write_csv_content(fid2, results);
    fclose(fid2);
    fprintf('[CSV] Saved %d trial results to %s\n', num_trials, csv_filename2);
end

end

%% ── Helper: Write CSV rows ───────────────────────────────────────────────
function write_csv_content(fid, results)
    fprintf(fid, 'trial_id,seed,outcome,time_to_goal,replan_count,min_clearance_achieved,max_lateral_position,num_potholes,num_agents,agent_types,steps_taken,innov_mean,innov_max,total_dropouts,error_message\n');
    for i = 1:length(results)
        r = results(i);
        t_goal = r.time_to_goal;
        if isnan(t_goal), t_str = ''; else, t_str = sprintf('%.2f', t_goal); end
        min_clr = r.min_clearance_achieved;
        if isinf(min_clr) || isnan(min_clr), clr_str = ''; else, clr_str = sprintf('%.3f', min_clr); end
        max_lat = r.max_lateral_position;
        if isnan(max_lat), lat_str = ''; else, lat_str = sprintf('%.3f', max_lat); end
        innov_m = r.innov_mean;
        if isnan(innov_m), im_str = ''; else, im_str = sprintf('%.4f', innov_m); end
        innov_x = r.innov_max;
        if isnan(innov_x), ix_str = ''; else, ix_str = sprintf('%.4f', innov_x); end

        % Clean message for CSV
        clean_msg = strrep(r.error_msg, ',', ' ');
        clean_msg = strrep(clean_msg, newline, ' ');

        fprintf(fid, '%d,%d,%s,%s,%d,%s,%s,%d,%d,"%s",%d,%s,%s,%d,"%s"\n', ...
            r.trial_id, r.seed, r.outcome, t_str, r.replan_count, ...
            clr_str, lat_str, r.num_potholes, r.num_agents, ...
            r.agent_types, r.steps_taken, im_str, ix_str, r.total_dropouts, clean_msg);
    end
end
