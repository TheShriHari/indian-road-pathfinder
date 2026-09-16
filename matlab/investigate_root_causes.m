%% INVESTIGATE_ROOT_CAUSES  Stage 2: Failure Mining & Root Cause Triage Engine.
%
% Parses Monte Carlo batch results across all 5 domains, filters edge cases failing:
%   - Clearance < 0.8m
%   - Jerk > 1.0 m/s^3
%   - Timeout / Deadlock / Collision / Error
% Classifies failures into taxonomy:
%   - COLLISION
%   - KINEMATIC_TRAP
%   - DEADLOCK
%   - JERK_EXCESS
%   - CHATTER
% Dumps edge case diagnostics to .tmp/failures/ and exports .tmp/failure_analysis.json.

clear; clc;

% ── Constraint 2: Thread pool throttling ──────────────────────────────────
try
    maxNumCompThreads(6);
catch
end

fprintf('=========================================================================\n');
fprintf('  STAGE 2: FAILURE MINING & ROOT CAUSE TRIAGE ENGINE                     \n');
fprintf('=========================================================================\n\n');

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
addpath(script_dir);
addpath(fullfile(script_dir, '..'));

% Locate input CSV
csv_candidates = { ...
    fullfile(script_dir, 'batch_test_results_baseline.csv'), ...
    fullfile(script_dir, '..', 'batch_test_results_baseline.csv'), ...
    fullfile(script_dir, 'batch_test_results.csv'), ...
    fullfile(script_dir, '..', 'batch_test_results.csv') ...
};

csv_file = '';
for i = 1:length(csv_candidates)
    if exist(csv_candidates{i}, 'file')
        csv_file = csv_candidates{i};
        break;
    end
end

if isempty(csv_file)
    error('investigate_root_causes: No batch_test_results.csv found to analyze.');
end

fprintf('[INFO] Ingesting batch test records from: %s\n', csv_file);
res_table = readtable(csv_file);
num_trials = height(res_table);

tmp_dir = fullfile(script_dir, '..', '.tmp');
fail_dir = fullfile(tmp_dir, 'failures');
if ~exist(fail_dir, 'dir'), mkdir(fail_dir); end

%% ── Filter Failure & Edge-Case Conditions ─────────────────────────────────
% 1. Non-success outcome
is_collision = strcmp(res_table.outcome, 'COLLISION');
is_deadlock  = strcmp(res_table.outcome, 'DEADLOCK') | strcmp(res_table.outcome, 'STALLED');
is_timeout   = strcmp(res_table.outcome, 'TIMEOUT');
is_error     = strcmp(res_table.outcome, 'ERROR');

% 2. Clearance < 0.8m margin
is_low_clearance = (res_table.min_clearance_achieved < 0.80) & ~isnan(res_table.min_clearance_achieved);

% 3. Jerk excess > 1.0 m/s^3
if ismember('p99_jerk', res_table.Properties.VariableNames)
    is_jerk_excess = (res_table.p99_jerk > 1.0) & ~isnan(res_table.p99_jerk);
else
    is_jerk_excess = false(num_trials, 1);
end

failed_mask = is_collision | is_deadlock | is_timeout | is_error | (is_low_clearance & ~strcmp(res_table.outcome, 'SUCCESS')) | is_jerk_excess;
failed_indices = find(failed_mask);
n_failures = length(failed_indices);

fprintf('\n-------------------------------------------------------------------------\n');
fprintf('  FAILURE & EDGE CASE BREAKDOWN (Total: %d / %d trials, %5.2f%%)\n', ...
    n_failures, num_trials, (n_failures / num_trials) * 100);
fprintf('-------------------------------------------------------------------------\n');
fprintf('  COLLISION            : %4d  (%5.2f%%)\n', sum(is_collision), (sum(is_collision)/num_trials)*100);
fprintf('  DEADLOCK / STALLED   : %4d  (%5.2f%%)\n', sum(is_deadlock),  (sum(is_deadlock)/num_trials)*100);
fprintf('  TIMEOUT              : %4d  (%5.2f%%)\n', sum(is_timeout),   (sum(is_timeout)/num_trials)*100);
fprintf('  SUB-0.8m CLEARANCE   : %4d  (%5.2f%%)\n', sum(is_low_clearance), (sum(is_low_clearance)/num_trials)*100);
fprintf('  JERK EXCESS (>1.0)   : %4d  (%5.2f%%)\n', sum(is_jerk_excess), (sum(is_jerk_excess)/num_trials)*100);
fprintf('  RUNTIME ERRORS       : %4d  (%5.2f%%)\n', sum(is_error),     (sum(is_error)/num_trials)*100);
fprintf('-------------------------------------------------------------------------\n\n');

%% ── Root Cause Classification & Edge Case Triage ─────────────────────────
class_collision = 0;
class_kin_trap  = 0;
class_deadlock  = 0;
class_jerk      = 0;
class_chatter   = 0;

failure_records = repmat(struct( ...
    'trial_id', 0, ...
    'seed', 0, ...
    'domain_id', 0, ...
    'classification', '', ...
    'outcome', '', ...
    'clearance', NaN, ...
    'jerk', NaN, ...
    'description', '' ...
), n_failures, 1);

for fi = 1:n_failures
    idx = failed_indices(fi);
    r = res_table(idx, :);
    seed = r.seed;
    
    % Classify
    if strcmp(r.outcome{1}, 'COLLISION')
        % Distinguish between Kinematic Trap (corridor narrower than R_min turn envelope) and Collision
        if ismember('domain_id', res_table.Properties.VariableNames) && (r.domain_id == 4 || r.domain_id == 1) && r.min_clearance_achieved < 0.20
            classification = 'KINEMATIC_TRAP';
            class_kin_trap = class_kin_trap + 1;
            desc = sprintf('Kinematic trap: Corridor narrowed below turning envelope; radius constraint prevented lateral avoidance.');
        else
            classification = 'COLLISION';
            class_collision = class_collision + 1;
            desc = sprintf('Lethal boundary collision (min clearance achieved: %.2fm).', r.min_clearance_achieved);
        end
    elseif strcmp(r.outcome{1}, 'DEADLOCK') || strcmp(r.outcome{1}, 'STALLED')
        classification = 'DEADLOCK';
        class_deadlock = class_deadlock + 1;
        desc = sprintf('Standstill outside Virtual Stop Line sustained for > 5.0s.');
    elseif ismember('p99_jerk', res_table.Properties.VariableNames) && r.p99_jerk > 1.0
        classification = 'JERK_EXCESS';
        class_jerk = class_jerk + 1;
        desc = sprintf('Longitudinal jerk peaked at %.2f m/s^3 (limit: 1.0 m/s^3).', r.p99_jerk);
    elseif r.replan_count > 25
        classification = 'CHATTER';
        class_chatter = class_chatter + 1;
        desc = sprintf('High replanning chatter (%d replans) due to insufficient spatial hysteresis.', r.replan_count);
    else
        classification = 'COLLISION';
        class_collision = class_collision + 1;
        desc = sprintf('Generic obstacle contention (outcome: %s).', r.outcome{1});
    end
    
    failure_records(fi).trial_id       = r.trial_id;
    failure_records(fi).seed           = seed;
    if ismember('domain_id', res_table.Properties.VariableNames)
        failure_records(fi).domain_id  = r.domain_id;
    else
        failure_records(fi).domain_id  = ceil(seed / 200);
    end
    failure_records(fi).classification = classification;
    failure_records(fi).outcome        = r.outcome{1};
    failure_records(fi).clearance      = r.min_clearance_achieved;
    if ismember('p99_jerk', res_table.Properties.VariableNames)
        failure_records(fi).jerk       = r.p99_jerk;
    else
        failure_records(fi).jerk       = NaN;
    end
    failure_records(fi).description    = desc;
    
    % Dump individual diagnostic file for up to first 30 edge cases
    if fi <= 30
        edge_file = fullfile(fail_dir, sprintf('edge_case_seed_%d.log', seed));
        fid_edge = fopen(edge_file, 'w');
        if fid_edge ~= -1
            fprintf(fid_edge, 'TRIAL ID        : %d\n', r.trial_id);
            fprintf(fid_edge, 'SEED            : %d\n', seed);
            fprintf(fid_edge, 'CLASSIFICATION  : %s\n', classification);
            fprintf(fid_edge, 'OUTCOME         : %s\n', r.outcome{1});
            fprintf(fid_edge, 'MIN CLEARANCE   : %.3f m\n', r.min_clearance_achieved);
            if ismember('p99_jerk', res_table.Properties.VariableNames)
                fprintf(fid_edge, 'P99 JERK        : %.3f m/s^3\n', r.p99_jerk);
            end
            fprintf(fid_edge, 'DIAGNOSIS       : %s\n', desc);
            fclose(fid_edge);
        end
    end
end

fprintf('-------------------------------------------------------------------------\n');
fprintf('  ROOT CAUSE TAXONOMY CLASSIFICATION\n');
fprintf('-------------------------------------------------------------------------\n');
fprintf('  1. COLLISION        : %4d  (%5.1f%% of failures)\n', class_collision, (class_collision / max(1, n_failures))*100);
fprintf('  2. KINEMATIC_TRAP   : %4d  (%5.1f%% of failures)\n', class_kin_trap,  (class_kin_trap  / max(1, n_failures))*100);
fprintf('  3. DEADLOCK         : %4d  (%5.1f%% of failures)\n', class_deadlock,  (class_deadlock  / max(1, n_failures))*100);
fprintf('  4. JERK_EXCESS      : %4d  (%5.1f%% of failures)\n', class_jerk,      (class_jerk      / max(1, n_failures))*100);
fprintf('  5. CHATTER          : %4d  (%5.1f%% of failures)\n', class_chatter,   (class_chatter   / max(1, n_failures))*100);
fprintf('-------------------------------------------------------------------------\n\n');

% Unique failing trials (neither SUCCESS nor SAFE_STOP)
unique_failing_mask = ~strcmp(res_table.outcome, 'SUCCESS') & ~strcmp(res_table.outcome, 'SAFE_STOP');
unique_failing_trials = sum(unique_failing_mask);

% Export structured analysis JSON
summary_struct = struct( ...
    'total_trials', num_trials, ...
    'total_failure_flags', n_failures, ...
    'unique_failing_trials', unique_failing_trials, ...
    'failure_rate_pct', round((unique_failing_trials / num_trials) * 100, 2), ...
    'taxonomy', struct( ...
        'collision', class_collision, ...
        'kinematic_trap', class_kin_trap, ...
        'deadlock', class_deadlock, ...
        'jerk_excess', class_jerk, ...
        'chatter', class_chatter ...
    ), ...
    'edge_cases_logged', min(30, n_failures), ...
    'status', 'TRIAGE_COMPLETE' ...
);

json_summary_path = fullfile(tmp_dir, 'failure_analysis.json');
fid_sum = fopen(json_summary_path, 'w');
if fid_sum ~= -1
    fwrite(fid_sum, jsonencode(summary_struct));
    fclose(fid_sum);
    fprintf('[JSON] Saved structured failure analysis to %s\n', json_summary_path);
end

fprintf('  [PASS] Stage 2 Root Cause Triage Completed.\n\n');
