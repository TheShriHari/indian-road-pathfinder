function audit_reflex_duty_cycle(batch_csv_path)
    if nargin < 1 || isempty(batch_csv_path)
        batch_csv_path = 'batch_test_results_jerk_fixed.csv';
    end
    script_dir = fileparts(mfilename('fullpath'));
    if isempty(script_dir), script_dir = pwd; end
    addpath(script_dir);
    addpath(fullfile(script_dir, '..'));
    
    fprintf('========================================================================\n');
    fprintf(' 1. AUDITING BATCH RESULTS CSV: TRIALS 400 TO 475\n');
    fprintf('========================================================================\n');
    
    if exist(batch_csv_path, 'file')
        T = readtable(batch_csv_path);
        total_rows = height(T);
        fprintf('Total trials recorded in CSV: %d\n', total_rows);
        
        if total_rows >= 475
            slice_400_475 = T(401:min(475, total_rows), :);
            n_slice = height(slice_400_475);
            n_coll = sum(strcmp(slice_400_475.outcome, 'COLLISION'));
            n_safe = sum(strcmp(slice_400_475.outcome, 'SAFE_STOP'));
            n_succ = sum(strcmp(slice_400_475.outcome, 'SUCCESS'));
            fprintf('Trials 401-%d Summary:\n', min(475, total_rows));
            fprintf('  Collisions:  %d / %d (%.2f%%)\n', n_coll, n_slice, (n_coll/n_slice)*100);
            fprintf('  Safe Stops:  %d / %d (%.2f%%)\n', n_safe, n_slice, (n_safe/n_slice)*100);
            fprintf('  Successes:   %d / %d (%.2f%%)\n', n_succ, n_slice, (n_succ/n_slice)*100);
        else
            fprintf('CSV only contains %d rows, showing all recorded:\n', total_rows);
            tabulate(T.outcome);
        end
    else
        fprintf('CSV file not found at %s. Skipping CSV table audit.\n', batch_csv_path);
    end
    
    fprintf('\n========================================================================\n');
    fprintf(' 2. AUDITING EMERGENCY REFLEX DUTY CYCLE (100-TRIAL SAMPLE)\n');
    fprintf('========================================================================\n');
    
    total_ticks = 0;
    reflex_ticks = 0;
    reflex_in_successful_runs = 0;
    trials_with_reflex = 0;
    N_audit = 100;
    
    for s = 1:N_audit
        sc = generate_random_scenario(s);
        r = run_single_scenario(sc, s, struct('max_steps', 350, 'verbose', false, 'record_telemetry', true));
        
        trial_reflex_count = 0;
        for k = 1:length(r.telemetry)
            total_ticks = total_ticks + 1;
            t = r.telemetry(k);
            if isfield(t, 'reflex_active') && t.reflex_active
                reflex_ticks = reflex_ticks + 1;
                trial_reflex_count = trial_reflex_count + 1;
            end
        end
        
        if trial_reflex_count > 0
            trials_with_reflex = trials_with_reflex + 1;
            if strcmp(r.outcome, 'SUCCESS')
                reflex_in_successful_runs = reflex_in_successful_runs + trial_reflex_count;
            end
        end
    end
    
    fprintf('Evaluated %d random seeds (%d total ticks):\n', N_audit, total_ticks);
    fprintf('  Total Emergency Reflex Ticks: %d / %d (%.3f%% duty cycle)\n', ...
            reflex_ticks, total_ticks, (reflex_ticks / max(1, total_ticks)) * 100);
    fprintf('  Trials Triggering Reflex:     %d / %d (%.1f%% of scenarios)\n', ...
            trials_with_reflex, N_audit, (trials_with_reflex / N_audit) * 100);
    fprintf('  Reflex Ticks in Successful Runs: %d / %d\n', ...
            reflex_in_successful_runs, max(1, reflex_ticks));
    fprintf('========================================================================\n');
end
