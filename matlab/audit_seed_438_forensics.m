function audit_seed_438_forensics()
    script_dir = fileparts(mfilename('fullpath'));
    if isempty(script_dir), script_dir = pwd; end
    addpath(script_dir);
    addpath(fullfile(script_dir, '..'));
    seed = 438;
    sc = generate_random_scenario(seed);
    
    fprintf('========================================================================\n');
    fprintf(' FORENSIC AUDIT: SEED 438 UNDER PURE 0.95 SLEW vs. ASYMMETRIC LIMITER\n');
    fprintf('========================================================================\n\n');

    % -------------------------------------------------------------------------
    % TEST 1: PURE 0.95 m/s^3 UNIFORM SLEW (No Emergency Exception)
    % -------------------------------------------------------------------------
    fprintf('--- RUNNING TEST 1: PURE UNIFORM 0.95 m/s^3 LIMITER ---\n');
    opts_pure = struct('max_steps', 350, 'verbose', false, 'record_telemetry', true, ...
                       'force_uniform_slew', true, 'slew_limit', 0.95);
    r_pure = run_single_scenario(sc, seed, opts_pure);
    fprintf('Result Pure: Outcome=%s, MinClearance=%.3fm, CollisionVel=%.3fm/s\n\n', ...
            r_pure.outcome, r_pure.min_clearance_achieved, ...
            ternary(strcmp(r_pure.outcome,'COLLISION') && ~isempty(r_pure.telemetry), r_pure.telemetry(end).v, 0.0));

    % -------------------------------------------------------------------------
    % TEST 2: ASYMMETRIC LIMITER (8.0 m/s^3 Emergency, 0.95 m/s^3 Comfort + Latch)
    % -------------------------------------------------------------------------
    fprintf('--- RUNNING TEST 2: ASYMMETRIC LIMITER + ANTI-CHATTER LATCH ---\n');
    opts_asym = struct('max_steps', 350, 'verbose', false, 'record_telemetry', true, ...
                       'force_uniform_slew', false);
    r_asym = run_single_scenario(sc, seed, opts_asym);
    fprintf('Result Asym: Outcome=%s, MinClearance=%.3fm, CollisionVel=%.3fm/s\n\n', ...
            r_asym.outcome, r_asym.min_clearance_achieved, ...
            ternary(strcmp(r_asym.outcome,'COLLISION') && ~isempty(r_asym.telemetry), r_asym.telemetry(end).v, 0.0));

    % -------------------------------------------------------------------------
    % PRINT PER-TICK TELEMETRY WINDOW (Critical 3.0-second slice)
    % -------------------------------------------------------------------------
    print_telemetry_table('PURE 0.95 SLEW LOG (COLLISION WINDOW)', r_pure.telemetry);
    print_telemetry_table('ASYMMETRIC LIMITER LOG (AVOIDANCE WINDOW)', r_asym.telemetry);
end

function print_telemetry_table(title_str, tel)
    if isempty(tel)
        fprintf('No telemetry recorded for %s\n', title_str);
        return;
    end
    
    fprintf('========================================================================================\n');
    fprintf(' %s\n', title_str);
    fprintf(' Tick | Time(s) | Speed(m/s) | ObsDist(m) |  TTC(s)  | Cmd_a(m/s2) | Act_a(m/s2) | Jerk(m/s3) | Latch\n');
    fprintf('----------------------------------------------------------------------------------------\n');
    
    % Find critical window: last 25 ticks or ticks where TTC < 2.5
    N = length(tel);
    start_idx = max(1, N - 25);
    
    for i = start_idx:N
        t = tel(i);
        obs_d = t.min_obstacle_dist;
        v = t.v;
        if v > 0.1
            ttc = obs_d / v;
        else
            ttc = 99.9;
        end
        latch_str = ternary(isfield(t, 'latch_active') && t.latch_active, 'TRUE', 'false');
        
        fprintf(' %4d | %7.2f | %10.3f | %10.3f | %8.2f | %11.3f | %11.3f | %10.3f | %5s\n', ...
                i, t.time, v, obs_d, min(ttc, 99.9), t.target_a, t.actual_a, t.jerk, latch_str);
    end
    fprintf('========================================================================================\n\n');
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
