%% TEST_SENSOR_LAYER  Rigorous mathematical test suite for Phase 1 perception layer.
%
% Validates the 3-Layer Architecture sensor degradation model against mathematical
% specifications and empirical statistical criteria:
%   1. Spatial Range Gate (R_max = 35.0 m boundary enforcement)
%   2. Azimuth Horizontal FoV Gate (140 deg width, +/- 70 deg with branch-cut wrapping)
%   3. Heteroscedastic Noise Distribution (N = 10,000 trials, range-scaled variance)
%   4. Non-Linear Stochastic Dropout (N = 10,000 trials, quadratic profile + class penalty)
%   5. Semantic Label Confusion Matrix (N = 10,000 trials, 3% Indian ODD mutation)
%   6. Transport Delay (FIFO Buffer exact 2-tick latency verification)
%   7. Canonical Closed-Loop Simulation with EKF tracking
%
% NOTE: All assertions throw hard error() calls to ensure non-zero exit codes in headless MATLAB.

clear; clc;
fprintf('=================================================================\n');
fprintf('  PHASE 1: SENSOR DEGRADATION & PERCEPTION SYNTHESIS TEST SUITE\n');
fprintf('=================================================================\n\n');

% Compute root directory and ensure .tmp exists at root
script_path = mfilename('fullpath');
if isempty(script_path)
    repo_root = pwd;
else
    repo_root = fileparts(fileparts(script_path));
end
tmp_dir = fullfile(repo_root, '.tmp');
if ~exist(tmp_dir, 'dir')
    mkdir(tmp_dir);
end

metrics = struct();


%% ── Test 1: Spatial Euclidean Range Gate ──────────────────────────────────
fprintf('--- [TEST 1] Spatial Range Gate (R_max = 35.0 m) ---\n');
simulate_sensor_detection([], [], struct('reset', true));

ego_pose = [0.0, 0.0, 0.0];
cfg_no_noise = struct('reset', true, 'latency_ticks', 0, 'p_base', 0.0, 'p_max', 0.0, ...
                      'std_pos_min', 0.0, 'std_pos_max', 0.0, 'std_vel', 0.0, ...
                      'misclass_prob', 0.0, 'verbose', false);

% Target just inside (34.9 m)
obs_inside = struct('id', 1, 'type', 'auto_rickshaw', 'position', [34.9, 0.0], ...
                    'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_inside, ~] = simulate_sensor_detection(obs_inside, ego_pose, cfg_no_noise);
assert(length(det_inside) == 1, 'TEST 1 FAILED: Target at 34.9 m must pass range gate.');

% Target exactly on boundary (35.0 m)
obs_boundary = struct('id', 2, 'type', 'auto_rickshaw', 'position', [35.0, 0.0], ...
                      'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_boundary, ~] = simulate_sensor_detection(obs_boundary, ego_pose, cfg_no_noise);
assert(length(det_boundary) == 1, 'TEST 1 FAILED: Target at 35.0 m must pass range gate.');

% Target just outside (35.1 m)
obs_outside = struct('id', 3, 'type', 'auto_rickshaw', 'position', [35.1, 0.0], ...
                     'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_outside, ~] = simulate_sensor_detection(obs_outside, ego_pose, cfg_no_noise);
assert(isempty(det_outside), 'TEST 1 FAILED: Target at 35.1 m must be pruned by range gate.');

metrics.range_gate_passed = true;
fprintf('  [PASS] Range gate strictly admits d <= 35.0 m and prunes d = 35.1 m.\n\n');


%% ── Test 2: Azimuth Horizontal FoV Gate (+/- 70 deg) ─────────────────────
fprintf('--- [TEST 2] Azimuth FoV Gate (140 deg, +/- 70 deg) ---\n');

% Angle within FoV (+65 deg) at 20 m
ang_in = 65 * pi / 180;
obs_fov_in = struct('id', 1, 'type', 'pedestrian', ...
                    'position', [20.0 * cos(ang_in), 20.0 * sin(ang_in)], ...
                    'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_fov_in, ~] = simulate_sensor_detection(obs_fov_in, [0, 0, 0], cfg_no_noise);
assert(length(det_fov_in) == 1, 'TEST 2 FAILED: Target at +65 deg must be within FoV.');

% Angle within FoV (-65 deg) at 20 m
ang_in_neg = -65 * pi / 180;
obs_fov_in_neg = struct('id', 2, 'type', 'pedestrian', ...
                        'position', [20.0 * cos(ang_in_neg), 20.0 * sin(ang_in_neg)], ...
                        'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_fov_in_neg, ~] = simulate_sensor_detection(obs_fov_in_neg, [0, 0, 0], cfg_no_noise);
assert(length(det_fov_in_neg) == 1, 'TEST 2 FAILED: Target at -65 deg must be within FoV.');

% Angle outside FoV (+75 deg) at 20 m
ang_out = 75 * pi / 180;
obs_fov_out = struct('id', 3, 'type', 'pedestrian', ...
                     'position', [20.0 * cos(ang_out), 20.0 * sin(ang_out)], ...
                     'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_fov_out, ~] = simulate_sensor_detection(obs_fov_out, [0, 0, 0], cfg_no_noise);
assert(isempty(det_fov_out), 'TEST 2 FAILED: Target at +75 deg must be pruned by FoV gate.');

% Angle outside FoV (-75 deg) at 20 m
ang_out_neg = -75 * pi / 180;
obs_fov_out_neg = struct('id', 4, 'type', 'pedestrian', ...
                         'position', [20.0 * cos(ang_out_neg), 20.0 * sin(ang_out_neg)], ...
                         'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_fov_out_neg, ~] = simulate_sensor_detection(obs_fov_out_neg, [0, 0, 0], cfg_no_noise);
assert(isempty(det_fov_out_neg), 'TEST 2 FAILED: Target at -75 deg must be pruned by FoV gate.');

% Branch-cut heading wrapping: Ego heading near +pi, target near -pi
ego_near_pi = [0.0, 0.0, 3.10]; % ~177.6 deg
% Obstacle at bearing -3.10 rad (-177.6 deg) -> relative bearing is -6.20 + 2*pi = +0.083 rad (~4.7 deg)
obs_branch = struct('id', 5, 'type', 'pedestrian', ...
                    'position', [10.0 * cos(-3.10), 10.0 * sin(-3.10)], ...
                    'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[det_branch, ~] = simulate_sensor_detection(obs_branch, ego_near_pi, cfg_no_noise);
assert(length(det_branch) == 1, 'TEST 2 FAILED: Branch cut wrapping across +/-pi failed.');

metrics.fov_gate_passed = true;
fprintf('  [PASS] FoV gate strictly admits +/-65 deg, prunes +/-75 deg, and wraps +/-pi.\n\n');


%% ── Test 3: Empirical Heteroscedastic Noise Distribution (N = 10,000) ────
fprintf('--- [TEST 3] Noise Distribution & Variance Scaling (N = 10,000) ---\n');
rng(12345); % deterministic seed

N_trials = 10000;
cfg_noise = struct('reset', true, 'latency_ticks', 0, 'p_base', 0.0, 'p_max', 0.0, ...
                   'std_pos_min', 0.15, 'std_pos_max', 0.30, 'std_vel', 0.20, ...
                   'misclass_prob', 0.0, 'verbose', false);

% Case 3A: Near-field (d = 0 m) -> theoretical std = 0.15 m
pos_err_near = zeros(N_trials, 2);
vel_err_near = zeros(N_trials, 2);
obs_0m = struct('id', 1, 'type', 'auto_rickshaw', 'position', [0.0, 0.0], ...
                'velocity', [2.0, -1.0], 'behavior_profile', 'nominal');

for i = 1:N_trials
    [d_out, ~] = simulate_sensor_detection(obs_0m, [0.0, 0.0, 0.0], cfg_noise);
    pos_err_near(i, :) = d_out.position - [0.0, 0.0];
    vel_err_near(i, :) = d_out.velocity - [2.0, -1.0];
end

mu_pos_near = mean(pos_err_near(:));
std_pos_near = std(pos_err_near(:));
std_vel_meas = std(vel_err_near(:));

fprintf('  d =  0.0 m: mean = %+.4f m (req: |mu| < 0.02), std = %.4f m (req: 0.15 +/- 0.015)\n', ...
        mu_pos_near, std_pos_near);
assert(abs(mu_pos_near) < 0.02, 'TEST 3A FAILED: Near-field position noise mean is biased.');
assert(abs(std_pos_near - 0.15) < 0.015, 'TEST 3A FAILED: Near-field position std deviates > 10%.');
assert(abs(std_vel_meas - 0.20) < 0.015, 'TEST 3A FAILED: Velocity noise std deviates > 10%.');

% Case 3B: Far-field (d = 35.0 m) -> theoretical std = 0.30 m
pos_err_far = zeros(N_trials, 2);
obs_35m = struct('id', 2, 'type', 'auto_rickshaw', 'position', [35.0, 0.0], ...
                 'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');

for i = 1:N_trials
    [d_out, ~] = simulate_sensor_detection(obs_35m, [0.0, 0.0, 0.0], cfg_noise);
    pos_err_far(i, :) = d_out.position - [35.0, 0.0];
end

mu_pos_far = mean(pos_err_far(:));
std_pos_far = std(pos_err_far(:));

fprintf('  d = 35.0 m: mean = %+.4f m (req: |mu| < 0.02), std = %.4f m (req: 0.30 +/- 0.020)\n', ...
        mu_pos_far, std_pos_far);
assert(abs(mu_pos_far) < 0.02, 'TEST 3B FAILED: Far-field position noise mean is biased.');
assert(abs(std_pos_far - 0.30) < 0.020, 'TEST 3B FAILED: Far-field position std deviates > 7%.');

metrics.noise_distribution_valid = true;
metrics.empirical_std_near = std_pos_near;
metrics.empirical_std_far  = std_pos_far;
fprintf('  [PASS] Noise distribution strictly matches heteroscedastic model.\n\n');


%% ── Test 4: Non-Linear Dropout Frequency (N = 10,000) ────────────────────
fprintf('--- [TEST 4] Quadratic Dropout Frequency (N = 10,000) ---\n');
cfg_drop = struct('reset', true, 'latency_ticks', 0, 'p_base', 0.05, 'p_max', 0.10, ...
                  'std_pos_min', 0.0, 'std_pos_max', 0.0, 'std_vel', 0.0, ...
                  'misclass_prob', 0.0, 'verbose', false);

% 4A: Nominal actor (auto_rickshaw) at d = 0 m -> P_drop = 0.05
drops_0m = 0;
for i = 1:N_trials
    [d_out, ~] = simulate_sensor_detection(obs_0m, [0.0, 0.0, 0.0], cfg_drop);
    if isempty(d_out), drops_0m = drops_0m + 1; end
end
p_emp_0m = drops_0m / N_trials;
fprintf('  auto_rickshaw @  0m: p_emp = %.4f (theory: 0.0500, margin +/- 0.015)\n', p_emp_0m);
assert(abs(p_emp_0m - 0.05) <= 0.015, 'TEST 4A FAILED: Near-field dropout out of tolerance.');

% 4B: Nominal actor (auto_rickshaw) at d = 35 m -> P_drop = 0.10
drops_35m = 0;
for i = 1:N_trials
    [d_out, ~] = simulate_sensor_detection(obs_35m, [0.0, 0.0, 0.0], cfg_drop);
    if isempty(d_out), drops_35m = drops_35m + 1; end
end
p_emp_35m = drops_35m / N_trials;
fprintf('  auto_rickshaw @ 35m: p_emp = %.4f (theory: 0.1000, margin +/- 0.015)\n', p_emp_35m);
assert(abs(p_emp_35m - 0.10) <= 0.015, 'TEST 4B FAILED: Far-field nominal dropout out of tolerance.');

% 4C: Organic small actor (pedestrian) at d = 35 m -> P_drop = 0.10 + 0.02 = 0.12
obs_ped_35m = struct('id', 3, 'type', 'pedestrian', 'position', [35.0, 0.0], ...
                     'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
drops_ped = 0;
for i = 1:N_trials
    [d_out, ~] = simulate_sensor_detection(obs_ped_35m, [0.0, 0.0, 0.0], cfg_drop);
    if isempty(d_out), drops_ped = drops_ped + 1; end
end
p_emp_ped = drops_ped / N_trials;
fprintf('  pedestrian    @ 35m: p_emp = %.4f (theory: 0.1200, margin +/- 0.015)\n', p_emp_ped);
assert(abs(p_emp_ped - 0.12) <= 0.015, 'TEST 4C FAILED: Small class penalty dropout out of tolerance.');

metrics.dropout_frequency_valid = true;
fprintf('  [PASS] Quadratic dropout probabilities match theory within 1.5%% margin.\n\n');


%% ── Test 5: Semantic Label Confusion (N = 10,000) ────────────────────────
fprintf('--- [TEST 5] Semantic Label Confusion Matrix (N = 10,000) ---\n');
cfg_conf = struct('reset', true, 'latency_ticks', 0, 'p_base', 0.0, 'p_max', 0.0, ...
                   'std_pos_min', 0.0, 'std_pos_max', 0.0, 'std_vel', 0.0, ...
                   'misclass_prob', 0.03, 'verbose', false);

n_misclassed = 0;
mutated_classes = {};
for i = 1:N_trials
    [d_out, ~] = simulate_sensor_detection(obs_0m, [0.0, 0.0, 0.0], cfg_conf);
    if ~strcmp(d_out.type, 'auto_rickshaw')
        n_misclassed = n_misclassed + 1;
        mutated_classes{end+1} = d_out.type; %#ok<AGROW>
    end
end
p_misclass = n_misclassed / N_trials;
fprintf('  Misclassification rate: %.4f (theory: 0.0300, margin +/- 0.010)\n', p_misclass);
assert(abs(p_misclass - 0.03) <= 0.010, 'TEST 5 FAILED: Misclassification rate out of bounds.');

% Verify mutated classes are all valid Indian ODD classes
valid_odd = {'cattle', 'auto_rickshaw', 'pedestrian', 'pushcart'};
for i = 1:length(mutated_classes)
    assert(any(strcmp(mutated_classes{i}, valid_odd)), ...
           sprintf('TEST 5 FAILED: Unknown mutated class %s', mutated_classes{i}));
end
metrics.misclassification_valid = true;
fprintf('  [PASS] Semantic confusion operates strictly at 3%% across Indian ODD classes.\n\n');


%% ── Test 6: Transport Delay (FIFO Buffer) ─────────────────────────────────
fprintf('--- [TEST 6] Transport Delay FIFO Queue (2-Tick Latency) ---\n');
cfg_fifo = struct('reset', false, 'latency_ticks', 2, 'p_base', 0.0, 'p_max', 0.0, ...
                  'std_pos_min', 0.0, 'std_pos_max', 0.0, 'std_vel', 0.0, ...
                  'misclass_prob', 0.0, 'verbose', false);

% Reset explicitly once before test begins
simulate_sensor_detection([], [], struct('reset', true));


% Tick 1: Provide target A (id=101)
t1_in = struct('id', 101, 'type', 'cattle', 'position', [10.0, 0.0], ...
               'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[d_t1, ~] = simulate_sensor_detection(t1_in, [0, 0, 0], cfg_fifo);
assert(isempty(d_t1), 'TEST 6 FAILED: Tick 1 must return empty (buffer warming up).');

% Tick 2: Provide target B (id=102)
t2_in = struct('id', 102, 'type', 'pushcart', 'position', [15.0, 0.0], ...
               'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[d_t2, ~] = simulate_sensor_detection(t2_in, [0, 0, 0], cfg_fifo);
assert(isempty(d_t2), 'TEST 6 FAILED: Tick 2 must return empty (buffer warming up).');

% Tick 3: Provide target C (id=103) -> Output must be Target A (Tick 1)
t3_in = struct('id', 103, 'type', 'pedestrian', 'position', [20.0, 0.0], ...
               'velocity', [0.0, 0.0], 'behavior_profile', 'nominal');
[d_t3, ~] = simulate_sensor_detection(t3_in, [0, 0, 0], cfg_fifo);
assert(length(d_t3) == 1 && d_t3(1).id == 101, ...
       'TEST 6 FAILED: Tick 3 must release Tick 1 observation (id=101).');

% Tick 4: Provide empty input -> Output must be Target B (Tick 2)
[d_t4, ~] = simulate_sensor_detection(struct([]), [0, 0, 0], cfg_fifo);
assert(length(d_t4) == 1 && d_t4(1).id == 102, ...
       'TEST 6 FAILED: Tick 4 must release Tick 2 observation (id=102).');

% Tick 5: Provide empty input -> Output must be Target C (Tick 3)
[d_t5, ~] = simulate_sensor_detection(struct([]), [0, 0, 0], cfg_fifo);
assert(length(d_t5) == 1 && d_t5(1).id == 103, ...
       'TEST 6 FAILED: Tick 5 must release Tick 3 observation (id=103).');

metrics.fifo_delay_ticks = 2;
fprintf('  [PASS] FIFO transport delay strictly enforces 2-tick (200 ms) latency.\n\n');


%% ── Test 7: Canonical Closed-Loop Simulation with EKF ─────────────────────
fprintf('--- [TEST 7] Canonical Scenario Closed-Loop Verification ---\n');

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

opts.max_steps        = 800;
opts.dt               = 0.1;
opts.collision_thresh = 1.0;
opts.goal_dist_tol    = 2.5;
opts.verbose          = false;

result = run_single_scenario(obs_cfg, 42, opts);
fprintf('  Simulation Outcome: %s (Time: %.1f s, Steps: %d)\n', ...
        result.outcome, result.time_to_goal, result.steps_taken);
fprintf('  Total Dropouts    : %d\n', result.total_dropouts);
fprintf('  Total Misclasses  : %d\n', result.total_misclasses);

if ~isempty(result.innov_log)
    fprintf('  Mean |innovation| : %.4f m\n', result.innov_mean);
    assert(result.innov_mean > 0.02, 'TEST 7 FAILED: EKF innovation is near zero.');
end

assert(strcmp(result.outcome, 'SUCCESS'), ...
       sprintf('TEST 7 FAILED: Closed loop simulation outcome was %s', result.outcome));
metrics.closed_loop_passed = true;
fprintf('  [PASS] Closed loop vehicle safely reached goal under sensor degradation.\n\n');


%% ── Export Structured Metrics to .tmp/sensor_metrics.json ─────────────────
json_str = sprintf(['{\n', ...
    '  "range_gate_passed": %s,\n', ...
    '  "fov_gate_passed": %s,\n', ...
    '  "noise_distribution_valid": %s,\n', ...
    '  "dropout_frequency_valid": %s,\n', ...
    '  "misclassification_valid": %s,\n', ...
    '  "fifo_delay_ticks": %d,\n', ...
    '  "closed_loop_passed": %s,\n', ...
    '  "empirical_std_near": %.4f,\n', ...
    '  "empirical_std_far": %.4f,\n', ...
    '  "status": "PASS"\n', ...
    '}\n'], ...
    tf2str(metrics.range_gate_passed), ...
    tf2str(metrics.fov_gate_passed), ...
    tf2str(metrics.noise_distribution_valid), ...
    tf2str(metrics.dropout_frequency_valid), ...
    tf2str(metrics.misclassification_valid), ...
    metrics.fifo_delay_ticks, ...
    tf2str(metrics.closed_loop_passed), ...
    metrics.empirical_std_near, ...
    metrics.empirical_std_far);

fid = fopen(fullfile(tmp_dir, 'sensor_metrics.json'), 'w');
if fid ~= -1
    fwrite(fid, json_str);
    fclose(fid);
end

% Also write to current directory .tmp if different
if ~strcmp(tmp_dir, fullfile(pwd, '.tmp'))
    local_tmp = fullfile(pwd, '.tmp');
    if ~exist(local_tmp, 'dir'), mkdir(local_tmp); end
    fid_local = fopen(fullfile(local_tmp, 'sensor_metrics.json'), 'w');
    if fid_local ~= -1
        fwrite(fid_local, json_str);
        fclose(fid_local);
    end
end


fprintf('=================================================================\n');
fprintf('  ALL PHASE 1 SENSOR LAYER TESTS PASSED SUCCESSFULLY!\n');
fprintf('=================================================================\n');

function s = tf2str(b)
    if b
        s = 'true';
    else
        s = 'false';
    end
end
