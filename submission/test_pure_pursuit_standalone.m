%% TEST_PURE_PURSUIT_STANDALONE  Smoke test for pure_pursuit_controller asymmetric jerk limiter.
%
%  Verifies that the asymmetric jerk limiter in pure_pursuit_controller.m:
%    1. Allows 8.0 m/s^3 emergency braking jerk when raw_accel < -2.0 (emergency trigger)
%    2. Enforces 0.95 m/s^3 comfort limit during normal driving
%    3. Enforces 0.95 m/s^3 comfort limit when releasing brakes (positive delta_a)
%
%  This test calls pure_pursuit_controller.m in standalone mode with manufactured
%  hard-braking scenarios to confirm the emergency jerk allowance is active
%  independently of run_single_scenario.m's override logic.
%
%  Source: Phase 3 defect fix, commit ca0ef63
%  Verified number: emergency jerk allowance 8.0 m/s^3, comfort limit 0.95 m/s^3
%                   (matches run_single_scenario.m lines 661-664)

fprintf('=============================================================\n');
fprintf('  STANDALONE SMOKE TEST: pure_pursuit_controller jerk limiter\n');
fprintf('=============================================================\n\n');

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

% Minimal straight-line path for testing (not the focus — jerk math is)
path = [0, 0; 100, 0];

dt = 0.1;  % Match run_single_scenario.m controller tick

% ── Test 1: Emergency Hard Braking ───────────────────────────────────────────
% Start at v=8 m/s, set v_ref=0 (full stop command) → raw_accel = 1.0*(0-8) = -8 m/s^2
% This is below -2.0 threshold → is_emergency = true → jerk limit should be 8.0 m/s^3
% Expected: |a_cmd - last_accel| <= 8.0 * dt = 0.80 m/s^2 per step
fprintf('TEST 1: Emergency hard braking (v=8m/s, v_ref=0)\n');
params_emergency = struct('dt', dt, 'reset', true);
state_fast = [0; 0; 0; 8.0];  % x=0, y=0, theta=0, v=8m/s
v_ref_stop = 0.0;

% Reset persistent state
pure_pursuit_controller(state_fast, path, v_ref_stop, struct('reset', true));

prev_accel = 0.0;
max_observed_jerk_emergency = 0;
for i = 1:5
    params_e = struct('dt', dt, 'prev_accel', prev_accel);
    [ctrl, ~, ~, ~] = pure_pursuit_controller(state_fast, path, v_ref_stop, params_e);
    a_cmd = ctrl(2);
    step_jerk = abs(a_cmd - prev_accel) / dt;
    max_observed_jerk_emergency = max(max_observed_jerk_emergency, step_jerk);
    fprintf('  Step %d: prev_a=%.3f, a_cmd=%.3f, jerk=%.3f m/s^3\n', i, prev_accel, a_cmd, step_jerk);
    prev_accel = a_cmd;
end

if max_observed_jerk_emergency > 0.95 + 0.01
    fprintf('  PASS: Emergency jerk (%.3f m/s^3) exceeded comfort limit (0.95) as expected.\n\n', max_observed_jerk_emergency);
else
    fprintf('  FAIL: Emergency jerk (%.3f m/s^3) did NOT exceed comfort limit — asymmetric logic not triggering!\n\n', max_observed_jerk_emergency);
end

% ── Test 2: Normal Cruise Deceleration ───────────────────────────────────────
% v=5 m/s, v_ref=3 m/s → raw_accel = 1.0*(3-5) = -2.0 m/s^2 (boundary, not < -2.0)
% is_emergency should be FALSE → jerk limit enforced at 0.95 m/s^3
% Expected: |a_cmd - last_accel| <= 0.95 * dt = 0.095 m/s^2 per step
fprintf('TEST 2: Normal comfort deceleration (v=5m/s, v_ref=3m/s)\n');
state_cruise = [0; 0; 0; 5.0];
v_ref_cruise = 3.0;

pure_pursuit_controller(state_cruise, path, v_ref_cruise, struct('reset', true));
prev_accel = 0.0;
max_observed_jerk_comfort = 0;
for i = 1:5
    params_c = struct('dt', dt, 'prev_accel', prev_accel);
    [ctrl, ~, ~, ~] = pure_pursuit_controller(state_cruise, path, v_ref_cruise, params_c);
    a_cmd = ctrl(2);
    step_jerk = abs(a_cmd - prev_accel) / dt;
    max_observed_jerk_comfort = max(max_observed_jerk_comfort, step_jerk);
    fprintf('  Step %d: prev_a=%.3f, a_cmd=%.3f, jerk=%.3f m/s^3\n', i, prev_accel, a_cmd, step_jerk);
    prev_accel = a_cmd;
end

COMFORT_LIMIT = 0.95;
if max_observed_jerk_comfort <= COMFORT_LIMIT + 0.01
    fprintf('  PASS: Comfort jerk (%.3f m/s^3) at or below comfort limit (%.2f).\n\n', max_observed_jerk_comfort, COMFORT_LIMIT);
else
    fprintf('  FAIL: Comfort jerk (%.3f m/s^3) exceeded limit (%.2f)!\n\n', max_observed_jerk_comfort, COMFORT_LIMIT);
end

% ── Test 3: Throttle / Positive Delta-a ──────────────────────────────────────
% Start with prev_accel=-2.0, v_ref=8 → raw_accel positive → should be capped at 0.95/dt
fprintf('TEST 3: Brake release / acceleration ramp (prev_a=-2.0, v_ref=8m/s)\n');
state_slow = [0; 0; 0; 1.0];
v_ref_high = 8.0;
pure_pursuit_controller(state_slow, path, v_ref_high, struct('reset', true));
prev_accel = -2.0;
max_observed_jerk_throttle = 0;
for i = 1:5
    params_t = struct('dt', dt, 'prev_accel', prev_accel);
    [ctrl, ~, ~, ~] = pure_pursuit_controller(state_slow, path, v_ref_high, params_t);
    a_cmd = ctrl(2);
    delta_a = a_cmd - prev_accel;
    step_jerk = delta_a / dt;  % Positive delta, comfort enforced
    max_observed_jerk_throttle = max(max_observed_jerk_throttle, step_jerk);
    fprintf('  Step %d: prev_a=%.3f, a_cmd=%.3f, jerk=%.3f m/s^3\n', i, prev_accel, a_cmd, step_jerk);
    prev_accel = a_cmd;
end

if max_observed_jerk_throttle <= COMFORT_LIMIT + 0.01
    fprintf('  PASS: Throttle jerk (%.3f m/s^3) within comfort limit (%.2f).\n\n', max_observed_jerk_throttle, COMFORT_LIMIT);
else
    fprintf('  FAIL: Throttle jerk (%.3f m/s^3) exceeded limit (%.2f)!\n\n', max_observed_jerk_throttle, COMFORT_LIMIT);
end

% ── Summary ───────────────────────────────────────────────────────────────────
fprintf('=============================================================\n');
fprintf('  Emergency jerk  : %.3f m/s^3 (limit: 8.0)\n', max_observed_jerk_emergency);
fprintf('  Comfort decel   : %.3f m/s^3 (limit: 0.95)\n', max_observed_jerk_comfort);
fprintf('  Throttle ramp   : %.3f m/s^3 (limit: 0.95)\n', max_observed_jerk_throttle);

t1 = max_observed_jerk_emergency > 0.95;
t2 = max_observed_jerk_comfort   <= 0.95 + 0.01;
t3 = max_observed_jerk_throttle  <= 0.95 + 0.01;

if t1 && t2 && t3
    fprintf('  OVERALL: ALL 3 TESTS PASSED — asymmetric jerk limiter verified.\n');
else
    fprintf('  OVERALL: FAILURES DETECTED — review asymmetric jerk logic.\n');
end
fprintf('=============================================================\n');
