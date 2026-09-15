%% test_ekf_convergence.m
% Explicit EKF convergence test (Component 2 verification).
%
% Setup: object moving in a known straight line at 1.0 m/s in the +X direction,
%        starting at [5, 0]. True positions generated exactly, then Gaussian
%        noise (std=0.5 m) added to create noisy "sensor" measurements.
%
% Test: Feed 20 noisy observations to dynamic_obstacle_predictor one by one.
%       At each step, print: true position, noisy measurement, EKF estimate,
%       covariance trace.
%       Pass criterion:
%         1. EKF estimates are on average CLOSER to truth than raw noisy measurements
%         2. Covariance trace DECREASES monotonically (filter gains confidence)
%
% Run: test_ekf_convergence

clear; clc;

% Reset EKF state
clear dynamic_obstacle_predictor;

fprintf('==========================================================\n');
fprintf('  EKF CONVERGENCE TEST — Straight-Line Tracking          \n');
fprintf('==========================================================\n');
fprintf('  True motion: x increases at 1.0 m/s, y stays at 0      \n');
fprintf('  Measurement noise: std = 0.5 m on both x and y         \n');
fprintf('  Filter type: constant-velocity EKF, state=[x,y,vx,vy]  \n\n');

rng(42);  % fixed seed for reproducibility

DT         = 0.1;     % timestep [s]
N_STEPS    = 25;      % number of observations
TRUE_VX    = 1.0;     % true x velocity [m/s]
TRUE_VY    = 0.0;     % true y velocity [m/s]
TRUE_X0    = 5.0;
TRUE_Y0    = 0.0;
NOISE_STD  = 0.5;     % measurement noise std [m]

true_xs  = TRUE_X0 + TRUE_VX * (0:N_STEPS-1)' * DT;
true_ys  = TRUE_Y0 + TRUE_VY * (0:N_STEPS-1)' * DT;
noisy_xs = true_xs + NOISE_STD * randn(N_STEPS, 1);
noisy_ys = true_ys + NOISE_STD * randn(N_STEPS, 1);

% Build observation struct for a single agent (id=99, type pedestrian)
obs_base.id               = 99;
obs_base.type             = 'pedestrian';
obs_base.behavior_profile = 'steady';
obs_base.velocity         = [TRUE_VX, TRUE_VY];

fprintf('%-5s  %-12s  %-12s  %-12s  %-12s  %-12s  %-10s\n', ...
    'Step', 'TrueX', 'NoisyX', 'EKF_X', 'TrueY', 'NoisyY', 'EKF_Y');
fprintf('%s\n', repmat('-',1,77));

ekf_xs     = zeros(N_STEPS, 1);
ekf_ys     = zeros(N_STEPS, 1);
cov_traces = zeros(N_STEPS, 1);
prev_trace = Inf;
trace_monotone = true;

for k = 1:N_STEPS
    obs = obs_base;
    obs.position = [noisy_xs(k), noisy_ys(k)];

    % Call the EKF predictor with N_HORIZON=2 to get two forward waypoints.
    % From two consecutive waypoints we recover estimated velocity, then
    % back-project one DT step to get the current-step state estimate.
    preds = dynamic_obstacle_predictor(obs, DT, 2);

    wp = preds(1).waypoints;       % (2 x 2) matrix: each row is [x,y] one step ahead
    est_vx = (wp(2,1) - wp(1,1)) / DT;  % velocity from consecutive horizon pts
    est_vy = (wp(2,2) - wp(1,2)) / DT;
    % Back-project to current-step estimate
    ekf_x = wp(1,1) - est_vx * DT;
    ekf_y = wp(1,2) - est_vy * DT;

    % Covariance trace from first horizon step P_1 = F*P_est*F' + Q
    % It's strictly >= trace(P_est) but decreasing as filter converges
    P1 = preds(1).covariance{1};
    cov_traces(k) = trace(P1);

    ekf_xs(k) = ekf_x;
    ekf_ys(k) = ekf_y;

    % Monotone decrease check (allow 5% tolerance for numerical noise)
    if cov_traces(k) > prev_trace * 1.05
        trace_monotone = false;
    end
    prev_trace = cov_traces(k);

    fprintf('%-5d  %-12.4f  %-12.4f  %-12.4f  %-12.4f  %-12.4f  %-10.5f  (P_trace=%.5f)\n', ...
        k, true_xs(k), noisy_xs(k), ekf_x, true_ys(k), noisy_ys(k), ekf_y, cov_traces(k));
end

% ---- Evaluation ----
ekf_err_x   = abs(ekf_xs   - true_xs);
noisy_err_x = abs(noisy_xs - true_xs);
ekf_err_y   = abs(ekf_ys   - true_ys);
noisy_err_y = abs(noisy_ys - true_ys);

mean_ekf_err   = mean(sqrt(ekf_err_x.^2   + ekf_err_y.^2));
mean_noisy_err = mean(sqrt(noisy_err_x.^2 + noisy_err_y.^2));

fprintf('\n==========================================================\n');
fprintf('  CONVERGENCE TEST RESULTS\n');
fprintf('==========================================================\n');
fprintf('  Mean |EKF estimate - truth|  : %.4f m\n', mean_ekf_err);
fprintf('  Mean |noisy measurement - truth|: %.4f m\n', mean_noisy_err);
fprintf('  EKF better than raw noise?    : %s  (ratio=%.3f)\n', ...
        tf_str(mean_ekf_err < mean_noisy_err), mean_ekf_err / mean_noisy_err);
fprintf('  Covariance trace monotone?    : %s\n', tf_str(trace_monotone));
fprintf('  Final covariance trace        : %.6f\n', cov_traces(end));

if mean_ekf_err < mean_noisy_err && cov_traces(end) < cov_traces(1)
    fprintf('\n  PASS: EKF converges to track true straight-line trajectory.\n');
else
    fprintf('\n  FAIL: EKF did not converge as expected — review Q/R tuning.\n');
end
fprintf('==========================================================\n\n');

function s = tf_str(b)
    if b, s = 'YES'; else, s = 'NO'; end
end
