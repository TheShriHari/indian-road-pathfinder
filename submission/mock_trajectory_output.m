function traj = mock_trajectory_output(ego_state, path, decision, params)
%% MOCK_TRAJECTORY_OUTPUT  Placeholder stub for Subsystem 3: Trajectory Output Interface.
%
%  STATUS: MOCK IMPLEMENTATION — authored to unblock integration testing.
%  This is NOT finished teammate code. Minimal stub only.
%
%  Real implementation owner: [Teammate name — Subsystem 3]
%  Interface contract:
%    Inputs:
%      ego_state : [x, y, theta, v]
%      path      : [N x 2] planned path waypoints [x, y]
%      decision  : decision struct from mock_tactical_decision (or real Subsystem 2)
%                    .maneuver, .v_target, .priority
%      params    : optional config struct
%    Outputs:
%      traj : struct with fields:
%               .waypoints   — [M x 3] array [x, y, v_target] (M <= 20 steps)
%               .horizon_s   — trajectory horizon in seconds
%               .feasible    — boolean: true if trajectory is kinematically feasible
%               .source      — string identifying the generator
%
%  Commit note: stub authored 2026-09-30 by integration harness — not teammate.

if nargin < 4, params = struct(); end

L_WB = 2.7; % Wheelbase (m)
DT   = 0.1; % Match run_single_scenario.m tick
HORIZON_STEPS = 20;

if isempty(path) || size(path,1) < 2
    traj.waypoints = [ego_state(1), ego_state(2), 0.0];
    traj.horizon_s = 0.0;
    traj.feasible  = false;
    traj.source    = 'mock_trajectory_output_stub_v0_empty_path';
    return;
end

% Clamp target velocity from decision
v_tgt = 5.0;
if isstruct(decision) && isfield(decision, 'v_target')
    v_tgt = max(0.0, decision.v_target);
end

% Simple forward projection along path at v_tgt
n_path = size(path, 1);
wps = zeros(min(HORIZON_STEPS, n_path), 3);
for i = 1:size(wps,1)
    wps(i,1) = path(i,1);
    wps(i,2) = path(i,2);
    wps(i,3) = v_tgt;
end

traj.waypoints = wps;
traj.horizon_s = size(wps,1) * DT;
traj.feasible  = true;
traj.source    = 'mock_trajectory_output_stub_v0';
traj.warning   = 'STUB: Replace with real Subsystem 3 implementation before deployment';
end
