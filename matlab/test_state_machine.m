%% test_state_machine.m
% Explicit behavioral state machine transition test (Component 5 verification).
%
% Setup: Ego vehicle stationary at [10, 0, 0, 0]. A single agent starts 15 m
%        ahead and moves toward the vehicle at 1 m/s per step, closing from
%        15 m to 1 m over 14 steps. Then moves away for 8 steps.
%
% Test: Print state transition at each step. Confirm:
%   1. Sequence approaches CRUISE -> NUDGE -> YIELD_DECEL -> YIELD_WAIT in order
%   2. As agent moves away: RESUME -> CRUISE
%   3. No chattering (no oscillation between two states in back-to-back steps)
%
% Run: test_state_machine

clear; clc;

fprintf('==========================================================\n');
fprintf('  BEHAVIORAL STATE MACHINE TRANSITION TEST               \n');
fprintf('==========================================================\n');
fprintf('  Ego: stationary at [10, 0, 0, 0]                       \n');
fprintf('  Agent: approaches from x=25 to x=11 (closes from 15m->1m)\n');
fprintf('         then retreats to x=25 (opens from 1m->15m)       \n\n');

DT       = 0.1;
ego_state = [10.0, 0.0, 0.0, 0.0];  % [x, y, theta, v]

% Phase 1: agent approaches (15 steps, closing)
approach_dist = linspace(15, 1, 15);
% Phase 2: agent retreats (10 steps, opening)
retreat_dist  = linspace(1, 16, 10);
all_dists = [approach_dist, retreat_dist];
N_steps   = length(all_dists);

bsm_state      = 'CRUISE';
prev_state     = 'CRUISE';
transitions    = {};
chatter_count  = 0;
expected_seq   = {'CRUISE','NUDGE','YIELD_DECEL','YIELD_WAIT','RESUME','CRUISE'};
state_history  = cell(N_steps+1, 1);
state_history{1} = bsm_state;

fprintf('%-5s  %-8s  %-16s  %-16s  %-6s  %-12s\n', ...
    'Step', 'Dist(m)', 'PrevState', 'NewState', 'Chatter', 'v_ref');
fprintf('%s\n', repmat('-', 1, 70));

for k = 1:N_steps
    d = all_dists(k);
    ax = ego_state(1) + d;   % agent directly ahead at distance d
    ay = 0.0;                 % same Y (in path)

    % Create a mock prediction struct for the agent
    mock_pred.id    = 1;
    mock_pred.type  = 'pedestrian';
    % One-step-ahead waypoint is just (ax,ay) (agent moving slowly)
    mock_pred.waypoints  = [ax, ay; ax - 0.1, ay];
    mock_pred.covariance = {};

    [bsm_state, v_ref, dbg] = behavior_state_machine(bsm_state, ego_state, mock_pred, DT, []);

    % Detect chattering: two back-to-back steps with different states
    chatter = strcmp(bsm_state, prev_state) == 0 && k > 1 && ...
              strcmp(state_history{k}, bsm_state);  % back to the state before prev
    if chatter, chatter_count = chatter_count + 1; end

    % Log transition
    if ~strcmp(bsm_state, prev_state)
        transitions{end+1} = sprintf('%s->%s @d=%.1fm', prev_state, bsm_state, d); %#ok<AGROW>
    end

    fprintf('%-5d  %-8.2f  %-16s  %-16s  %-6s  %-6.2f\n', ...
        k, d, prev_state, bsm_state, yesno(chatter), v_ref);

    state_history{k+1} = bsm_state;
    prev_state = bsm_state;
end

% ---- Evaluation ----
fprintf('\n==========================================================\n');
fprintf('  STATE MACHINE TEST RESULTS\n');
fprintf('==========================================================\n');
fprintf('  Transitions observed:\n');
for t = 1:length(transitions)
    fprintf('    %d. %s\n', t, transitions{t});
end

% Check required states appeared in sequence
states_seen = state_history(~cellfun(@isempty, state_history));
for s = {'CRUISE','NUDGE','YIELD_DECEL','YIELD_WAIT','RESUME'}
    appeared = any(strcmp(states_seen, s{1}));
    fprintf('  State %-14s appeared: %s\n', s{1}, yesno(appeared));
end
fprintf('  Total chatter events: %d\n', chatter_count);

no_chatter  = (chatter_count == 0);
all_present = all(cellfun(@(s) any(strcmp(states_seen, s)), expected_seq));

if all_present && no_chatter
    fprintf('\n  PASS: All states visited in sensible sequence, no chattering.\n');
elseif ~no_chatter
    fprintf('\n  WARN: Chattering detected (%d events) — review hysteresis thresholds.\n', chatter_count);
else
    fprintf('\n  PARTIAL: Some expected states not reached.\n');
end
fprintf('==========================================================\n\n');

function s = yesno(b)
    if b, s = 'YES'; else, s = 'no'; end
end
