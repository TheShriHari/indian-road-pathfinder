%% SIH PS-26037 — Main Closed-Loop Simulation (All 5 Scenarios)
%
% Runs the complete pipeline for each scenario:
%   generate_scenarios -> dynamic_obstacle_predictor (EKF)
%   -> adaptive_path_planner (Hybrid A*) -> behavior_state_machine
%   -> pure_pursuit_controller -> vehicle_kinematics -> evaluate_metrics
%
% No Automated Driving Toolbox, Navigation Toolbox, or Robotics System
% Toolbox calls — runs on MATLAB Online base install.
%
% Run with:  main_simulation
% or headlessly: matlab -batch main_simulation

clear; clc;

fprintf('==========================================================\n');
fprintf('  SIH 2026 PS-26037: Adaptive Path Planning              \n');
fprintf('  Unstructured Indian Roads — Closed-Loop Simulation     \n');
fprintf('==========================================================\n\n');

% ---- Global simulation parameters ----
DT               = 0.1;    % timestep [s]
MAX_STEPS        = 600;    % max steps per scenario (60 s)
SAFETY_RADIUS    = 2.0;    % collision detection threshold [m]
GOAL_REACH_DIST  = 2.5;    % goal-reached threshold [m]
N_HORIZON        = 20;     % EKF prediction horizon [steps]
REPLAN_PERIOD    = 8;      % periodic replan every N steps
REPLAN_THREAT_R  = 3.0;    % replan if predicted agent within this distance of path [m]

% Summary storage
all_metrics = cell(5, 1);

for sc_id = 1:5

    % Reset EKF persistent state at start of each scenario
    clear dynamic_obstacle_predictor;

    fprintf('\n==========  SCENARIO %d  ==========================================\n', sc_id);
    scenario = generate_scenarios(sc_id);
    fprintf('  Title      : %s\n', scenario.title);
    fprintf('  Description: %s\n', scenario.description);
    fprintf('  Start      : [%.1f, %.1f, %.3f rad]\n', scenario.start_pose(1), ...
            scenario.start_pose(2), scenario.start_pose(3));
    fprintf('  Goal       : [%.1f, %.1f, %.3f rad]\n', scenario.goal_pose(1), ...
            scenario.goal_pose(2), scenario.goal_pose(3));
    fprintf('  Agents     : %d\n', length(scenario.obstacles));
    for k = 1:length(scenario.obstacles)
        obs = scenario.obstacles(k);
        fprintf('    [%d] %-14s  pos=[%5.1f,%5.1f]  vel=[%+.2f,%+.2f]  profile=%s\n', ...
            obs.id, obs.type, obs.position(1), obs.position(2), ...
            obs.velocity(1), obs.velocity(2), obs.behavior_profile);
    end

    % ---- Initialise vehicle state [x; y; theta; v] ----
    state = [scenario.start_pose(1);
             scenario.start_pose(2);
             scenario.start_pose(3);
             0.0];

    % Working copy of obstacle positions/velocities (integrated each step)
    obs_live = scenario.obstacles;

    % ---- Initial path plan (no dynamic obstacles yet) ----
    fprintf('\n  [t=0] Initial Hybrid A* plan:\n  ');
    [path, ~, lat0] = adaptive_path_planner( ...
        scenario.start_pose, scenario.goal_pose, ...
        scenario.static_map, [], scenario.grid_res);

    % ---- Logging arrays ----
    time_hist  = zeros(1, MAX_STEPS);
    state_hist = zeros(MAX_STEPS, 4);
    ctrl_hist  = zeros(MAX_STEPS, 2);
    lat_hist   = [];      % only nonzero replanning latencies
    obs_hist   = cell(1, MAX_STEPS);

    bsm_state      = 'CRUISE';
    prev_bsm_state = '';
    replan_count   = 0;
    collision_flag = false;
    min_clearance  = Inf;
    step_done      = 0;

    % ---- Closed-loop simulation ----
    for step = 1:MAX_STEPS
        t = step * DT;

        % (a) Advance obstacle positions using simple Euler + behavior profile noise
        for k = 1:length(obs_live)
            % Apply behavior-specific velocity perturbation
            switch obs_live(k).behavior_profile
                case 'erratic'
                    obs_live(k).velocity = obs_live(k).velocity + ...
                        0.25 * (rand(1,2) - 0.5);
                case 'weaving'
                    obs_live(k).velocity(2) = obs_live(k).velocity(2) + ...
                        0.12 * (rand() - 0.5);
                case 'slow'
                    % pushcart: very small random direction wobble
                    obs_live(k).velocity = obs_live(k).velocity + ...
                        0.03 * (rand(1,2) - 0.5);
                % 'steady' and others: no perturbation
            end
            % Clamp velocity to reasonable range
            obs_live(k).velocity = max(min(obs_live(k).velocity, 5.0), -5.0);
            % Integrate position
            obs_live(k).position = obs_live(k).position + obs_live(k).velocity * DT;
            % Clamp to world bounds
            obs_live(k).position(1) = min(max(obs_live(k).position(1), ...
                scenario.bounds(1)), scenario.bounds(2));
            obs_live(k).position(2) = min(max(obs_live(k).position(2), ...
                scenario.bounds(3)), scenario.bounds(4));
        end

        % (b) EKF: predict dynamic agent trajectories
        %     Prints [EKF] line for first agent on every step (verify Kalman math)
        predictions = dynamic_obstacle_predictor(obs_live, DT, N_HORIZON);

        % (c) Decide whether to replan
        needs_replan = (mod(step, REPLAN_PERIOD) == 0);  % periodic replan
        if ~isempty(predictions) && ~needs_replan
            for k = 1:length(predictions)
                wp = predictions(k).waypoints;
                if isempty(wp), continue; end
                % Check if any predicted waypoint is close to current planned path
                d_to_path = min(hypot(path(:,1) - wp(1,1), path(:,2) - wp(1,2)));
                if d_to_path < REPLAN_THREAT_R
                    needs_replan = true;
                    break;
                end
            end
        end

        if needs_replan
            cur_pose = [state(1), state(2), state(3)];
            [path_new, ~, lat_ms] = adaptive_path_planner( ...
                cur_pose, scenario.goal_pose, ...
                scenario.static_map, predictions, scenario.grid_res);
            if size(path_new, 1) > 2
                path = path_new;
                replan_count = replan_count + 1;
                lat_hist(end+1) = lat_ms; %#ok<AGROW>
            end
        end

        % (d) Behavioral state machine
        [bsm_state, v_ref, ~] = behavior_state_machine( ...
            bsm_state, state', predictions, DT, []);
        if ~strcmp(bsm_state, prev_bsm_state)
            fprintf('  [t=%5.1f s | step %3d] BSM: %-14s -> %-14s  (v_ref=%.1f m/s)\n', ...
                    t, step, prev_bsm_state, bsm_state, v_ref);
            prev_bsm_state = bsm_state;
        end

        % (e) Pure pursuit lateral + longitudinal control
        if size(path, 1) >= 2
            [ctrl, ~, ~] = pure_pursuit_controller(state', path, v_ref, []);
        else
            ctrl = [0; 0];
        end

        % (f) Integrate vehicle state (Euler integration)
        sd    = vehicle_kinematics(state, ctrl, []);
        state = state + sd * DT;

        % Log
        time_hist(step)     = t;
        state_hist(step, :) = state';
        ctrl_hist(step, :)  = ctrl';
        obs_hist{step}      = obs_live;
        step_done           = step;

        % (g) Collision check
        for k = 1:length(obs_live)
            d = norm(state(1:2)' - obs_live(k).position);
            if d < min_clearance, min_clearance = d; end
            if d < SAFETY_RADIUS
                collision_flag = true;
                fprintf('  *** COLLISION DETECTED at t=%.1f s! ego=[%.1f,%.1f] obs=[%.1f,%.1f] d=%.2f m\n', ...
                    t, state(1), state(2), obs_live(k).position(1), obs_live(k).position(2), d);
            end
        end

        % (h) Goal check
        goal_dist = norm(state(1:2) - [scenario.goal_pose(1); scenario.goal_pose(2)]);
        if goal_dist < GOAL_REACH_DIST
            fprintf('\n  *** GOAL REACHED at step %d (t=%.1f s)  pos=[%.2f, %.2f]  dist_to_goal=%.3f m\n', ...
                    step, t, state(1), state(2), goal_dist);
            break;
        end
    end

    % ---- Trim logs ----
    n = step_done;
    time_hist_  = time_hist(1:n);
    state_hist_ = state_hist(1:n, :);
    ctrl_hist_  = ctrl_hist(1:n, :);
    obs_hist_   = obs_hist(1:n);

    % ---- Metrics ----
    metrics = evaluate_metrics(time_hist_, state_hist_, ctrl_hist_, ...
                               lat_hist, obs_hist_, scenario.goal_pose);
    all_metrics{sc_id} = metrics;

    final_pos = state(1:2);
    goal_d    = norm(final_pos' - scenario.goal_pose(1:2));
    fprintf('  Replans: %d  |  Min clearance: %.3f m  |  Final-to-goal: %.3f m  |  Collision: %s\n', ...
        replan_count, min_clearance, goal_d, bool2str(collision_flag));
end

% ---- Final Summary Table ----
fprintf('\n\n==========================================================\n');
fprintf('          FINAL SUMMARY — ALL 5 SCENARIOS               \n');
fprintf('==========================================================\n');
fprintf('%-5s  %-35s  %-8s  %-12s  %-10s  %-18s\n', ...
    'Sc#', 'Title', 'Replans', 'MinClear(m)', 'GoalDist(m)', 'Status');
fprintf('%s\n', repmat('-', 1, 95));
names = {'Unmarked Village Road', 'Signal-less Intersection', ...
         'Highway Merge', 'Dense Market Area', 'Cattle Crossing'};
for sc_id = 1:5
    m = all_metrics{sc_id};
    if isempty(m), continue; end
    fprintf('%-5d  %-35s  %-8s  %-12.3f  %-10.3f  %s\n', ...
        sc_id, names{sc_id}, '(see log)', ...
        m.min_safety_clearance_m, m.final_goal_dist_m, m.completion_status);
end
fprintf('==========================================================\n\n');

% ---- Local helper ----
function s = bool2str(b)
    if b, s = 'YES'; else, s = 'NO'; end
end
