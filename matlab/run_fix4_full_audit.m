function audit_results = run_fix4_full_audit(csv_path)
%% RUN_FIX4_FULL_AUDIT  Audits Fix #4 across all successful trials in the benchmark.
% Specifically identifies every trial where:
% 1. A dynamic agent had clearance in [0.50, 1.00) m
% 2. Ego vehicle was in a yielding/safe-stop state (v < 1.5 m/s, YIELD_WAIT/DECEL/safe_stop)
% 3. Under the baseline (un-gated 1.0m threshold), this would have triggered COLLISION,
%    but under Fix #4 (0.50m threshold), it succeeded.
%
% For every such trial, logs and prints:
% - Exact timestamp t of minimum clearance
% - Ego state: [x, y, theta, v], BSM state
% - Agent state: [x, y, vx, vy], id, type
% - Distance d_agent
% - Body-frame relative coordinates [x_body, y_body]
% - Whether ego was fully stopped (v < 0.1 m/s)
% - Physical bounding box overlap check (length 4.5m, width 1.85m)
% - Classification: 'FALSE_POSITIVE_CORRECTED' vs 'GENUINE_COLLISION_RECLASSIFIED'

if nargin < 1 || isempty(csv_path)
    csv_path = 'batch_test_results_1000_unseen.csv';
end

T = readtable(csv_path);

% Filter to successful trials with dynamic agents and min_clearance < 1.0m
cand = T(strcmp(T.outcome, 'SUCCESS') & T.num_agents > 0 & T.min_clearance_achieved < 1.0, :);
fprintf('=========================================================================\n');
fprintf('  AUDITING FIX #4: Stationary/Yield Clearance Gate                       \n');
fprintf('  Candidate SUCCESS trials with min_clearance < 1.0m and agents > 0: %d\n', height(cand));
fprintf('=========================================================================\n\n');

deciding_trials = struct([]);
n_deciding = 0;
n_stopped = 0;
n_footprint_overlap = 0;

L_WB = 2.8;
ego_length = 4.5;
ego_width = 1.85;
dt = 0.1;

for idx = 1:height(cand)
    seed = cand.seed(idx);
    sc = generate_random_scenario(seed);
    
    % Replay scenario to inspect dynamic agent clearances
    potholes = sc.potholes;
    dynamic_agents = sc.dynamic_agents;
    if isempty(dynamic_agents), continue; end
    
    % Run single scenario in silent mode but with sim_opts
    % Fast replay of kinematics to detect if agent was within [0.50, 1.0)
    % We can call run_single_scenario with a callback or step tracker
    % To be 100% exact, run step by step with same seed
    clear dynamic_obstacle_predictor simulate_sensor_detection;
    rng(seed);
    
    sensor_cfg.max_detection_range = 35.0;
    sensor_cfg.field_of_view_deg   = 140.0;
    sensor_cfg.base_dropout_prob   = 0.05;
    sensor_cfg.std_pos_base        = 0.3;
    sensor_cfg.std_pos_max         = 0.5;
    sensor_cfg.std_vel             = 0.2;
    sensor_cfg.misclass_prob       = 0.03;
    sensor_cfg.latency_ticks       = 2;
    sensor_cfg.verbose             = false;

    map_cfg.grid_res  = 0.2;
    map_cfg.range_fwd = 50.0;
    map_cfg.range_bwd = 10.0;
    map_cfg.range_lat = 15.0;

    sensor_det.road_boundaries = sc.road_boundaries;
    sensor_det.potholes        = potholes;
    sensor_det.static_boxes    = struct([]);
    sensor_det.static_points   = [];

    start_pose = sc.start_pose;
    goal_pose  = sc.goal_pose;
    ego_state  = [start_pose(1); start_pose(2); start_pose(3); 0.0];
    mock_x     = ego_state(1);
    mock_y     = ego_state(2);
    mock_theta = ego_state(3);
    mock_v     = 0.0;

    path = [ego_state(1:2)'; ego_state(1:2)' + [5, 0]];
    prev_target = [NaN, NaN];
    bsm_state = 'CRUISE';
    safe_stop_active = false;
    last_replan_step = -99;
    replan_cnt = 0;
    stopped_wait_steps = 0;

    deciding_in_this_trial = false;
    min_agent_dist_yield = Inf;
    snap = struct([]);

    for step = 1:400
        t = step * dt;

        raw_obs = struct([]);
        for k = 1:length(dynamic_agents)
            pos = dynamic_agents(k).position + dynamic_agents(k).velocity * t;
            raw_obs(k).id               = dynamic_agents(k).id;
            raw_obs(k).type             = dynamic_agents(k).type;
            raw_obs(k).position         = pos;
            raw_obs(k).velocity         = dynamic_agents(k).velocity;
            raw_obs(k).behavior_profile = dynamic_agents(k).behavior_profile;
        end

        [rolling_costmap, grid_meta] = local_occupancy_grid_builder(ego_state(1:3)', sensor_det, map_cfg);
        [detected_obs, ~] = simulate_sensor_detection(raw_obs, ego_state, sensor_cfg);
        [predictions, ~] = dynamic_obstacle_predictor(detected_obs, dt, 35);

        can_replan = (size(path, 1) < 2) || ((step - last_replan_step) >= 3);
        needs_replan = (size(path, 1) < 2) || (mod(step, 8) == 0);
        if safe_stop_active && (ego_state(4) < 0.2) && (mod(step, 4) == 0)
            needs_replan = true;
        end

        if can_replan && ~needs_replan && ~isempty(potholes) && size(path, 1) >= 2
            for j = 1:length(potholes)
                d_pot = hypot(path(:, 1) - potholes(j).x, path(:, 2) - potholes(j).y) - potholes(j).radius;
                if min(d_pot) < 1.2, needs_replan = true; break; end
            end
        end

        if can_replan && ~needs_replan && ~isempty(predictions) && size(path, 1) >= 2
            vx_ego = ego_state(4) * cos(ego_state(3));
            vy_ego = ego_state(4) * sin(ego_state(3));
            for ii = 1:length(predictions)
                wp = predictions(ii).waypoints;
                if isempty(wp), continue; end
                vx_ag = 0; vy_ag = 0;
                for k_ag = 1:length(raw_obs)
                    if raw_obs(k_ag).id == predictions(ii).id
                        vx_ag = raw_obs(k_ag).velocity(1);
                        vy_ag = raw_obs(k_ag).velocity(2);
                        break;
                    end
                end
                rel_v = hypot(vx_ego - vx_ag, vy_ego - vy_ag);
                trig_d = max(3.0, min(8.0, rel_v * 0.8));
                for h = 1:size(wp, 1)
                    if min(hypot(path(:,1) - wp(h,1), path(:,2) - wp(h,2))) < trig_d
                        needs_replan = true; break;
                    end
                end
                if needs_replan, break; end
            end
        end

        if needs_replan
            last_replan_step = step;
            cur_p = [ego_state(1), ego_state(2), ego_state(3)];
            lg = [min(goal_pose(1), ego_state(1) + 28.0), 0.0, 0.0];
            grid_o = [grid_meta.x_min, grid_meta.y_min];
            [p_new, ~, ~, plan_ok] = adaptive_path_planner(cur_p, lg, rolling_costmap, predictions, map_cfg.grid_res, grid_o);
            if plan_ok && size(p_new, 1) > 2
                path = p_new;
                replan_cnt = replan_cnt + 1;
                safe_stop_active = false;
                stopped_wait_steps = 0;
            elseif ~plan_ok
                safe_stop_active = true;
            end
        end

        [vstop_act, ~] = universal_bottleneck_decider(ego_state', path, rolling_costmap, grid_meta, predictions);
        bsm_p.virtual_stop_active = vstop_act;
        [bsm_state, v_ref, ~] = behavior_state_machine(bsm_state, ego_state, predictions, dt, bsm_p);
        if safe_stop_active, v_ref = 0.0; end

        % Pure pursuit
        if safe_stop_active
            if ego_state(4) > 0.05
                accel = -2.5;
            else
                accel = 0.0; mock_v = 0.0;
            end
            steer_rad = 0.0;
            prev_target = [NaN, NaN];
        elseif size(path, 1) >= 2
            pp_params.L             = L_WB;
            pp_params.k_lookahead   = 0.45;
            pp_params.min_lookahead = 2.2;
            pp_params.Kp_v          = 1.0;
            pp_params.max_steer     = deg2rad(35);
            pp_params.prev_target   = prev_target;
            pp_params.dt            = dt;
            [ctrl_out, ~, ~, target_pt] = pure_pursuit_controller(ego_state', path, v_ref, pp_params);
            steer_rad = ctrl_out(1);
            accel     = ctrl_out(2);
            prev_target = target_pt;
        else
            steer_rad = 0.0; accel = -2.0; prev_target = [NaN, NaN];
        end
        if ~safe_stop_active && v_ref <= 0.0, accel = -3.5; end

        mock_v     = max(0.0, min(8.0, mock_v + accel * dt));
        mock_x     = mock_x + mock_v * cos(mock_theta) * dt;
        mock_y     = mock_y + mock_v * sin(mock_theta) * dt;
        mock_theta = mock_theta + (mock_v / L_WB) * tan(steer_rad) * dt;
        ego_state  = [mock_x; mock_y; mock_theta; mock_v];

        ego_center = [ego_state(1) + 0.5 * L_WB * cos(ego_state(3)), ...
                      ego_state(2) + 0.5 * L_WB * sin(ego_state(3))];

        is_yielding = (ego_state(4) < 1.5) && (safe_stop_active || strcmp(bsm_state, 'YIELD_WAIT') || strcmp(bsm_state, 'YIELD_DECEL'));

        for k = 1:length(raw_obs)
            d_ag = norm(ego_center - raw_obs(k).position);
            
            % Fix #4 deciding factor: d_ag in [0.50, 1.00) while yielding
            if is_yielding && (d_ag < 1.00) && (d_ag >= 0.50)
                deciding_in_this_trial = true;
                if d_ag < min_agent_dist_yield
                    min_agent_dist_yield = d_ag;

                    dx = raw_obs(k).position(1) - ego_center(1);
                    dy = raw_obs(k).position(2) - ego_center(2);
                    th = ego_state(3);
                    x_body =  dx * cos(th) + dy * sin(th);
                    y_body = -dx * sin(th) + dy * cos(th);

                    % Footprint overlap
                    in_fp = (abs(x_body) <= ego_length / 2) && (abs(y_body) <= ego_width / 2);

                    snap = struct( ...
                        'trial_id', cand.trial_id(idx), ...
                        'seed', seed, ...
                        't', t, ...
                        'ego_pos', ego_state(1:2)', ...
                        'ego_center', ego_center, ...
                        'ego_v', ego_state(4), ...
                        'ego_theta', ego_state(3), ...
                        'bsm_state', bsm_state, ...
                        'agent_id', raw_obs(k).id, ...
                        'agent_type', raw_obs(k).type, ...
                        'agent_pos', raw_obs(k).position, ...
                        'agent_v', raw_obs(k).velocity, ...
                        'distance', d_ag, ...
                        'rel_body_pos', [x_body, y_body], ...
                        'in_footprint', in_fp ...
                    );
                end
            end
        end

        if norm(ego_state(1:2) - goal_pose(1:2)') < 2.5
            break;
        end
    end

    if deciding_in_this_trial && ~isempty(snap)
        n_deciding = n_deciding + 1;
        deciding_trials(n_deciding).data = snap;
        if snap.ego_v < 0.1
            n_stopped = n_stopped + 1;
        end
        if snap.in_footprint
            n_footprint_overlap = n_footprint_overlap + 1;
        end

        % Print relative trajectory for audited trial
        fprintf('-------------------------------------------------------------------------\n');
        fprintf('[AUDIT] Trial #%d (Seed: %d) — Fix #4 Deciding Factor Encounter\n', snap.trial_id, snap.seed);
        fprintf('  Encounter Time      : t = %.1f s\n', snap.t);
        fprintf('  Ego Pose            : x = %6.2f m, y = %5.2f m, theta = %+5.2f rad\n', snap.ego_pos(1), snap.ego_pos(2), snap.ego_theta);
        if snap.ego_v < 0.1
            stopped_str = 'CONFIRMED FULLY STOPPED';
        else
            stopped_str = 'SLOW DECELERATING';
        end
        fprintf('  Ego Stopped Check   : %s  (v = %.3f m/s)\n', stopped_str, snap.ego_v);
        fprintf('  Dynamic Agent       : #%d (%s)\n', snap.agent_id, snap.agent_type);
        fprintf('  Agent Position      : x = %6.2f m, y = %5.2f m\n', snap.agent_pos(1), snap.agent_pos(2));
        fprintf('  Agent Velocity      : vx = %+5.2f m/s, vy = %+5.2f m/s\n', snap.agent_v(1), snap.agent_v(2));
        fprintf('  Min Center Distance : %.3f m  (in [0.50m, 1.00m) threshold window)\n', snap.distance);
        fprintf('  Body-Frame Offset   : dx_body = %+5.2f m, dy_body = %+5.2f m\n', snap.rel_body_pos(1), snap.rel_body_pos(2));
        fprintf('  Physical Footprint  : Half-L = 2.25m, Half-W = 0.925m\n');
        if snap.in_footprint
            fprintf('  Overlap Assessment  : [WARNING] FOOTPRINT OVERLAP (Genuine Collision Reclassified)\n');
        else
            fprintf('  Overlap Assessment  : [CONFIRMED] NO FOOTPRINT OVERLAP (Legitimate False-Positive Corrected)\n');
        end
    end
end

fprintf('\n=========================================================================\n');
fprintf('  FIX #4 AUDIT SUMMARY (1,000 Unseen Trials)\n');
fprintf('=========================================================================\n');
fprintf('  Total Deciding-Factor Trials : %d / 1000  (%.2f%% of all trials)\n', n_deciding, (n_deciding / 1000) * 100);
fprintf('  Ego Confirmed Stopped (v<0.1): %d / %d   (%.1f%%)\n', n_stopped, max(1, n_deciding), (n_stopped / max(1, n_deciding)) * 100);
fprintf('  No Footprint Overlap (Safe)  : %d / %d   (%.1f%%)\n', n_deciding - n_footprint_overlap, max(1, n_deciding), ((n_deciding - n_footprint_overlap) / max(1, n_deciding)) * 100);
fprintf('  Footprint Overlap (Flagged)  : %d / %d   (%.1f%%)\n', n_footprint_overlap, max(1, n_deciding), (n_footprint_overlap / max(1, n_deciding)) * 100);
fprintf('=========================================================================\n');

audit_results.total_deciding      = n_deciding;
audit_results.total_stopped       = n_stopped;
audit_results.total_footprint_ovl = n_footprint_overlap;
audit_results.trials              = deciding_trials;

end
