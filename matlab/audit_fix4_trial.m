function audit_info = audit_fix4_trial(seed)
%% AUDIT_FIX4_TRIAL  Inspects whether Fix #4 was the deciding factor for a given seed.
% Re-runs the scenario and monitors step-by-step clearance to dynamic agents.
% Checks whether clearance was in [0.50, 1.00) while ego was yielding/stationary.
% Computes ego body-frame relative position and checks physical footprint overlap.

scenario = generate_random_scenario(seed);
clear dynamic_obstacle_predictor simulate_sensor_detection;

% Run single scenario in silent mode
sim_opts.max_steps = 400;
sim_opts.verbose   = false;
res = run_single_scenario(scenario, seed, sim_opts);

audit_info.seed                   = seed;
audit_info.outcome                = res.outcome;
audit_info.min_clearance_achieved = res.min_clearance_achieved;
audit_info.deciding_factor        = false;
audit_info.min_agent_dist         = Inf;
audit_info.step_data              = struct([]);

% If trial did not succeed, Fix #4 was not the deciding factor for a SUCCESS
if ~strcmp(res.outcome, 'SUCCESS')
    return;
end

% Scenario replay to find minimum dynamic clearance
clear dynamic_obstacle_predictor simulate_sensor_detection;

dt = 0.1;
max_steps = 400;
L_WB = 2.8;
ego_length = 4.5;
ego_width = 1.85;

ego_state = [scenario.start_pose(1); scenario.start_pose(2); scenario.start_pose(3); 0.0];
goal_pose = scenario.goal_pose;
potholes = scenario.potholes;
dynamic_agents = scenario.dynamic_agents;

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

sensor_det.road_boundaries = scenario.road_boundaries;
sensor_det.potholes        = potholes;
sensor_det.static_boxes    = struct([]);
sensor_det.static_points   = [];

bsm_state = 'CRUISE';
safe_stop_active = false;
path = [ego_state(1:2)'; ego_state(1:2)' + [5, 0]];
prev_target = [NaN, NaN];
mock_x = ego_state(1); mock_y = ego_state(2); mock_theta = ego_state(3); mock_v = 0.0;

min_d_yield = Inf;
critical_snap = struct([]);

for step = 1:max_steps
    t = step * dt;
    
    % Update agents
    raw_obs = struct([]);
    for k = 1:length(dynamic_agents)
        pos = dynamic_agents(k).position + dynamic_agents(k).velocity * t;
        raw_obs(k).id               = dynamic_agents(k).id;
        raw_obs(k).type             = dynamic_agents(k).type;
        raw_obs(k).position         = pos;
        raw_obs(k).velocity         = dynamic_agents(k).velocity;
        raw_obs(k).behavior_profile = dynamic_agents(k).behavior_profile;
    end
    
    % Costmap & perception
    [rolling_costmap, grid_meta] = local_occupancy_grid_builder(ego_state(1:3)', sensor_det, map_cfg);
    [detected_obs, ~] = simulate_sensor_detection(raw_obs, ego_state, sensor_cfg);
    [predictions, ~] = dynamic_obstacle_predictor(detected_obs, dt, 35);
    
    % Check clearance to agents
    ego_center = [ego_state(1) + 0.5 * L_WB * cos(ego_state(3)), ...
                  ego_state(2) + 0.5 * L_WB * sin(ego_state(3))];
              
    is_yielding = (ego_state(4) < 1.5) && (safe_stop_active || strcmp(bsm_state, 'YIELD_WAIT') || strcmp(bsm_state, 'YIELD_DECEL'));
    
    for k = 1:length(raw_obs)
        d_ag = norm(ego_center - raw_obs(k).position);
        if is_yielding && d_ag < min_d_yield
            min_d_yield = d_ag;
            
            % Body-frame relative coordinates
            dx = raw_obs(k).position(1) - ego_center(1);
            dy = raw_obs(k).position(2) - ego_center(2);
            th = ego_state(3);
            x_body =  dx * cos(th) + dy * sin(th);
            y_body = -dx * sin(th) + dy * cos(th);
            
            % Check physical bounding box overlap
            in_footprint = (abs(x_body) <= ego_length / 2) && (abs(y_body) <= ego_width / 2);
            
            critical_snap = struct( ...
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
                'in_footprint', in_footprint ...
            );
        end
    end
    
    % Replan check
    needs_replan = (mod(step, 5) == 0) || size(path, 1) < 2;
    if needs_replan
        lg = [min(goal_pose(1), ego_state(1) + 28.0), 0.0, 0.0];
        grid_org = [grid_meta.x_min, grid_meta.y_min];
        [p_out, ~, ~, plan_ok] = adaptive_path_planner( ...
            ego_state(1:3)', lg, rolling_costmap, predictions, ...
            map_cfg.grid_res, grid_org);
        if plan_ok
            path = p_out;
        end
    end
    
    % UBD & BSM
    [vstop_active, ~] = universal_bottleneck_decider(ego_state', path, rolling_costmap, grid_meta, predictions);
    bsm_params.virtual_stop_active = vstop_active;
    [bsm_state, v_ref, ~] = behavior_state_machine(bsm_state, ego_state, predictions, dt, bsm_params);
    if safe_stop_active, v_ref = 0.0; end
    
    % Controller & Integration
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
    if ~safe_stop_active && v_ref <= 0.0, accel = -3.5; end
    
    mock_v     = max(0.0, min(8.0, mock_v + accel * dt));
    mock_x     = mock_x + mock_v * cos(mock_theta) * dt;
    mock_y     = mock_y + mock_v * sin(mock_theta) * dt;
    mock_theta = mock_theta + (mock_v / L_WB) * tan(steer_rad) * dt;
    ego_state  = [mock_x; mock_y; mock_theta; mock_v];
    
    if norm(ego_state(1:2) - goal_pose(1:2)') < 2.5
        break;
    end
end

audit_info.min_agent_dist = min_d_yield;
if min_d_yield < 1.0 && min_d_yield >= 0.50
    audit_info.deciding_factor = true;
    audit_info.step_data       = critical_snap;
end

end
