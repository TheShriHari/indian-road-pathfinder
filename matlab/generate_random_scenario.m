function scenario = generate_random_scenario(seed, domain_override)
%% GENERATE_RANDOM_SCENARIO  Generates randomized Indian road scenarios across 5 domains.
%
% Syntax:
%   scenario = generate_random_scenario(trial_id)
%   scenario = generate_random_scenario(seed, domain_id)
%
% 5 Domains (200 seeds each = 1,000 trials total):
%   1. Unmarked Rural Road (trials 1-200)
%   2. Signal-less Urban Intersection (trials 201-400)
%   3. Highway Merge with Slow Vehicles (trials 401-600)
%   4. Dense Market Squeeze (trials 601-800)
%   5. Sudden Cattle Crossing (trials 801-1000)

if nargin < 1 || isempty(seed)
    seed = 1;
end

% Determine domain (1..5) and internal seed
if nargin >= 2 && ~isempty(domain_override)
    domain_id = domain_override;
    internal_seed = seed;
else
    if seed <= 200
        domain_id = 1;
        internal_seed = seed;
    elseif seed <= 400
        domain_id = 2;
        internal_seed = seed - 200;
    elseif seed <= 600
        domain_id = 3;
        internal_seed = seed - 400;
    elseif seed <= 800
        domain_id = 4;
        internal_seed = seed - 600;
    else
        domain_id = 5;
        internal_seed = mod(seed - 801, 200) + 1;
    end
end

% Seed RNG for strict determinism
rng(seed);

scenario.seed      = seed;
scenario.domain_id = domain_id;
scenario.start_pose = [2.0, 0.0, 0.0];
scenario.goal_pose  = [55.0, 0.0, 0.0];

% Domain dispatch
switch domain_id
    case 1
        %% ── Domain 1: Unmarked Rural Road (Seeds 1–200) ───────────────────
        scenario.domain_name = 'Unmarked Rural Road';
        % Eroded width W in U(3.2, 4.5) m
        W = 3.2 + rand() * (4.5 - 3.2);
        scenario.road_width = round(W, 2);
        
        % Road boundaries
        xs = (-10:70)';
        half_w = W / 2.0;
        scenario.road_boundaries = [xs, repmat(-half_w, length(xs), 1); ...
                                    xs, repmat( half_w, length(xs), 1)];
        
        % 2 to 4 potholes, radius R in U(0.4, 0.9) m
        n_potholes = randi([2, 4]);
        potholes = struct('x', {}, 'y', {}, 'radius', {});
        for j = 1:n_potholes
            px = 12.0 + rand() * 38.0; % x in [12, 50]
            py = (-half_w + 0.6) + rand() * (W - 1.2);
            pr = 0.40 + rand() * 0.50;
            potholes(j).x      = round(px, 2);
            potholes(j).y      = round(py, 2);
            potholes(j).radius = round(pr, 2);
        end
        scenario.potholes = potholes;
        
        % Oncoming tractor: v in U(-6.0, -3.0) m/s
        v_trac = -(3.0 + rand() * 3.0);
        x_trac = 40.0 + rand() * 15.0;
        y_trac = 0.5 + rand() * (half_w - 0.7);
        scenario.dynamic_agents = [struct( ...
            'id', 1, ...
            'type', 'tractor', ...
            'position', [round(x_trac, 2), round(y_trac, 2)], ...
            'velocity', [round(v_trac, 2), round(-0.1 + rand() * 0.2, 2)], ...
            'accel', [0.0, 0.0], ...
            'behavior_profile', 'steady' ...
        )];

    case 2
        %% ── Domain 2: Signal-less Urban Intersection (Seeds 201–400) ──────
        scenario.domain_name = 'Signal-less Urban Intersection';
        W = 7.5;
        scenario.road_width = W;
        xs = (-10:70)';
        half_w = W / 2.0;
        scenario.road_boundaries = [xs, repmat(-half_w, length(xs), 1); ...
                                    xs, repmat( half_w, length(xs), 1)];
        scenario.potholes = struct([]);
        
        % 2 crossing auto-rickshaws (v in U(4.0, 8.0) m/s) + 1 jaywalking pedestrian (v in U(0.8, 1.4) m/s)
        v_auto1 = -(4.0 + rand() * 4.0);
        x_auto1 = 28.0 + rand() * 10.0;
        y_auto1 = 6.0 + rand() * 3.0;
        
        v_auto2 = -(3.0 + rand() * 2.5);
        x_auto2 = 38.0 + rand() * 12.0;
        y_auto2 = -1.5 + rand() * 1.0;
        
        v_ped = 0.8 + rand() * 0.6;
        x_ped = 24.0 + rand() * 8.0;
        y_ped = -(half_w + 0.5);
        
        scenario.dynamic_agents = [ ...
            struct('id', 1, 'type', 'auto_rickshaw', ...
                   'position', [round(x_auto1, 2), round(y_auto1, 2)], ...
                   'velocity', [round(0.2 - rand()*0.4, 2), round(v_auto1, 2)], ...
                   'accel', [0.0, 0.0], ...
                   'behavior_profile', 'weaving'), ...
            struct('id', 2, 'type', 'auto_rickshaw', ...
                   'position', [round(x_auto2, 2), round(y_auto2, 2)], ...
                   'velocity', [round(v_auto2, 2), round(0.1 - rand()*0.2, 2)], ...
                   'accel', [0.0, 0.0], ...
                   'behavior_profile', 'weaving'), ...
            struct('id', 3, 'type', 'pedestrian', ...
                   'position', [round(x_ped, 2), round(y_ped, 2)], ...
                   'velocity', [round(-0.1 + rand()*0.2, 2), round(v_ped, 2)], ...
                   'accel', [0.0, 0.0], ...
                   'behavior_profile', 'steady') ...
        ];

    case 3
        %% ── Domain 3: Highway Merge with Slow Vehicles (Seeds 401–600) ───
        scenario.domain_name = 'Highway Merge';
        W = 9.0;
        scenario.road_width = W;
        xs = (-10:70)';
        half_w = W / 2.0;
        scenario.road_boundaries = [xs, repmat(-half_w, length(xs), 1); ...
                                    xs, repmat( half_w, length(xs), 1)];
        scenario.potholes = struct([]);
        
        % High-speed lead vehicle v in U(16.0, 22.0) m/s
        v_lead = 16.0 + rand() * 6.0;
        x_lead = 32.0 + rand() * 12.0;
        y_lead = 1.5 + rand() * 1.5;
        
        % Merging truck a in U(-1.0, 0.5) m/s^2, initial v in U(3.0, 6.0) m/s
        v_truck = 3.0 + rand() * 3.0;
        a_truck = -1.0 + rand() * 1.5;
        x_truck = 20.0 + rand() * 12.0;
        y_truck = -3.5 + rand() * 0.8;
        
        scenario.dynamic_agents = [ ...
            struct('id', 1, 'type', 'lead_car', ...
                   'position', [round(x_lead, 2), round(y_lead, 2)], ...
                   'velocity', [round(v_lead, 2), 0.0], ...
                   'accel', [0.0, 0.0], ...
                   'behavior_profile', 'steady'), ...
            struct('id', 2, 'type', 'merging_truck', ...
                   'position', [round(x_truck, 2), round(y_truck, 2)], ...
                   'velocity', [round(v_truck, 2), 0.3], ...
                   'accel', [round(a_truck, 2), 0.0], ...
                   'behavior_profile', 'slow') ...
        ];

    case 4
        %% ── Domain 4: Dense Market Squeeze (Seeds 601–800) ────────────────
        scenario.domain_name = 'Dense Market Squeeze';
        % Lateral road clearance W in U(2.4, 3.2) m
        W = 2.4 + rand() * (3.2 - 2.4);
        scenario.road_width = round(W, 2);
        xs = (-10:70)';
        half_w = W / 2.0;
        scenario.road_boundaries = [xs, repmat(-half_w, length(xs), 1); ...
                                    xs, repmat( half_w, length(xs), 1)];
        scenario.potholes = struct([]);
        
        % 3 pushcarts: v in U(0.5, 1.2) m/s
        % 4 pedestrians with random cross angles
        agents = struct('id', {}, 'type', {}, 'position', {}, 'velocity', {}, 'accel', {}, 'behavior_profile', {});
        
        % Pushcarts along shoulder
        for pc = 1:3
            x_pc = 15.0 + (pc - 1) * 12.0 + rand() * 5.0;
            side_pc = (rand() > 0.5) * 2 - 1;
            y_pc = side_pc * (half_w - 0.4);
            v_pc = (0.5 + rand() * 0.7) * ((rand() > 0.6) * 2 - 1);
            agents(pc).id = pc;
            agents(pc).type = 'pushcart';
            agents(pc).position = [round(x_pc, 2), round(y_pc, 2)];
            agents(pc).velocity = [round(v_pc, 2), 0.0];
            agents(pc).accel = [0.0, 0.0];
            agents(pc).behavior_profile = 'slow';
        end
        
        % 4 Pedestrians
        for pd = 1:4
            idx = 3 + pd;
            x_pd = 18.0 + (pd - 1) * 9.0 + rand() * 4.0;
            y_pd = -half_w + rand() * W;
            speed_pd = 0.6 + rand() * 0.7;
            ang_pd = -pi/4 + rand() * (pi/2);
            agents(idx).id = idx;
            agents(idx).type = 'pedestrian';
            agents(idx).position = [round(x_pd, 2), round(y_pd, 2)];
            agents(idx).velocity = [round(speed_pd * cos(ang_pd), 2), round(speed_pd * sin(ang_pd), 2)];
            agents(idx).accel = [0.0, 0.0];
            agents(idx).behavior_profile = 'steady';
        end
        scenario.dynamic_agents = agents;

    case 5
        %% ── Domain 5: Sudden Cattle Crossing (Seeds 801–1000) ─────────────
        scenario.domain_name = 'Sudden Cattle Crossing';
        W = 5.5;
        scenario.road_width = W;
        xs = (-10:70)';
        half_w = W / 2.0;
        scenario.road_boundaries = [xs, repmat(-half_w, length(xs), 1); ...
                                    xs, repmat( half_w, length(xs), 1)];
        scenario.potholes = struct([]);
        
        % Cattle entering corridor at d in U(8.0, 16.0) m ahead of ego
        d_ahead = 8.0 + rand() * 8.0;
        side_c = (rand() > 0.5) * 2 - 1;
        x_c = 2.0 + d_ahead;
        y_c = side_c * (half_w + 0.8);
        vy_c = -side_c * (0.8 + rand() * 1.0); % lateral speed in U(0.8, 1.8) m/s
        vx_c = -0.2 + rand() * 0.4;
        
        scenario.dynamic_agents = [struct( ...
            'id', 1, ...
            'type', 'cattle', ...
            'position', [round(x_c, 2), round(y_c, 2)], ...
            'velocity', [round(vx_c, 2), round(vy_c, 2)], ...
            'accel', [0.0, 0.0], ...
            'behavior_profile', 'erratic' ...
        )];
end

end
