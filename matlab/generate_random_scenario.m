function scenario = generate_random_scenario(seed)
%% GENERATE_RANDOM_SCENARIO  Generates a randomized Indian road scenario with seed reproducibility.
%
% Syntax:
%   scenario = generate_random_scenario(seed)
%
% Randomizes:
%   - Number of potholes (0-4), position x in [10, 60], y in [-2.3, 2.3], radius in [0.5, 1.2]
%   - Number of dynamic agents (0-3), typed (cattle/auto_rickshaw/pedestrian),
%     realistic starting position and velocities appropriate for each type
%   - Road boundaries: standard 5m corridor (y = [-2.5, +2.5]) from x = -5 to 120m
%   - Start pose: [2.0, 0.0, 0.0], Goal pose: [70.0, 0.0, 0.0]

if nargin >= 1 && ~isempty(seed)
    rng(seed);
end

% 1. Potholes (0 to 4)
n_potholes = randi([0, 4]);
potholes = struct('x', {}, 'y', {}, 'radius', {});
for j = 1:n_potholes
    px = 10.0 + rand() * 50.0;     % x in [10, 60]
    py = -2.3 + rand() * 4.6;      % y in [-2.3, 2.3]
    pr =  0.5 + rand() * 0.7;      % radius in [0.5, 1.2]
    potholes(j).x      = round(px, 2);
    potholes(j).y      = round(py, 2);
    potholes(j).radius = round(pr, 2);
end

% 2. Dynamic Agents (0 to 3)
n_agents = randi([0, 3]);
agents = struct('id', {}, 'type', {}, 'position', {}, 'velocity', {}, 'behavior_profile', {});
types = {'cattle', 'auto_rickshaw', 'pedestrian'};

for k = 1:n_agents
    atype = types{randi(length(types))};
    switch atype
        case 'cattle'
            % Cattle: wanders from shoulder/verge across the road
            side = (rand() > 0.5) * 2 - 1; % +1 or -1
            x0 = 15.0 + rand() * 40.0;
            y0 = side * (2.6 + rand() * 0.9);
            vy = -side * (0.4 + rand() * 0.6); % 0.4 to 1.0 m/s lateral
            vx = -0.2 + rand() * 0.4;          % slow wander longitudinal
            profile = 'erratic';

        case 'auto_rickshaw'
            % Auto-rickshaw: 75% oncoming in opposite lane, 25% slower leading vehicle
            if rand() < 0.75
                x0 = 35.0 + rand() * 30.0;     % [35, 65]
                y0 = 0.5 + rand() * 1.3;       % Opposing lane [0.5, 1.8]
                vx = -(2.5 + rand() * 1.5);    % -2.5 to -4.0 m/s oncoming
                vy = -0.3 + rand() * 0.6;      % slight lateral weave
            else
                x0 = 15.0 + rand() * 20.0;     % Ahead of ego
                y0 = -1.5 + rand() * 1.2;      % In-lane
                vx = 1.8 + rand() * 1.2;       % 1.8 to 3.0 m/s leading
                vy = -0.2 + rand() * 0.4;
            end
            profile = 'weaving';

        case 'pedestrian'
            % Pedestrian: crossing the road from verge at walking speed (0.7-1.4 m/s)
            side = (rand() > 0.5) * 2 - 1;
            x0 = 15.0 + rand() * 40.0;
            y0 = side * (2.2 + rand() * 0.8);
            vy = -side * (0.7 + rand() * 0.6); % 0.7 to 1.3 m/s
            vx = -0.15 + rand() * 0.3;
            profile = 'steady';
    end

    agents(k).id               = k;
    agents(k).type             = atype;
    agents(k).position         = [round(x0, 2), round(y0, 2)];
    agents(k).velocity         = [round(vx, 2), round(vy, 2)];
    agents(k).behavior_profile = profile;
end

% 3. Assemble scenario struct
scenario.seed           = seed;
scenario.potholes       = potholes;
scenario.dynamic_agents = agents;
scenario.start_pose     = [2.0, 0.0, 0.0];
scenario.goal_pose      = [70.0, 0.0, 0.0];

% Standard road boundaries: y = -2.5 and +2.5 from x = -5 to 120m
xs = (-5:120)';
scenario.road_boundaries = [xs, repmat(-2.5, length(xs), 1); xs, repmat( 2.5, length(xs), 1)];
end
