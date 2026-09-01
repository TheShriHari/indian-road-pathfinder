function scenario = generate_scenarios(scenario_id)
% GENERATE_SCENARIOS Generates 5 realistic Indian Road Test Scenarios
%   scenario = generate_scenarios(scenario_id)
%
% Scenarios:
%   1: Unmarked Village Road (narrow, potholes, dynamic cattle crossing)
%   2: Signal-less Urban Intersection (auto-rickshaws, pedestrians, multi-directional flow)
%   3: Highway Merge with Slow Vehicles (pushcarts, slow tractor, speed differential)
%   4: Dense Market Area (crowded pedestrians, auto-rickshaws, narrow clearance)
%   5: Sudden Cattle Crossing Event (unpredictable fast lateral movement)

if nargin < 1, scenario_id = 1; end

scenario.id = scenario_id;
scenario.grid_res = 0.2; % 20 cm per cell
scenario.bounds = [0, 60, -10, 10]; % [Xmin Xmax Ymin Ymax] meters

switch scenario_id
    case 1 % Unmarked Village Road
        scenario.title = 'Unmarked Village Road';
        scenario.description = 'Narrow paved road with missing boundaries, potholes, and roaming cattle.';
        scenario.start_pose = [2, 0, 0];
        scenario.goal_pose = [55, 0, 0];
        scenario.road_width = 5.0; % meters
        
        % Static Map: 300x100 grid (60m x 20m)
        scenario.static_map = zeros(100, 300);
        % Road edges (unmarked boundaries)
        scenario.static_map(1:35, :) = 1; % Left off-road
        scenario.static_map(65:100, :) = 1; % Right off-road
        % Potholes (circle obstacles)
        scenario = add_pothole(scenario, 20, 1.0, 0.8);
        scenario = add_pothole(scenario, 35, -0.8, 1.0);
        
        % Dynamic Obstacles (Cattle & Auto)
        scenario.obstacles = [
            struct('id', 1, 'type', 'cattle', 'position', [30, -3.5], 'velocity', [0.2, 1.2]), ...
            struct('id', 2, 'type', 'auto_rickshaw', 'position', [45, 1.2], 'velocity', [-2.5, -0.1])
        ];

    case 2 % Signal-less Urban Intersection
        scenario.title = 'Signal-less Urban Intersection';
        scenario.description = 'Unregulated 4-way junction with auto-rickshaws and pedestrians.';
        scenario.start_pose = [2, -1.5, 0];
        scenario.goal_pose = [55, -1.5, 0];
        scenario.road_width = 8.0;
        
        scenario.static_map = zeros(100, 300);
        % Corner buildings / sidewalk bounds
        scenario.static_map(1:30, 1:120) = 1;
        scenario.static_map(70:100, 1:120) = 1;
        scenario.static_map(1:30, 180:300) = 1;
        scenario.static_map(70:100, 180:300) = 1;
        
        scenario.obstacles = [
            struct('id', 1, 'type', 'auto_rickshaw', 'position', [30, 8.0], 'velocity', [0.0, -3.0]), ...
            struct('id', 2, 'type', 'pedestrian', 'position', [28, -6.0], 'velocity', [0.3, 1.0]), ...
            struct('id', 3, 'type', 'auto_rickshaw', 'position', [42, -1.5], 'velocity', [-3.5, 0.2])
        ];

    case 3 % Highway Merge with Slow Vehicles
        scenario.title = 'Highway Merge with Slow Vehicles';
        scenario.description = 'Merging onto high-speed road obstructed by pushcarts and tractors.';
        scenario.start_pose = [2, -4.0, 0.1];
        scenario.goal_pose = [55, 2.0, 0];
        scenario.road_width = 10.0;
        
        scenario.static_map = zeros(100, 300);
        scenario.static_map(1:20, :) = 1;
        scenario.static_map(80:100, :) = 1;
        
        scenario.obstacles = [
            struct('id', 1, 'type', 'pushcart', 'position', [25, -3.5], 'velocity', [0.8, 0.1]), ...
            struct('id', 2, 'type', 'auto_rickshaw', 'position', [40, 2.0], 'velocity', [4.0, 0.0])
        ];

    case 4 % Dense Market Area
        scenario.title = 'Dense Market Area';
        scenario.description = 'Extremely constrained space with close-proximity pedestrians and vendors.';
        scenario.start_pose = [2, 0, 0];
        scenario.goal_pose = [55, 0, 0];
        scenario.road_width = 4.5;
        
        scenario.static_map = zeros(100, 300);
        scenario.static_map(1:38, :) = 1;
        scenario.static_map(62:100, :) = 1;
        
        scenario.obstacles = [
            struct('id', 1, 'type', 'pedestrian', 'position', [15, 1.0], 'velocity', [0.2, -0.4]), ...
            struct('id', 2, 'type', 'pedestrian', 'position', [22, -1.2], 'velocity', [0.1, 0.5]), ...
            struct('id', 3, 'type', 'pushcart', 'position', [32, 0.8], 'velocity', [0.3, -0.1]), ...
            struct('id', 4, 'type', 'pedestrian', 'position', [42, -0.5], 'velocity', [-0.3, 0.3])
        ];

    case 5 % Sudden Cattle Crossing Event
        scenario.title = 'Sudden Cattle Crossing Event';
        scenario.description = 'Vehicle driving at speed when cattle suddenly bolts into path from roadside foliage.';
        scenario.start_pose = [2, 0, 0];
        scenario.goal_pose = [55, 0, 0];
        scenario.road_width = 6.0;
        
        scenario.static_map = zeros(100, 300);
        scenario.static_map(1:32, :) = 1;
        scenario.static_map(68:100, :) = 1;
        
        scenario.obstacles = [
            struct('id', 1, 'type', 'cattle', 'position', [28, -4.5], 'velocity', [0.5, 2.8]) % High lateral speed
        ];
end
end

function sc = add_pothole(sc, center_x, center_y, radius)
    gx = round(center_x / sc.grid_res);
    gy = round((center_y + 10) / sc.grid_res);
    r_cells = ceil(radius / sc.grid_res);
    
    [r_rows, r_cols] = size(sc.static_map);
    for x = max(1, gx-r_cells):min(r_cols, gx+r_cells)
        for y = max(1, gy-r_cells):min(r_rows, gy+r_cells)
            if hypot((x-gx)*sc.grid_res, (y-gy)*sc.grid_res) <= radius
                sc.static_map(y, x) = 1;
            end
        end
    end
end
