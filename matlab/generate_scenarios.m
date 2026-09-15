function scenario = generate_scenarios(scenario_id)
% GENERATE_SCENARIOS  Produces the 5 officially required SIH PS-26037 test scenarios.
%   scenario = generate_scenarios(scenario_id)   scenario_id in {1..5}
%
% No toolbox dependencies — runs on MATLAB Online base install.
%
% Output struct fields:
%   .id               - scenario number (1-5)
%   .title            - human-readable name
%   .description      - brief description
%   .start_pose       - [x, y, theta_rad]  ego vehicle start
%   .goal_pose        - [x, y, theta_rad]  ego vehicle goal
%   .road_width       - usable road width (m)
%   .grid_res         - costmap cell size (m)   = 0.2 m
%   .bounds           - [Xmin Xmax Ymin Ymax]  world bounds (m)
%   .static_map       - binary occupancy grid (rows=Y, cols=X); 1 = obstacle
%   .obstacles        - struct array, each element:
%                         .id               unique integer
%                         .type             string: 'cattle','auto_rickshaw','pedestrian','pushcart'
%                         .position         [x, y]   initial position (m)
%                         .velocity         [vx, vy] initial velocity (m/s)
%                         .behavior_profile string: 'erratic','weaving','steady','slow'

if nargin < 1, scenario_id = 1; end

scenario.id       = scenario_id;
scenario.grid_res = 0.2;              % 20 cm per cell
scenario.bounds   = [0, 60, -10, 10]; % [Xmin Xmax Ymin Ymax] metres

% Grid dimensions derived from bounds + resolution
nX = round((scenario.bounds(2) - scenario.bounds(1)) / scenario.grid_res); % 300
nY = round((scenario.bounds(4) - scenario.bounds(3)) / scenario.grid_res); % 100

switch scenario_id

    % ------------------------------------------------------------------
    case 1  %  Unmarked Village Road
    % ------------------------------------------------------------------
    scenario.title       = 'Unmarked Village Road';
    scenario.description = ['Narrow paved road: missing lane boundaries, potholes, ' ...
                            'roaming cattle. Speed limit ~20 km/h.'];
    scenario.start_pose  = [2.0,   0.0, 0.0];
    scenario.goal_pose   = [55.0,  0.0, 0.0];
    scenario.road_width  = 5.0;

    m = zeros(nY, nX);
    % Usable pavement width = 5.0m: y in [-2.5, +2.5]
    % World Y: [-10, 10], res=0.2m -> row = (y - (-10))/0.2 + 1
    % y <= -2.5m -> row <= 38; y >= +2.5m -> row >= 63
    m(1:37, :)    = 1;   % off-road shoulder (y < -2.5 m)
    m(64:nY, :)   = 1;   % off-road shoulder (y > +2.5 m)
    m = add_circle(m, scenario.grid_res, scenario.bounds, 20.0,  1.0, 0.8);  % Pothole 1 (Left Lane)
    m = add_circle(m, scenario.grid_res, scenario.bounds, 35.0, -0.8, 1.0);  % Pothole 2 (Right Lane)
    scenario.static_map = m;

    % Dynamic Agents in Spatio-Temporal Conflict Zone:
    % - Cattle: crossing corridor at X=30m, v_cattle = +0.7 m/s lateral
    % - Auto-Rickshaw: oncoming in opposing lane at X=45m, v_auto = -3.2 m/s longitudinal
    scenario.obstacles = make_obs( ...
        {1, 'cattle',       [30.0, -3.5], [ 0.0,  0.7], 'erratic'}, ...
        {2, 'auto_rickshaw',[45.0,  1.2], [-3.2,  0.0], 'weaving'});

    % ------------------------------------------------------------------
    case 2  %  Signal-less Urban Intersection
    % ------------------------------------------------------------------
    scenario.title       = 'Signal-less Urban Intersection';
    scenario.description = ['Unregulated 4-way junction: auto-rickshaws and pedestrians ' ...
                            'crossing from multiple directions without signals.'];
    scenario.start_pose  = [2.0,  -1.5, 0.0];
    scenario.goal_pose   = [55.0, -1.5, 0.0];
    scenario.road_width  = 8.0;

    m = zeros(nY, nX);
    % Corner buildings / sidewalk blocks (leave central junction open)
    m(1:30,  1:120)  = 1;
    m(71:nY, 1:120)  = 1;
    m(1:30,  181:nX) = 1;
    m(71:nY, 181:nX) = 1;
    scenario.static_map = m;

    scenario.obstacles = make_obs( ...
        {1, 'auto_rickshaw',[30.0,  8.0], [ 0.0,-3.0], 'weaving'}, ...
        {2, 'pedestrian',   [28.0, -6.0], [ 0.3, 1.0], 'steady'}, ...
        {3, 'auto_rickshaw',[42.0, -1.5], [-3.5, 0.2], 'weaving'});

    % ------------------------------------------------------------------
    case 3  %  Highway Merge with Slow Vehicles
    % ------------------------------------------------------------------
    scenario.title       = 'Highway Merge with Slow Vehicles';
    scenario.description = ['Ego vehicle merges onto higher-speed road blocked by pushcarts ' ...
                            'and a slow auto-rickshaw. High speed differential.'];
    scenario.start_pose  = [2.0,  -4.0, 0.1];
    scenario.goal_pose   = [55.0,  2.0, 0.0];
    scenario.road_width  = 10.0;

    m = zeros(nY, nX);
    m(1:20,  :) = 1;   % highway boundary top
    m(81:nY, :) = 1;   % highway boundary bottom
    scenario.static_map = m;

    scenario.obstacles = make_obs( ...
        {1, 'pushcart',     [25.0, -3.5], [0.8, 0.1], 'slow'}, ...
        {2, 'auto_rickshaw',[40.0,  2.0], [4.0, 0.0], 'weaving'});

    % ------------------------------------------------------------------
    case 4  %  Dense Market Area
    % ------------------------------------------------------------------
    scenario.title       = 'Dense Market Area';
    scenario.description = ['Extremely narrow street flanked by market stalls. ' ...
                            'High pedestrian density, sub-metre clearance margins.'];
    scenario.start_pose  = [2.0,  0.0, 0.0];
    scenario.goal_pose   = [55.0, 0.0, 0.0];
    scenario.road_width  = 4.5;

    m = zeros(nY, nX);
    m(1:38,  :) = 1;   % market stalls / wall left  (y < -2.4 m)
    m(63:nY, :) = 1;   % market stalls / wall right (y >  2.4 m)
    scenario.static_map = m;

    scenario.obstacles = make_obs( ...
        {1, 'pedestrian',[15.0,  1.0], [ 0.2,-0.4], 'steady'}, ...
        {2, 'pedestrian',[22.0, -1.2], [ 0.1, 0.5], 'steady'}, ...
        {3, 'pushcart',  [32.0,  0.8], [ 0.3,-0.1], 'slow'}, ...
        {4, 'pedestrian',[42.0, -0.5], [-0.3, 0.3], 'steady'});

    % ------------------------------------------------------------------
    case 5  %  Sudden Cattle Crossing Event
    % ------------------------------------------------------------------
    scenario.title       = 'Sudden Cattle Crossing Event';
    scenario.description = ['Ego vehicle cruising at speed when cattle bolts laterally ' ...
                            'from roadside foliage. Requires emergency evasive action.'];
    scenario.start_pose  = [2.0,  0.0, 0.0];
    scenario.goal_pose   = [55.0, 0.0, 0.0];
    scenario.road_width  = 6.0;

    m = zeros(nY, nX);
    m(1:32,  :) = 1;   % off-road boundary
    m(69:nY, :) = 1;
    scenario.static_map = m;

    scenario.obstacles = make_obs( ...
        {1, 'cattle',[28.0, -4.5], [0.5, 2.8], 'erratic'});  % high lateral speed

    otherwise
        error('generate_scenarios: scenario_id must be 1-5, got %d', scenario_id);
end

% ---- Sanity check: start and goal must NOT overlap static obstacles ----
assert_clear(scenario, 'start', scenario.start_pose);
assert_clear(scenario, 'goal',  scenario.goal_pose);
end

% ======================================================================
%  Helper: paint a filled circle of value 1 into a binary grid
% ======================================================================
function m = add_circle(m, res, bounds, cx, cy, r)
    [nY, nX] = size(m);
    col_c   = round((cx - bounds(1)) / res) + 1;
    row_c   = round((cy - bounds(3)) / res) + 1;
    r_cells = ceil(r / res);
    for col = max(1, col_c - r_cells) : min(nX, col_c + r_cells)
        for row = max(1, row_c - r_cells) : min(nY, row_c + r_cells)
            if hypot((col - col_c)*res, (row - row_c)*res) <= r
                m(row, col) = 1;
            end
        end
    end
end

% ======================================================================
%  Helper: build obstacle struct array from variable-length cell list
% ======================================================================
function obs = make_obs(varargin)
    obs = struct('id',{},'type',{},'position',{},'velocity',{},'behavior_profile',{});
    for k = 1:nargin
        c = varargin{k};
        obs(k).id               = c{1};
        obs(k).type             = c{2};
        obs(k).position         = c{3};   % [x, y]
        obs(k).velocity         = c{4};   % [vx, vy]
        obs(k).behavior_profile = c{5};   % 'erratic','weaving','steady','slow'
    end
end

% ======================================================================
%  Helper: assert a pose does NOT fall on a static obstacle cell
% ======================================================================
function assert_clear(scenario, label, pose)
    res    = scenario.grid_res;
    bounds = scenario.bounds;
    m      = scenario.static_map;
    col = min(max(round((pose(1) - bounds(1)) / res) + 1, 1), size(m,2));
    row = min(max(round((pose(2) - bounds(3)) / res) + 1, 1), size(m,1));
    if m(row, col)
        error('Scenario %d: %s pose [%.1f, %.1f] overlaps a static obstacle cell!', ...
              scenario.id, label, pose(1), pose(2));
    end
end
