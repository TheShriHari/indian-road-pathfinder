function [new_state, v_ref, debug_info] = behavior_state_machine(current_state, ego_state, predicted_agents, dt, params)
%% BEHAVIOR_STATE_MACHINE 5-State Finite State Machine with Spatial Hysteresis (Phase 4).
%   [new_state, v_ref, debug_info] = behavior_state_machine(current_state, ego_state, predicted_agents, dt, params)
%
% States:
%   'CRUISE'      - Nominal cruising speed (v_cruise = 8.0 m/s)
%   'NUDGE'       - Lateral detour around static hazard / cattle (v_nudge = 4.0 m/s)
%   'YIELD_DECEL' - Smooth deceleration toward Virtual Stop Line (a = -1.5 m/s^2)
%   'YIELD_WAIT'  - Standstill at Virtual Stop Line (v_wait = 0.0 m/s)
%   'RESUME'      - Controlled launch clearing bottleneck (v_resume = 3.5 m/s)
%
% Key Features:
%   - Explicit persistent state reset via params.reset = true
%   - Spatial hysteresis unlatching: W_free must reach W_crit + 0.50 = 3.05m to exit NUDGE -> CRUISE
%   - Full yield cycle: CRUISE -> YIELD_DECEL -> YIELD_WAIT -> RESUME -> CRUISE with zero chattering

% ── Configuration Defaults ─────────────────────────────────────────────────
defaults.v_cruise   = 8.0;   % nominal cruising velocity (m/s)
defaults.v_nudge    = 4.0;   % capped detour velocity during NUDGE (m/s)
defaults.v_decel    = 1.5;   % approach crawl velocity during YIELD_DECEL (m/s)
defaults.v_wait     = 0.0;   % full standstill velocity (m/s)
defaults.v_resume   = 3.5;   % controlled launch velocity during RESUME (m/s)

defaults.W_crit     = 2.55;  % critical corridor clearance (m)
defaults.W_hyst     = 0.50;  % spatial hysteresis margin (m) -> unlatch at 3.05m

defaults.d_nudge    = 12.0;  % threat distance triggering NUDGE (m)
defaults.d_clear    = 10.0;  % obstacle clearance distance (m)
defaults.d_lat_path = 1.30;  % lateral offset threshold for path contention (m)

if nargin < 4 || isempty(dt), dt = 0.1; end
if nargin < 5 || isempty(params), params = struct(); end

if isfield(params, 'delta_hyst') && ~isempty(params.delta_hyst)
    params.W_hyst = params.delta_hyst;
end

fnames = fieldnames(defaults);
for fi = 1:length(fnames)
    if ~isfield(params, fnames{fi})
        params.(fnames{fi}) = defaults.(fnames{fi});
    end
end

% ── Explicit State Isolation Reset ─────────────────────────────────────────
if isfield(params, 'reset') && params.reset
    new_state = 'CRUISE';
    v_ref     = params.v_cruise;
    debug_info = struct('min_dist', Inf, 'nearest_lon', Inf, 'nearest_lat', Inf, ...
                        'agent_in_path', false, 'virtual_stop_active', false, ...
                        'stop_line_dist', Inf, 'W_free', Inf);
    return;
end

if isempty(current_state)
    current_state = 'CRUISE';
end

ego_x     = ego_state(1);
ego_y     = ego_state(2);
if length(ego_state) >= 3, ego_theta = ego_state(3); else, ego_theta = 0.0; end
if length(ego_state) >= 4, ego_v = ego_state(4);     else, ego_v = 0.0;     end

% ── Perception & Decider Telemetry Parsing ─────────────────────────────────
virtual_stop_active = false;
stop_line_dist      = Inf;
if isfield(params, 'virtual_stop_active'), virtual_stop_active = params.virtual_stop_active; end
if isfield(params, 'stop_line_dist'),      stop_line_dist      = params.stop_line_dist;      end

W_free = Inf;
if isfield(params, 'W_free'),              W_free = params.W_free;
elseif isfield(params, 'corridor_width'),  W_free = params.corridor_width;
elseif isfield(params, 'min_width'),       W_free = params.min_width;
end

% ── Scan Dynamic Predicted Agents ──────────────────────────────────────────
min_dist      = Inf;
nearest_lon   = Inf;
nearest_lat   = Inf;
agent_in_path = false;

if ~isempty(predicted_agents)
    for i = 1:length(predicted_agents)
        wp = predicted_agents(i).waypoints;
        if isempty(wp), continue; end
        
        d0 = hypot(wp(1, 1) - ego_x, wp(1, 2) - ego_y);
        lon0 =  (wp(1, 1) - ego_x) * cos(ego_theta) + (wp(1, 2) - ego_y) * sin(ego_theta);
        lat0 = abs(-(wp(1, 1) - ego_x) * sin(ego_theta) + (wp(1, 2) - ego_y) * cos(ego_theta));
        if d0 < min_dist
            min_dist = d0;
            nearest_lon = lon0;
            nearest_lat = lat0;
        end
        
        for h = 1:min(20, size(wp, 1))
            lon =  (wp(h, 1) - ego_x) * cos(ego_theta) + (wp(h, 2) - ego_y) * sin(ego_theta);
            lat = abs(-(wp(h, 1) - ego_x) * sin(ego_theta) + (wp(h, 2) - ego_y) * cos(ego_theta));
            if lon > 0.0 && lon < 30.0 && lat < params.d_lat_path
                agent_in_path = true;
                break;
            end
        end
    end
end

debug_info.min_dist            = min_dist;
debug_info.nearest_lon         = nearest_lon;
debug_info.nearest_lat         = nearest_lat;
debug_info.agent_in_path       = agent_in_path;
debug_info.virtual_stop_active = virtual_stop_active;
debug_info.stop_line_dist      = stop_line_dist;
debug_info.W_free              = W_free;

% ── 5-State Finite State Machine Transitions ───────────────────────────────
W_unlatch = params.W_crit + params.W_hyst; % 2.55 + 0.50 = 3.05 m

switch current_state
    
    case 'CRUISE'
        if virtual_stop_active
            if ego_v <= 0.1 && stop_line_dist <= 1.0
                new_state = 'YIELD_WAIT';
            else
                new_state = 'YIELD_DECEL';
            end
        elseif W_free < params.W_crit || (agent_in_path && min_dist < params.d_nudge)
            new_state = 'NUDGE';
        else
            new_state = 'CRUISE';
        end
        
    case 'NUDGE'
        if virtual_stop_active
            if ego_v <= 0.1 && stop_line_dist <= 1.0
                new_state = 'YIELD_WAIT';
            else
                new_state = 'YIELD_DECEL';
            end
        elseif (W_free >= W_unlatch) && (~agent_in_path || min_dist >= params.d_clear)
            % Spatial hysteresis unlatch: requires W_free >= 3.05m to return to CRUISE
            new_state = 'CRUISE';
        else
            % Stays in NUDGE when W_free is in [2.55m, 3.05m)
            new_state = 'NUDGE';
        end
        
    case 'YIELD_DECEL'
        if ~virtual_stop_active
            new_state = 'RESUME';
        elseif ego_v <= 0.1 && stop_line_dist <= 1.0
            new_state = 'YIELD_WAIT';
        else
            new_state = 'YIELD_DECEL';
        end
        
    case 'YIELD_WAIT'
        if ~virtual_stop_active
            new_state = 'RESUME';
        else
            new_state = 'YIELD_WAIT';
        end
        
    case 'RESUME'
        if virtual_stop_active
            new_state = 'YIELD_DECEL';
        elseif (W_free >= W_unlatch) && (~agent_in_path || min_dist >= params.d_clear)
            new_state = 'CRUISE';
        elseif W_free < params.W_crit || agent_in_path
            new_state = 'NUDGE';
        else
            new_state = 'RESUME';
        end
        
    otherwise
        new_state = 'CRUISE';
end

% ── Reference Velocity Generation ──────────────────────────────────────────
switch new_state
    case 'CRUISE'
        v_ref = params.v_cruise;
    case 'NUDGE'
        v_ref = params.v_nudge;
    case 'YIELD_DECEL'
        % Approach virtual stop line smoothly under comfortable deceleration
        if stop_line_dist > 1.0 && ~isinf(stop_line_dist)
            v_approach = min(params.v_decel, sqrt(2.0 * 1.5 * max(0.1, stop_line_dist - 0.5)));
            v_ref = max(0.8, v_approach);
        elseif isinf(stop_line_dist)
            v_ref = params.v_decel;
        else
            v_ref = 0.0;
        end
    case 'YIELD_WAIT'
        v_ref = params.v_wait;
    case 'RESUME'
        % Smooth acceleration at +1.0 m/s^2 up to resume cap (3.5 m/s)
        v_ref = min(params.v_resume, max(1.5, ego_v + 1.0 * dt));
    otherwise
        v_ref = params.v_cruise;
end

end
