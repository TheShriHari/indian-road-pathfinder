function [new_state, v_ref, debug_info] = behavior_state_machine(current_state, ego_state, predicted_agents, dt, params)
% BEHAVIOR_STATE_MACHINE  State-based behavioral decision layer for ego vehicle.
%   [new_state, v_ref, debug_info] = behavior_state_machine(
%       current_state, ego_state, predicted_agents, dt, params)
%
% DESIGN INSPIRATION:
%   State structure conceptually inspired by Autoware Universe's
%   behavior_velocity_planner (autoware_universe/planning/behavior_velocity_planner).
%   Specifically: the yield/nudge/resume state-machine pattern, and the idea of
%   computing longitudinal/lateral distance to predicted agent positions to drive
%   transitions. This is an INDEPENDENT REIMPLEMENTATION in plain MATLAB —
%   no Autoware code was ported or integrated; it was studied as an architectural
%   reference only.
%
% States:
%   'CRUISE'      Normal speed; no nearby agent within threat envelope
%   'NUDGE'       Agent detected at medium range; reduce speed, slight lateral shift
%   'YIELD_DECEL' Agent in path at closer range; decelerate to yield speed
%   'YIELD_WAIT'  Agent very close or crossing; slow to near-stop
%   'RESUME'      Agent cleared; accelerate back toward cruise speed
%
% Inputs:
%   current_state    : current state string (one of the 5 above)
%   ego_state        : [x, y, theta, v]  (row or column vector)
%   predicted_agents : struct array from dynamic_obstacle_predictor
%   dt               : timestep (s)   [unused internally; reserved for future hysteresis timers]
%   params           : optional struct; defaults used if empty/missing
%
% Outputs:
%   new_state  : next state string
%   v_ref      : reference velocity for this timestep (m/s)
%   debug_info : struct with nearest agent distance, lat/lon decomposition

% ── Build defaults, then merge any caller-supplied fields on top ──────────
% Merging (not replacing) guarantees every field exists even when the caller
% passes a partial struct (e.g. only virtual_stop_active and stop_line_dist).
defaults.v_cruise      = 5.0;   % m/s  nominal cruise speed
defaults.v_nudge       = 3.5;   % m/s  reduced speed during NUDGE
defaults.v_decel       = 1.5;   % m/s  slow yield approach
defaults.v_wait        = 0.0;   % m/s  full stop during YIELD_WAIT to avoid creeping into in-path obstacles
defaults.v_resume      = 2.5;   % m/s  initial resume speed
defaults.d_nudge       = 12.0;  % m  threshold to enter NUDGE
defaults.d_decel       = 8.5;   % m  threshold to enter YIELD_DECEL
defaults.d_wait        = 6.0;   % m  threshold to enter YIELD_WAIT (provides 2.4m front bumper buffer before braking)
defaults.d_clear       = 10.0;  % m  agent must exceed this to allow RESUME->CRUISE
% d_lat_path: set to 1.30 m (vehicle half-width 0.925 m + 0.375 m margin)
% for crossing agents and dynamic objects.
defaults.d_lat_path    = 1.30;  % m  max lateral offset to consider agent "in path"
if nargin < 5 || isempty(params)
    params = defaults;
else
    % Fill in any field that is absent from the caller's partial struct
    fnames = fieldnames(defaults);
    for fi = 1:length(fnames)
        if ~isfield(params, fnames{fi})
            params.(fnames{fi}) = defaults.(fnames{fi});
        end
    end
end

ego_x     = ego_state(1);
ego_y     = ego_state(2);
ego_theta = ego_state(3);

% ---- Find nearest predicted agent and check if any agent enters ego path ----
min_dist     = Inf;
nearest_lon  = Inf;
nearest_lat  = Inf;
agent_in_path = false;

for i = 1:length(predicted_agents)
    wp = predicted_agents(i).waypoints;
    if isempty(wp), continue; end
    
    % Current position (h = 1)
    ax0 = wp(1, 1);
    ay0 = wp(1, 2);
    dx0 = ax0 - ego_x;
    dy0 = ay0 - ego_y;
    d0  = hypot(dx0, dy0);
    lon0 =  dx0 * cos(ego_theta) + dy0 * sin(ego_theta);
    lat0 = abs(-dx0 * sin(ego_theta) + dy0 * cos(ego_theta));
    if d0 < min_dist
        min_dist = d0;
        nearest_lon = lon0;
        nearest_lat = lat0;
    end
    
    % Scan prediction horizon (up to 20 steps = 2.0s) for corridor intersection
    for h = 1:min(20, size(wp, 1))
        ax = wp(h, 1);
        ay = wp(h, 2);
        dx = ax - ego_x;
        dy = ay - ego_y;
        lon =  dx * cos(ego_theta) + dy * sin(ego_theta);
        lat = abs(-dx * sin(ego_theta) + dy * cos(ego_theta));
        
        % If predicted to be ahead and in path corridor
        if lon > 0 && lon < 30.0 && lat < params.d_lat_path
            agent_in_path = true;
            d_pred = hypot(dx, dy);
            if d_pred < min_dist || (nearest_lat >= params.d_lat_path)
                min_dist = min(min_dist, d_pred);
                nearest_lon = lon;
                nearest_lat = lat;
            end
        end
    end
end

% Backward compatibility: if no predicted agent entered corridor, but immediate position is in path
if ~agent_in_path && nearest_lon > 0 && nearest_lat < params.d_lat_path
    agent_in_path = true;
end

debug_info.min_dist      = min_dist;
debug_info.nearest_lon   = nearest_lon;
debug_info.nearest_lat   = nearest_lat;
debug_info.agent_in_path = agent_in_path;

% ---- Dynamic Spatial-Temporal Bottleneck Decider ----
% Adaptable to ANY 3D CARLA scene without prior coordinate knowledge.
% If an external universal_bottleneck_decider provided the flag via params, respect it.
% Otherwise, compute dynamic corridor squeeze relative to current ego heading & path.
virtual_stop_active = false;
stop_line_dist = 4.0;

if isfield(params, 'virtual_stop_active')
    virtual_stop_active = params.virtual_stop_active;
    if isfield(params, 'stop_line_dist'), stop_line_dist = params.stop_line_dist; end
else
    % Fallback only when no external decider is provided:
    % Halt only if an agent is directly obstructing the immediate travel lane (lat_d < 0.9m)
    for i = 1:length(predicted_agents)
        wp = predicted_agents(i).waypoints;
        if isempty(wp), continue; end
        for step_h = 1:min(5, size(wp, 1))
            ag_x = wp(step_h, 1);
            ag_y = wp(step_h, 2);
            dx = ag_x - ego_x;
            dy = ag_y - ego_y;
            lon_d = dx * cos(ego_theta) + dy * sin(ego_theta);
            lat_d = abs(-dx * sin(ego_theta) + dy * cos(ego_theta));
            
            if lon_d > 2.0 && lon_d < 6.0 && lat_d < 0.9
                virtual_stop_active = true;
                stop_line_dist = max(1.5, lon_d - 3.5);
                break;
            end
        end
        if virtual_stop_active, break; end
    end
end

debug_info.virtual_stop_active = virtual_stop_active;
debug_info.stop_line_dist = stop_line_dist;

% Compute closing speed for approaching/oncoming agents to adapt safety margins
max_closing_speed = 0.0;
v_ego_fwd = ego_state(4);
for i = 1:length(predicted_agents)
    if isfield(predicted_agents(i), 'velocity') && ~isempty(predicted_agents(i).velocity)
        vx_ag = predicted_agents(i).velocity(1);
        if vx_ag < 0
            v_rel = v_ego_fwd - vx_ag;
            if v_rel > max_closing_speed
                max_closing_speed = v_rel;
            end
        end
    end
end

% Adaptive threshold scaling for oncoming traffic
d_wait_eff  = params.d_wait  + min(3.5, max(0.0, (max_closing_speed - 3.0) * 0.7));
d_decel_eff = params.d_decel + min(4.0, max(0.0, (max_closing_speed - 3.0) * 0.9));

% ---- State machine transitions ----
% Hysteresis: transitions from safer to more cautious states are trigger-immediate;
% transitions back to less cautious states require distance to exceed the "clear"
% threshold, preventing chattering at boundary distances.
if virtual_stop_active
    % Virtual stop line takes precedence over standard distance threshold
    new_state = 'YIELD_WAIT';
else
    switch current_state

        case 'CRUISE'
            if agent_in_path && min_dist < d_wait_eff
                new_state = 'YIELD_WAIT';
            elseif agent_in_path && min_dist < d_decel_eff
                new_state = 'YIELD_DECEL';
            elseif agent_in_path && min_dist < params.d_nudge
                new_state = 'NUDGE';
            else
                new_state = 'CRUISE';
            end

        case 'NUDGE'
            if agent_in_path && min_dist < d_wait_eff
                new_state = 'YIELD_WAIT';
            elseif agent_in_path && min_dist < d_decel_eff
                new_state = 'YIELD_DECEL';
            elseif (~agent_in_path) || (min_dist >= params.d_clear)
                new_state = 'CRUISE';
            else
                new_state = 'NUDGE';
            end

        case 'YIELD_DECEL'
            if agent_in_path && min_dist < d_wait_eff
                new_state = 'YIELD_WAIT';
            elseif (~agent_in_path && min_dist >= d_decel_eff) || (min_dist >= params.d_clear)
                new_state = 'RESUME';
            else
                new_state = 'YIELD_DECEL';
            end

        case 'YIELD_WAIT'
            if (~agent_in_path && min_dist >= d_wait_eff + 1.5) || (min_dist >= d_decel_eff)
                new_state = 'RESUME';
            else
                new_state = 'YIELD_WAIT';
            end

        case 'RESUME'
            if agent_in_path && min_dist < d_wait_eff
                new_state = 'YIELD_WAIT';
            elseif agent_in_path && min_dist < d_decel_eff
                new_state = 'YIELD_DECEL';
            elseif min_dist >= params.d_clear
                new_state = 'CRUISE';
            else
                new_state = 'RESUME';
            end

        otherwise
            % Unknown state: safe default
            new_state = 'CRUISE';
    end
end

% ---- Map state to reference velocity ----
switch new_state
    case 'CRUISE',       v_ref = params.v_cruise;
    case 'NUDGE',        v_ref = params.v_nudge;
    case 'YIELD_DECEL',  v_ref = params.v_decel;
    case 'YIELD_WAIT'
        if virtual_stop_active
            v_ref = 0.0; % Complete halt before Virtual Stop Line
        else
            v_ref = params.v_wait;
        end
    case 'RESUME',       v_ref = params.v_resume;
    otherwise,           v_ref = params.v_cruise;
end
end
