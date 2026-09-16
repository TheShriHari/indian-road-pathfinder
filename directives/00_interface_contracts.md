# Interface Contracts — SIH PS-26037 Autonomous Pathfinder
## Layer 1 Directive · Version 1.0 · FROZEN

> **IMPORTANT — READ THIS FIRST.**
> These contracts are **FROZEN**. They are the shared language between all 5 subsystems.
> Any subsystem that needs another subsystem's output as its input **must build a local mock
> generator** that produces a struct matching this exact shape. You must **never** import or call
> another subsystem's real `.m` file until the Master Integrator does the final merge on `master`.
> This rule exists so that 5 teammates can work completely independently in parallel without
> waiting on each other or accidentally breaking each other's work.

---

## What Is an Interface Contract?

Think of each contract as a **standardised envelope**. Subsystem 1 puts its results in a specific
envelope. Subsystem 2 reads from that same envelope shape — but during development it uses a
**fake envelope filled with dummy numbers** instead of waiting for Subsystem 1 to finish.
On integration day, the Master Integrator swaps the fake envelope for the real one.
As long as the envelope shape is identical, everything fits together perfectly.

---

## Contract 1 — `FrenetOutput`

**Produced by:** Subsystem 1 (Map & Environment)  
**Consumed by:** Subsystems 2, 3, 5

```matlab
% FrenetOutput struct — describes where the ego vehicle sits
%   relative to the road centreline at a given instant.
FrenetOutput.s          = 0.0;   % double — longitudinal distance (m) along the road
                                 %           from the start of the planned path.
                                 %           Think of it as "how far down the road are we".
FrenetOutput.d          = 0.0;   % double — lateral offset (m) from road centreline.
                                 %           Positive = left of centre, negative = right.
FrenetOutput.theta_road = 0.0;   % double — road heading angle (radians) at station s.
                                 %           This is the direction the road itself is pointing.
FrenetOutput.kappa      = 0.0;   % double — road curvature (1/m) at station s.
                                 %           0 = straight road. 0.2222 = sharpest allowed curve.
```

**Mock generator stub (copy into your own folder, never import from Subsystem 1):**
```matlab
function out = mock_frenet_output(s_val, d_val)
    out.s          = s_val;
    out.d          = d_val;
    out.theta_road = 0.0;
    out.kappa      = 0.0;
end
```

---

## Contract 2 — `TacticalDecision`

**Produced by:** Subsystem 2 (Tactical Behavior)  
**Consumed by:** Subsystems 4, 5

```matlab
% TacticalDecision struct — the high-level driving instruction for this moment.
%   The motion controller reads this to know what the car should be doing.
TacticalDecision.active_state   = 'CRUISE';  % string — current FSM state.
                                             % Must be one of exactly 8 values:
                                             %   'CRUISE'        — normal driving
                                             %   'NUDGE'         — slight lateral detour around hazard
                                             %   'YIELD_DECEL'   — slowing down for obstacle/junction
                                             %   'YIELD_WAIT'    — fully stopped, waiting to proceed
                                             %   'RESUME'        — starting to move again after waiting
                                             %   'LANE_KEEP'     — highway lane-keeping mode
                                             %   'JUNCTION_TURN' — executing a junction turn (left/right/sharp)
                                             %   'HIGHWAY_MERGE' — merging onto a faster road
TacticalDecision.v_ref          = 8.0;      % double — target speed (m/s) the car should maintain.
TacticalDecision.yield_required = false;    % bool   — true if the car must hold its position.
TacticalDecision.hold_time_est  = 0.0;     % double — estimated seconds until it is safe to proceed.
                                             %           0.0 if yield_required is false.
```

**Mock generator stub:**
```matlab
function out = mock_tactical_decision(state_str, v)
    out.active_state   = state_str;
    out.v_ref          = v;
    out.yield_required = false;
    out.hold_time_est  = 0.0;
end
```

---

## Contract 3 — `TrajectoryOutput`

**Produced by:** Subsystem 3 (Trajectory Generation)  
**Consumed by:** Subsystems 4, 5

```matlab
% TrajectoryOutput struct — the planned smooth path plus speed schedule.
%   The motion controller follows this path. The visualiser draws it on screen.
TrajectoryOutput.path             = zeros(80, 2); % double [Nx2] — sequence of (x, y) waypoints (m).
                                                   %                N = 80 points over 25 m horizon.
TrajectoryOutput.speed_profile    = zeros(80, 1); % double [Nx1] — target speed (m/s) at each waypoint.
                                                   %                Speed drops automatically before curves.
TrajectoryOutput.max_kappa        = 0.0;          % double — maximum path curvature (1/m) along entire path.
                                                   %          Must stay ≤ 0.2222 m⁻¹ or path is invalid.
TrajectoryOutput.is_collision_free = true;        % bool   — true if the swept vehicle envelope (1.85m
                                                   %          + 0.35m cushion) is clear of all obstacles.
```

**Mock generator stub:**
```matlab
function out = mock_trajectory_output(num_pts)
    if nargin < 1, num_pts = 80; end
    t = linspace(0, 1, num_pts)';
    out.path             = [t * 25.0, zeros(num_pts, 1)];
    out.speed_profile    = ones(num_pts, 1) * 8.0;
    out.max_kappa        = 0.05;
    out.is_collision_free = true;
end
```

---

## Contract 4 — `ControlOutput`

**Produced by:** Subsystem 4 (Motion Control)  
**Consumed by:** Subsystem 5 (Telemetry & Web)

```matlab
% ControlOutput struct — the actual commands sent to the vehicle actuators.
%   Steering wheel angle + throttle pedal + brake pedal.
ControlOutput.steer_rad  = 0.0; % double — steering angle in radians.
                                 %          Range: [-0.5236, +0.5236] (i.e. ±30 degrees).
                                 %          Positive = steer left, negative = steer right.
ControlOutput.throttle   = 0.0; % double — throttle pedal position, 0.0 to 1.0.
                                 %          0.0 = not pressing, 1.0 = fully pressed.
ControlOutput.brake      = 0.0; % double — brake pedal position, 0.0 to 1.0.
                                 %          0.0 = no braking, 1.0 = maximum braking.
                                 %          throttle and brake should not both be > 0 simultaneously.
```

**Mock generator stub:**
```matlab
function out = mock_control_output()
    out.steer_rad = 0.0;
    out.throttle  = 0.5;
    out.brake     = 0.0;
end
```

---

## Contract 5 — `TelemetryFrame`

**Produced by:** Subsystem 5 (Telemetry & Web)  
**Consumed by:** Web visualiser JSON player

```matlab
% TelemetryFrame struct — one timestamped snapshot of everything happening in the simulation.
%   This is what gets saved to JSON and replayed in the browser.
%   It is a flat union of all the above contracts plus bookkeeping fields.

% --- Bookkeeping ---
TelemetryFrame.timestamp    = 0.0;        % double — simulation time in seconds since scenario start.
TelemetryFrame.scenario_id  = 'SC-1';    % string — which of the 5 official scenarios this frame belongs to.
                                           %          One of: 'SC-1','SC-2','SC-3','SC-4','SC-5'.

% --- From FrenetOutput ---
TelemetryFrame.s            = 0.0;        % double — longitudinal road distance (m)
TelemetryFrame.d            = 0.0;        % double — lateral road offset (m)
TelemetryFrame.theta_road   = 0.0;        % double — road heading (rad)
TelemetryFrame.kappa        = 0.0;        % double — road curvature (1/m)

% --- From TacticalDecision ---
TelemetryFrame.active_state   = 'CRUISE'; % string — current FSM state (one of 8 states)
TelemetryFrame.v_ref          = 0.0;      % double — target speed (m/s)
TelemetryFrame.yield_required = false;    % bool
TelemetryFrame.hold_time_est  = 0.0;     % double

% --- From TrajectoryOutput ---
TelemetryFrame.max_kappa        = 0.0;   % double — max path curvature
TelemetryFrame.is_collision_free = true; % bool

% --- From ControlOutput ---
TelemetryFrame.steer_rad  = 0.0;         % double — actual steering command (rad)
TelemetryFrame.throttle   = 0.0;         % double — throttle [0,1]
TelemetryFrame.brake      = 0.0;         % double — brake [0,1]

% --- Derived metrics (computed in Subsystem 5 from consecutive frames) ---
TelemetryFrame.ego_x          = 0.0;    % double — ego vehicle X position (m)
TelemetryFrame.ego_y          = 0.0;    % double — ego vehicle Y position (m)
TelemetryFrame.ego_theta      = 0.0;    % double — ego heading (rad)
TelemetryFrame.ego_v          = 0.0;    % double — ego speed (m/s)
TelemetryFrame.cross_track_err = 0.0;  % double — lateral deviation from planned path (m)
TelemetryFrame.lon_jerk       = 0.0;   % double — longitudinal jerk (m/s³)
TelemetryFrame.lat_jerk       = 0.0;   % double — lateral jerk (m/s³)
```

---

## Summary Table

| Contract | Produced By | Consumed By | Key Constraint |
|---|---|---|---|
| `FrenetOutput` | Subsystem 1 | 2, 3, 5 | `s ≥ 0`, `kappa ≤ 0.2222` |
| `TacticalDecision` | Subsystem 2 | 4, 5 | `active_state` must be one of exactly 8 strings |
| `TrajectoryOutput` | Subsystem 3 | 4, 5 | `max_kappa ≤ 0.2222`, `is_collision_free = true` before use |
| `ControlOutput` | Subsystem 4 | 5 | `steer_rad ∈ [-0.5236, 0.5236]`, `throttle + brake ≤ 1.0` |
| `TelemetryFrame` | Subsystem 5 | Browser | All fields present, no NaN, no Inf |

---

*Document maintained by: Master Integrator (Shri Hari)*  
*Last frozen: 2026-09-16*  
*Branch: `master` — do not edit this file from any subsystem branch.*
