# Subsystem 2 — Tactical Behavior Decision Making
## Directive for: **Rithika**
## Branch: `feature/subsystem2-rithika`

---

> **INDEPENDENCE RULE:** This subsystem has **ZERO dependency on any other subsystem's real
> code**. Build and test entirely against mocks matching `directives/00_interface_contracts.md`.
> Push to your own branch `feature/subsystem2-rithika` — **do not merge to master yourself**.
> The Master Integrator handles final integration.

---

## Plain English: What Is This Subsystem Doing?

Think of this subsystem as the **car's brain** — specifically, the part that makes tactical
driving decisions every 100 ms:

- "Is there a vehicle coming at me? Should I slow down and wait?"
- "Is the gap in highway traffic large enough to safely merge?"
- "Am I approaching an intersection? Do I have right-of-way, or should I yield?"
- "What speed should I be driving right now?"

The car already has a 5-state brain (`CRUISE`, `NUDGE`, `YIELD_DECEL`, `YIELD_WAIT`, `RESUME`)
from our earlier work. Your job is to **add 3 new states** for more complex situations
(`LANE_KEEP`, `JUNCTION_TURN`, `HIGHWAY_MERGE`) and write the **logic** that decides whether
it is safe to merge or turn at a junction.

You do not need to worry about how the road geometry is computed (that is Subsystem 1's job)
or how the car physically steers (that is Subsystem 4's job). You work with **mock road data**
and output a `TacticalDecision` struct.

---

## Scope (What You Build — Exactly This, Nothing More)

### In Scope ✅

**New function: `gap_accept_merge()`**
- Checks if there is a safe enough gap in highway traffic to merge onto the faster road.
- This is specifically for **Scenario 3 (Highway Merge)** only.
- Uses Time-To-Collision (TTC) maths — see formula below.

**New function: `intersection_yield()`**
- Checks if any agent (pedestrian, auto-rickshaw) will enter the intersection clear zone in the
  next few seconds.
- This is for **Scenario 2 (Signal-less Urban Intersection)**.
- Returns: `yield = true/false` and `hold_time_est` (how many seconds to wait).

**Extended FSM — add 3 new states to the existing `behavior_state_machine.m`:**
- `LANE_KEEP` — for highway driving; basically the same as `CRUISE` but with tighter lateral
  control. Activated when `params.scenario_type = 'highway'`.
- `JUNCTION_TURN` — for executing a turn at an intersection. Activated when
  `params.junction_active = true` AND `intersection_yield()` says it is safe. Speed: 3.5 m/s.
  Exits when `params.junction_cleared = true`.
- `HIGHWAY_MERGE` — for merging. Activated when `params.merge_zone = true` AND
  `gap_accept_merge()` says the gap is safe. Exits when ego passes the merge completion point.

**All existing 5 states are preserved 100%.** Do not change any existing state logic.

### Commented Stubs for Deferred Features ❌
Place these stubs in `tactical_behavior_decider.m` — **do not implement them**:
```matlab
% TODO: overtake_decider() — deferred, see MOBIL model spec in this directive.
%       Will decide if it is safe to overtake a slow vehicle by pulling into
%       the opposite lane. Not needed for the 5 official PS-26037 scenarios.

% TODO: roundabout_yield() — deferred. Will place a Virtual Stop Line at
%       roundabout entry when a circulating vehicle is < 3.5s away.
%       Not needed for the 5 official PS-26037 scenarios.
```

### Explicitly OUT of Scope ❌
- **NO** roundabout logic.
- **NO** MOBIL model overtaking.
- **NO** `OVERTAKE_APPROACH` or `OVERTAKE_PASS` states.
- **NO** importing any file from Subsystem 1, 3, 4, or 5.

---

## Your Mock Input (Build This Locally)

Since Subsystem 1 is being built in parallel, you need a **fake Frenet output** to test against.
Create this file **inside your own working folder** (do not import from Subsystem 1):

```matlab
% matlab/mock_frenet_output.m — local mock matching Contract 1 in 00_interface_contracts.md
function out = mock_frenet_output(s_val, d_val, kappa_val)
    if nargin < 1, s_val    = 10.0; end
    if nargin < 2, d_val    = 0.0;  end
    if nargin < 3, kappa_val = 0.0; end
    out.s          = s_val;
    out.d          = d_val;
    out.theta_road = 0.0;
    out.kappa      = kappa_val;
end
```

When the Master Integrator merges everything, this mock is deleted and replaced with the
real `frenet_coordinate_converter.m` output from Subsystem 1. Your code must work identically
with either — as long as it reads from the struct fields listed in the contract.

---

## The Maths You Need to Implement

### 1. Gap Acceptance — Time-To-Collision (TTC) for Highway Merge

Used in: `gap_accept_merge(ego_s, ego_v, agents, ttc_safe)`

For each agent vehicle in the list, compute:

$$\text{TTC} = \frac{s_{\text{other}} - s_{\text{ego}}}{v_{\text{rel}}}$$

where $v_{\text{rel}} = v_{\text{ego}} - v_{\text{other}}$ (positive when ego is gaining on the agent).

**Decision rule:**
- If `TTC > ttc_safe` (use `ttc_safe = 3.5 s`) **AND** no agent is within 15 m **behind** ego in
  the target lane → return `accept = true`.
- Otherwise → return `accept = false`.

**Edge cases:**
- If `v_rel ≤ 0` (agent is moving away or same speed), set `TTC = Inf` (gap is safe from that agent).
- If `s_other < s_ego` (agent is behind ego already), skip — it is not blocking the merge.

```matlab
function [accept, min_ttc] = gap_accept_merge(ego_s, ego_v, agents, ttc_safe)
    % ego_s   : double — ego longitudinal position in Frenet frame (m)
    % ego_v   : double — ego speed (m/s)
    % agents  : struct array, each with fields .s (position) and .v (speed)
    % ttc_safe: double — minimum acceptable TTC (s), default 3.5
    if nargin < 4, ttc_safe = 3.5; end
    % ... your implementation here ...
end
```

### 2. Intersection Clear-Zone Yield Check

Used in: `intersection_yield(ego_pose, conflict_agents, clear_zone_radius, horizon_s)`

**What the clear zone is:** A circle of radius `clear_zone_radius = 5.0 m` centred on the
intersection centre point. If any agent is predicted to enter this circle within the next
`horizon_s = 4.0 seconds`, the ego must yield.

**Agent trajectory prediction:** Assume constant velocity (straight-line extrapolation):
$$x_{\text{agent}}(t) = x_0 + v_x \cdot t, \quad y_{\text{agent}}(t) = y_0 + v_y \cdot t$$

Sample at `t = 0.1, 0.2, ..., 4.0 s`. If at any sample time the agent is inside the circle,
yield = true.

**Yield hold time estimate:** Time until the agent exits the clear zone again.

### 3. FSM State Addition (in `behavior_state_machine.m`)

Add these new cases to the existing `switch current_state` block:

```matlab
case 'LANE_KEEP'
    % Same as CRUISE but only active when params.scenario_type == 'highway'
    % If merge zone detected: transition to 'HIGHWAY_MERGE' (pending gap acceptance)
    if isfield(params,'merge_zone') && params.merge_zone
        [gap_ok, ~] = gap_accept_merge(params.ego_s, ego_v, params.merge_agents, 3.5);
        if gap_ok
            new_state = 'HIGHWAY_MERGE';
        else
            new_state = 'LANE_KEEP';  % Stay and wait for gap
        end
    else
        new_state = 'LANE_KEEP';
    end
    v_ref = params.v_cruise;

case 'JUNCTION_TURN'
    % Slow turn through intersection, exit when junction is cleared
    if isfield(params,'junction_cleared') && params.junction_cleared
        new_state = 'CRUISE';
    else
        new_state = 'JUNCTION_TURN';
    end
    v_ref = 3.5;  % 3.5 m/s ≈ 12.6 km/h — safe junction speed

case 'HIGHWAY_MERGE'
    % Merging onto faster road; exit when s exceeds merge completion point
    if isfield(params,'merge_complete_s') && params.ego_s > params.merge_complete_s
        new_state = 'LANE_KEEP';
    else
        new_state = 'HIGHWAY_MERGE';
    end
    v_ref = min(params.v_cruise, params.v_merge_cap);
```

**Hysteresis guard:** The existing `+0.5 m` hysteresis (`W_unlatch = W_crit + 0.50 m`) is
preserved. New states do not use corridor width hysteresis (they have their own entry/exit guards).

---

## Deliverables

| File | Type | What It Contains |
|---|---|---|
| `matlab/tactical_behavior_decider.m` | MATLAB function file | `gap_accept_merge()` and `intersection_yield()` functions, with stubs for deferred features |
| `matlab/behavior_state_machine.m` | MATLAB function (MODIFIED) | Existing 5 states untouched + 3 new states added |
| `matlab/mock_frenet_output.m` | MATLAB function (local mock) | Fake FrenetOutput for your own tests only |
| `matlab/test_subsystem2.m` | MATLAB test script | Runs all 3 tests and prints numeric results |

**Output struct produced** (see `directives/00_interface_contracts.md` for exact field types):
```matlab
TacticalDecision.active_state   % string (one of 8 FSM states)
TacticalDecision.v_ref          % double (m/s)
TacticalDecision.yield_required % bool
TacticalDecision.hold_time_est  % double (s)
```

---

## Required Tests — You Must Print Real Numbers

Run with: `matlab -batch "addpath('matlab'); test_subsystem2"` from repo root.

### Test T1 — Merge Gap Reject (Oncoming Vehicle Too Close)

**Scenario:** Ego is at position `s=0`, speed `8 m/s`. An oncoming vehicle is at `s=20 m`,
moving at `−10 m/s` (toward ego). Relative speed = 18 m/s.

$$\text{TTC} = \frac{20 - 0}{8 - (-10)} = \frac{20}{18} = 1.11\,\text{s}$$

Since 1.11 s < 3.5 s threshold → **must reject the merge**.

**Required printed output:**
```
[T1] Merge gap rejection test:
     ego_s=0m, ego_v=8m/s | agent_s=20m, agent_v=-10m/s
     Computed TTC : 1.11 s
     Threshold    : 3.50 s
     Decision     : REJECT
     Result       : PASS   (or FAIL if decision is ACCEPT)
```

### Test T2 — Merge Gap Accept (Distant Slow Vehicle)

**Scenario:** Ego at `s=0`, speed `8 m/s`. Agent at `s=60 m`, speed `−5 m/s`.
Relative speed = 13 m/s.

$$\text{TTC} = \frac{60 - 0}{8 - (-5)} = \frac{60}{13} = 4.62\,\text{s}$$

Since 4.62 s > 3.5 s threshold → **must accept the merge**.

**Required printed output:**
```
[T2] Merge gap acceptance test:
     ego_s=0m, ego_v=8m/s | agent_s=60m, agent_v=-5m/s
     Computed TTC : 4.62 s
     Threshold    : 3.50 s
     Decision     : ACCEPT
     Result       : PASS   (or FAIL if decision is REJECT)
```

> **Note on the original spec:** The prior planning document quoted TTC = 9.23 s for this test
> case. Using `agent_v = -5 m/s` gives TTC = 4.62 s. The corrected numbers above are
> arithmetically consistent. Use these numbers — do not use 9.23 s.

### Test T3 — Intersection Yield Logic

**Sub-test A (yield required):** An auto-rickshaw is at position `(35.0, 8.0)` with velocity
`(0.0, −3.0) m/s`. Intersection centre is at `(30.0, 0.0)`. Clear-zone radius = 5.0 m.
The agent reaches the clear-zone boundary in approximately **2.5 s** → `yield = true`.

**Sub-test B (no yield needed):** Same agent but starting at `(35.0, 30.0)` with velocity
`(0.0, −1.0) m/s`. At this slow speed it takes **>8 s** to reach the clear zone → `yield = false`.

**Required printed output:**
```
[T3] Intersection yield logic:
     Sub-test A: agent arrives at clear zone in ~2.5s
                 yield_required = true          [PASS/FAIL]
                 hold_time_est  = X.X s
     Sub-test B: agent arrives at clear zone in >8s
                 yield_required = false         [PASS/FAIL]
```

---

## Zero-Toolbox Rule

Use **only** base MATLAB functions. Allowed: `struct()`, `isfield()`, `switch/case`,
`for`, `if`, `hypot()`, `min()`, `max()`, `linspace()`, `Inf`.

---
