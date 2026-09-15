# Layer 1 Directive: Kinematic Planning & Behavioral Arbitration

## 1. Scope & Objective
This directive specifies the mathematical formulation, geometric representations, behavioral state transitions, and tracking controller algorithms for:
1. Non-holonomic kinematic path planner with Ackerman constraints and $C^2$ spline smoothing ([`matlab/adaptive_path_planner.m`](file:///c:/Users/toshr/OneDrive/Desktop/Hari@Work/Shri%20Hari%20-%20Official%20Documents/IIT%20KGP/SIH/indian-road-pathfinder/matlab/adaptive_path_planner.m)).
2. 5-state behavioral arbitration state machine with spatial hysteresis unlatching ([`matlab/behavior_state_machine.m`](file:///c:/Users/toshr/OneDrive/Desktop/Hari@Work/Shri%20Hari%20-%20Official%20Documents/IIT%20KGP/SIH/indian-road-pathfinder/matlab/behavior_state_machine.m)).
3. Speed-adaptive Pure Pursuit tracking controller with steering slew rate and longitudinal jerk clamping ([`matlab/pure_pursuit_controller.m`](file:///c:/Users/toshr/OneDrive/Desktop/Hari@Work/Shri%20Hari%20-%20Official%20Documents/IIT%20KGP/SIH/indian-road-pathfinder/matlab/pure_pursuit_controller.m)).

The system guarantees continuous-curvature, kinematically executable trajectories across unstructured Indian roadway geometries, arbitrates bottlenecks without state chattering, limits jerk to $|j(t)| < 1.0\,\text{m/s}^3$, limits steering rate to $|\Delta \delta / \Delta t| \le 15^\circ/\text{s}$, and keeps replanning latency under $50\,\text{ms}$ (P99).

---

## 2. Mathematical Formulation

### 2.1 Kinematic Feasibility & Bounded Curvature
To ensure kinematic executability by an autonomous Ackerman ground vehicle:
* Wheelbase: $L = 2.7\,\text{m}$
* Maximum steering angle: $\delta_{\max} = \pm 30^\circ = \pm 0.5236\,\text{rad}$
* Minimum turning radius:
  $$R_{\min} = \frac{L}{\tan(\delta_{\max})} = \frac{2.7}{\tan(30^\circ)} \approx 4.6765\,\text{m} \quad (\text{clamped bound: } 4.50\,\text{m})$$
* Maximum path curvature bound:
  $$\kappa_{\max} = \frac{1}{R_{\min}} \approx 0.2222\,\text{m}^{-1}$$

**Kinematic Motion Primitives:**
Given state node $\mathbf{s} = [x, y, \theta]^T$, expand candidate motion primitives across discrete steering inputs $\delta \in \{-\delta_{\max}, -\frac{2}{3}\delta_{\max}, -\frac{1}{3}\delta_{\max}, 0, \frac{1}{3}\delta_{\max}, \frac{2}{3}\delta_{\max}, \delta_{\max}\}$ over forward step length $\Delta s = 1.0\,\text{m}$:
$$x' = x + \Delta s \cos\left(\theta + \frac{\Delta s \tan\delta}{2 L}\right)$$
$$y' = y + \Delta s \sin\left(\theta + \frac{\Delta s \tan\delta}{2 L}\right)$$
$$\theta' = \theta + \frac{\Delta s \tan\delta}{L}$$

**Node Evaluation Cost Function:**
$$f(n) = g(n) + h(n) + w_{\text{cost}} \cdot C(y', x') + w_{\text{steer}} \cdot \delta^2 + w_{\text{switch}} \cdot |\delta - \delta_{\text{prev}}|$$
Where:
* $C(y', x') \in [0, 100]$ is sampled directly from the Phase 3 ego-centric costmap.
* Cells with $C \ge 90$ are classified as hard obstacles (`HARD_BLOCK`) and immediately pruned.
* Forward search horizon is bounded to $25.0\,\text{m}$ ahead of ego to guarantee sub-$50\,\text{ms}$ latency.
* Collision-free path warm-starting is utilized when the previous path remains clear ($C < 40$).

---

### 2.2 $C^2$ Spline Curvature Smoothing & Jerk Minimization
Raw discrete waypoints from graph search are smoothed using a parametric cubic spline parameterized by normalized arc length $s \in [0, 1]$:
$$\mathbf{p}(s) = [x(s), y(s)]^T$$

Analytical curvature at each station $s$:
$$\kappa(s) = \frac{x'(s) y''(s) - y'(s) x''(s)}{\left(x'(s)^2 + y'(s)^2\right)^{3/2}}$$

**Kinematic Constraints:**
1. **Curvature Clamp:** $|\kappa(s)| \le \kappa_{\max} = 0.2222\,\text{m}^{-1}$.
2. **Lateral Acceleration Clamp:** $a_{\text{lat}}(s) = v^2 \kappa(s) \le a_{\text{lat, max}} = 2.0\,\text{m/s}^2$.
   Velocity profile scales dynamically with curvature:
   $$v_{\text{target}}(s) = \min\left(v_{\text{nominal}}, \; \sqrt{\frac{a_{\text{lat, max}}}{|\kappa(s)| + \epsilon}}\right)$$
3. **Longitudinal Jerk Minimization:** Acceleration slew is rate-limited:
   $$|j(t)| = \left|\frac{da}{dt}\right| \le 0.9\,\text{m/s}^3 < 1.0\,\text{m/s}^3$$

---

### 2.3 5-State Behavioral Arbitration with Spatial Hysteresis
Vehicle behaviors are arbitrated through a deterministic finite-state machine:
$$\mathcal{S} = \{\text{CRUISE}, \; \text{NUDGE}, \; \text{YIELD\_DECEL}, \; \text{YIELD\_WAIT}, \; \text{RESUME}\}$$

```
            ┌──────────────┐
 ┌─────────►│    CRUISE    │◄─────────┐
 │          └──────┬───────┘          │
 │                 │                  │
Clearance OK    Corridor Pinch      Clearance Restored
(W_free > 3.05) (W_free < 2.55)    (W_free > 3.05 & Obstacle Passed)
 │                 ▼                  │
 │          ┌──────────────┐          │
 ├──────────┤    NUDGE     ├──────────┤
 │          └──────┬───────┘          │
 │                 │                  │
 │          Dynamic Bottleneck        │
 │          (VSL Active)              │
 │                 ▼                  │
 │          ┌──────────────┐          │
 │          │ YIELD_DECEL  │          │
 │          └──────┬───────┘          │
 │                 │                  │
 │          v <= 0.1 m/s              │
 │          & dist_vsl <= 1.0m        │
 │                 ▼                  │
 │          ┌──────────────┐          │
 │          │  YIELD_WAIT  │          │
 │          └──────┬───────┘          │
 │                 │                  │
 │          Oncoming Cleared          │
 │          (VSL Dropped)             │
 │                 ▼                  │
 │          ┌──────────────┐          │
 └──────────┤    RESUME    ├──────────┘
            └──────────────┘
```

#### State Transition & Guard Logic:
* **`CRUISE`:** Nominal speed ($v_{\text{ref}} = 8.0\,\text{m/s}$ or configured $v_{\text{cruise}}$). Transitions to `NUDGE` when $W_{\text{free}} < 2.55\,\text{m}$ (detour required) or an obstacle is detected within the lateral corridor envelope ($d_{\text{lat}} < 1.30\,\text{m}$).
* **`NUDGE`:** Lateral detour around static hazard or wide-clearance obstacle. Speed capped at $4.0\,\text{m/s}$. Transitions to `YIELD_DECEL` immediately if `virtual_stop_active == true`.
* **`YIELD_DECEL`:** Controlled approach toward Virtual Stop Line at target deceleration $a_{\text{decel}} = -1.5\,\text{m/s}^2$ ($jerk \ge -0.8\,\text{m/s}^3$). Transitions to `YIELD_WAIT` when $v \le 0.1\,\text{m/s}$ and distance to VSL $\le 1.0\,\text{m}$.
* **`YIELD_WAIT`:** Full stop ($v = 0.0\,\text{m/s}, a = 0.0\,\text{m/s}^2$). Transitions to `RESUME` when `virtual_stop_active == false` (oncoming actor cleared pinch point).
* **`RESUME`:** Controlled acceleration at $a_{\text{accel}} = +1.0\,\text{m/s}^2$ up to $3.5\,\text{m/s}$ while clearing the pinch zone.
* **Spatial Hysteresis Unlatching:** To prevent rapid oscillation between `CRUISE` and `NUDGE`, returning to `CRUISE` strictly requires:
  $$W_{\text{free}} \ge W_{\text{crit}} + \delta_{\text{hyst}} = 2.55\,\text{m} + 0.50\,\text{m} = 3.05\,\text{m}$$
* **State Isolation Contract:** Providing `params.reset = true` resets the state to `CRUISE` cleanly without inter-test leakage.

---

### 2.4 Speed-Adaptive Pure Pursuit Tracking Controller
Given path waypoints $\mathbf{P}_{\text{path}}$, the controller computes steering angle $\delta$ and acceleration $a$:

**Speed-Scheduled Lookahead Distance:**
$$L_d(v) = \operatorname{clip}\left(K_v \cdot v_{\text{ego}}, \; L_{\min}, \; L_{\max}\right)$$
Where $K_v = 1.2\,\text{s}$, $L_{\min} = 3.0\,\text{m}$, $L_{\max} = 12.0\,\text{m}$.

**Lookahead Target Selection:**
Find lookahead target point $\mathbf{p}_{\text{target}} = [x_t, y_t]^T$ on the forward path at distance $L_d$ from the rear axle.
* **Standstill Boundary Condition:** When vehicle reaches complete stop ($v \to 0$), $L_d$ scales to $L_{\min} = 3.0\,\text{m}$. Target projection is strictly enforced along or forward of the ego heading axis to ensure heading error $\alpha$ does not flip across $\pm \pi$ when approaching zero headway to the VSL target.

**Steering Angle Command:**
$$\alpha = \operatorname{atan2}(y_t - y_{\text{ego}}, \; x_t - x_{\text{ego}}) - \theta_{\text{ego}}, \quad \alpha \in [-\pi, \pi]$$
$$\delta_{\text{cmd}} = \operatorname{atan2}\left(2 L \sin\alpha, \; L_d\right)$$

**Actuation Clamping & Slew Rate Limits:**
1. Steering saturation: $\delta_{\text{cmd}} \in [-\delta_{\max}, \delta_{\max}] = [-30^\circ, 30^\circ]$.
2. Steering slew rate limit:
   $$\left|\frac{\Delta \delta}{\Delta t}\right| \le 15^\circ/\text{s} = 0.2618\,\text{rad/s}$$
3. Longitudinal acceleration jerk limit:
   $$a_{\text{cmd}} = \operatorname{clip}\left(a_{\text{raw}}, \; a_{\text{prev}} - j_{\max} \Delta t, \; a_{\text{prev}} + j_{\max} \Delta t\right), \quad j_{\max} = 0.9\,\text{m/s}^3$$

---

## 3. Data Interface Contract

### Inputs & Outputs

#### `adaptive_path_planner.m`
* **Inputs:** `start_pose` ($[x, y, \theta]$), `goal_pose` ($[x, y, \theta]$), `costmap` ($[150 \times 300]$ matrix), `dynamic_predictions` (struct array), `grid_res` ($0.2\,\text{m}$), `grid_origin` ($[x_{\min}, y_{\min}]$).
* **Outputs:** `path` ($[N \times 2]$ smoothed coordinates), `costmap` (internal cost grid), `latency_ms` (elapsed execution time), `plan_ok` (boolean flag).

#### `behavior_state_machine.m`
* **Inputs:** `current_state` (string), `ego_state` ($[x, y, \theta, v]$), `predicted_agents` (struct array), `dt` (seconds), `params` (struct with `.W_free`, `.virtual_stop_active`, `.stop_line_dist`, `.reset`).
* **Outputs:** `new_state` (string), `v_ref` (target speed m/s), `debug_info` (diagnostics struct).

#### `pure_pursuit_controller.m`
* **Inputs:** `state` ($[x, y, \theta, v]$), `path` ($[N \times 2]$), `v_ref` (target velocity m/s), `params` (config struct with `.L`, `.k_lookahead`, `.min_lookahead`, `.max_lookahead`, `.dt`, `.prev_steer`, `.prev_accel`).
* **Outputs:** `control` ($[\delta; a]$ vector), `e_y` (cross-track error m), `e_theta` (heading error rad), `target_pt` ($[x_t, y_t]$ target point).

---

## 4. Grounding Literature References
- **Snider (CMU-RI-TR-09-08, 2009):** Automatic Steering Methods for Autonomous Automobile Path Tracking (Pure Pursuit derivation and lookahead tuning).
- **Likhachev & Ferguson (ICRA 2009):** Planning Long Dynamically Feasible Maneuvers for Autonomous Vehicles (Ackerman lattice primitive expansions).
- **Paden et al. (IEEE Trans. IV 2016):** A Survey of Motion Planning and Control Techniques for Self-Driving Urban Vehicles.
