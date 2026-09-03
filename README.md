# Adaptive Path Planning & Collision Avoidance for Unstructured Indian Roads

**Smart Vehicles Theme · MathWorks Problem Statement ID: 26037**

---

> **METRICS TABLE NOTE:** The performance numbers in Section 4 are marked
> **"to be measured"** for any value not yet printed from an actual MATLAB run.
> Once you run `main_simulation` in MATLAB Online, paste the printed output
> here and replace the placeholders. No numbers were invented or estimated.

---

## 1. Executive Summary

Autonomous driving in India presents unique challenges: unstructured traffic, missing lane markings,
high agent diversity (cattle, auto-rickshaws, pushcarts, pedestrians), narrow village roads, and
sudden dynamic obstacles. This repository contains a **genuinely algorithmic, map-agnostic** closed-loop
simulation pipeline that dynamically adapts to **ANY 3D CARLA world, road geometry, or obstacle layout**
without prior coordinate knowledge.

---

## 2. System Architecture

```
+--------------------------------------------------------------------------------+
|  1. Online Perception & Local Rolling Grid   local_occupancy_grid_builder.m    |
|     Sensor-driven rolling costmap (0-255) built online; ZERO hardcoded maps.   |
|     Extracts road boundaries, potholes, and static clutter dynamically.       |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  2. Dynamic Trajectory Predictor   dynamic_obstacle_predictor.m                |
|     REAL Extended Kalman Filter: state [x,y,vx,vy], F/H matrices,             |
|     predict+update steps, per-agent persistent covariance via containers.Map   |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  3. Spatio-Temporal Corridor Analyzer   universal_bottleneck_decider.m         |
|     Evaluates continuous traversable width W_free(s) along planned horizon.    |
|     Injects dynamic Virtual Stop Line if W_free(s) < W_car + 2*margin.         |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  4. Adaptive Path Planner   adaptive_path_planner.m                            |
|     REAL Hybrid A* grid search: bicycle-model arc expansion, 5 steer options,  |
|     matrix-based open/closed sets, costmap soft/hard penalty, pchip smoothing  |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  5. Behavioral Decision Layer   behavior_state_machine.m                       |
|     CRUISE / NUDGE / YIELD_DECEL / YIELD_WAIT / RESUME state machine          |
|     Dynamic hysteresis and clearance-aware speed modulation.                   |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  6. Motion Control   pure_pursuit_controller.m                                 |
|     Standard Pure Pursuit lateral control + P-gain longitudinal speed control  |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  7. Vehicle Dynamics / CARLA Actuation   vehicle_kinematics.m / bridge        |
|     Kinematic bicycle model: [ẋ, ẏ, θ̇, v̇] = f(state, [delta, a])             |
+--------------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------------+
|  7. Evaluation   evaluate_metrics.m                                            |
|     Latency, jerk, smoothness, min clearance — computed from real logged data  |
|     Completion checked against actual scenario goal_pose, not a hardcoded value|
+--------------------------------------------------------------------------------+
```

---

## 3. Five Mandated Test Scenarios

| # | Scenario | Challenge | Key Agent |
|---|---|---|---|
| 1 | **Unmarked Village Road** | Missing lane markings, potholes, roaming cattle | Cattle (erratic) + Auto-rickshaw |
| 2 | **Signal-less Urban Intersection** | Unregulated 4-way crossing, multi-directional flow | 2× Auto-rickshaw + Pedestrian |
| 3 | **Highway Merge with Slow Vehicles** | High speed differential, blocked merge lane | Pushcart + Auto-rickshaw |
| 4 | **Dense Market Area** | Sub-metre clearance, crowded pedestrian density | 3× Pedestrian + Pushcart |
| 5 | **Sudden Cattle Crossing** | High-speed lateral bolt from roadside | Cattle (high-vy) |

---

## 4. Performance Metrics (from real MATLAB run)

| Metric | Target | Measured |
|---|---|---|
| Replanning Latency (avg) | < 50 ms | **to be measured** |
| Path Smoothness (jerk) | < 1.0 m/s³ | **to be measured** |
| Min Safety Clearance | > 0.8 m | **to be measured** |
| Scenario Completion Rate | 100% | **to be measured** |

> Replace "to be measured" values by running `main_simulation` in MATLAB Online
> and copying the printed `evaluate_metrics` table lines here.

---

## 5. How to Run

### MATLAB Simulation (primary — actual algorithms)

```matlab
% Navigate to the matlab/ directory
cd matlab/

% Run all 5 scenarios with full metrics
main_simulation

% Component-level tests
test_ekf_convergence       % EKF: prints per-step truth vs estimate vs covariance trace
test_hybrid_astar_detour   % A*:  prints all path waypoints, confirms detour around wall
test_state_machine         % BSM: prints state transitions CRUISE->NUDGE->YIELD_DECEL->...
```

### CARLA Co-Simulation Mode

To run closed-loop pathfinding against a CARLA 3D rural road:

1. **Start CARLA Simulator** (e.g. `CarlaUE4.exe -quality-level=Low`).
2. **Start the Python Bridge Server** (in project root):
   ```bash
   python carla_bridge.py --host 127.0.0.1 --port 2000 --bridge-port 20000
   ```
3. **In MATLAB**, open `matlab/carla_simulation_bridge.m`:
   - Set `MODE = 'LIVE_CARLA'` to receive live telemetry and transmit steering/throttle commands.
   - Or keep `MODE = 'MOCK_SOCKET'` to verify the full telemetry/actuator loop offline.
   - Run:
   ```matlab
   carla_simulation_bridge
   ```

**Requirements:** MATLAB base install (no Automated Driving Toolbox, Navigation Toolbox,
or Robotics System Toolbox required — all algorithms are self-contained).

### Web Visualizer (secondary — illustrative presentation tool only)

```bash
python server.py   # then open http://localhost:8000
```

> ⚠️ **The web visualizer is an independent illustrative prototype for presentation purposes.**
> It does NOT run any MATLAB code, does not execute Hybrid A*, EKF, or vehicle_kinematics,
> and does NOT produce the metrics shown in Section 4. It mirrors the scenario names and BSM
> state labels from the MATLAB codebase for visual consistency only. The two tools are not
> connected in any way.

---

## 6. File Inventory

| File | Purpose | Algorithm |
|---|---|---|
| `generate_scenarios.m` | 5 test scenarios | Occupancy grid construction |
| `dynamic_obstacle_predictor.m` | Agent trajectory prediction | **Real EKF** (predict+update) |
| `adaptive_path_planner.m` | Path planning | **Real Hybrid A*** (grid search) |
| `behavior_state_machine.m` | Behavioral decisions | State machine (CRUISE/NUDGE/YIELD/RESUME) |
| `pure_pursuit_controller.m` | Lateral/longitudinal control | Pure Pursuit (unchanged) |
| `vehicle_kinematics.m` | Vehicle state integration | Kinematic bicycle model (unchanged) |
| `evaluate_metrics.m` | Metrics computation | From real logged simulation data |
| `main_simulation.m` | Closed-loop integration | Runs all 5 scenarios end-to-end |
| `test_ekf_convergence.m` | EKF verification | Convergence on straight-line track |
| `test_hybrid_astar_detour.m` | A* verification | Wall-obstacle detour test |
| `test_state_machine.m` | BSM verification | State transition sequence test |

---

## 7. Attribution & Open-Source References

### Hybrid A* Path Planner

Adapted from two files in **PythonRobotics** (Atsushi Sakai et al., MIT License):

- **`PythonRobotics/PathPlanning/HybridAStar/hybrid_a_star.py`** — node structure
  (x_ind, y_ind, yaw_ind), A* open/closed set pattern, priority-queue-based expansion,
  parent-chain path reconstruction, `calc_cost()` with g + H_COST × h design.
- **`PythonRobotics/PathPlanning/HybridAStar/car.py`** — `move()` bicycle kinematic
  integration function (x += dist·cos(yaw); y += dist·sin(yaw);
  yaw += dist·tan(steer)/WB) adapted directly into the arc-expansion inner loop.

**Key differences from the reference (not just renamed):**
- No Reeds-Shepp analytic expansion (avoids external dependency unavailable in MATLAB Online)
- 5 fixed steer options ±30°, ±15°, 0° instead of 20 uniformly spaced options
- MATLAB matrix arrays used for closed/g-cost sets instead of Python dicts (performance)
- Costmap-based soft/hard traversal cost added (not in reference)
- Goal check is position-only, not heading-matched (appropriate for our use case)

### Extended Kalman Filter Predictor

Adapted from **PythonRobotics**:

- **`PythonRobotics/Localization/extended_kalman_filter/extended_kalman_filter.py`**
  (author: Atsushi Sakai, @Atsushi_twi, MIT License) — predict/update cycle structure,
  innovation computation (y = z - H·x_pred), Kalman gain (K = P·Hᵀ·S⁻¹), state update
  (x_est = x_pred + K·y), covariance update.

**Key differences:**
- State vector changed from [x, y, yaw, v] to [x, y, vx, vy] (constant-velocity model;
  no control input because we observe agents, not control them)
- Linear F and H matrices replace Jacobians (CV model is already linear; no EKF linearisation needed)
- Per-agent persistent state via `containers.Map` (Python reference used local variables)
- Per-agent-type process noise Q tuning added
- Joseph-form covariance update for numerical stability
- N_horizon forward propagation added for trajectory prediction output

### Behavioral State Machine

Conceptually inspired by the **yield/nudge/resume state-machine pattern** in
**Autoware Universe's** `behavior_velocity_planner`
(`autoware_universe/planning/behavior_velocity_planner/`).
Specifically: the idea of computing longitudinal/lateral distance to predicted agent
positions to drive state transitions, and the YIELD_WAIT → RESUME → CRUISE recovery sequence.

> **Important:** This is an **independent MATLAB reimplementation** from first principles.
> No Autoware source code was copied, ported, or directly translated.
> Autoware is referenced as architectural inspiration only.

### Cubic Spline Smoothing

The pchip spline control-point deformation in `adaptive_path_planner.m` (Section 3 of
that file) is unchanged from the original project's smoothing code, which was inspired by
cubic spline curvature equations from **PythonRobotics/PathPlanning/CubicSpline/**.

---

## 8. What Is Genuinely Implemented vs. What Was Previously a Placeholder

| Component | Previous Version | This Version |
|---|---|---|
| Path planning | Straight-line lerp + pchip smoothing | Real Hybrid A* grid search with arc expansion |
| Obstacle prediction | `switch`-case with raw `randn()` noise | EKF with predict/update cycle, persistent P |
| Behavioral decisions | Inline distance threshold in main loop | Dedicated state machine with 5 states + hysteresis |
| Metrics goal check | Hardcoded `final_x >= 50.0` | Actual `norm(pos - goal_pose)` comparison |
| Web visualizer claim | Implied integration with MATLAB | Explicitly labelled as standalone illustrative tool |
