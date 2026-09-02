# Autonomous Driving Algorithms - Integration Guide & Reference Map

This document provides a comprehensive architectural audit, mathematical breakdown, and porting roadmap for path planning, obstacle behavior deciders, velocity profiling, and vehicle controllers extracted from **Baidu Apollo**, **Autoware Universe**, and **PythonRobotics**.

---

## 1. Summary Integration Reference Map

| Component | Source Repository | File Path in Repo | Algorithm / Math Model | Porting Target (JS / MATLAB) |
| :--- | :--- | :--- | :--- | :--- |
| **S-T Graph Speed & Collision Deciders** | Baidu Apollo | `modules/planning/tasks/deciders/speed_decider/speed_decider.cc` | $S$-$T$ Boundary Mapping & Pass/Yield Region Search | `web_simulation/index.html` |
| **Piecewise Jerk Path & Speed QP** | Baidu Apollo | `modules/planning/tasks/optimizers/piecewise_jerk_speed/` | Quadratic Programming ($L_2$ Jerk & Accel Minimization) | `matlab/adaptive_path_planner.m` |
| **Station-Decoupled PID & Feedforward** | Baidu Apollo | `modules/control/controller/lon_controller.cc` | Dual-loop PID with deadband & acceleration feedforward | `matlab/pure_pursuit_controller.m` |
| **LQR Lateral Steering Controller** | Baidu Apollo | `modules/control/controller/lat_controller.cc` | 4-State LQR with curvature feedforward $\delta_{\text{ff}}$ | `matlab/pure_pursuit_controller.m` |
| **Virtual Stop Line & Yielding Module** | Autoware Universe | `planning/behavior_velocity_planner/src/scene_module/` | State Machine & Margin Stop Line Insertion | `web_simulation/index.html` |
| **Jerk-Constrained Velocity Smoother** | Autoware Universe | `planning/motion_velocity_smoother/src/` | Forward-Backward Filtering ($a_{\max}, j_{\max}$) | `matlab/adaptive_path_planner.m` |
| **Model Predictive Trajectory (MPT)** | Autoware Universe | `planning/obstacle_avoidance_planner/src/mpt_optimizer.cpp` | Convex Corridor Bounds & Vehicle Footprint Circles | `matlab/adaptive_path_planner.m` |
| **Frenet Optimal Trajectory Planner** | PythonRobotics | `PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py` | Quintic $s$-$d$ Polynomials & Cost Minimization | `matlab/adaptive_path_planner.m` |
| **Cubic Spline Curvature Generator** | PythonRobotics | `PathPlanning/CubicSpline/cubic_spline_planner.py` | $C^2$ Parametric Spline & Curvature $\kappa(s)$ | `web_simulation/index.html` |
| **Stanley Front-Axle Lateral Tracking** | PythonRobotics | `PathTracking/stanley_controller/stanley_controller.py` | Front-axle cross-track error + yaw steering | `matlab/pure_pursuit_controller.m` |
| **Dynamic Window Approach (DWA)** | PythonRobotics | `PathPlanning/DynamicWindowApproach/dynamic_window_approach.py` | Velocity Space Search $(v, \omega)$ under accel limits | `web_simulation/index.html` |

---

## Deep-Dive Implementations & Mathematical Formulations

### 1. Baidu Apollo: S-T Graph Speed Decider
- **Source Location:** `modules/planning/tasks/deciders/speed_decider/speed_decider.cc`
- **Key Functions / Classes:** `StBoundaryMapper`, `SpeedDecider::MakeObjectDecision()`
- **Mathematical Principle:**
  Maps dynamic obstacles onto the Station-Time ($S$-$T$) plane by projecting obstacle boundary polygons along the ego reference line $s(t)$. For each obstacle $k$, an $S$-$T$ region $ST_k = \{(s, t) \mid s_{\text{lower}}(t) \le s \le s_{\text{upper}}(t), t \in [0, T]\}$ is created.
  The decider evaluates whether the ego trajectory should pass **ABOVE** (Overtake) or **BELOW** (Yield) the region:
  $$\text{Yield Decision: } s_{\text{ego}}(t) \le s_{\text{lower}}(t) - d_{\text{buffer}}$$

- **Standalone JavaScript Port (`web_simulation/index.html`):**
```javascript
class ApolloSTGraphDecider {
    constructor(bufferMeters = 3.5) {
        this.buffer = bufferMeters;
    }
    evaluateYield(egoStation, egoSpeed, obstaclePath, horizon = 5.0, dt = 0.1) {
        let minStopStation = Infinity;
        let yieldRequired = false;

        for (let t = 0; t <= horizon; t += dt) {
            const obsPos = obstaclePath.predict(t);
            const obsStation = obstaclePath.getStation(obsPos);
            const egoPredStation = egoStation + egoSpeed * t;

            if (obsStation > egoStation && Math.abs(egoPredStation - obsStation) < this.buffer) {
                yieldRequired = true;
                minStopStation = Math.min(minStopStation, obsStation - this.buffer);
            }
        }
        return { yieldRequired, stopStation: yieldRequired ? minStopStation : null };
    }
}
```

---

### 2. Baidu Apollo: Piecewise Jerk Quadratic Programming (QP) Speed Profiler
- **Source Location:** `modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.cc`
- **Key Functions / Classes:** `PiecewiseJerkSpeedProblem::Optimize()`
- **Mathematical Principle:**
  Formulates longitudinal velocity optimization as a Quadratic Program minimizing weighted displacement deviation, velocity tracking error, acceleration, and jerk:
  $$\min_{x, \dot{x}, \ddot{x}} \sum_{i=0}^{N-1} \left( w_s (x_i - s_{\text{ref}, i})^2 + w_v (\dot{x}_i - v_{\text{ref}, i})^2 + w_a \ddot{x}_i^2 + w_j \left(\frac{\ddot{x}_{i+1} - \ddot{x}_i}{\Delta t}\right)^2 \right)$$
  Subject to physical constraints:
  $$x_i + \dot{x}_i \Delta t \le x_{i+1}, \quad a_{\min} \le \ddot{x}_i \le a_{\max}, \quad j_{\min} \le \frac{\ddot{x}_{i+1} - \ddot{x}_i}{\Delta t} \le j_{\max}$$

- **Standalone MATLAB Port (`matlab/adaptive_path_planner.m`):**
```matlab
function [s, v, a] = optimize_piecewise_jerk_speed(s_ref, v_ref, s_bounds, dt)
    N = length(s_ref);
    % State vector X = [s_0..s_N, v_0..v_N, a_0..a_N]
    H = blkdiag(10*eye(N), 5*eye(N), 1*eye(N));
    f = [-10*s_ref(:); -5*v_ref(:); zeros(N,1)];
    
    % Linear equality & inequality constraints
    Aeq = []; beq = [];
    A = []; b = [];
    
    lb = [s_bounds(:,1); zeros(N,1); -4.5*ones(N,1)];
    ub = [s_bounds(:,2); 15*ones(N,1); 2.5*ones(N,1)];
    
    X = quadprog(H, f, A, b, Aeq, beq, lb, ub);
    s = X(1:N); v = X(N+1:2*N); a = X(2*N+1:3*N);
end
```

---

### 3. Autoware Universe: Virtual Stop Lines & Crosswalk Module
- **Source Location:** `planning/behavior_velocity_planner/src/scene_module/crosswalk/scene_crosswalk.cpp`
- **Key Functions / Classes:** `CrosswalkModule::modifyPathVelocity()`
- **Mathematical Principle:**
  Inserts a virtual stop line at $s_{\text{stop}} = s_{\text{crosswalk}} - d_{\text{margin}}$ whenever a pedestrian or animal is inside or approaching the attention area. Calculates a smooth deceleration profile:
  $$v_{\text{target}}(s) = \sqrt{\max\left(0, v_{\text{curr}}^2 - 2 \cdot a_{\text{decel}} \cdot (s_{\text{stop}} - s)\right)}$$

- **Standalone JavaScript Port (`web_simulation/index.html`):**
```javascript
function applyVirtualStopLine(ego, stopStation, marginMeters = 3.5) {
    const distToStop = stopStation - ego.station - marginMeters;
    if (distToStop <= 0.5) {
        return { targetSpeed: 0, accel: -3.5, state: "YIELD_WAIT" };
    }
    const decelRequired = (ego.vx * ego.vx) / (2 * distToStop);
    return {
        targetSpeed: 0,
        accel: -Math.min(4.5, Math.max(1.5, decelRequired)),
        state: "YIELD_DECEL"
    };
}
```

---

### 4. PythonRobotics: Frenet Optimal Trajectory Planner
- **Source Location:** `PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py`
- **Key Functions / Classes:** `frenet_optimal_planning()`, `FrenetPath`
- **Mathematical Principle:**
  Generates candidate trajectories in Frenet frame coordinates $(s, d)$. Lateral offset $d(t)$ is represented as a 5th-degree polynomial (Quintic):
  $$d(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5$$
  Longitudinal position $s(t)$ is represented as a 4th-degree polynomial (Quantic) for speed keeping.
  Cost function for trajectory selection:
  $$C = k_j J_d + k_t T + k_d d_{D}^2 + k_v (v_D - v_{\text{target}})^2$$
  Where $J_d = \int \dddot{d}(t)^2 dt$ is lateral jerk.

- **Standalone MATLAB Port (`matlab/adaptive_path_planner.m`):**
```matlab
function path = generate_frenet_trajectory(s0, c_speed, d0, d_target, T)
    % Quintic polynomial solver for lateral movement d(t)
    A = [T^3 T^4 T^5; 3*T^2 4*T^3 5*T^4; 6*T 12*T^2 20*T^3];
    b = [d_target - d0; 0; 0];
    x = A \ b;
    
    t = 0:0.1:T;
    path.d = d0 + x(1)*t.^3 + x(2)*t.^4 + x(3)*t.^5;
    path.s = s0 + c_speed*t;
end
```

---

### 5. PythonRobotics: Stanley Steering Controller
- **Source Location:** `PathTracking/stanley_controller/stanley_controller.py`
- **Key Functions / Classes:** `stanley_control()`
- **Mathematical Principle:**
  Calculates lateral steering angle $\delta(t)$ using front-axle position reference:
  $$\delta(t) = \theta_e(t) + \arctan\left(\frac{k \cdot e_{fa}(t)}{v(t) + \epsilon}\right)$$
  Where:
  - $\theta_e(t) = \theta_{\text{path}} - \theta_{\text{vehicle}}$ is path heading error.
  - $e_{fa}(t)$ is lateral cross-track error measured from the front axle to the nearest path segment.
  - $k$ is gain parameter.

- **Standalone MATLAB Port (`matlab/pure_pursuit_controller.m`):**
```matlab
function delta = stanley_controller(state, path, k, k_soft)
    front_x = state.x + state.L * cos(state.yaw);
    front_y = state.y + state.L * sin(state.yaw);
    
    [nearest_pt, path_yaw, e_fa] = find_nearest_front_axle(front_x, front_y, path);
    
    yaw_error = path_yaw - state.yaw;
    yaw_error = atan2(sin(yaw_error), cos(yaw_error)); % normalize
    
    cross_track_steering = atan2(k * e_fa, state.v + k_soft);
    delta = yaw_error + cross_track_steering;
end
```

---

## 3. Integration Roadmap & Verification Checkpoints

1. **Phase 1: Web Studio (JavaScript)**
   - ✅ Integrated `STGraphSpeedDecider` with hysteresis state latching.
   - ✅ Implemented Nudge vs. Yield decision logic.
   - ✅ Added real-time CSV exporter & Chart.js dynamic plots.

2. **Phase 2: MATLAB Engine (`matlab/`)**
   - 🔄 Incorporate `optimize_piecewise_jerk_speed` into `adaptive_path_planner.m`.
   - 🔄 Add `stanley_controller` as a dual lateral controller option alongside Pure Pursuit in `pure_pursuit_controller.m`.
   - 🔄 Run full batch evaluation across all 5 Indian road scenarios using `plot_simulation_results.m`.
