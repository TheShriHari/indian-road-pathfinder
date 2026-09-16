# Phase 5 Directive: Overnight Autonomous Monte Carlo Batch, Failure Mining & Auto-Tuning Engine

## 1. Scope & Objective
This directive governs the autonomous Phase 5 multi-stage closed-loop validation and hyperparameter auto-tuning pipeline for the SIH PS-26037 Indian Road Autonomous Pathfinder. The pipeline executes 1,000-trial Monte Carlo batches across 5 randomized Indian road domains, mines and triages failure modes, runs coordinate descent hyperparameter optimization on kinematic planner and controller parameters, verifies the upgraded parameter set on a fresh 1,000-trial batch, and packages telemetry for web visual playback.

---

## 2. 5-Domain Scenario Parameter Randomization
Every 1,000-trial batch evaluates exactly 200 randomized seeds across 5 distinct operational design domains ($k \in \{1, \dots, 200\}$ per domain, indexed $1 \dots 1000$ overall):

1. **Scenario 1: Unmarked Rural Road (Seeds 1–200)**
   * Usable pavement width $W \sim \mathcal{U}(3.2, 4.5)\text{ m}$ (eroded shoulders: $y \in [-W/2, +W/2]$).
   * Potholes: $N_{\text{pot}} \in \{2, 3, 4\}$, radius $R \sim \mathcal{U}(0.4, 0.9)\text{ m}$, longitudinally distributed $x \in [12.0, 50.0]\text{ m}$.
   * Oncoming tractor: position $x_0 \sim \mathcal{U}(40.0, 55.0)\text{ m}$, velocity $v_x \sim \mathcal{U}(-6.0, -3.0)\text{ m/s}$, lateral weaving $v_y \sim \mathcal{U}(-0.2, 0.2)\text{ m/s}$.

2. **Scenario 2: Signal-less Urban Intersection (Seeds 201–400)**
   * Crossing auto-rickshaws (2 agents):
     * Agent 1: $x_0 \sim \mathcal{U}(25.0, 35.0)\text{ m}, y_0 \sim \mathcal{U}(6.0, 9.0)\text{ m}, v_y \sim \mathcal{U}(-8.0, -4.0)\text{ m/s}$.
     * Agent 2: $x_0 \sim \mathcal{U}(35.0, 48.0)\text{ m}, y_0 \sim \mathcal{U}(-1.8, -0.8)\text{ m}, v_x \sim \mathcal{U}(-4.5, -2.5)\text{ m/s}$.
   * Jaywalking pedestrian: $x_0 \sim \mathcal{U}(22.0, 30.0)\text{ m}, y_0 \sim \mathcal{U}(-5.0, -3.0)\text{ m}, v_y \sim \mathcal{U}(0.8, 1.4)\text{ m/s}$.

3. **Scenario 3: Highway Merge with Slow Moving Vehicles (Seeds 401–600)**
   * Lead vehicle (high speed): $x_0 \sim \mathcal{U}(30.0, 45.0)\text{ m}$, velocity $v \sim \mathcal{U}(16.0, 22.0)\text{ m/s}$.
   * Merging truck / auto: enters ramp at $x \sim \mathcal{U}(20.0, 32.0)\text{ m}$ with acceleration $a \sim \mathcal{U}(-1.0, 0.5)\text{ m/s}^2$ and initial speed $v \sim \mathcal{U}(3.0, 6.0)\text{ m/s}$.

4. **Scenario 4: Dense Market Squeeze (Seeds 601–800)**
   * Lateral corridor width $W \sim \mathcal{U}(2.4, 3.2)\text{ m}$ flanked by stalls and static obstacles.
   * 3 Pushcarts: speed $v \sim \mathcal{U}(0.5, 1.2)\text{ m/s}$ moving along road shoulders.
   * 4 Pedestrians: cross-walk trajectories at random angles $\theta \sim \mathcal{U}(-\pi/4, \pi/4)$ and speed $v \sim \mathcal{U}(0.6, 1.3)\text{ m/s}$.

5. **Scenario 5: Sudden Cattle Crossing (Seeds 801–1000)**
   * Cattle obstacle stepping abruptly into corridor from foliage at distance $d \sim \mathcal{U}(8.0, 16.0)\text{ m}$ ahead of ego.
   * Lateral wander velocity $v_y \sim \mathcal{U}(0.8, 1.8)\text{ m/s}$ with unpredictable stop/turn behavior.

---

## 3. Failure Categorization Taxonomy & Root Cause Triage
Any simulation run that does not achieve `SUCCESS` with all KPI bounds satisfied is isolated and classified:

* **`COLLISION`**:
  * Vehicle geometric footprint intersects lethal cost $C = 100$ (static obstacle/pothole rim penetration $d_{\text{edge}} < 0\text{ m}$ or dynamic agent boundary distance $d_{\text{agent}} < 0.55\text{ m}$).
* **`DEADLOCK`**:
  * Vehicle speed drops below $0.1\text{ m/s}$ outside of a Virtual Stop Line (VSL) yield zone for $> 5.0\text{ s}$ ($> 50$ consecutive steps).
* **`KINEMATIC_VIOLATION`**:
  * Steering actuation rate $|\dot{\delta}| > 15^\circ/\text{s}$ ($0.2618\text{ rad/s}$) or path curvature $|\kappa(s)| > 0.2222\text{ m}^{-1}$ ($R_{\min} < 4.5\text{ m}$).
* **`JERK_EXCESS`**:
  * Longitudinal or lateral jerk $|j(t)| = |\dot{a}(t)| > 1.0\text{ m/s}^3$.
* **`CHATTER` / `OSCILLATION`**:
  * Rapid state alternation in FSM (e.g., $NUDGE \leftrightarrow CRUISE$) exceeding 3 state transitions within $1.5\text{ s}$ without spatial unlatching.

---

## 4. Crash Recovery & Safe Headless Execution Rules
1. **Per-Trial Error Isolation:**
   Each trial is wrapped in an internal MATLAB `try-catch` block. If an unhandled numerical or geometric exception occurs, the error stack trace is serialized to `.tmp/failures/error_seed_<seed>.log` and recorded as outcome `ERROR` without crashing the batch.
2. **Resource Management:**
   Computational thread pool is explicitly capped via `maxNumCompThreads(6);` at batch initialization to eliminate thread contention and thermal throttling on 6-core Ryzen 5 7530U processors.
3. **Session Persistence:**
   MATLAB sessions are maintained continuously across each stage. No per-trial process spawning is permitted.

---

## 5. Hyperparameter Coordinate Descent Optimization
The optimization engine sweeps hyperparameter vector $\mathbf{\theta} = [K_v, L_{\min}, w_{\text{steer}}, \delta_{\text{hyst}}, \alpha_{\text{cost}}]$ to minimize:

$$J(\mathbf{\theta}) = w_1 (1 - \text{CompletionRate}) + w_2 \frac{\bar{j}}{1.0} + w_3 \frac{\bar{\tau}_{\text{replan}}}{50.0} + w_4 \max\left(0, \, 0.8 - d_{\min}\right)$$

Where:
* Weights: $w_1 = 100.0, w_2 = 15.0, w_3 = 10.0, w_4 = 50.0$.
* **$K_v$ (Pure Pursuit Speed Lookahead Gain):** $\in [1.0, 1.5]\text{ s}$ (nominal: $1.2\text{ s}$).
* **$L_{\min}$ (Minimum Lookahead Distance):** $\in [3.0, 4.0]\text{ m}$ (nominal: $3.2\text{ m}$).
* **$w_{\text{steer}}$ (Hybrid A* Steering Penalty Weight):** $\in [0.02, 0.10]$ (nominal: $0.06$).
* **$\delta_{\text{hyst}}$ (FSM Spatial Hysteresis Unlatch Margin):** $\in [0.35, 0.70]\text{ m}$ (nominal: $0.50\text{ m}$).
* **$\alpha_{\text{cost}}$ (Costmap Exponential Decay Rate):** $\in [2.2, 3.1]\text{ m}^{-1}$ (nominal: $2.50\text{ m}^{-1}$).

Upon convergence, optimal parameters $\mathbf{\theta}^*$ are serialized to `matlab/best_params.mat` and verified on a fresh 1,000-trial batch.
