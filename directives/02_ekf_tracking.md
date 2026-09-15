# Layer 1 Directive: Multi-Class EKF Tracking & Trajectory Rollout

## 1. Scope & Objective
This directive governs the mathematical formulation, interfaces, and operational contracts of the Multi-Class Extended Kalman Filter (EKF) tracker and trajectory rollout engine (`matlab/dynamic_obstacle_predictor.m`).

The module consumes noisy, delayed, and dropout-prone detection feeds from Phase 1 (`matlab/simulate_sensor_detection.m`), maintains consistent track identities across occlusions, filters position and Doppler velocity measurements, and rolls forward active obstacle states over a 2.0-second prediction horizon ($H = 20$ steps at $\Delta t = 0.1\,\text{s}$) with spatial $2\sigma$ uncertainty ellipses for downstream collision checking.

---

## 2. Mathematical Formulation & Algorithmic Pipeline

For every simulation tick $\Delta t = 0.1\,\text{s}$, the tracker receives an array of detections and updates active tracks through 6 core algorithmic steps:

```
Incoming Noisy Detections Z_k = [z_x, z_y, z_vx, z_vy]^T
       │
       ▼
[1. Track State & Covariance Predict]  ──> x_pred = F * x,  P_pred = F * P * F' + Q(class)
       │
       ▼
[2. Mahalanobis Gating & GNN]           ──> d_M^2 = y^T S^-1 y <= 9.488 (χ^2_4, 0.05)
       │
       ▼
[3. Joseph-Form Measurement Update]    ──> P = (I - KH) P_pred (I - KH)' + K R K'
       │
       ▼
[4. Coasting on Missed Detections]     ──> Predict-only for miss_count <= 15; prune if > 15
       │
       ▼
[5. Track Birth & Confirmation]        ──> P_0 = diag(0.25, 0.25, 4.0, 4.0)
       │
       ▼
[6. 2.0s Trajectory & Ellipse Rollout] ──> 20-step rollout with 2σ confidence ellipses
       │
       ▼
Predicted Trajectories & Covariances (to Motion Planner & BSM)
```

### Stage 1: 4D Constant Velocity (CV) Kinematic Model
Each tracked actor $i$ is modeled by a 4D state vector in Cartesian world coordinates:
$$\mathbf{x} = [p_x, p_y, v_x, v_y]^T$$

State transition over discrete sampling interval $\Delta t = 0.1\,\text{s}$:
$$\mathbf{x}_{k} = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_{k-1}$$
$$\mathbf{F} = \begin{bmatrix} 
1 & 0 & \Delta t & 0 \\ 
0 & 1 & 0 & \Delta t \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 
\end{bmatrix}$$

Measurement model: Phase 1 sensor synthesis delivers position and Doppler velocity:
$$\mathbf{z}_k = [z_x, z_y, z_{vx}, z_{vy}]^T = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{H} = \mathbf{I}_4$$

### Stage 2: Class-Conditioned Continuous White Noise Acceleration (CWNA)
Process noise $\mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$ models deviations from constant velocity. Continuous acceleration spectral density $\tilde{q}_{\text{class}} = \sigma_{a, \text{class}}^2$ integrated over $\Delta t$ yields:

$$\mathbf{Q} = \sigma_{a, \text{class}}^2 \begin{bmatrix} 
\frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0 \\ 
0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} \\ 
\frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0 \\ 
0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2 
\end{bmatrix}$$

**Class Spectral Densities ($\sigma_{a, \text{class}}$):**
- `'cattle'`: $\sigma_a = 2.2\,\text{m/s}^2$ — Erratic lateral darting and sudden stopping.
- `'auto_rickshaw'`: $\sigma_a = 1.8\,\text{m/s}^2$ — Aggressive weaving, cut-ins, and rapid speed differentials.
- `'pedestrian'`: $\sigma_a = 0.8\,\text{m/s}^2$ — High turning agility, low top speeds.
- `'pushcart'`: $\sigma_a = 0.3\,\text{m/s}^2$ — High rolling inertia, near-constant trajectory.
- Default / Unknown: $\sigma_a = 1.0\,\text{m/s}^2$.

### Stage 3: Dynamic Measurement Covariance ($\mathbf{R}$) & Joseph-Form Update
Measurement noise covariance $\mathbf{R}_k$ is injected dynamically based on observation range $d_k$:
$$\mathbf{R}_k = \operatorname{diag}\left(\sigma_p^2(d_k), \sigma_p^2(d_k), \sigma_v^2, \sigma_v^2\right)$$
where $\sigma_p(d_k) = 0.15 + 0.15\left(\frac{d_k}{35}\right)\,\text{m}$ and $\sigma_v = 0.20\,\text{m/s}$.

To guarantee numerical stability, positive semi-definiteness, and symmetry under finite-precision floating-point math, the covariance update uses the **Joseph Form** followed by explicit symmetrization:
$$\tilde{\mathbf{y}}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}$$
$$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_k$$
$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \tilde{\mathbf{y}}_k$$
$$\mathbf{P}_{k|k} = (\mathbf{I}_4 - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1} (\mathbf{I}_4 - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$
$$\mathbf{P}_{k|k} = \frac{1}{2}\left(\mathbf{P}_{k|k} + \mathbf{P}_{k|k}^T\right)$$

### Stage 4: Mahalanobis Gating & GNN Association
Incoming detections are evaluated against existing predicted tracks using the squared Mahalanobis distance:
$$d_M^2 = \tilde{\mathbf{y}}_k^T \mathbf{S}_k^{-1} \tilde{\mathbf{y}}_k$$

For a 4D Gaussian state, gating threshold $\gamma = 9.488$ corresponds to the 95% confidence interval ($\chi^2_{4, 0.05}$).
- Observations with matching `.id` are validated against $d_M^2 \le 9.488$.
- Unassociated observations and tracks are resolved using greedy Global Nearest Neighbor (GNN) on valid pairs ($d_M^2 \le 9.488$) sorted in ascending order of $d_M^2$.

### Stage 5: Track Lifecycle & Occlusion Coasting
1. **Track Birth:** An unassociated valid detection spawns a tentative track with initial covariance:
   $$\mathbf{P}_0 = \operatorname{diag}\left(0.5^2, 0.5^2, 2.0^2, 2.0^2\right) = \operatorname{diag}(0.25, 0.25, 4.0, 4.0)$$
   Tracks confirmed on second consecutive hit or when sensor provides confirmed ID.
2. **Coasting on Dropout:** When an active track receives no measurement (sensor dropout / occlusion):
   $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1}, \quad \mathbf{P}_{k|k} = \mathbf{P}_{k|k-1}$$
   $$\text{miss\_count} = \text{miss\_count} + 1$$
3. **Pruning:** A track is maintained while $\text{miss\_count} \le 15$ ticks ($1.5\,\text{s}$). If $\text{miss\_count} > 15$, the track is pruned from the registry (deleted on tick 16).
4. **Re-acquisition:** When an associated measurement arrives for a coasting track, $\text{miss\_count}$ resets to 0.

### Stage 6: 2.0-Second Trajectory Rollout & $2\sigma$ Confidence Ellipses
For each active track (both updated and coasting), project state and uncertainty forward over $H = 20$ steps ($2.0\,\text{s}$ at $\Delta t = 0.1\,\text{s}$):
$$\hat{\mathbf{x}}_{k+h} = \mathbf{F}^h \hat{\mathbf{x}}_{k|k}$$
$$\mathbf{P}_{k+h} = \mathbf{F} \mathbf{P}_{k+h-1} \mathbf{F}^T + \mathbf{Q}$$

Extract the spatial positional sub-covariance $\mathbf{P}_{\text{pos}, h} = \mathbf{P}_{k+h}(1:2, 1:2)$:
- Compute eigenvalues $\lambda_1 \ge \lambda_2 \ge 0$ and eigenvectors of $\mathbf{P}_{\text{pos}, h}$.
- $2\sigma$ semi-major axis: $a_h = 2\sqrt{\lambda_1}$.
- $2\sigma$ semi-minor axis: $b_h = 2\sqrt{\lambda_2}$.
- Ellipse orientation angle: $\phi_h = \operatorname{atan2}(v_{y, 1}, v_{x, 1})$.

---

## 3. Data Interface Contract

### Inputs
1. `observations`: Struct array from `simulate_sensor_detection.m`:
   - `.id`: Unique integer agent ID.
   - `.type`: Class string (`'cattle'`, `'auto_rickshaw'`, `'pedestrian'`, `'pushcart'`).
   - `.position`: $[x, y]$ vector in meters.
   - `.velocity`: $[v_x, v_y]$ vector in m/s.
   - `.behavior_profile`: Profile descriptor string.
   - `.noise_std_pos`: Range-dependent position noise $\sigma_p(d_k)$ (optional, computed if missing).
   - `.dist_to_ego`: Distance to ego in meters (optional, computed if missing).
2. `dt`: Timestep in seconds (default: `0.1`).
3. `N_horizon`: Number of forward rollout steps (default: `20`).
4. `cfg`: Optional configuration struct:
   - `.reset`: Boolean flag. When `true`, resets persistent track registry.
   - `.gating_threshold`: Chi-squared threshold (default: `9.488`).
   - `.max_coast_ticks`: Max missed ticks before deletion (default: `15`).

### Outputs
1. `predicted_trajectories`: Struct array containing:
   - `.id`: Track integer ID.
   - `.type`: Semantic class string.
   - `.waypoints`: $(H \times 2)$ array of $[x, y]$ future coordinates.
   - `.covariance`: $(H \times 1)$ cell array of $(4 \times 4)$ future covariance matrices.
   - `.semi_major`: $(H \times 1)$ vector of $2\sigma$ semi-major axes ($a_h$).
   - `.semi_minor`: $(H \times 1)$ vector of $2\sigma$ semi-minor axes ($b_h$).
   - `.orientation`: $(H \times 1)$ vector of ellipse orientations in radians.
   - `.x_est`: $(4 \times 1)$ filtered state estimate $[p_x, p_y, v_x, v_y]^T$.
   - `.P_est`: $(4 \times 4)$ filtered covariance matrix.
   - `.is_coasting`: Boolean flag (`true` if track coasted this frame).
2. `innov_stats`: Struct array containing per-track innovation diagnostics:
   - `.id`: Track ID.
   - `.innov`: $(4 \times 1)$ innovation vector.
   - `.innov_norm`: Scalar norm of innovation.
   - `.predict_only`: Boolean flag (`true` if coasting without measurement update).

---

## 4. Grounding Literature References
- **Darms, Rybski & Urmson (IEEE IVS 2008):** Continuous White Noise Acceleration model, Joseph-form covariance stability, and Mahalanobis gating.
- **Argoverse Motion Forecasting Dataset:** 2.0-second ($H = 20$) horizon standard and non-linear multi-agent uncertainty growth (`resources/web_docs/argoverse_overview.md`).
- **Varma et al. (IDD 2018):** Class-specific maneuver dynamics, acceleration variances, and road user mobility hierarchy (`resources/papers_md/Varma_2018_IDD_IndiaDrivingDataset_arXiv1811.10200.md`).
- **Paden et al. (2016):** Motion planning survey on occupancy envelopes and kinematic state representation (`resources/papers_md/Paden_2016_MotionPlanningSurvey_arXiv1604.07446.md`).
