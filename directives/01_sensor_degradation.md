# Layer 1 Directive: Sensor Degradation & Perception Synthesis

## 1. Scope & Objective
This directive governs the mathematical formulation, interfaces, and operational contracts of the synthetic perception and sensor degradation layer (`matlab/simulate_sensor_detection.m`). 

The primary role of this layer is to bridge idealized ground-truth simulation environments with the empirical sensor degradation characteristics observed on unstructured Indian roads. This forces the downstream Extended Kalman Filter (EKF) tracker (`matlab/dynamic_obstacle_predictor.m`) and behavioral motion planner to handle realistic sensing imperfections rather than relying on an oracle feed.

---

## 2. Mathematical Formulation & Algorithmic Pipeline

For every simulation tick $\Delta t = 0.1\,\text{s}$ at ego state $\mathbf{x}_{\text{ego}} = [x_0, y_0, \theta_0]^T$, an incoming set of $K$ ground-truth obstacles $\{\mathbf{O}_k\}_{k=1}^K$ with position $\mathbf{p}_k = [x_k, y_k]^T$, velocity $\mathbf{v}_k = [v_{x, k}, v_{y, k}]^T$, and ground-truth semantic class $c_k^* \in \mathcal{C}$ is processed sequentially through 6 stages:

```
Ground Truth Actors 
       │
       ▼
[1. Euclidean Range Gate]    ──> Discard if d_k > 35.0 m
       │
       ▼
[2. Horizontal FoV Gate]     ──> Discard if |θ_rel,k| > 70° (with 4-quadrant atan2 wrap)
       │
       ▼
[3. Quadratic Bernoulli Dropout] ──> Drop frame if u_k < P_drop(d_k, class)
       │
       ▼
[4. Heteroscedastic Noise]   ──> Add w_p ~ N(0, σ_p(d_k)^2 I_2), w_v ~ N(0, σ_v^2 I_2)
       │
       ▼
[5. Semantic Confusion Matrix]──> 3% mutation across Indian ODD classes
       │
       ▼
[6. 2-Tick FIFO Buffer]      ──> Buffer latency Z_out(t) = Z_in(t - 2Δt)
       │
       ▼
Delayed Noisy Detections (to EKF Tracker)
```

### Stage 1: Euclidean Distance Gate
Compute relative Euclidean distance:
$$d_k = \|\mathbf{p}_k - [x_0, y_0]^T\|_2 = \sqrt{(x_k - x_0)^2 + (y_k - y_0)^2}$$
- **Gate Condition:** The detection is discarded if $d_k > R_{\max}$, where $R_{\max} = 35.0\,\text{m}$.
- **Boundary Contract:** Targets at $d_k \le 35.0\,\text{m}$ are retained; targets at $d_k > 35.0\,\text{m}$ are strictly excluded.

### Stage 2: Azimuth Field-of-View (FoV) Gate with Branch-Cut Wrapping
Global bearing angle to target:
$$\beta_k = \operatorname{atan2}(y_k - y_0, x_k - x_0)$$
Relative bearing $\theta_{\text{rel}, k}$ with respect to ego heading $\theta_0$ wrapped cleanly to $[-\pi, \pi]$:
$$\theta_{\text{rel}, k} = \operatorname{atan2}(\sin(\beta_k - \theta_0), \cos(\beta_k - \theta_0))$$
- **Gate Condition:** The detection passes if:
$$|\theta_{\text{rel}, k}| \le \frac{\Phi}{2}, \quad \Phi = 140^\circ \implies \frac{\Phi}{2} = 70^\circ \approx 1.221730476\,\text{rad}$$
- **Branch-Cut Invariance:** Using 4-quadrant $\operatorname{atan2}(\sin\Delta, \cos\Delta)$ prevents false pruning when vehicle headings cross the $\pm \pi$ discontinuity.

### Stage 3: Non-Linear Stochastic Dropout (Bernoulli Sampling)
Target dropout probability incorporates baseline sensor limits, quadratic beam dispersal/occlusion over range, and target cross-section penalties:
$$P_{\text{drop}}(d_k) = P_{\text{base}} + (P_{\max} - P_{\text{base}})\left(\frac{d_k}{R_{\max}}\right)^2 + \delta_{\text{class}}$$
- $P_{\text{base}} = 0.05$ (5% baseline near-field dropout)
- $P_{\max} = 0.10$ (10% far-field baseline dropout)
- $R_{\max} = 35.0\,\text{m}$
- $\delta_{\text{class}} = +0.02$ for `'pedestrian'` and `'cattle'` (low Radar Cross-Section / non-rigid organic reflectors); $\delta_{\text{class}} = 0.00$ otherwise.
- For each in-gate obstacle, sample $u_k \sim \mathcal{U}(0, 1)$. If $u_k < P_{\text{drop}}(d_k)$, the detection is dropped for the current tick.

### Stage 4: Heteroscedastic Range-Dependent Measurement Noise
Measurement accuracy degrades with distance due to camera disparity resolution limits and radar beam divergence:
$$\mathbf{z}_{p, k} = \mathbf{p}_k + \mathbf{w}_{p, k}, \quad \mathbf{w}_{p, k} \sim \mathcal{N}\left(\mathbf{0}, \sigma_p^2(d_k) \mathbf{I}_2\right)$$
$$\sigma_p(d_k) = \sigma_{\min} + (\sigma_{\max} - \sigma_{\min})\left(\frac{d_k}{R_{\max}}\right)$$
- $\sigma_{\min} = 0.15\,\text{m}$ (near-field standard deviation)
- $\sigma_{\max} = 0.30\,\text{m}$ (far-field standard deviation at $35.0\,\text{m}$)
- Yields the measurement covariance $R(d_k) = \operatorname{diag}(\sigma_p^2(d_k), \sigma_p^2(d_k))$ supplied to the downstream EKF.

Velocity measurements (Doppler radar / dense optical flow):
$$\mathbf{z}_{v, k} = \mathbf{v}_k + \mathbf{w}_{v, k}, \quad \mathbf{w}_{v, k} \sim \mathcal{N}\left(\mathbf{0}, \sigma_v^2 \mathbf{I}_2\right), \quad \sigma_v = 0.20\,\text{m/s}$$

### Stage 5: Semantic Label Confusion Matrix
To model camera classifier errors caused by road dust, sun glare, and unstructured actor geometry:
$$P(\hat{c}_k \ne c_k^*) = 0.03 \quad (3\%\text{ misclassification})$$
When triggered, mutate $\hat{c}_k$ uniformly from the remaining Indian ODD class set:
$$\mathcal{C} = \{\text{'cattle'}, \text{'auto\_rickshaw'}, \text{'pedestrian'}, \text{'pushcart'}\} \setminus \{c_k^*\}$$

### Stage 6: Transport Delay (FIFO Buffer)
To reflect embedded perception computation latency (camera exposure + CNN backbone inference + clustering):
$$\Delta t_{\text{latency}} = 200\,\text{ms} = 2 \text{ ticks at } \Delta t = 0.1\,\text{s}$$
$$\mathbf{Z}_{\text{out}}(t) = \mathbf{Z}_{\text{in}}(t - 2\Delta t)$$
- For $t \le 2$, the FIFO buffer is warming up; `detections = struct([])` (empty).
- For $t > 2$, released detections represent observations captured at tick $t - 2$.

---

## 3. Data Interface Contract

### Inputs
1. `ground_truth_obs`: Struct array with fields:
   - `.id`: Unique positive integer.
   - `.type`: String matching one of `'cattle'`, `'auto_rickshaw'`, `'pedestrian'`, `'pushcart'` (or legacy `'bicycle'`, `'dog'`).
   - `.position`: $[x, y]$ vector in meters (global frame).
   - `.velocity`: $[v_x, v_y]$ vector in m/s (global frame).
   - `.behavior_profile`: String descriptor (e.g. `'erratic'`, `'weaving'`, `'nominal'`).
2. `ego_state`: Vector $[x, y, \theta]$ or $[x, y, \theta, v]$ in meters and radians.
3. `sensor_cfg`: Struct with optional configuration parameters:
   - `.max_detection_range` (default: `35.0`)
   - `.field_of_view_deg` (default: `140.0`)
   - `.p_base` (default: `0.05`)
   - `.p_max` (default: `0.10`)
   - `.std_pos_min` (default: `0.15`)
   - `.std_pos_max` (default: `0.30`)
   - `.std_vel` (default: `0.20`)
   - `.misclass_prob` (default: `0.03`)
   - `.latency_ticks` (default: `2`)
   - `.verbose` (default: `false` or `true`)
   - `.reset` (boolean flag: when `true`, clears persistent FIFO state immediately)

### Outputs
1. `detections`: Struct array with fields:
   - `.id`: Preserved obstacle ID.
   - `.type`: Possibly mutated semantic class.
   - `.position`: $[x + w_x, y + w_y]$ in meters.
   - `.velocity`: $[v_x + w_{vx}, v_y + w_{vy}]$ in m/s.
   - `.behavior_profile`: Preserved profile string.
   - `.gt_type`: Ground truth type for diagnostic metrics.
   - `.noise_std_pos`: Theoretical $\sigma_p(d_k)$ used for downstream EKF covariance $R$.
   - `.dist_to_ego`: Distance $d_k$ at observation time.
2. `sensor_log`: Struct containing diagnostic counters:
   - `.n_in_range`: Count passing range + FoV.
   - `.n_dropped`: Count dropped by Bernoulli sampling.
   - `.n_misclassed`: Count mutated by confusion matrix.
   - `.events`: Cell array of formatted log strings.

---

## 4. Edge Case Handling & Policies

1. **Exact Range Boundary ($d = 35.0\,\text{m}$):**
   - $d \le R_{\max}$ strictly accepted; $d > R_{\max}$ pruned. At $d = 35.0\,\text{m}$, noise is evaluated at $\sigma_p(35) = 0.30\,\text{m}$.
2. **Heading Discontinuity ($|\theta_0| \to \pi$):**
   - $\theta_{\text{rel}}$ must be evaluated via $\operatorname{atan2}(\sin(\beta - \theta), \cos(\beta - \theta))$ to ensure angle differences wrap smoothly across the $\pm \pi$ branch cut.
3. **Empty Scene (`ground_truth_obs = []`):**
   - Must gracefully output empty struct array `struct([])` without indexing errors.
4. **FIFO State Isolation & Resets:**
   - Persistent FIFO buffers must support instant re-initialization via `cfg.reset = true` to prevent state leakage between independent test suites or Monte Carlo runs.

---

## 5. Grounding Literature References
- **IDD (India Driving Dataset):** Varma et al., arXiv:1811.10200 (`resources/papers_md/Varma_2018_IDD_IndiaDrivingDataset_arXiv1811.10200.md`). Ground truth class distribution and occlusion characteristics.
- **DARPA Urban Challenge:** Darms, Rybski & Urmson, IEEE IVS 2008. Range-dependent radar/vision heteroscedastic noise model.
- **Motion Planning Survey:** Paden et al., 2016, arXiv:1604.07446 (`resources/papers_md/Paden_2016_MotionPlanningSurvey_arXiv1604.07446.md`). Sensor horizon & computational latency budget.
