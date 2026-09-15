# Layer 1 Directive: Rolling Costmap & Universal Bottleneck Decider

## 1. Scope & Objective
This directive governs the mathematical formulation, geometric representations, and arbitration logic of:
1. The zero-toolbox ego-centric local costmap builder (`matlab/local_occupancy_grid_builder.m`).
2. The forward corridor width profiler and bottleneck decider (`matlab/universal_bottleneck_decider.m`).

The module eliminates all dependencies on external toolboxes (e.g. Navigation Toolbox `binaryOccupancyMap`), creates a $150 \times 300$ grid ($0.2\,\text{m}$ resolution) covering $X \in [-10, +50]\,\text{m}$ and $Y \in [-15, +15]\,\text{m}$, applies continuous exponential inflation fields, profiles lateral free corridor widths $W_{\text{free}}(s)$ along forward stations $s \in [0, 30]\,\text{m}$, and arbitrates among static detour replanning, dynamic Virtual Stop Line (VSL) holding, and complete road blockage halts.

---

## 2. Mathematical Formulation & Algorithmic Pipeline

### 2.1 Zero-Toolbox Ego-Centric Grid Discretization
* **Longitudinal Span:** $X \in [-10.0, +50.0]\,\text{m}$ ($L_x = 60.0\,\text{m}$)
* **Lateral Span:** $Y \in [-15.0, +15.0]\,\text{m}$ ($L_y = 30.0\,\text{m}$)
* **Grid Resolution:** $\Delta g = 0.2\,\text{m/cell}$
* **Matrix Dimensions:** $N_y \times N_x = 150 \times 300$, initialized to zero cost $\mathbf{C} = \mathbf{0}_{150 \times 300}$.

**Coordinate-to-Matrix Indexing (Local Frame $\to$ 1-Indexed Row/Col):**
$$\operatorname{col}(x) = \operatorname{clip}\left(\left\lfloor \frac{x - (-10.0)}{\Delta g} \right\rfloor + 1, \, 1, \, 300\right)$$
$$\operatorname{row}(y) = \operatorname{clip}\left(\left\lfloor \frac{y - (-15.0)}{\Delta g} \right\rfloor + 1, \, 1, \, 150\right)$$

**Grid Cell Center Coordinates (Row/Col $\to$ Local Frame):**
$$x_{\text{cell}}(c) = -10.0 + (c - 0.5) \cdot \Delta g$$
$$y_{\text{cell}}(r) = -15.0 + (r - 0.5) \cdot \Delta g$$

### 2.2 Local Bounding Box Sub-Matrix Cropping
To preserve a sub-$5\,\text{ms}$ build time per frame and avoid scanning all 45,000 cells for every obstacle, inflation is strictly computed over a cropped sub-grid:
$$r \in [\operatorname{row}(y_h - R_{\text{infl}}), \; \operatorname{row}(y_h + R_{\text{infl}})]$$
$$c \in [\operatorname{col}(x_h - R_{\text{infl}}), \; \operatorname{col}(x_h + R_{\text{infl}})]$$
where $R_{\text{infl}} = R_h + d_{\text{margin}} = R_h + 1.2\,\text{m}$.

### 2.3 Hazard Footprints & Continuous Exponential Inflation Field
Costs in $\mathbf{C}$ range continuously from $0$ (free space) to $100$ (lethal obstacle):

1. **Static Hazards (Potholes, Debris, Stopped Obstacles):**
   Centered at $(x_h, y_h)$ with core radius $R_h$:
   - Distance: $d_E = \sqrt{(x - x_h)^2 + (y - y_h)^2}$
   - **Lethal Core ($C = 100$):** For $d_E \le R_h$.
   - **Continuous Exponential Decay ($0 < C < 100$):** For $R_h < d_E \le R_h + d_{\text{margin}}$:
     $$C(x, y) = \max\left(C(x, y), \; 100 \cdot \exp\left(-\alpha (d_E - R_h - d_{\text{safe}})\right)\right)$$
     where decay rate $\alpha = 2.5\,\text{m}^{-1}$, safe cushion $d_{\text{safe}} = 0.35\,\text{m}$, and margin $d_{\text{margin}} = 1.2\,\text{m}$.
     Values are clamped to $[0, 100]$. Cells with $d_E \le R_h + d_{\text{safe}}$ saturate at $100$.

2. **Dynamic Obstacle Confidence Ellipses (from Phase 2 EKF):**
   For each confirmed track with state $[p_x, p_y]^T$, heading angle $\psi$, and $2\sigma$ semi-axes $(a, b)$:
   - Rotate local relative offset:
     $$\tilde{x} = (x - p_x)\cos\psi + (y - p_y)\sin\psi, \quad \tilde{y} = -(x - p_x)\sin\psi + (y - p_y)\cos\psi$$
     $$d_{\text{ell}} = \frac{\tilde{x}^2}{a^2} + \frac{\tilde{y}^2}{b^2}$$
   - **Lethal Core ($C = 100$):** For $d_{\text{ell}} \le 1.0$.
   - **Safety Field:** For $1.0 < d_{\text{ell}} \le 2.25$ (i.e. up to $1.5\times$ semi-axes):
     $$C(x, y) = \max\left(C(x, y), \; 100 \cdot \exp\left(-\alpha_{\text{dyn}} (\sqrt{d_{\text{ell}}} - 1.0)\right)\right), \quad \alpha_{\text{dyn}} = 2.0$$

3. **Road Verge / Shoulder Boundaries:**
   Cells outside lateral road boundaries $[Y_{\text{right\_edge}}(x), Y_{\text{left\_edge}}(x)]$ are assigned $C = 100$.

---

### 2.4 Forward Corridor Width Profiling & Disjoint Gap Selection
The ego vehicle evaluates the forward path from station $s = 0\,\text{m}$ to $s = 30.0\,\text{m}$ in steps of $\Delta s = 1.0\,\text{m}$:
1. Map station $s$ to column index $c_s = \operatorname{col}(s)$.
2. Extract vertical cost slice $\mathbf{c} = \mathbf{C}(:, c_s)$.
3. Identify all contiguous drivable vertical intervals $\mathcal{I}_m = [y_{\text{lower}, m}, y_{\text{upper}, m}]$ where $\mathbf{c}(r) < C_{\text{threshold}} = 40$.
4. **Disjoint Free Opening Rule:** If an obstacle splits the roadway into multiple clear gaps (e.g. left gap $2.2\,\text{m}$ and right gap $2.5\,\text{m}$), **do not sum them**. Select the single contiguous interval $\mathcal{I}^*$ that contains or is closest to the path reference lateral offset $y_{\text{ref}}(s)$:
   $$W_{\text{free}}(s) = y_{\text{upper}}^* - y_{\text{lower}}^*$$

---

### 2.5 Bottleneck Decider & Virtual Stop Line Arbitration
Critical vehicle corridor clearance threshold:
$$W_{\text{crit}} = W_{\text{veh}} + 2 \cdot \delta_{\text{clear}} = 1.85\,\text{m} + 2(0.35\,\text{m}) = 2.55\,\text{m}$$

At any station $s \in [0, 30]\,\text{m}$, if $W_{\text{free}}(s) < 2.55\,\text{m}$, the system identifies a squeeze point at station $s_{\text{pinch}} = s$. The decider arbitrates across three distinct cases:

#### Condition 1: Static Narrowing Only (Pothole / Edge Encroachment)
* **Trigger:** $1.85\,\text{m} \le W_{\text{free}}(s) < 2.55\,\text{m}$, and NO dynamic EKF track (or $2.0\,\text{s}$ rollout) has approaching relative velocity ($v_{\text{rel}} < -0.5\,\text{m/s}$) in $[s - 2.0\,\text{m}, s + 4.0\,\text{m}]$.
* **Decision:**
  - `virtual_stop_active = false`
  - `detour_required = true`
  - `pinch_station = s`
* **Behavior:** Enables the Hybrid A* planner to steer around the static hazard without halting the vehicle.

#### Condition 2: Dynamic Squeeze Point (Oncoming Actor in Narrow Corridor)
* **Trigger:** $W_{\text{free}}(s) < 2.55\,\text{m}$, AND an active dynamic EKF track has an approaching relative velocity ($v_{\text{rel}} < -0.5\,\text{m/s}$) with its 2.0s rollout intersecting $[s - 2.0\,\text{m}, s + 4.0\,\text{m}]$.
* **Decision:**
  - `virtual_stop_active = true`
  - `detour_required = false`
  - `vsl_station = \max(0.5, \, s - 3.5\,\text{m})`
  - `vsl_pose = [vsl_station, \, y_{\text{center}}(s), \, \theta_{\text{ego}}]`
* **Upstream Clamping:** If $s_{\text{pinch}} < 3.5\,\text{m}$, clamp $s_{\text{vsl}} = \max(0.5, s - 3.5)$. If $s_{\text{pinch}} \le 1.0\,\text{m}$, command immediate standstill.
* **Behavior:** Ego vehicle yields upstream of the bottleneck until the oncoming actor clears the squeeze point.

#### Condition 3: Complete Road Blockage ($W_{\text{free}}(s) < 1.85\,\text{m}$)
* **Trigger:** Physical drivable gap is narrower than the chassis width itself ($W_{\text{free}} < 1.85\,\text{m}$).
* **Decision:**
  - `virtual_stop_active = true`
  - `detour_required = false`
  - `vsl_station = \max(0.5, \, s - 3.5\,\text{m})`
* **Behavior:** Halts the vehicle safely upstream to prevent attempting impossible detours into ditches.

---

## 3. Data Interface Contract

### Inputs
1. `ego_state`: Vector $[x, y, \theta, v]$ in meters, radians, and m/s.
2. `sensor_detections`: Struct containing:
   - `.potholes`: Struct array with `.x`, `.y`, `.radius`.
   - `.static_boxes`: Struct array with `.x`, `.y`, `.radius`.
   - `.road_boundaries`: $[N \times 2]$ array of $[x, y]$ boundary points, or struct with `.left`, `.right`.
   - `.static_points`: Optional $[N \times 2]$ raw LiDAR hits.
3. `dynamic_predictions`: Struct array from `dynamic_obstacle_predictor.m`:
   - `.id`: Track integer ID.
   - `.type`: Class string.
   - `.waypoints`: $[H \times 2]$ predicted trajectory.
   - `.covariance`: $[H \times 1]$ cell array of covariance matrices.
   - `.semi_major`: $[H \times 1]$ $2\sigma$ semi-major axis.
   - `.semi_minor`: $[H \times 1]$ $2\sigma$ semi-minor axis.
   - `.orientation`: $[H \times 1]$ ellipse orientation angles.
   - `.x_est`: $[4 \times 1]$ state vector $[px, py, vx, vy]^T$.

### Outputs
1. `local_costmap`: $[150 \times 300]$ matrix of costs in $[0, 100]$.
2. `grid_meta`: Struct with coordinate transforms `.world2grid`, `.grid2world`, `.res = 0.2`, `.x_min = -10.0`, etc.
3. `virtual_stop_active`: Boolean flag indicating ego vehicle must halt.
4. `stop_pose`: $[x_{\text{stop}}, y_{\text{stop}}, \theta_{\text{stop}}]$ coordinates of Virtual Stop Line.
5. `bottleneck_info`: Struct containing `.station_s`, `.min_width`, `.detour_required`, `.vsl_station`, and `.reason`.

---

## 4. Grounding Literature References
- **Autoware Open-Source Architecture:** Forward corridor extraction, costmap inflation layers, and stop line placement (`resources/repos/Autoware`).
- **Ososinski & Labrosse (2015):** Navigating ill-defined roads lacking lane striping via boundary-constrained corridor tracking (`resources/papers_md/Ososinski_2015_IllDefinedRoads_preprint.md`).
- **Paden et al. (2016):** Motion planning and clearance margins for non-holonomic vehicle navigation (`resources/papers_md/Paden_2016_MotionPlanningSurvey_arXiv1604.07446.md`).
