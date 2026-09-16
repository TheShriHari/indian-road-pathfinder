# Subsystem 1 — Map & Environment
## Directive for: **Vetrivel**
## Branch: `feature/subsystem1-vetrivel`

---

> **INDEPENDENCE RULE:** This subsystem has **ZERO dependency on any other subsystem's real
> code**. Build and test entirely against your own scene data and your own exported road
> geometry. Push to your own branch `feature/subsystem1-vetrivel` — **do not merge to
> master yourself**. The Master Integrator handles final integration.

---

## Plain English: What Is This Subsystem Doing?

Imagine you are given a map of a road. Before the car can drive on it, someone needs to
"mathematically describe" that road in a way the computer can understand.

Right now the car only knows about obstacles near it (from the camera/LiDAR layer we already
built). It does **not** know the shape of the road far ahead, or how curved it is, or where the
centre of the lane is.

Your job is to:
1. **Build the road scenes** in RoadRunner (the 3D road design tool) for two of our five test
   scenarios and export them.
2. **Write the maths** that can take any point on the road and convert it into a simple
   (distance-along-road, offset-from-centre) coordinate pair. This is called a **Frenet frame**.
3. **Write the waypoint builder** for junction turns (left, right, sharp, U-turn) so the
   planner knows which path to follow through an intersection.

You own both the scene files and the code that reads them, so you can validate against your
own real data — no mocking needed on your end.

---

## Scope (What You Build — Exactly This, Nothing More)

### In Scope ✅
- **RoadRunner scene building:**
  - Scene A: Unmarked Village Road (Scenario 1 — narrow paved road, eroded shoulders)
  - Scene B: Signal-less Urban Intersection (Scenario 2 — 4-way uncontrolled junction)
  - Export each scene as a boundary point `.csv` file with columns `[x_left, y_left, x_right, y_right]`
    (one row per 0.5 m station along the road).
- **Frenet coordinate converter** (`matlab/frenet_coordinate_converter.m`):
  - Input: array of `(x, y)` world coordinates + road centreline definition
  - Output: `FrenetOutput` struct per point (fields: `s`, `d`, `theta_road`, `kappa`)
  - Bidirectional: also convert `(s, d)` back to `(x, y)` for the round-trip test.
- **Road network loader** (`matlab/road_network_loader.m`):
  - Reads the exported `.csv` boundary file.
  - Computes centreline as midpoint of left and right boundaries.
  - Fits a parametric cubic spline through the centreline points.
  - Returns spline coefficients, arc-length array, and `kappa(s)` array.
- **Junction turn waypoint generator** (function inside `road_network_loader.m`):
  - `build_junction_turn(entry_pose, turn_type)` where `turn_type ∈ {'left','right','sharp','uturn'}`
  - Returns an `[N×2]` waypoint array the planner can directly follow.
  - Uses a Bezier curve approximation (no toolbox required — Bezier is just polynomial math).

### Explicitly OUT of Scope ❌
- **NO roundabout ring topology.** Do not build circular roundabout paths. Leave this stub comment where the function would go:
  ```matlab
  % TODO: build_roundabout_ring() — deferred to post-SIH phase. See Master Integrator.
  ```
- **NO Scenes 3, 4, 5.** Those are handled by the scenario generator already in `matlab/generate_scenarios.m`.
- **NO dependency on any other teammate's code.**

---

## The Maths You Need to Implement

### 1. Centreline Spline Fitting

Given `N` centreline points `[cx_1..cx_N, cy_1..cy_N]`:

1. Compute cumulative arc-length parameter:
   $$s_0 = 0, \quad s_i = s_{i-1} + \sqrt{(cx_i - cx_{i-1})^2 + (cy_i - cy_{i-1})^2}$$

2. Fit two independent cubic splines: `x(s)` and `y(s)` using MATLAB's built-in `spline()` function.
   (This is base MATLAB — no toolbox needed.)

3. Curvature at any station `s`:
   $$\kappa(s) = \frac{x'(s)\,y''(s) - y'(s)\,x''(s)}{\bigl(x'(s)^2 + y'(s)^2\bigr)^{3/2}}$$
   where `x'(s)` and `x''(s)` are first and second derivatives from the spline (use `ppval` + `gradient`).

### 2. Cartesian → Frenet Conversion

For each query point `(qx, qy)`:

1. Find the arc-length `s*` at which the spline is closest to `(qx, qy)`:
   - Sample the spline at 500 uniform stations to get a coarse estimate of the nearest station.
   - Refine with **Newton iteration** (max 10 steps, stop when correction < 1e-5 m):
     $$s_{k+1} = s_k - \frac{(x(s_k)-qx)\cdot x'(s_k) + (y(s_k)-qy)\cdot y'(s_k)}{x'(s_k)^2 + y'(s_k)^2 + (x(s_k)-qx)\cdot x''(s_k) + (y(s_k)-qy)\cdot y''(s_k)}$$

2. Lateral offset `d`:
   $$d = -(qx - x(s^*))\sin\theta_{road}(s^*) + (qy - y(s^*))\cos\theta_{road}(s^*)$$
   where $\theta_{road}(s^*) = \text{atan2}(y'(s^*), x'(s^*))$.

3. Pack into `FrenetOutput` struct (see `directives/00_interface_contracts.md`).

### 3. Frenet → Cartesian Conversion (for round-trip test)

$$x = x(s) - d\cdot\sin\theta_{road}(s), \quad y = y(s) + d\cdot\cos\theta_{road}(s)$$

### 4. Junction Turn Bezier Path

For a 90° right turn at an intersection:
- Control points: `P0` = entry point, `P1` = corner apex (scaled by turn radius `R`), `P2` = exit point.
- Quadratic Bezier: $B(t) = (1-t)^2 P_0 + 2t(1-t)P_1 + t^2 P_2$, sampled at `t = linspace(0,1,30)`.
- Output: `[30×2]` waypoint matrix.
- For U-turn: semicircle approximated with a 6-control-point Bezier (so the curve stays smooth).

---

## Deliverables

| File | Type | What It Contains |
|---|---|---|
| `matlab/frenet_coordinate_converter.m` | MATLAB function | Bidirectional `(x,y) ↔ (s,d)` converter |
| `matlab/road_network_loader.m` | MATLAB function | Reads `.csv` boundaries, fits spline, returns `kappa(s)`, contains `build_junction_turn()` |
| `scenes/village_road_boundaries.csv` | Data file | Left/right boundary points for Scenario 1 |
| `scenes/urban_intersection_boundaries.csv` | Data file | Left/right boundary points for Scenario 2 |
| `matlab/test_subsystem1.m` | MATLAB test script | Runs both required tests and prints numeric results |

**Output struct produced** (see `directives/00_interface_contracts.md` for exact field types):
```matlab
FrenetOutput.s          % double
FrenetOutput.d          % double
FrenetOutput.theta_road % double
FrenetOutput.kappa      % double
```

---

## Required Tests — You Must Print Real Numbers

Run these by executing `matlab -batch "addpath('matlab'); test_subsystem1"` from the repo root.

### Test T1 — Coordinate Round-Trip Accuracy

**What it checks:** Convert 1,000 random points on an S-curve road from `(x,y)` to `(s,d)` and back
to `(x,y)`. The reconstructed point must be within 1 mm of the original.

**How to generate the S-curve for testing:**
```matlab
% S-curve centreline: 60m road with a sinusoidal lateral deviation
s_test = linspace(0, 60, 200)';
cx = s_test;
cy = 2.0 * sin(s_test * pi / 30.0);  % peak lateral swing ±2m
```

**Pass threshold:** Max positional error < **0.001 m** (1 mm).

**Required printed output:**
```
[T1] Round-trip test: 1000 points on S-curve road
     Max error    : X.XXXe-YY m
     Mean error   : X.XXXe-YY m
     Result       : PASS   (or FAIL if max error >= 0.001m)
```

### Test T2 — Curvature Accuracy on 90° Turn

**What it checks:** Build a circular arc of radius 5.0 m subtending 90°. Compute `kappa(s)` from
your spline at 50 evenly spaced stations. Ground truth is `kappa_true = 1/5.0 = 0.2000 m⁻¹`.

**Pass threshold:** Every computed `kappa(s)` value must be within **±0.005 m⁻¹** of 0.2000, and
no single value must spike above **2× the mean** value (no numerical blow-ups allowed).

**Required printed output:**
```
[T2] Curvature accuracy on 90-degree arc (R=5.0m, kappa_true=0.2000):
     Max kappa    : X.XXXX m^-1
     Min kappa    : X.XXXX m^-1
     Max deviation: X.XXXX m^-1   (threshold: <= 0.0050)
     Max spike    : X.XX x mean   (threshold: <= 2.0x)
     Result       : PASS   (or FAIL)
```

---

## Zero-Toolbox Rule

Use **only** base MATLAB functions. The following are allowed:  
`spline()`, `ppval()`, `gradient()`, `hypot()`, `atan2()`, `linspace()`, `cumsum()`, `diff()`,
`zeros()`, `ones()`, `polyval()`.

Do **not** use: Navigation Toolbox, Automated Driving Toolbox, Robotics System Toolbox,
Curve Fitting Toolbox, or any function that requires a licence beyond base MATLAB.

---


*Reference contracts: `directives/00_interface_contracts.md`*
