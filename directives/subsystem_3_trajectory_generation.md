# Subsystem 3 — Kinematic Trajectory Generation & Smoothing
## Directive for: **Keerthana**
## Branch: `feature/subsystem3-keerthana`

---

> **INDEPENDENCE RULE:** This subsystem has **ZERO dependency on any other subsystem's real
> code**. Build and test entirely against mocks matching `directives/00_interface_contracts.md`.
> Push to your own branch `feature/subsystem3-keerthana` — **do not merge to master yourself**.
> The Master Integrator handles final integration.

---

## Plain English: What Is This Subsystem Doing?

The brain (Subsystem 2) has decided "we need to nudge left around that pothole" or "we need to
merge right onto the highway". Now someone has to draw the **actual smooth curve** the car will
follow.

That is your job. You take a starting position and a desired ending position (both described in
the Frenet frame — distance along the road + lateral offset) and produce:

1. **A smooth path** — a curve that goes from A to B without any sharp kinks, and that the car
   can physically execute (it can't turn tighter than a 4.5 m radius).
2. **A speed plan** — how fast should the car go at each point along that curve? The car must
   automatically slow down before sharp corners and speed up on straights.

You also check: will the car hit anything while following this path?

You do **not** need to know how the road geometry was computed (Subsystem 1) or how the
steering wheel is physically moved (Subsystem 4). You just produce the `TrajectoryOutput` struct
and hand it off.

---

## Scope (What You Build — Exactly This, Nothing More)

### In Scope ✅

**Quintic polynomial lateral trajectory generator** (`matlab/frenet_trajectory_generator.m`):
- Takes a Frenet start state and a Frenet target state.
- Fits a 5th-order (quintic) polynomial `d(s)` describing how the lateral offset changes along
  the path.
- Converts the resulting Frenet path back to Cartesian `(x, y)` waypoints.
- Checks the swept vehicle envelope (car width 1.85 m + 0.35 m safety cushion = **2.20 m total**)
  against the costmap for 25 m ahead.
- Outputs a `TrajectoryOutput` struct.

**Curvature-based speed profiler** (`matlab/curvature_speed_profiler.m`):
- Takes the generated path and computes how fast the car should go at each point.
- Speed is limited by both the road speed limit and the lateral comfort constraint (the car must
  not feel like it is sliding sideways).
- Outputs an `[N×1]` speed array matched to the path waypoints.

### Explicitly OUT of Scope ❌
- **NO** roundabout path generation.
- **NO** MOBIL overtaking trajectories.
- **NO** importing any file from Subsystem 1, 2, 4, or 5.
- **NO** map loading or scene reading — you work with mock Frenet inputs.

---

## Your Mock Input (Build This Locally)

You need fake Frenet data to test your trajectory generator. Create this locally:

```matlab
% matlab/mock_frenet_output.m — local mock (do NOT import from Subsystem 1)
function out = mock_frenet_output(s_val, d_val, kappa_val)
    if nargin < 1, s_val    = 0.0;  end
    if nargin < 2, d_val    = 0.0;  end
    if nargin < 3, kappa_val = 0.05; end
    out.s          = s_val;
    out.d          = d_val;
    out.theta_road = 0.0;
    out.kappa      = kappa_val;
end
```

When the Master Integrator merges everything, your trajectory generator will receive real
`FrenetOutput` structs from Subsystem 1. Your code must read from the struct fields —
never hardcode specific values.

---

## The Maths You Need to Implement

### 1. Quintic Polynomial Trajectory (`matlab/frenet_trajectory_generator.m`)

**What is a quintic polynomial?** It is a smooth S-shaped curve described by 6 numbers
(coefficients `a0` through `a5`). We choose these 6 numbers so that the curve starts and
ends exactly where we want, with the right angle and the right curvature at both ends.
This guarantees smoothness — no kinks, no jerks.

**The trajectory formula:**

$$d(s) = a_0 + a_1 s + a_2 s^2 + a_3 s^3 + a_4 s^4 + a_5 s^5$$

where `s` is the longitudinal distance along the road (0 = start, S = end).

**The 6 boundary conditions** (3 at start, 3 at end):

| Condition | Formula | Meaning |
|---|---|---|
| $d(0) = d_0$ | given | Start lateral offset |
| $d'(0) = d_0'$ | given | Start lateral slope (from ego heading vs road heading) |
| $d''(0) = d_0''$ | given | Start lateral curvature |
| $d(S) = d_S$ | given | Target lateral offset |
| $d'(S) = 0$ | zero | Arrive tangent to road (no lateral drift at endpoint) |
| $d''(S) = 0$ | zero | Arrive with zero curvature change |

**Solving for coefficients:** Set up a 6×6 linear system $A \mathbf{a} = \mathbf{b}$:

$$A = \begin{bmatrix}
1 & 0 & 0    & 0      & 0       & 0        \\
0 & 1 & 0    & 0      & 0       & 0        \\
0 & 0 & 2    & 0      & 0       & 0        \\
1 & S & S^2  & S^3    & S^4     & S^5      \\
0 & 1 & 2S   & 3S^2   & 4S^3    & 5S^4     \\
0 & 0 & 2    & 6S     & 12S^2   & 20S^3
\end{bmatrix}, \quad
\mathbf{b} = \begin{bmatrix} d_0 \\ d_0' \\ d_0'' \\ d_S \\ 0 \\ 0 \end{bmatrix}$$

Solve: `a = A \ b` — this is exact linear algebra, no iteration, no toolbox.

**Converting Frenet path back to Cartesian:**

For each station `s_i = linspace(0, S, N)` (use N=80 points for 25 m horizon):
$$x_i = x_{\text{road}}(s_i) - d(s_i)\cdot\sin\theta_{\text{road}}(s_i)$$
$$y_i = y_{\text{road}}(s_i) + d(s_i)\cdot\cos\theta_{\text{road}}(s_i)$$

Since you are working with a mock (no real spline from Subsystem 1), approximate:
- `x_road(s) = s_start + s * cos(theta_road)` (straight road approximation for testing).
- For integration, Subsystem 1's spline is substituted in by the Master Integrator.

**Curvature of the generated path** (for collision check and T1 verification):

$$\kappa_{\text{path}}(s) = \frac{d''(s)}{\bigl(1 + d'(s)^2\bigr)^{3/2}} + \kappa_{\text{road}}(s)$$

where `d'(s) = a1 + 2a2*s + 3a3*s² + 4a4*s³ + 5a5*s⁴` and
`d''(s) = 2a2 + 6a3*s + 12a4*s² + 20a5*s³`.

**Collision envelope check:**

For each waypoint `(x_i, y_i)`, look up the costmap value. Also check lateral offsets of
±1.10 m (half of 2.20 m total width). If any cell has cost ≥ 90 (hard obstacle), set
`is_collision_free = false`.

```matlab
function traj = frenet_trajectory_generator(start_frenet, target_d, S_horizon, road_kappa, costmap, grid_res, grid_origin)
    % start_frenet : FrenetOutput struct (mock or real)
    % target_d     : double — target lateral offset at end of trajectory (m)
    % S_horizon    : double — longitudinal planning distance (m), default 25.0
    % road_kappa   : double — approximate road curvature (use start_frenet.kappa)
    % costmap      : [M x N] occupancy grid (0-100 scale)
    % grid_res     : double — metres per cell (default 0.2)
    % grid_origin  : [x_min, y_min] world coords of costmap bottom-left
    %
    % Returns: TrajectoryOutput struct (see 00_interface_contracts.md)
end
```

---

### 2. Speed Profiler (`matlab/curvature_speed_profiler.m`)

**What it does:** For each point along the trajectory, computes how fast the car can safely go
given the curvature at that point and the road speed limit.

**Two limiting factors:**
1. Road speed limit: `v_limit` (passed as parameter, default 8.0 m/s for urban, 13.9 m/s for highway)
2. Lateral comfort: the centripetal acceleration must stay ≤ 2.0 m/s²

$$v(s) = \min\!\left(v_{\text{limit}},\; \sqrt{\frac{a_{\text{lat,max}}}{|\kappa(s)| + \varepsilon}}\right)$$

where $a_{\text{lat,max}} = 2.0\,\text{m/s}^2$ and $\varepsilon = 10^{-4}$ (prevents divide-by-zero
on straight roads).

**What this means in practice:**
- On a straight road: `kappa ≈ 0` → `sqrt(2.0 / 1e-4) ≈ 141 m/s` → limited by `v_limit` = 8 m/s.
- At `kappa = 0.2222 m⁻¹` (sharpest allowed curve): `sqrt(2.0 / 0.2222) ≈ 3.0 m/s` → car slows to 3 m/s ≈ 10.8 km/h.

```matlab
function speed_profile = curvature_speed_profiler(kappa_array, v_limit, a_lat_max)
    % kappa_array : [Nx1] double — curvature at each path station
    % v_limit     : double — road speed limit (m/s)
    % a_lat_max   : double — lateral comfort limit (m/s²), default 2.0
    % Returns: [Nx1] double — target speed at each station
    if nargin < 3, a_lat_max = 2.0; end
    eps = 1e-4;
    speed_profile = min(v_limit, sqrt(a_lat_max ./ (abs(kappa_array) + eps)));
end
```

---

## Deliverables

| File | Type | What It Contains |
|---|---|---|
| `matlab/frenet_trajectory_generator.m` | MATLAB function | Quintic polynomial solver, Frenet→Cartesian conversion, collision check |
| `matlab/curvature_speed_profiler.m` | MATLAB function | Speed array from curvature profile |
| `matlab/mock_frenet_output.m` | MATLAB function (local mock) | Fake FrenetOutput for your own tests only |
| `matlab/test_subsystem3.m` | MATLAB test script | Runs all 3 tests and prints numeric results |

**Output struct produced** (see `directives/00_interface_contracts.md`):
```matlab
TrajectoryOutput.path             % [Nx2] double — (x,y) waypoints
TrajectoryOutput.speed_profile    % [Nx1] double — speed at each waypoint (m/s)
TrajectoryOutput.max_kappa        % double — max curvature along path (m⁻¹)
TrajectoryOutput.is_collision_free % bool
```

---

## Required Tests — You Must Print Real Numbers

Run with: `matlab -batch "addpath('matlab'); test_subsystem3"` from repo root.

### Test T1 — Maximum Curvature Bound

**What it checks:** Generate 50 different avoidance trajectories (vary `target_d` from −1.0 m
to +1.0 m in steps, and `S_horizon` from 10 m to 25 m in steps). For each trajectory, compute
`max(|kappa_path(s)|)` across all 80 sample points. Every single trajectory must satisfy
`max_kappa ≤ 0.2222 m⁻¹`.

**Pass threshold:** `max(max_kappa across all 50 trajectories) ≤ 0.2222 m⁻¹`

**Required printed output:**
```
[T1] Maximum curvature bound across 50 generated trajectories:
     Max kappa observed : X.XXXX m^-1
     Threshold          : 0.2222 m^-1
     Trajectories over  : X / 50
     Result             : PASS   (or FAIL — must be 0/50 over threshold)
```

### Test T2 — Jerk Bounds

**What it checks:** For a representative trajectory (e.g. `target_d = 1.0 m`, `S_horizon = 20 m`,
`v = 8 m/s`), compute:
- Longitudinal jerk: `|Δa_lon / Δt|` — change in forward acceleration per second.
- Lateral jerk: `|Δa_lat / Δt|` — change in sideways acceleration per second.

Both must remain strictly below **1.0 m/s³** at every point along the trajectory.

Derive accelerations from speed profile: `a_lon(i) = (v(i+1) - v(i)) / dt` where `dt = S_step / v_mean`.
Lateral acceleration: `a_lat(i) = v(i)² * kappa(i)`.
Jerk: finite difference of acceleration.

**Pass threshold:** `max(|lon_jerk|) < 1.0 m/s³` AND `max(|lat_jerk|) < 1.0 m/s³`

**Required printed output:**
```
[T2] Jerk bounds on representative avoidance trajectory:
     Max longitudinal jerk : X.XXX m/s^3   (threshold: < 1.000)
     Max lateral jerk      : X.XXX m/s^3   (threshold: < 1.000)
     Result                : PASS   (or FAIL)
```

### Test T3 — Generation Speed

**What it checks:** Time how long it takes to generate one trajectory (single call to
`frenet_trajectory_generator()`). Must complete in under **15 ms** on a standard CPU.

Use MATLAB's `tic` / `toc` timing. Average over 20 calls to get a stable measurement.

**Pass threshold:** Mean generation time < **15 ms**

**Required printed output:**
```
[T3] Trajectory generation time (20-call average):
     Mean time : X.X ms
     Max time  : X.X ms
     Threshold : 15.0 ms
     Result    : PASS   (or FAIL)
```

---

## Zero-Toolbox Rule

Use **only** base MATLAB functions. Allowed: `linspace()`, `cumsum()`, `gradient()`,
`hypot()`, `atan2()`, `min()`, `max()`, `zeros()`, `ones()`, `tic`, `toc`, `abs()`,
`sqrt()`, backslash operator (`\`) for linear solve.

Do **not** use: `lsqcurvefit()`, `fmincon()`, `polyfit()` (for trajectory fitting),
Optimization Toolbox, or any spline toolbox functions for the polynomial solver.

---

