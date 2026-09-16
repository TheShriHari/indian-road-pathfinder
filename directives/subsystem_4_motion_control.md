# Subsystem 4 — Motion Control (Lateral & Longitudinal)
## Directive for: **Akshaya**
## Branch: `feature/subsystem4-akshaya`

---

> **INDEPENDENCE RULE:** This subsystem has **ZERO dependency on any other subsystem's real
> code**. Build and test entirely against mocks matching `directives/00_interface_contracts.md`.
> Push to your own branch `feature/subsystem4-akshaya` — **do not merge to master yourself**;
> the Master Integrator handles final integration.

---

## Plain English: What Is This Subsystem Doing?

Think of Subsystems 1, 2, and 3 as the car's **eyes**, **tactical brain**, and **navigator**:
- Subsystem 1 knows where the road boundaries are.
- Subsystem 2 decides: *"We should cruise at 8 m/s, but slow down if an obstacle appears."*
- Subsystem 3 draws the exact curved path on the map and plans the target speeds.

Now, someone has to be the **hands on the steering wheel** and the **feet on the accelerator and brake pedals**.

**That is your job.** You take the virtual path and speed target, and you calculate the physical steering angle (in radians), the throttle pedal position (0.0 to 1.0), and the brake pedal position (0.0 to 1.0) needed to guide a real 1.5-tonne car smoothly along that line.

### Why Is This Tricky on Indian Roads?
1. **Potholes and sudden nudges require quick steering**, but if you turn the steering wheel too fast, the car will violently lurch or skid sideways. Passengers get car-sick, and real car actuators break.
2. **If you only look right in front of the car**, you will wobble back and forth like a novice driver ("hunting").
3. **If you only look far ahead**, you will cut corners and clip curbs.
4. **Real physical steering motors take time to turn** — they don't jump from 0° to 20° in zero milliseconds.

To solve this, you will build a **blended steering controller** that combines the best of two proven algorithms:
- **Pure Pursuit:** Looks ahead down the road (like a human looking into a curve) to guarantee stability.
- **Stanley Controller:** Measures the exact error at the front wheels to eliminate lane drift.
- **Actuator Lag & Slew Filter:** Makes sure the steering wheel turns smoothly (never exceeding 15° per second) and respects physical motor response.
- **Longitudinal PID:** Smoothly presses the gas or brake to match the desired speed without harsh jerky lurches.

---

## Scope (What You Build — Exactly This, Nothing More)

### In Scope ✅

1. **Lateral Controller — Blended Pure Pursuit + Stanley** (`matlab/motion_controller.m`):
   - Computes Pure Pursuit steering angle $\delta_{\text{PP}}$ using speed-adaptive lookahead distance.
   - Computes Stanley steering angle $\delta_{\text{Stanley}}$ using front-axle cross-track and heading error.
   - Dynamically blends the two based on vehicle velocity: Pure Pursuit dominates at low speeds / sharp turns; Stanley dominates at cruising speeds for pinpoint lane tracking.
   - Clamps steering command to physical limits: $\pm 30^\circ$ ($\pm 0.5236\text{ rad}$).

2. **Longitudinal Controller — PID with Throttle/Brake Split**:
   - Compares current ego vehicle speed $v$ against target reference speed $v_{\text{ref}}$ from `TacticalDecision` or `TrajectoryOutput`.
   - PID control loop computes desired longitudinal acceleration $a_{\text{cmd}} \in [-3.5, +2.5]\text{ m/s}^2$.
   - Enforces jerk limit: $|da/dt| \le 0.90\text{ m/s}^3$ (strictly below the $1.0\text{ m/s}^3$ discomfort threshold).
   - Maps acceleration into independent throttle $[0, 1]$ and brake $[0, 1]$ commands. Throttle and brake must never be applied simultaneously.

3. **First-Order Actuator Lag Filter & Slew Limiter**:
   - Enforces physical actuator slew-rate ceiling: $|\dot{\delta}| \le 15^\circ/\text{s} = 0.2618\text{ rad/s}$.
   - Applies discrete-time first-order low-pass filter ($\tau = 0.08\text{ s}$) to emulate realistic steering motor mechanical response.

4. **Output Packager**:
   - Packages final values into the frozen `ControlOutput` struct (`steer_rad`, `throttle`, `brake`).

### Explicitly OUT of Scope ❌
- **NO** trajectory generation, polynomial curve fitting, or spline math (that is Keerthana's Subsystem 3).
- **NO** obstacle detection, sensor fusion, or FSM state changes (that is Rithika's Subsystem 2).
- **NO** RoadRunner maps or road geometry loading (that is Vetrivel's Subsystem 1).
- **NO** telemetry JSON serialization or web dashboard code (that is Archana's Subsystem 5).
- **NO** direct hardware communication or CAN-bus drivers (this runs in MATLAB algorithmic simulation).

---

## Your Mock Inputs (Build These Locally)

Subsystems 2 and 3 are being built in parallel on their own branches. You **must not import their files**.
Instead, create these two mock files in your local workspace:

### Mock 1: `matlab/mock_tactical_decision.m`
```matlab
% Local mock matching Contract 2 in directives/00_interface_contracts.md
function out = mock_tactical_decision(state_str, v)
    if nargin < 1, state_str = 'CRUISE'; end
    if nargin < 2, v = 8.0; end
    out.active_state   = state_str;
    out.v_ref          = v;
    out.yield_required = false;
    out.hold_time_est  = 0.0;
end
```

### Mock 2: `matlab/mock_trajectory_output.m`
```matlab
% Local mock matching Contract 3 in directives/00_interface_contracts.md
function out = mock_trajectory_output(num_pts, lateral_offset)
    if nargin < 1, num_pts = 80; end
    if nargin < 2, lateral_offset = 0.0; end
    
    % Generates a straight or nudged 25m test path
    x = linspace(0, 25.0, num_pts)';
    y = ones(num_pts, 1) * lateral_offset;
    
    out.path              = [x, y];
    out.speed_profile     = ones(num_pts, 1) * 8.0;
    out.max_kappa         = 0.02;
    out.is_collision_free = true;
end
```

---

## Mathematical Specifications & Exact Formulas

### 1. Vehicle Physical Dimensions
```matlab
L         = 2.7;               % Wheelbase (m) - distance between front and rear axles
max_steer = deg2rad(30.0);     % Maximum physical steering angle: 0.5236 rad
max_slew  = deg2rad(15.0);     % Maximum steering slew rate: 0.2618 rad/s
max_jerk  = 0.90;              % Maximum longitudinal jerk: 0.90 m/s³
dt        = 0.10;              % Controller timestep: 100 ms (10 Hz nominal tick)
```

---

### 2. Pure Pursuit Formulation (Rear Axle Reference)
Pure pursuit calculates the steering angle needed to aim the car toward a lookahead point on the path ahead:

$$\text{Lookahead Distance:}\quad L_d(v) = \text{clip}\big(K_v \cdot \max(0.1, v),\, L_{\min},\, L_{\max}\big)$$
- $K_v = 1.2\text{ s}$
- $L_{\min} = 3.0\text{ m}$ (ensures stable steering even when stopped at a junction)
- $L_{\max} = 12.0\text{ m}$

$$\text{Heading to Lookahead Target:}\quad \alpha = \text{atan2}(y_{\text{tgt}} - y,\, x_{\text{tgt}} - x) - \theta$$
$$\text{Pure Pursuit Steering Command:}\quad \delta_{\text{PP}} = \text{atan2}\left(\frac{2 L \sin(\alpha)}{L_d},\, 1.0\right)$$

*Standstill / VSL boundary protection:* If $v < 0.2\text{ m/s}$ or target point projection along vehicle heading is $\le 0.2\text{ m}$, project the target point strictly forward along the car's heading axis to avoid sudden $\pm \pi$ angle flips.

---

### 3. Stanley Steering Formulation (Front Axle Reference)
The Stanley controller computes steering based on errors measured at the **front axle**:

$$\text{Front Axle Position:}\quad x_f = x + L \cos(\theta),\quad y_f = y + L \sin(\theta)$$

Find the nearest path waypoint $(x_{p}, y_{p})$ to $(x_f, y_f)$ and compute:
1. **Path Heading:** $\theta_p = \text{atan2}(y_{p+1} - y_p,\, x_{p+1} - x_p)$
2. **Heading Error:** $e_\theta = \text{angdiff}(\theta_p,\, \theta)$
3. **Cross-Track Error:** Measured perpendicular to path heading:
   $$e_y = (y_f - y_p)\cos(\theta_p) - (x_f - x_p)\sin(\theta_p)$$

$$\text{Stanley Steering Command:}\quad \delta_{\text{Stanley}} = e_\theta + \text{atan2}\left(k_e \cdot e_y,\, v + v_{\text{soft}}\right)$$
- Gain $k_e = 0.8$
- Softening parameter $v_{\text{soft}} = 1.0\text{ m/s}$ (prevents infinite steering angle at near-zero speeds)

---

### 4. Dynamic Blending Formula
Pure Pursuit handles tight curves and low-speed navigation with rock-solid stability, while Stanley provides razor-sharp lateral lane-keeping at cruising speed. We blend them dynamically:

$$w(v) = \text{clip}\left(\frac{v - 2.0}{8.0 - 2.0},\, 0.0,\, 1.0\right)$$
$$\delta_{\text{blend}} = (1.0 - w(v)) \cdot \delta_{\text{PP}} + w(v) \cdot \delta_{\text{Stanley}}$$

- At $v \le 2.0\text{ m/s}$: $w = 0.0 \implies 100\%$ Pure Pursuit (maximum stability, no jitter).
- At $v \ge 8.0\text{ m/s}$: $w = 1.0 \implies 100\%$ Stanley (pinpoint path tracking).
- In between: smooth, continuous, shock-free transition.

---

### 5. Slew Rate Limiter & First-Order Actuator Lag Filter

#### Step A: Saturation Clamp
$$\delta_{\text{sat}} = \text{clip}(\delta_{\text{blend}},\, -0.5236,\, +0.5236)$$

#### Step B: Slew Rate Limiting (Max 15°/s)
$$\Delta \delta_{\max} = \text{slew\_rate} \cdot dt = 0.2618 \cdot dt$$
$$\delta_{\text{slew}}(k) = \text{clip}\big(\delta_{\text{sat}},\, \delta_{\text{prev}} - \Delta \delta_{\max},\, \delta_{\text{prev}} + \Delta \delta_{\max}\big)$$

#### Step C: First-Order Mechanical Actuator Lag
Real steering actuators act as a low-pass filter with time constant $\tau = 0.08\text{ s}$:
$$\alpha_{\text{lag}} = \exp\left(-\frac{dt}{\tau}\right) = \exp\left(-\frac{0.10}{0.08}\right) \approx 0.2865$$
$$\delta_{\text{cmd}}(k) = \alpha_{\text{lag}} \cdot \delta_{\text{act}}(k-1) + (1.0 - \alpha_{\text{lag}}) \cdot \delta_{\text{slew}}(k)$$

Update state: $\delta_{\text{prev}} = \delta_{\text{slew}}(k)$, $\delta_{\text{act}}(k) = \delta_{\text{cmd}}(k)$.

---

### 6. Longitudinal PID & Throttle/Brake Split

#### Step A: Speed Tracking Error & PID
$$e_v(k) = v_{\text{ref}} - v(k)$$
$$a_{\text{PID}} = K_p e_v + K_i \sum (e_v \cdot dt) + K_d \frac{e_v(k) - e_v(k-1)}{dt}$$
- Gains: $K_p = 1.0$, $K_i = 0.05$, $K_d = 0.01$
- Anti-windup clamping on integral sum: $\text{clip}\big(\sum e_v dt,\, -5.0,\, +5.0\big)$

#### Step B: Acceleration Clamping & Longitudinal Jerk Limiting
$$\text{Physical envelope clamp:}\quad a_{\text{clamped}} = \text{clip}(a_{\text{PID}},\, -3.5,\, +2.5)\text{ m/s}^2$$
$$\Delta a_{\max} = \text{jerk\_max} \cdot dt = 0.90 \cdot dt$$
$$a_{\text{cmd}}(k) = \text{clip}\big(a_{\text{clamped}},\, a_{\text{prev}} - \Delta a_{\max},\, a_{\text{prev}} + \Delta a_{\max}\big)$$

#### Step C: Throttle / Brake Split (No Overlapping Actuation)
$$\text{If } a_{\text{cmd}} \ge 0:\quad \text{throttle} = \text{clip}\left(\frac{a_{\text{cmd}}}{2.5},\, 0.0,\, 1.0\right),\quad \text{brake} = 0.0$$
$$\text{If } a_{\text{cmd}} < 0:\quad \text{throttle} = 0.0,\quad \text{brake} = \text{clip}\left(\frac{-a_{\text{cmd}}}{3.5},\, 0.0,\, 1.0\right)$$

*Guarantee:* $\text{throttle} \times \text{brake} \equiv 0$ at all times.

---

## Deliverable Files

You will deliver the following files in `matlab/`:

| File | Purpose |
|---|---|
| `matlab/motion_controller.m` | Complete controller function matching `ControlOutput` contract |
| `matlab/mock_tactical_decision.m` | Local mock generator for Subsystem 2 data |
| `matlab/mock_trajectory_output.m` | Local mock generator for Subsystem 3 data |
| `matlab/test_subsystem4_motion_control.m` | Unit test suite executing the 2 required tests below |

### Main Function Signature (`matlab/motion_controller.m`)
```matlab
function [control_out, debug_info] = motion_controller(ego_state, trajectory, tactical, params)
% MOTION_CONTROLLER Blended Pure Pursuit + Stanley controller with longitudinal PID
%
% Inputs:
%   ego_state  : [x; y; theta; v] (4x1 double array)
%   trajectory : TrajectoryOutput struct (Contract 3)
%   tactical   : TacticalDecision struct (Contract 2)
%   params     : configuration struct (optional, contains gains/limits)
%
% Outputs:
%   control_out: ControlOutput struct (Contract 4)
%                .steer_rad  [-0.5236, +0.5236]
%                .throttle   [0.0, 1.0]
%                .brake      [0.0, 1.0]
%   debug_info : struct with cross-track error, heading error, jerk, raw commands
```

---

## Required Verification Tests (Must Pass Before Merge)

You must run `matlab/test_subsystem4_motion_control.m` and verify that both tests pass 100%:

### Test 1 — Step-Response Lateral Convergence
- **Scenario:** The car is traveling along a straight road at $v = 8.0\text{ m/s}$, but begins with a lateral offset of $e_y = +0.50\text{ m}$ from the path centerline.
- **Duration:** 4.0 seconds simulation (40 steps at $dt = 0.1\text{ s}$).
- **Pass Threshold:**
  $$\text{Cross-track error } |e_y| < 0.05\text{ m within } 3.0\text{ seconds of start}$$
- **Fail Condition:** If $|e_y| \ge 0.05\text{ m}$ at $t = 3.0\text{ s}$ or the vehicle oscillates unstably across the centerline.

### Test 2 — Actuator Slew Rate & Passenger Jerk Compliance
- **Scenario:** Run a demanding 50-step simulation through a sharp obstacle nudge trajectory ($0.8\text{ m}$ lateral deviation over $10\text{ m}$) followed by an emergency braking deceleration ($v_{\text{ref}}$ drops from $8.0\text{ m/s}$ to $0.0\text{ m/s}$).
- **Pass Thresholds:**
  1. **Steering Slew Rate:**
     $$\left|\frac{\delta(k) - \delta(k-1)}{dt}\right| \le 0.2618\text{ rad/s } (15.0^\circ/\text{s}) \quad \text{for } 100\% \text{ of timesteps}$$
  2. **Longitudinal Jerk:**
     $$\left|\frac{a(k) - a(k-1)}{dt}\right| \le 0.90\text{ m/s}^3 \quad \text{for } 100\% \text{ of timesteps}$$
  3. **Actuator Conflict:** $\text{throttle} \times \text{brake} == 0$ at all timesteps.
- **Fail Condition:** Any single timestep where slew exceeds $15.01^\circ/\text{s}$ or jerk exceeds $0.901\text{ m/s}^3$.

---

## What NOT to Do (Common Traps)

1. ❌ **Do NOT call `frenet_trajectory_generator.m` or `behavior_state_machine.m` directly.** Always use `mock_trajectory_output()` and `mock_tactical_decision()`.
2. ❌ **Do NOT compute Stanley cross-track error at the rear axle.** Stanley math requires the error at the **front axle** ($x + L\cos\theta, y + L\sin\theta$). If you compute it at the rear axle, the vehicle will fish-tail and oscillate violently.
3. ❌ **Do NOT allow both throttle and brake to be positive.** Always branch: if accelerating $\implies \text{brake} = 0$; if decelerating $\implies \text{throttle} = 0$.
4. ❌ **Do NOT forget anti-windup on the speed PID.** If the car is stopped at a virtual stop line, speed error $(v_{\text{ref}} - v)$ will accumulate in the integrator if unclamped, causing the car to blast off when released. Clamp the integral sum to $[-5.0, 5.0]$.
5. ❌ **Do NOT bypass the slew limiter during emergency stops.** Slew limits protect actuator health and vehicle stability under all circumstances.

---

