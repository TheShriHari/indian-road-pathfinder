# Subsystem 5 — Telemetry Packaging & Web Visualizer Dashboard
## Directive for: **Archana**
## Branch: `feature/subsystem5-archana`

---

> **INDEPENDENCE RULE:** This subsystem has **ZERO dependency on any other subsystem's real
> code**. Build and test entirely against mocks matching `directives/00_interface_contracts.md`.
> Push to your own branch `feature/subsystem5-archana` — **do not merge to master yourself**;
> the Master Integrator handles final integration.

---

## Plain English: What Is This Subsystem Doing?

Subsystems 1 to 4 are running heavy mathematics inside MATLAB:
- Subsystem 1 computes complex road curves and Frenet frames.
- Subsystem 2 evaluates Time-To-Collision (TTC) and makes FSM tactical decisions.
- Subsystem 3 calculates 5th-order polynomials and speed profiles.
- Subsystem 4 runs blended Pure Pursuit and Stanley control loops.

If all of that math runs invisibly inside a terminal with numbers scrolling past at 50 lines a second, **no judge, evaluator, or passenger can understand what the vehicle is thinking or doing**.

**That is where you come in.** You build the **flight recorder, mission control dashboard, and live web visualizer**:
1. **The Telemetry Exporter:** Takes the simulation output (positions, speeds, planned curves, steering angles, sensor predictions) and packages them into clean, standardized JSON files (`TelemetryFrame` sequence). It guarantees that there are no broken numbers (`NaN`, `Inf`, or missing fields).
2. **The Web Visualizer:** An interactive browser dashboard (HTML5 Canvas + modern CSS/JS) that loads the telemetry and plays it back smoothly at 60 FPS. Anyone opening a web browser can watch:
   - The car navigating around cows, potholes, and oncoming traffic.
   - The planned trajectory curve stretching 25 meters ahead.
   - Live telemetry gauges: vehicle speed vs target speed, longitudinal and lateral jerk dials, and cross-track deviation.
   - A color-coded **Tactical Behavior Badge** showing whether the car is in `CRUISE`, `NUDGE`, `JUNCTION_TURN`, `YIELD_WAIT`, etc.
   - The **Frenet frame coordinates** ($s, d$) displaying where the car is along the road.
   - **EKF uncertainty ellipses** drawn around obstacles showing how confident the tracker is about their future motion.
   - A **Scenario Selector** dropdown allowing instant switching between Scenarios 1 through 5.

---

## Scope (What You Build — Exactly This, Nothing More)

### In Scope ✅

1. **Telemetry Packaging & Export Script** (`execution/export_web_telemetry.py`):
   - Ingests simulation runs or mock outputs.
   - Decimates high-frequency data if needed (e.g. 100 Hz simulation downsampled to 20 Hz / 50 ms frames for clean web playback).
   - Validates that every frame contains 100% of required fields matching the frozen `TelemetryFrame` contract.
   - Computes derived metrics:
     - Cross-track error: lateral distance from current ego position to planned path centerline ($e_y$).
     - Longitudinal jerk: $\frac{a(k) - a(k-1)}{\Delta t}$.
     - Lateral jerk: $\frac{a_{\text{lat}}(k) - a_{\text{lat}}(k-1)}{\Delta t}$, where $a_{\text{lat}} = v^2 \cdot \kappa_{\text{path}}$.
   - Exports formatted JSON to `web_simulation/data/scenario_playback.json`.

2. **Web Visualizer Dashboard Extensions** (`web_simulation/index.html`, CSS, JS):
   - **Scenario Selector:** Dropdown menu allowing selection of Scenarios 1 to 5 (`SC-1`, `SC-2`, `SC-3`, `SC-4`, `SC-5`).
   - **HUD Instrument Cluster:**
     - Digital speedometer ($v_{\text{ego}}$ vs $v_{\text{ref}}$ in km/h and m/s).
     - Jerk gauges (longitudinal and lateral jerk with visual comfort zone indicator $< 0.9\text{ m/s}^3$).
     - Cross-track error gauge (deviation in cm/m with green/amber/red status).
   - **FSM Tactical State Badge:**
     - Prominent status badge displaying the current `active_state`.
     - Color-coded:
       - Green: `CRUISE`, `LANE_KEEP`, `RESUME`
       - Yellow / Amber: `NUDGE`, `HIGHWAY_MERGE`, `JUNCTION_TURN`
       - Red: `YIELD_DECEL`, `YIELD_WAIT`
   - **Frenet Overlay Box:**
     - Displays current station $s$ (m), lateral offset $d$ (m), road heading $\theta_{\text{road}}$ (deg), and curvature $\kappa$ ($m^{-1}$).
   - **EKF Uncertainty Ellipses:**
     - Visualizes obstacle covariance ellipses ($2\sigma$ uncertainty bounds) dynamically expanding/contracting around tracked dynamic obstacles.
   - **Playback Controls:**
     - Play, Pause, Step Forward/Back, Playback Speed (0.5x, 1.0x, 2.0x), and a time scrubber bar.

### Explicitly OUT of Scope ❌
- **NO** 3D game engines (no Three.js or heavy WebGL libraries). Use clean, ultra-responsive HTML5 2D Canvas and vanilla CSS/JS.
- **NO** modifications to the fields of `TelemetryFrame` in `directives/00_interface_contracts.md`.
- **NO** implementing motion controllers, path planners, or EKF filters (you only visualize their output).
- **NO** importing MATLAB engine libraries into Python (use standard JSON/CSV file exchange).

---

## Your Mock Inputs (Build These Locally)

To develop and test your exporter and dashboard without waiting for Subsystems 1–4, you will create a synthetic telemetry generator:

### Mock Generator: `execution/mock_telemetry_generator.py`
```python
#!/usr/bin/env python3
"""
execution/mock_telemetry_generator.py
Generates 100 frames of synthetic simulation data matching Contract 5 in directives/00_interface_contracts.md.
Used by Subsystem 5 for standalone dashboard development and testing.
"""
import json
import math
from pathlib import Path

def generate_mock_telemetry(num_frames=100, scenario_id="SC-1"):
    frames = []
    dt = 0.05  # 20 Hz
    
    for k in range(num_frames):
        t = round(k * dt, 3)
        s = round(k * 0.4, 2)
        d = round(0.3 * math.sin(k * 0.1), 3)  # Gentle lateral nudge
        v = 8.0 if k < 60 else max(0.0, 8.0 - (k - 60) * 0.25)
        
        state = "CRUISE"
        if 20 <= k < 45:
            state = "NUDGE"
        elif 60 <= k < 80:
            state = "YIELD_DECEL"
        elif k >= 80:
            state = "YIELD_WAIT"
            
        frame = {
            "timestamp": t,
            "scenario_id": scenario_id,
            "s": s,
            "d": d,
            "theta_road": round(0.02 * math.cos(k * 0.05), 4),
            "kappa": round(0.01 * math.sin(k * 0.05), 4),
            "active_state": state,
            "v_ref": 8.0 if state != "YIELD_WAIT" else 0.0,
            "yield_required": (state in ["YIELD_DECEL", "YIELD_WAIT"]),
            "hold_time_est": round(max(0.0, 4.0 - (k - 60) * dt), 2) if state == "YIELD_WAIT" else 0.0,
            "max_kappa": 0.045,
            "is_collision_free": True,
            "steer_rad": round(0.08 * math.sin(k * 0.1), 4),
            "throttle": round(0.4 if v > 2.0 else 0.0, 2),
            "brake": round(0.6 if state in ["YIELD_DECEL", "YIELD_WAIT"] else 0.0, 2),
            "ego_x": round(s, 2),
            "ego_y": round(d, 2),
            "ego_theta": round(0.02 * math.cos(k * 0.05), 4),
            "ego_v": round(v, 2),
            "cross_track_err": round(d, 3),
            "lon_jerk": round(0.3 * math.sin(k * 0.2), 3),
            "lat_jerk": round(0.2 * math.cos(k * 0.2), 3),
            "obstacles": [
                {
                    "id": 1,
                    "x": round(s + 12.0, 2),
                    "y": 0.5,
                    "vx": -1.2,
                    "vy": 0.0,
                    "cov_major": 1.2,
                    "cov_minor": 0.6,
                    "cov_angle": 0.1
                }
            ]
        }
        frames.append(frame)
        
    out_dir = Path("web_simulation/data")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "scenario_playback.json"
    
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump({"scenario_id": scenario_id, "frames": frames}, f, indent=2)
        
    print(f"[SUCCESS] Generated {len(frames)} synthetic frames in {out_file}")

if __name__ == "__main__":
    generate_mock_telemetry()
```

---

## Data Contract Specifications (`TelemetryFrame`)

Each frame in the exported telemetry array must conform strictly to **Contract 5** of `directives/00_interface_contracts.md`:

```json
{
  "timestamp": 1.45,
  "scenario_id": "SC-1",
  "s": 14.50,
  "d": -0.32,
  "theta_road": 0.015,
  "kappa": 0.008,
  "active_state": "NUDGE",
  "v_ref": 6.50,
  "yield_required": false,
  "hold_time_est": 0.0,
  "max_kappa": 0.035,
  "is_collision_free": true,
  "steer_rad": -0.062,
  "throttle": 0.35,
  "brake": 0.0,
  "ego_x": 14.50,
  "ego_y": -0.32,
  "ego_theta": 0.012,
  "ego_v": 6.42,
  "cross_track_err": -0.041,
  "lon_jerk": 0.24,
  "lat_jerk": 0.18,
  "obstacles": []
}
```

### Critical Rules for JSON Generation:
1. **Zero NaN / Inf / Null:** JSON does not support `NaN` or `Infinity`. Any invalid float is a fatal syntax error in web browsers. If a mathematical calculation produces `NaN`, clamp it to `0.0` or previous valid value.
2. **Numeric Precision:** Round floating point numbers to at most 4 decimal places (e.g. `round(val, 4)`) to prevent bloated multi-megabyte JSON payloads.
3. **Array Structure:** The top-level JSON file must be formatted as:
   ```json
   {
     "scenario_id": "SC-1",
     "total_duration": 20.0,
     "frame_rate_hz": 20,
     "frames": [ { ... }, { ... } ]
   }
   ```

---

## Deliverable Files

You will deliver the following files:

| File | Purpose |
|---|---|
| `execution/export_web_telemetry.py` | Python packaging script converting simulation data to valid JSON |
| `web_simulation/index.html` | Updated interactive visualizer with HUD, gauges, FSM badge, and Frenet box |
| `web_simulation/style.css` (or inline) | Premium dark-mode glassmorphism theme with high-visibility instrument dials |
| `web_simulation/visualizer.js` (or inline) | 60 FPS Canvas rendering loop, EKF ellipse renderer, and playback controller |
| `execution/test_subsystem5_telemetry.py` | Automated unit test suite verifying schema compliance and zero-NaN |

---

## Required Verification Tests (Must Pass Before Merge)

You must run `execution/test_subsystem5_telemetry.py` and verify both tests pass:

### Test 1 — JSON Schema & Zero-NaN Integrity (Automated)
- **Execution:** Run `python execution/test_subsystem5_telemetry.py`
- **Validation Criteria:**
  - Load 100 consecutive frames from `scenario_playback.json`.
  - Validate that 100% of required fields from Contract 5 are present in every frame.
  - Assert that **zero** fields contain `None`, `null`, `"NaN"`, `"Infinity"`, or `-Infinity`.
  - Assert that `active_state` is one of the 8 approved FSM strings:
    `{'CRUISE','NUDGE','YIELD_DECEL','YIELD_WAIT','RESUME','LANE_KEEP','JUNCTION_TURN','HIGHWAY_MERGE'}`.
  - Assert that `throttle` and `brake` are in $[0.0, 1.0]$ and never both positive.
- **Pass Threshold:** 100/100 frames pass validation with 0 errors.

### Test 2 — Web Visualizer 60 FPS Playback Performance (Browser)
- **Execution:** Open `web_simulation/index.html` in Chrome/Firefox with a 500-frame test dataset.
- **Validation Criteria:**
  - Measure frame render time using `requestAnimationFrame()` performance timestamps:
    $$\Delta t_{\text{render}} \le 16.6\text{ ms (sustained } \ge 58\text{ FPS)}$$
  - The vehicle, planned path, Frenet coordinates, and EKF ellipses must render smoothly without UI freeze, stutter, or memory growth.
  - Toggling between Scenarios 1 to 5 via the dropdown must switch datasets within $< 200\text{ ms}$.
- **Pass Threshold:** Sustained $\ge 58\text{ FPS}$ playback across the entire scenario duration.

---

## What NOT to Do (Common Traps)

1. ❌ **Do NOT leave `NaN` in exported JSON.** `NaN` is not valid JSON and will crash `JSON.parse()` in the browser. Always sanitize with `math.isnan()` before serializing.
2. ❌ **Do NOT load heavyweight 3D libraries (Three.js, Babylon).** The visualizer must load instantaneously on any laptop, mobile tablet, or presentation screen without GPU overhead. Use 2D HTML5 Canvas.
3. ❌ **Do NOT change the spelling of FSM states.** The state names must match the exact 8 uppercase strings specified in Contract 2 (`CRUISE`, `NUDGE`, etc.).
4. ❌ **Do NOT hardcode screen dimensions.** Make the canvas responsive to different display aspect ratios and window sizes.
5. ❌ **Do NOT wait for Subsystems 1–4 to finish.** Use `execution/mock_telemetry_generator.py` to build and test the entire UI immediately.

---
