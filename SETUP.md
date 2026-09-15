# Setup & Quick-Start Guide

**SIH PS 26037 · Adaptive Path Planning for Unstructured Indian Roads**

---

## Requirements

| Tool | Version | Notes |
|---|---|---|
| **MATLAB** | R2021b or later | Base install only — no paid toolboxes required |
| **MATLAB Online** | Any | Works identically — upload the `matlab/` folder |
| **Webots** *(optional)* | 2023b+ | Only for 3D physics co-simulation |
| **Python** *(optional)* | 3.8+ | Only for `server.py` web visualiser or CARLA/Webots bridge |

> All core simulation runs on **base MATLAB** with no toolbox dependencies.  
> The only MATLAB built-ins used: `binaryOccupancyMap`, `plannerAStarGrid` (for the demo scripts only — the full pipeline uses pure algorithmic implementations).

---

## Quickstart — MATLAB Simulation

### 1. Clone the repository
```powershell
git clone https://github.com/TheShriHari/indian-road-pathfinder.git
cd indian-road-pathfinder
```

### 2. Open MATLAB and navigate to the project
```matlab
cd('path\to\indian-road-pathfinder\matlab')
```

### 3. Run the full 5-scenario pipeline
```matlab
main_simulation
```
Runs all 5 Indian road scenarios end-to-end through the complete 7-module pipeline and prints an evaluation metrics report.

### 4. Run the 1000-trial batch validator
```matlab
results = run_batch_tests(1000);
```
Generates a `batch_test_results.csv` with per-trial outcome, replanning count, clearance, EKF innovation stats, and dropout counts. Takes ~5–10 minutes.

### 5. Run a single reproducible scenario
```matlab
run_single_scenario(42)   % seed 42 is fully reproducible
```

### 6. Run the live-animation demo (saves GIF)
```matlab
sih26037_demo
```

---

## Module Test Scripts

Run these individually to validate each pipeline module in isolation:

| Script | What It Tests |
|---|---|
| `test_ekf_convergence.m` | EKF innovation shrinks over time (filter converging) |
| `test_adaptive_bottleneck_decider.m` | Corridor width detection and Virtual Stop Line placement |
| `test_state_machine.m` | All 5 FSM state transitions and hysteresis |
| `test_sensor_layer.m` | Dropout rate, noise std, FoV/range gating |
| `test_hybrid_astar_detour.m` | Planner finds detour path around obstacle |

```matlab
% Run all unit tests
test_ekf_convergence
test_adaptive_bottleneck_decider
test_state_machine
test_sensor_layer
test_hybrid_astar_detour
```

---

## Interactive Web Visualiser

No MATLAB needed — runs in any browser.

```powershell
# Option A: direct file open
start web_simulation\index.html

# Option B: local server (recommended for Chrome)
python server.py
# Then open: http://localhost:8000
```

Features:
- All 5 Indian road scenarios
- Real-time Chart.js telemetry (latency, smoothness, clearance)
- Sensor dropout toggle
- CSV telemetry export
- Drag-and-drop obstacle spawning

---

## Webots 3D Physics Co-simulation *(Optional)*

See [`WEBOTS_GUIDE.md`](WEBOTS_GUIDE.md) for full instructions.

### Prerequisites
1. Install [Webots 2023b+](https://cyberbotics.com/)
2. Open `webots/worlds/indian_rural_road.wbt` in Webots
3. Start the simulation (press Play)

### Connect MATLAB
```matlab
cd matlab
webots_simulation_bridge    % connects to Webots on localhost:10020
```

The bridge streams sensor data from Webots → MATLAB planning stack → steering/throttle commands back to Webots at 50 Hz.

---

## File Reference

### Core Pipeline (run in order by `main_simulation.m`)

| File | Role | Key Parameters |
|---|---|---|
| `simulate_sensor_detection.m` | Sensor noise layer | `max_range=35m`, `FoV=140°`, `dropout=5%` |
| `dynamic_obstacle_predictor.m` | EKF tracker | Per-class Q matrix, 15-tick coast |
| `local_occupancy_grid_builder.m` | Rolling costmap | `res=0.2m`, `range_fwd=50m` |
| `universal_bottleneck_decider.m` | Corridor width | `vehicle_width=1.85m`, `min_clearance=0.35m` |
| `adaptive_path_planner.m` | Path planner | Hybrid A* + Catmull-Rom spline |
| `behavior_state_machine.m` | Behavioral FSM | 5 states, hysteresis thresholds |
| `pure_pursuit_controller.m` | Lateral+speed control | `k_lookahead=0.5`, `Kp_v=1.0` |
| `vehicle_kinematics.m` | Bicycle model | `L=2.7m`, `max_steer=30°` |

### Scenario & Evaluation

| File | Role |
|---|---|
| `generate_scenarios.m` | 5 fixed Indian road scenarios |
| `generate_random_scenario.m` | Seeded random scenario (potholes + dynamic agents) |
| `run_batch_tests.m` | 1000-trial Monte Carlo harness |
| `run_single_scenario.m` | Single reproducible run |
| `evaluate_metrics.m` | Official MathWorks metrics |
| `investigate_root_causes.m` | Batch failure classifier |

---

## Troubleshooting

### `binaryOccupancyMap` not found
This is in the Navigation Toolbox. The main pipeline (`main_simulation.m`) does **not** use it. Only the standalone demo (`sih26037_demo.m`) uses it. Either:
- Use MATLAB Online (toolboxes included), or
- Run `main_simulation.m` instead (no toolbox calls)

### Webots connection refused
- Ensure Webots is running and the world is playing (not paused)
- Check the port in `webots_simulation_bridge.m` matches Webots External Controller port (default `10020`)

### Batch test runs slowly
The mock closed-loop in `run_batch_tests` has no graphics overhead. If it's slow, reduce `MAX_STEPS` inside the script (default: 400 steps per trial) or run fewer trials:
```matlab
results = run_batch_tests(100);   % quick 100-trial smoke test
```
