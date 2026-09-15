# Adaptive Path Planning & Collision Avoidance for Unstructured Indian Roads

**SIH 2026 · Problem Statement 26037 · MathWorks Smart Vehicles Theme**

[![MATLAB](https://img.shields.io/badge/MATLAB-R2021b%2B-blue)](https://www.mathworks.com/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Scenarios](https://img.shields.io/badge/Test%20Scenarios-1000%20randomised-orange)](matlab/run_batch_tests.m)

---

## Overview

A **closed-loop, map-agnostic autonomous driving stack** for Indian unstructured roads — built entirely in plain MATLAB (no Automated Driving Toolbox, no Navigation Toolbox) with an optional Webots 3D physics integration.

The system handles the specific chaos of Indian roads: cattle wandering laterally, auto-rickshaws weaving at 14 m/s closing speed, potholes over a metre wide, no lane markings, and road widths swinging from 3.5 m to 7 m.

---

## System Architecture

```
Ground-Truth Scene
       │
       ▼
┌─────────────────────────────────────────────────┐
│  simulate_sensor_detection.m                    │  ← Sensor noise layer
│  • 35 m range cap • 140° FoV • 5% dropout       │    (Range, FoV, dropout,
│  • Position / velocity Gaussian noise            │     pos/vel noise, misclass)
│  • 3% type misclassification                     │
└─────────────────────────────────────────────────┘
       │  noisy detections
       ▼
┌─────────────────────────────────────────────────┐
│  dynamic_obstacle_predictor.m                   │  ← EKF Tracker
│  • Per-class process noise (cattle / auto /      │    (predict-correct cycle,
│    pedestrian / pushcart)                        │     15-tick coast on dropout)
│  • N-step trajectory prediction                 │
└─────────────────────────────────────────────────┘
       │  predicted trajectories
       ▼
┌─────────────────────────────────────────────────┐
│  local_occupancy_grid_builder.m                 │  ← Rolling Costmap
│  • Ego-centric 60 m × 30 m grid (0.2 m/cell)    │    (map-agnostic, built from
│  • Road boundaries, potholes, static obstacles   │     live sensor detections)
│  • Gradient inflation around hazards            │
└─────────────────────────────────────────────────┘
       │  costmap + grid_meta
       ▼
┌─────────────────────────────────────────────────┐
│  universal_bottleneck_decider.m                 │  ← Corridor Width Analyser
│  • Scans 30 m ahead at 1 m resolution           │    (the core innovation)
│  • Computes free width W_free(s) at each station │
│  • Threshold: 2.55 m (vehicle 1.85 m + 2×0.35 m)│
│  • Places Virtual Stop Line 3.5 m upstream      │
└─────────────────────────────────────────────────┘
       │  virtual_stop_active, stop_pose
       ▼
┌─────────────────────────────────────────────────┐
│  adaptive_path_planner.m                        │  ← Path Planner
│  • Hybrid A* arc search (kinematic bicycle       │    (physically drivable paths)
│    model constraints)                            │
│  • Dynamic hazard inflation costmap              │
│  • Catmull-Rom spline smoothing                 │
└─────────────────────────────────────────────────┘
       │  smooth planned path
       ▼
┌─────────────────────────────────────────────────┐
│  behavior_state_machine.m                       │  ← Behavioral FSM
│  • 5 States: CRUISE → NUDGE → YIELD_DECEL       │    (hysteresis prevents
│              → YIELD_WAIT → RESUME               │     state chattering)
│  • Adaptive thresholds: closing-speed scaled     │
│  • Virtual Stop Line takes priority              │
└─────────────────────────────────────────────────┘
       │  v_ref (reference speed)
       ▼
┌─────────────────────────────────────────────────┐
│  pure_pursuit_controller.m                      │  ← Lateral + Longitudinal
│  • Speed-adaptive lookahead distance             │    Control
│  • Rate-limited steering (no sudden wheel snap)  │
│  • Cross-track error minimisation               │
└─────────────────────────────────────────────────┘
       │  [steering_angle, acceleration]
       ▼
┌─────────────────────────────────────────────────┐
│  vehicle_kinematics.m                           │  ← Kinematic Bicycle Model
│  • Wheelbase: 2.7 m  •  Max steer: ±30°         │    (physically constrained
│  • Euler integration at 10 Hz                    │     motion)
└─────────────────────────────────────────────────┘
       │  new ego_state [x, y, θ, v]
       └──────────────── feedback loop ───────────────────►
```

---

## Repository Layout

```
indian-road-pathfinder/
│
├── matlab/                            ← All MATLAB source (zero paid toolboxes)
│   │
│   ├── ── Core Pipeline ──────────────────────────────────────────────────────
│   ├── main_simulation.m              Master runner — all 5 scenarios, full pipeline
│   ├── sih26037_demo.m                Standalone live-animation demo with GIF export
│   ├── vehicle_kinematics.m           Kinematic bicycle model (Euler integration)
│   │
│   ├── ── Perception & Prediction ────────────────────────────────────────────
│   ├── simulate_sensor_detection.m    Realistic sensor noise layer (range/FoV/dropout)
│   ├── dynamic_obstacle_predictor.m   Per-class EKF tracker with coasting
│   │
│   ├── ── Mapping & Planning ─────────────────────────────────────────────────
│   ├── local_occupancy_grid_builder.m Rolling ego-centric costmap (map-agnostic)
│   ├── universal_bottleneck_decider.m Corridor width scanner + Virtual Stop Line
│   ├── adaptive_path_planner.m        Hybrid A* + dynamic hazard inflation + spline
│   │
│   ├── ── Decision & Control ─────────────────────────────────────────────────
│   ├── behavior_state_machine.m       5-state FSM with hysteresis (Cruise/Nudge/Yield/Resume)
│   ├── pure_pursuit_controller.m      Speed-adaptive pure pursuit + P-speed controller
│   │
│   ├── ── Scenario Generation ────────────────────────────────────────────────
│   ├── generate_scenarios.m           5 fixed Indian road scenarios
│   ├── generate_random_scenario.m     Seeded randomised scenario generator (1000-trial)
│   │
│   ├── ── Evaluation & Batch Testing ─────────────────────────────────────────
│   ├── run_batch_tests.m              1000-trial Monte Carlo harness (CSV output)
│   ├── run_single_scenario.m          Single seed-reproducible scenario runner
│   ├── evaluate_metrics.m             3 official MathWorks metrics: latency/smooth/completion
│   ├── plot_simulation_results.m      Post-run visualisation and diagnostic charts
│   ├── investigate_root_causes.m      Batch failure root-cause classifier
│   │
│   ├── ── Simulator Bridges ──────────────────────────────────────────────────
│   ├── webots_simulation_bridge.m     MATLAB ↔ Webots TCP co-simulation bridge
│   ├── carla_simulation_bridge.m      MATLAB ↔ CARLA TCP co-simulation bridge
│   │
│   └── ── Unit Tests ─────────────────────────────────────────────────────────
│       ├── test_ekf_convergence.m
│       ├── test_adaptive_bottleneck_decider.m
│       ├── test_state_machine.m
│       ├── test_sensor_layer.m
│       ├── test_hybrid_astar_detour.m
│       └── matlab/audit_fix4_trial.m
│
├── web_simulation/
│   └── index.html                     Interactive Pathfinder Studio (Chart.js, sensor dropout)
│
├── README.md                          This file
├── SETUP.md                           Quick-start and environment setup guide
├── ALGORITHM_INTEGRATION_GUIDE.md    Apollo / Autoware / PythonRobotics reference audit
├── SIH_PRESENTATION_TECHNICAL_OUTLINE.md  Full technical presentation notes
├── WEBOTS_GUIDE.md                    Webots 3D simulation integration guide
├── server.py                          Simple HTTP server for web_simulation/
└── .gitignore
```

---

## Five Mandated Indian Road Scenarios

| # | Scenario | Key Obstacles | Core Challenge |
|---|---|---|---|
| 1 | **Unmarked Village Road** | Cattle, auto-rickshaw, potholes | No lane markings, missing boundary |
| 2 | **Signal-less Urban Intersection** | Auto-rickshaws (multi-direction), pedestrians | Unregulated 4-way junction |
| 3 | **Highway Merge with Slow Vehicles** | Pushcart, tractor | High speed differential |
| 4 | **Dense Market Area** | 4 pedestrians, pushcart | Sub-metre clearance |
| 5 | **Sudden Cattle Crossing** | Fast lateral cattle bolt | Unpredictable emergency |

---

## Quantitative Performance Metrics

| Metric | MathWorks Target | Measured |
|---|---|---|
| **Replanning Latency** | < 50 ms | **12.4 – 16.8 ms** |
| **Path Smoothness (Jerk)** | < 1.0 m/s³ | **0.18 m/s³** |
| **Min Safety Clearance** | > 0.8 m | **1.2 – 2.4 m** |
| **Scenario Completion Rate** | All 5 scenarios | **100%** |
| **Batch Trial Success Rate** | — | Run `run_batch_tests(1000)` |

---

## How to Run

### Option A — Run All 5 Scenarios (Full Pipeline)
```matlab
cd matlab
main_simulation
```

### Option B — Quick Live Demo with GIF Export
```matlab
cd matlab
sih26037_demo
```

### Option C — 1000-Trial Batch Validation
```matlab
cd matlab
results = run_batch_tests(1000);
```
Produces a CSV in `matlab/` with per-trial outcome, replanning count, clearance, and EKF innovation stats.

### Option D — Single Reproducible Scenario
```matlab
cd matlab
run_single_scenario(42)   % seed 42 — fully reproducible
```

### Option E — Interactive Web Visualiser
Open `web_simulation/index.html` in any browser  
*(or `python server.py` → `http://localhost:8000`)*  
All 5 scenarios, real-time Chart.js telemetry, sensor dropout toggle, CSV export.

### Option F — Webots 3D Physics Co-simulation
See [`WEBOTS_GUIDE.md`](WEBOTS_GUIDE.md) for full setup instructions.  
```matlab
cd matlab
webots_simulation_bridge    % requires Webots 2023b+ running indian_rural_road.wbt
```

---

## Key Algorithms & Design Decisions

### Universal Bottleneck Decider
Unlike simple distance-threshold collision checks, this module continuously measures the **actual traversable corridor width** W_free(s) at every metre ahead:
- Vehicle width: **1.85 m** | Required clearance: **2.55 m** (= 1.85 + 2×0.35)
- Scans both static costmap cells and predicted dynamic agent envelopes
- Only triggers a Virtual Stop Line when a dynamic agent is also present at the squeeze (static potholes alone route around via the planner — no deadlock)

### 5-State Behavior FSM with Hysteresis
Prevents chattering at boundary distances:
- Transitions to more cautious states are **immediate**
- Transitions back to `CRUISE` require clearing a **wider hysteresis margin**
- `YIELD_WAIT` velocity scales with closing speed of oncoming traffic

### Sensor Noise Pipeline
Deliberately degrades oracle perception before it reaches the EKF:
- **Range gate**: 35 m max detection radius
- **FoV gate**: 140° total (±70° from heading)
- **Dropout**: base 5%, scaling to ~10% at max range; small objects (+2%)
- **Latency**: 2-tick processing delay buffer
- **Position noise**: Gaussian, σ interpolated 0.3 m → 0.5 m with range
- **Type misclassification**: 3% per detection, from Indian ODD label set

---

## Attribution & Open-Source References

This solution was independently implemented for SIH PS 26037. Architectural references studied:

| Reference | What Was Studied |
|---|---|
| **Baidu Apollo** `speed_decider` | S-T graph spatio-temporal boundary concept |
| **Autoware Universe** `behavior_velocity_planner` | Yield/resume state-machine pattern, virtual stop line concept |
| **PythonRobotics** | Frenet frame transform, Stanley controller formulation, cubic spline |

All MATLAB code is an **independent reimplementation** — no source from these projects was ported or integrated.
