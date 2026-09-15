# Webots 3D Simulation Guide: Indian Rural Road Scenario

**SIH Problem Statement 26037 · Adaptive Path Planning for Unstructured Indian Roads**

This module provides a realistic **Cyberbotics Webots 3D simulation environment** tailored specifically for unstructured Indian rural roads. It completely replaces CARLA while offering native 3D visual observation, high-fidelity physics, zero GPU driver lockups, and full co-simulation compatibility with MATLAB and Python.

---

## 1. 3D Scene Architecture (`webots/worlds/indian_rural_road.wbt`)

The scene models a **typical Indian rural road corridor** with realistic roadside features and safety-critical unstructured hazards:

```
[X = 0m]         [X = 18m]           [X = 26m]              [X = 48m]             [X = 65m]              [X = 75m]
 Ego Vehicle  ──► Pothole Crater  ──► Pedestrian 1       ──► Pedestrian 2     ◄── Oncoming Auto-Rickshaw   Goal Target
 (Silver Car)     (In ego lane)       (Crossing Left->Right) (Crossing Right->Left)(Indian 3-Wheeler)     (Arrival)
```

### Road & Environmental Elements
| Element | Coordinates / Dimensions | Visual Representation & Role |
| :--- | :--- | :--- |
| **Rural Asphalt Road** | Length: 140m, Width: 7.0m | Dark bituminous weathered surface with faded broken centerline and edge lines. |
| **Dirt Shoulders** | Width: 2.5m (Left & Right) | Unpaved mud/gravel shoulders (`baseColor: earthy brown`). |
| **Trees & Flora** | X = 5m to 70m along shoulders | Roadside neem/banyan-style trees with natural canopies providing visual depth. |
| **Milestone** | X = 10m, Y = -4.2m | Standard Indian yellow-and-white rounded highway distance marker. |
| **Ego Vehicle** | X = 0.0m, Y = -1.75m (Left Lane) | Autonomous sedan with roof-mounted LiDAR pod, headlights, brake lights, and third-person chase camera. |
| **Pothole Hazard** | X = 18.0m, Y = -1.20m (Ego Lane) | Recessed road crater with dark cracked bitumen rim (0.7m radius) and deep puddle/depression. |
| **Pedestrian 1** | X = 26.0m (Crosses Y: +3.8m ➔ -3.8m) | Rural pedestrian in blue kurta crossing from the left shoulder across ego's path. |
| **Pedestrian 2** | X = 48.0m (Crosses Y: -3.8m ➔ +3.8m) | Pedestrian in saffron/orange shirt crossing from the right shoulder. |
| **Oncoming Auto-Rickshaw**| X = 65.0m, Y = +1.75m (Opposite Lane) | Iconic Indian 3-wheeler (green chassis, bright yellow canopy roof, single front wheel) cruising at ~3.6 m/s in -X direction. |

---

## 2. Quick Start: Launching & Observing the Simulation

### Option A: Standalone Autonomous Mode (Pure Python + Webots 3D GUI)
Open PowerShell or Command Prompt in the repository root:

```powershell
# Using the PowerShell launcher
.\run_webots_sim.ps1

# Or using the Windows Batch launcher
.\run_webots_sim.bat

# Or direct Python
python webots_sim.py
```

**What you will observe in Webots:**
1. The Webots 3D viewport opens with the camera tracking the ego vehicle from a third-person chase angle.
2. The ego car accelerates along the rural road at ~5.0 m/s.
3. At X = 10m, the planner detects the **pothole hazard** at X = 18m and executes a smooth **`NUDGE_RIGHT` swerve** into the center corridor, avoiding chassis impact.
4. Near X = 20m, **Pedestrian 1** steps onto the road from the left. The vehicle transitions to **`YIELD_PEDESTRIAN`**, decelerates, and waits for safe clearance.
5. As the vehicle resumes cruise, the **oncoming auto-rickshaw** approaches on the opposite lane. The bottleneck decider ensures lateral clearance.
6. Near X = 45m, **Pedestrian 2** crosses from the right. The ego car safely negotiates and halts/yields if necessary before crossing the finish line at **X = 75m (`[SUCCESS]`)**.

---

### Option B: MATLAB Co-Simulation Mode (Hybrid A* + EKF + BSM)

In this mode, MATLAB executes your complete path planning algorithm suite while Webots provides the 3D physics, sensor telemetry, and visual feedback.

#### Terminal 1: Start Webots with Bridge Server
```powershell
python webots_sim.py --bridge
```
*(Webots starts in 3D and listens on TCP port `20000`)*

#### Terminal 2: Run MATLAB Co-Simulation
```matlab
% In MATLAB Command Window:
cd matlab
webots_simulation_bridge
```
*(Or from PowerShell directly:)*
```powershell
matlab -batch "cd('matlab'); webots_simulation_bridge"
```

---

## 3. Command-Line Options for `webots_sim.py`

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--mode` | `realtime` | Startup execution mode: `realtime`, `fast`, or `pause`. |
| `--bridge` | Disabled | Enables TCP socket server on port `20000` to interface with MATLAB. |
| `--batch` | `True` | Prevents blocking GUI modal popups on startup. |
| `--no-rendering` | Disabled | Disables 3D viewport rendering for ultra-fast headless benchmark runs. |
| `--world` | `indian_rural_road.wbt` | Explicit path to a custom Webots world file. |

---

## 4. File Layout

```
indian-road-pathfinder/
├── webots/
│   ├── worlds/
│   │   └── indian_rural_road.wbt      # 3D Indian Rural Road world description
│   └── controllers/
│       └── indian_road_supervisor/
│           └── indian_road_supervisor.py  # Webots Supervisor (agents, physics, bridge)
├── matlab/
│   └── webots_simulation_bridge.m    # MATLAB co-simulation client
├── webots_sim.py                     # Universal launcher & bridge manager
├── run_webots_sim.bat                 # 1-Click Windows batch launcher
├── run_webots_sim.ps1                 # PowerShell launcher
└── WEBOTS_GUIDE.md                   # This documentation
```
