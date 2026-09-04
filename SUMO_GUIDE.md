# Eclipse SUMO Simulation Guide for Indian Road Pathfinder

**SIH Problem Statement 26037 · Adaptive Path Planning for Unstructured Indian Roads**

This branch (`sumo`) integrates the open-source **Eclipse SUMO (Simulation of Urban MObility)** traffic simulator with our adaptive autonomous navigation stack.

---

## 🚀 Quick Start (Under 10 Seconds)

### Option 1: Standalone Autonomous Mode (Fastest & Simplest)
Runs the complete Indian road obstacle scenario in SUMO with the autonomous Python controller (handling obstacle nudge, bottleneck yielding for oncoming rickshaws, and braking for pedestrians):

```powershell
# Run with visual GUI:
python sumo_sim.py --gui

# Or run headless (ultra-fast):
python sumo_sim.py --headless
```

---

### Option 2: MATLAB Co-Simulation via TraCI TCP Bridge
Runs the high-fidelity MATLAB Hybrid A* planner, EKF predictor, and Behavior State Machine connected to SUMO:

#### Step 1: In Terminal 1 (Launch SUMO Bridge Server)
```powershell
python sumo_sim.py --gui --bridge
```
*(SUMO opens the visual window, sets up the Indian road corridor and obstacles, and waits for MATLAB on `127.0.0.1:20000`)*

#### Step 2: In Terminal 2 or MATLAB
```powershell
# Command-line execution:
matlab -batch "cd('matlab'); sumo_simulation_bridge"
```
*Or inside the MATLAB Command Window:*
```matlab
cd('matlab')
sumo_simulation_bridge
```

---

## 🎯 Obstacles Modeled in the SUMO Scenario

| Obstacle | Type | Placement / Dynamics | Required Autonomous Response |
| :--- | :--- | :--- | :--- |
| **Ego Vehicle** | Passenger Car | Starts at $X = 2.0\,\text{m}, Y = -1.75\,\text{m}$ | Navigates corridor to $X = 75.0\,\text{m}$ |
| **Left Roadblock** | Parked / Stationary | Located at $X = 20.0\,\text{m}, Y = -1.75\,\text{m}$ | Ego must plan a nudge into opposing lane |
| **Crossing Pedestrian** | Pedestrian Agent | Crosses road laterally at $X = 28.0\,\text{m}$ | Emergency deceleration / pedestrian yield |
| **Right Hazard** | Debris / Barrier | Located at $X = 38.0\,\text{m}, Y = +1.2\,\text{m}$ | Cleared after roadblock re-centering |
| **Oncoming Auto-Rickshaw** | 3-Wheeler Traffic | Starts at $X = 58.0\,\text{m}, V_x = -3.2\,\text{m/s}$ | Bottleneck decider yields behind roadblock until clear |
| **Potholes** | Surface Hazard | $X=20\,\text{m}, 35\,\text{m}$ (radii $0.8\,\text{m}, 1.0\,\text{m}$) | Dynamic cost penalty avoided by path planner |

---

## 📁 Repository Structure for SUMO

- [`sumo_sim.py`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/sumo_sim.py): Core simulation runner and bridge.
- [`sumo/indian_road.sumocfg`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/sumo/indian_road.sumocfg): SUMO main configuration file.
- [`sumo/indian_road.net.xml`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/sumo/indian_road.net.xml): Undivided 2-lane 7.0m road network.
- [`sumo/indian_road.rou.xml`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/sumo/indian_road.rou.xml): Vehicle types and routing definitions.
- [`sumo/viewsettings.xml`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/sumo/viewsettings.xml): High-contrast GUI camera settings.
- [`matlab/sumo_simulation_bridge.m`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/matlab/sumo_simulation_bridge.m): MATLAB co-simulation driver.
- [`run_sumo_sim.bat`](file:///C:/Users/toshr/.gemini/antigravity-ide/scratch/indian-road-pathfinder/run_sumo_sim.bat): 1-click batch launcher.
