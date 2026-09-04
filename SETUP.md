# Webots 3D, Eclipse SUMO & MATLAB Simulation Setup Guide

**SIH Problem Statement 26037 · Adaptive Path Planning for Unstructured Indian Roads**

This guide provides the exact commands to run the **Cyberbotics Webots 3D simulation** or **Eclipse SUMO** environment, either **standalone in Python** or **co-simulated with MATLAB**.

---

## 📋 Quick Table of Options

| Simulation Platform | Standalone Autonomous Mode | MATLAB Co-Simulation Mode |
| :--- | :--- | :--- |
| **Webots 3D Simulator (Recommended)** | `python webots_sim.py`<br>*(or `.\run_webots_sim.bat`)* | **Terminal 1**: `python webots_sim.py --bridge`<br>**Terminal 2**: `matlab -batch "cd('matlab'); webots_simulation_bridge"` |
| **Eclipse SUMO Simulator** | `python sumo_sim.py --gui` | **Terminal 1**: `python sumo_sim.py --gui --bridge`<br>**Terminal 2**: `matlab -batch "cd('matlab'); sumo_simulation_bridge"` |


---

## 🛠️ Step 1: Clone the `sumo` Branch

### Fresh Clone on any Computer:
```powershell
git clone -b sumo https://github.com/TheShriHari/indian-road-pathfinder.git
cd indian-road-pathfinder
```

### If you already have the repository cloned:
```powershell
cd indian-road-pathfinder
git fetch origin
git checkout sumo
git pull origin sumo
```

---

## 📦 Step 2: Install SUMO & Python TraCI Packages

Run this in PowerShell or Command Prompt. It installs the complete official SUMO binaries (`sumo.exe`, `sumo-gui.exe`, `netconvert.exe`) and Python TraCI libraries:

```powershell
pip install eclipse-sumo traci sumolib
```

*(No admin rights or external installer required; PyPI bundles precompiled binaries!)*

---

## 🚀 Step 3: Run the Simulation

### Option A: Standalone Autonomous Mode (Fastest — No MATLAB Needed)

Runs the entire Indian road obstacle scenario in SUMO with autonomous path planning, nudging, and yielding directly in Python:

```powershell
# 1. Run with visual SUMO-GUI window:
python sumo_sim.py --gui

# 2. Or run the 1-click batch launcher:
.\run_sumo_sim.bat

# 3. Or run headless in background (fastest batch execution):
python sumo_sim.py --headless
```

---

### 🌾 Option B: 10 Rural Road Scenario Combinations (2 Pedestrians + 1 Pothole + 1 Oncoming Auto)

Runs the 10 distinct rural road interaction scenarios testing multiple crossing speeds, dynamic yielding, and pothole detours:

```powershell
# Run all 10 combinations with visual GUI:
python sumo_rural_batch.py --gui

# Or 1-click batch launcher:
.\run_rural_10_trials.bat

# Or run headless (fastest automated benchmark):
python sumo_rural_batch.py --headless

# Or run a single specific trial (e.g. combo 3):
python sumo_rural_batch.py --gui --trial 3
```

*(Results will automatically be saved to `sumo_rural_10_trials.csv` and summarized in your console!)*

---

### Option C: Full Co-Simulation (SUMO GUI + Python + MATLAB)

Runs the 3D/2D SUMO world while MATLAB executes the **Hybrid A* Planner**, **EKF Trajectory Predictor**, and **Behavior State Machine**:

#### Terminal 1: Launch SUMO & Python Bridge Server
In PowerShell from the repository root:
```powershell
python sumo_sim.py --gui --bridge
```
> *(This launches the visual SUMO GUI, sets up the 2-lane Indian road corridor, spawns the roadblocks, pedestrian, and oncoming rickshaw, and listens on port `20000`)*

#### Terminal 2: Run the MATLAB Navigation Stack
**From PowerShell / Command Prompt:**
```powershell
matlab -batch "cd('matlab'); sumo_simulation_bridge"
```

**Or directly inside the MATLAB GUI Command Window:**
```matlab
cd('matlab')
sumo_simulation_bridge
```

---

## 🎯 Obstacles Modeled in the SUMO Scenario

| Obstacle | Type | Placement / Dynamics | Required Autonomous Response |
| :--- | :--- | :--- | :--- |
| **Ego Vehicle** | Passenger Car | Starts at $X = 2.0\,\text{m}, Y = -1.75\,\text{m}$ | Navigates corridor to $X = 75.0\,\text{m}$ |
| **Left Roadblock** | Parked / Stationary | Located at $X = 20.0\,\text{m}, Y = -1.75\,\text{m}$ | Ego plans a nudge into opposing lane |
| **Crossing Pedestrian** | Pedestrian Agent | Crosses road laterally at $X = 28.0\,\text{m}$ | Emergency deceleration / pedestrian yield |
| **Right Hazard** | Debris / Barrier | Located at $X = 38.0\,\text{m}, Y = +1.2\,\text{m}$ | Cleared after roadblock re-centering |
| **Oncoming Auto-Rickshaw** | 3-Wheeler Traffic | Starts at $X = 58.0\,\text{m}, V_x = -3.2\,\text{m/s}$ | Bottleneck decider yields behind roadblock until clear |
| **Potholes** | Surface Hazard | $X=20\,\text{m}, 35\,\text{m}$ (radii $0.8\,\text{m}, 1.0\,\text{m}$) | Dynamic cost penalty avoided by path planner |

---

## 🧹 Troubleshooting & Clean Restart

If a simulation is interrupted and port `20000` or a background SUMO process remains open:

### PowerShell:
```powershell
Stop-Process -Name sumo, sumo-gui, python, MATLAB -Force -ErrorAction SilentlyContinue
```

### Windows Command Prompt (CMD):
```cmd
taskkill /F /IM sumo.exe /IM sumo-gui.exe /IM python.exe /IM MATLAB.exe /T 2>nul
```
