# CARLA Simulation Setup Guide (Option A: All-in-One Machine)

This guide walks through setting up and running the autonomous driving navigation stack (MATLAB + Python bridge) alongside the CARLA simulator on a single machine.

---

## 1. Prerequisites

Make sure the following tools and software are installed on your machine:

1. **CARLA Simulator** (v0.9.13 or compatible)
2. **Python** (v3.8 – 3.11 with `pip`)
3. **MATLAB** (R2020b or later)
4. **Git**

---

## 2. Clone the Repository

Clone the project and checkout the `carla` branch:

```bash
git clone -b carla https://github.com/TheShriHari/indian-road-pathfinder.git
cd indian-road-pathfinder
```

---

## 3. Install Python Dependencies

Install the required Python packages for the bridge:

```bash
pip install numpy opencv-python
```

> **Note:** Ensure your CARLA Python API egg/whl is discoverable by Python. If you installed CARLA, this is usually found in `<CARLA_DIR>/PythonAPI/carla/dist/`. You can either add it to your `PYTHONPATH` or copy the egg/whl to your Python environment.

---

## 4. Execution Steps

### Step 1: Start the CARLA Simulator
Open a terminal, navigate to your CARLA root directory, and launch the simulator:

```powershell
# Windows
cd C:\path\to\CARLA_0.9.X
.\CarlaUE4.exe -windowed -ResX=800 -ResY=600 -quality-level=Low
```

```bash
# Linux
cd /path/to/CARLA_0.9.X
./CarlaUE4.sh -windowed -ResX=800 -ResY=600 -quality-level=Low
```

Wait for the CARLA viewport to open and the world to finish loading.

---

### Step 2: Start the Python Bridge (`carla_bridge.py`)
Open a new terminal, navigate to the cloned repository root, and run:

```bash
python carla_bridge.py --show-cam
```

*Optional flags:*
- `--map Town01` (or `Town02`, `Town04`, etc. to select a specific map)
- `--vehicle vehicle.tesla.model3` (choose ego vehicle blueprint)
- `--matlab-port 20000` (default port)

You should see:
```text
[CARLA] Connected — map: Town01
[CARLA] Ego vehicle spawned: vehicle.tesla.model3
[SENSORS] RGB camera spawned
[SENSORS] Collision sensor spawned
[CARLA] Camera ready.
[BRIDGE] Listening for MATLAB on port 20000...
```

---

### Step 3: Run the MATLAB Navigation Stack (`carla_simulation_bridge.m`)
1. Open MATLAB.
2. In the MATLAB file browser or command window, navigate to the `matlab/` directory:
   ```matlab
   cd('C:/path/to/indian-road-pathfinder/matlab')
   ```
3. Open `carla_simulation_bridge.m` and ensure `MODE` is set to `'LIVE'`:
   ```matlab
   MODE = 'LIVE';
   BRIDGE_HOST = '127.0.0.1';
   BRIDGE_PORT = 20000;
   ```
4. Run the script:
   ```matlab
   carla_simulation_bridge
   ```

---

## 5. What to Observe

1. **Terminal / MATLAB Window**:
   You will see real-time closed-loop logs showing ego pose, speed, state machine status (`CRUISE`, `NUDGE`, `YIELD`), steering angle ($\delta$), curvature ($\kappa$), and camera status:
   ```text
   [t=  0.5s #  5] Pos:[  2.4,  0.0]  3.8m/s | CRUISE      | δ=+0.02 (κ=0.007) T=0.45 B=0.00 CAM:OK
   ```
2. **OpenCV Camera Window**:
   Displays the live forward-facing RGB camera view from the hood of the ego vehicle.
3. **CARLA Viewport**:
   The ego vehicle will drive autonomously, avoiding pedestrians, cattle, or opposing vehicles and pausing at virtual stop lines if narrow corridors/bottlenecks are detected.
