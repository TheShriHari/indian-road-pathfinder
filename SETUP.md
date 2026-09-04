# CARLA Co-Simulation Setup Guide: Two-Computer & All-in-One Setup

**SIH Problem Statement 26037 · Adaptive Path Planning for Unstructured Indian Roads**

This guide provides step-by-step instructions for running the autonomous pathfinding stack across two computers (CARLA on your friend's computer, MATLAB on your computer) or on a single computer.

---

## 1. System Architecture (Two-Computer Network Setup)

Both computers must be connected to the **same local network (Wi-Fi or Ethernet LAN)**.

```
       [ FRIEND'S COMPUTER ]                                   [ YOUR COMPUTER ]
┌──────────────────────────────────────────────┐        ┌──────────────────────────────────────────────┐
│ 1. CARLA Simulator (CarlaUE4.exe)            │        │                                              │
│    Port 2000 · 3D Unreal Engine Simulation   │        │                                              │
│    ▲                                         │        │                                              │
│    │ (Python API)                            │        │                                              │
│    ▼                                         │        │                                              │
│ 2. python obs_sim.py                         │  LAN   │ 3. MATLAB (R2020b+)                          │
│    - Spawns Indian Road Scene (Roadblocks,   │  Wi-Fi │    cd matlab/                                │
│      Cones, Oncoming Rickshaw, Pedestrian)   │ ◄────► │    carla_simulation_bridge.m                 │
│    - Camera Perception (RGB + Depth + YOLO)  │  TCP   │    - BRIDGE_HOST = '<FRIEND_IP>'             │
│    - 3D Spectator Chase Camera View          │ 20000  │    - Real-time EKF Tracking                  │
│    - TCP Socket Bridge Server (0.0.0.0:20000)│        │    - Hybrid A* Arc Search                    │
└──────────────────────────────────────────────┘        │    - Spatio-Temporal Corridor Decider        │
                                                        │    - Pure Pursuit Lateral Control            │
                                                        └──────────────────────────────────────────────┘
```

---

## 2. Prerequisites

### On Friend's Computer (Running CARLA Simulator):
1. **CARLA Simulator** (v0.9.13 – v0.9.16)
2. **Python 3.8 – 3.11** with pip
3. Required Python packages:
   ```powershell
   pip install numpy opencv-python ultralytics
   ```
   *(Note: CARLA Python API egg or wheel must be installed or discoverable)*

### On Your Computer (Running MATLAB Navigation Stack):
1. **MATLAB** (R2020b or later, pure base install — no extra toolboxes needed).
2. Git clone of the repository.

---

## 3. Step-by-Step Execution: Two-Computer Setup

### Part A: On Your Friend's Computer (CARLA Host)

#### Step 1: Find the Friend's Local IP Address
In PowerShell or Command Prompt, run:
```powershell
ipconfig
```
Look for **IPv4 Address** under your active Wi-Fi or Ethernet adapter (e.g. `192.168.1.50`).  
**Note this IP address** — you will enter it in MATLAB.

#### Step 2: Allow Port 20000 in Windows Firewall
To let MATLAB on your PC communicate with the Python bridge on your friend's PC, allow port 20000:
```powershell
# Run PowerShell as Administrator:
New-NetFirewallRule -DisplayName "CARLA-MATLAB-Bridge" -Direction Inbound -LocalPort 20000 -Protocol TCP -Action Allow
```
*(Or click "Allow access" if Windows Defender pops up when the script runs).*

#### Step 3: Launch CARLA Simulator
Navigate to the CARLA root folder and run:
```powershell
cd C:\path\to\CARLA_0.9.X
.\CarlaUE4.exe -windowed -ResX=1280 -ResY=720 -quality-level=Low
```
Wait for the Unreal Engine CARLA window to open and the world to load.

#### Step 4: Launch the Indian Road Scene & Bridge
In a new terminal, navigate to the `indian-road-pathfinder` repository root:
```powershell
# Checkout simulation branch (or pull latest)
git checkout simulation
git pull origin simulation

# Launch the Indian road scene + bridge
python obs_sim.py --show-cam
```
You will see output:
```text
======================================================================
  CARLA INDIAN ROAD SCENE SIMULATOR  (SIH PS-26037)
======================================================================
[CONNECT] Connected to CARLA Server (Map: Town01)
[EGO] Spawned ego vehicle: vehicle.tesla.model3
[SCENE] Generating Indian Road corridor obstacles...
[SCENE] Static Obstacle 1 (Left Roadblock) spawned at +18m
[SCENE] Static Obstacle 2 (Right Hazard) spawned at +38m
[SCENE] Dynamic Obstacle 1 (Oncoming Rickshaw Surrogate) spawned at +55m
[SCENE] Dynamic Obstacle 2 (Crossing Pedestrian/Cattle) spawned at +28m
[SENSORS] Camera stream synchronized successfully.
[BRIDGE READY] Waiting for MATLAB on port 20000...
```

---

### Part B: On Your Computer (MATLAB Navigation Controller)

#### Step 5: Configure the Friend's IP in MATLAB
1. Open MATLAB.
2. Navigate to the `matlab/` folder inside the project:
   ```matlab
   cd('C:/path/to/indian-road-pathfinder/matlab')
   ```
3. Open `carla_simulation_bridge.m`.
4. In the **Config** section near line 42, set `BRIDGE_HOST` to your friend's IP:
   ```matlab
   MODE        = 'LIVE';
   BRIDGE_HOST = '192.168.1.50';   % <-- Replace with friend's IPv4 address
   BRIDGE_PORT = 20000;
   ```

#### Step 6: Run the Controller
In MATLAB command window, run:
```matlab
carla_simulation_bridge
```

---

## 4. What You Will Observe During the Run

### 1. In the CARLA Simulator Window (Friend's Screen):
- The **Spectator Camera** automatically switches into a **third-person 3D chase camera** tracking behind the Tesla Model 3.
- You will see the vehicle drive forward autonomously:
  - It detects the **parked roadblock at +18m** and performs a smooth **rightward nudge**.
  - It evaluates the **oncoming auto-rickshaw/vehicle at +55m** and decelerates if the corridor traversable width drops below safety limits.
  - It navigates past the **construction hazard at +38m** and recovers back to the center corridor.
  - If a crossing pedestrian cuts in front, the vehicle yields at a virtual stop line.

### 2. In the OpenCV Camera Window (Friend's Screen):
- Displays the live forward-facing RGB camera view from the hood of the vehicle with bounding boxes.

### 3. In the MATLAB Telemetry HUD (Your Screen):
- Real-time closed-loop logs printed at 10 Hz:
  ```text
  [t= 2.4s # 24] Pos:[ 12.3,  0.4]  4.2m/s | NUDGE       | δ=+0.12 (κ=0.042) T=0.40 B=0.00 CAM:OK
  [t= 3.8s # 38] Pos:[ 18.5,  0.9]  3.5m/s | YIELD_DECEL | δ=+0.04 (κ=0.015) T=0.00 B=0.65 CAM:OK
  ```

---

## 5. Alternative: Single-Machine Setup (All-in-One)

If running both CARLA and MATLAB on the same machine:
1. Start CARLA: `.\CarlaUE4.exe -windowed -ResX=800 -ResY=600 -quality-level=Low`
2. Start Bridge: `python obs_sim.py --show-cam`
3. In MATLAB `carla_simulation_bridge.m`, keep:
   ```matlab
   BRIDGE_HOST = '127.0.0.1';
   ```
4. Run `carla_simulation_bridge`.

---

## 6. CLI Arguments Reference for `obs_sim.py`

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--carla-host` | `127.0.0.1` | IP address of CARLA server |
| `--carla-port` | `2000` | Port of CARLA server |
| `--matlab-port`| `20000` | Port to accept MATLAB TCP socket connections |
| `--map` | `None` (current) | Specific CARLA map to load (e.g. `Town01`, `Town04`) |
| `--vehicle` | `vehicle.tesla.model3` | Ego vehicle blueprint |
| `--show-cam` | `False` | Opens an OpenCV window showing live camera perception |
| `--standalone` | `False` | Spawns scene only without waiting for MATLAB bridge |

---

## 7. Troubleshooting & Network Verification

1. **Cannot connect from MATLAB to Friend's PC**:
   - Test connectivity from your computer using PowerShell:
     ```powershell
     Test-NetConnection -ComputerName 192.168.1.50 -Port 20000
     ```
   - If `TcpTestSucceeded : False`, check:
     - Friend's Windows Firewall rule for port 20000.
     - Both machines are on the same Wi-Fi network (not on separate guest networks).
2. **CARLA Client/Server Version Mismatch Warning**:
   - If CARLA prints `"WARNING: Client and server versions mismatch"`, this is a standard CARLA notice (e.g. 0.9.15 server vs 0.9.16 client) and does not impede execution.
3. **OpenCV Display Error**:
   - If `cv2.imshow` fails on headless systems, simply omit the `--show-cam` flag.
