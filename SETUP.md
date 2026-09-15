# CARLA Simulation Execution Guide: Single-Machine & Two-Computer Setups

**SIH Problem Statement 26037 · Adaptive Path Planning for Unstructured Indian Roads**

You can run this project in two ways:
1. **Option 1 (Simplest): Everything runs on your friend's computer** (no network connection between two computers needed!).
2. **Option 2: Two computers connected over Wi-Fi/LAN** (CARLA on friend's PC, MATLAB on your PC).

---

## Quick Summary of Options

| Scenario | What to Run on Friend's Computer | What to Run on Your Computer | Network Connection Needed? |
| :--- | :--- | :--- | :--- |
| **Option 1A: Friend has CARLA + MATLAB** | 1. CARLA Simulator<br>2. `python obs_sim.py --show-cam`<br>3. MATLAB: `carla_simulation_bridge` | *Nothing* | **No** (runs on localhost `127.0.0.1`) |
| **Option 1B: Friend has CARLA only (No MATLAB)** | 1. CARLA Simulator<br>2. `python obs_sim.py --autonomous --show-cam` | *Nothing* | **No** (pure Python autonomous navigation!) |
| **Option 2: CARLA on Friend's PC, MATLAB on Your PC** | 1. CARLA Simulator<br>2. `python obs_sim.py --show-cam` | MATLAB: `carla_simulation_bridge`<br>(set `BRIDGE_HOST = '<FRIEND_IP>'`) | **Yes** (Same Wi-Fi or LAN network) |

---

## Option 1: Everything Runs on Your Friend's Computer (No Network Needed)

Your friend clones this repository directly onto their computer where CARLA is installed:
```powershell
git clone https://github.com/TheShriHari/indian-road-pathfinder.git
cd indian-road-pathfinder
git checkout simulation
```

### Sub-Option 1A: If Your Friend Has MATLAB Installed

#### Step 1: Launch CARLA Simulator in Low-Graphics / Low-Effort Mode
To prevent GPU overload, VRAM exhaustion, and Unreal Engine `D3D device being lost` crashes:
```powershell
# Option A: Use the included Python launcher (auto-detects CarlaUE4)
python start_carla.py

# Option B: Run the batch or PowerShell script
.\start_carla_low_effort.bat
# or: .\start_carla_low_effort.ps1

# Option C: Manual launch with recommended low-effort flags
cd C:\path\to\CARLA_0.9.X
.\CarlaUE4.exe -dx11 -quality-level=Low -benchmark -fps=20 -windowed -ResX=800 -ResY=600
```
*(Tip: Add `-RenderOffScreen` if you want headless mode with zero window rendering overhead!)*

#### Step 2: Launch the Scene & Bridge Server (Terminal 1)
In the repository root:
```powershell
python obs_sim.py --show-cam
```
This spawns the Indian road scene (ego vehicle, left roadblock at +18m, right hazard at +38m, oncoming auto-rickshaw at +55m, and crossing pedestrian at +28m), attaches the camera sensor suite, positions the 3D chase spectator camera, and waits for MATLAB on localhost (`127.0.0.1:20000`).

#### Step 3: Launch the MATLAB Navigation Stack (MATLAB Window)
Open MATLAB on the same computer:
```matlab
cd('C:/path/to/indian-road-pathfinder/matlab')
carla_simulation_bridge
```
*(By default `BRIDGE_HOST = '127.0.0.1'`, so it connects immediately on the same machine!)*

---

### Sub-Option 1B: If Your Friend Does NOT Have MATLAB Installed

If your friend does not have MATLAB installed, they can run the **standalone autonomous Python navigation controller** directly:

#### Step 1: Launch CARLA Simulator in Low-Graphics Mode
```powershell
python start_carla.py
# Or: .\start_carla_low_effort.bat
```

#### Step 2: Run Autonomous Python Navigation
In the repository root:
```powershell
python obs_sim.py --autonomous --show-cam
```
- The car uses neural camera perception (YOLOv8 + depth back-projection) to detect roadblocks and traffic.
- It autonomously tracks the road corridor, executes rightward nudges past the left roadblock, slows down/yields for oncoming vehicles, and brakes for pedestrians.
- The CARLA Unreal Engine window automatically tracks the vehicle in 3D third-person view!

---

## Option 2: Two Computers Connected Over Local Network

If you want to run **CARLA on your friend's PC** while running **MATLAB on your PC**:

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

### Steps for Option 2:

1. **On Friend's PC**: Connect to the same Wi-Fi. In PowerShell, type `ipconfig` to find friend's IPv4 address (e.g. `192.168.1.50`).
2. **On Friend's PC**: Allow port 20000 in Windows Firewall:
   ```powershell
   New-NetFirewallRule -DisplayName "CARLA-MATLAB-Bridge" -Direction Inbound -LocalPort 20000 -Protocol TCP -Action Allow
   ```
3. **On Friend's PC**: Start CARLA and run `python obs_sim.py --show-cam`.
4. **On Your PC**: Open `matlab/carla_simulation_bridge.m`, set `BRIDGE_HOST = '192.168.1.50'` (friend's IP), and run `carla_simulation_bridge`.

---

## 3. What You Will See During the Run

1. **In the CARLA Simulator Window**:
   - The spectator camera automatically positions itself in **third-person 3D chase view** behind the Tesla Model 3.
   - The vehicle drives forward, senses the obstacles using hood-mounted RGB + Depth cameras, and maneuvers around the road hazards in real-time.
2. **In the OpenCV Window (`--show-cam`)**:
   - Live bounding-box object detection feed from the hood camera.
3. **In the Terminal / MATLAB Window**:
   - Real-time 10 Hz telemetry showing ego speed, steering angle ($\delta$), corridor clearance, and behavior state (`CRUISE`, `NUDGE`, `YIELD`).

---

## 4. CLI Arguments Reference for `obs_sim.py`

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--autonomous` | `False` | Run standalone Python autonomous controller (No MATLAB required!) |
| `--duration` | `60.0` | Simulation duration in seconds for autonomous mode |
| `--show-cam` | `False` | Display live forward camera perception stream in an OpenCV window |
| `--matlab-port`| `20000` | Port for MATLAB TCP socket co-simulation |
| `--map` | `None` (current) | Specific CARLA map to load (e.g. `Town01`, `Town04`) |
| `--vehicle` | `vehicle.tesla.model3` | Ego vehicle blueprint name |
| `--standalone` | `False` | Spawn scene only without starting the bridge server |
| `--clean-scene` | `False` | Purge all previous ego vehicles and obstacles before running |

---

## 5. Troubleshooting: Unreal Engine "D3D Device Lost" (0x887A0020)

If you see:
```
LowLevelFatalError [File:Unknown] [Line: 198]
Unreal Engine is exiting due to D3D device being lost. (Error: 0x887A0020 - 'INTERNAL_ERROR')
```

This error happens when Windows TDR (Timeout Detection and Recovery) resets the GPU driver because CARLA took longer than 2 seconds to render a frame.

### Quick Fix Checklist:
1. **Always launch CARLA with low-effort settings**:
   ```powershell
   python start_carla.py
   # Or: .\start_carla_low_effort.bat
   ```
   This automatically caps framerate to 20 FPS, switches to DX11, lowers graphics quality to `Low`, and resizes the window to `800x600`.
2. **Increase Windows GPU Timeout (`TdrDelay`)**:
   - Open `regedit` and go to `HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\GraphicsDrivers`.
   - Create a `DWORD (32-bit)` named `TdrDelay` with value `10` (Decimal).
   - Create a `DWORD (32-bit)` named `TdrDdiDelay` with value `10` (Decimal).
   - Restart your PC.
3. **Run Headless (Zero Window Rendering)**:
   If you only care about data/telemetry and don't need a spectator window:
   ```powershell
   python start_carla.py --headless
   ```

