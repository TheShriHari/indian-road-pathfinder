"""
sumo_sim.py — Eclipse SUMO Co-Simulation & Autonomous Navigation Driver
=============================================================================
SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads

Features:
  1. Auto-configures SUMO_HOME and TraCI bindings (supports eclipse-sumo PyPI).
  2. Builds & compiles the Indian Road network (undivided 2-lane corridor).
  3. Spawns all required Indian Road scenario obstacles:
     - Ego Vehicle (role='hero', starts at X=2.0m)
     - Left Roadblock / Parked Vehicle (blocks lane at X=20.0m)
     - Crossing Pedestrian / Cattle (crossing at X=28.0m)
     - Construction Hazard / Debris (at X=38.0m)
     - Oncoming Auto-Rickshaw (westbound at X=55.0m, speed=3.5 m/s)
     - Potholes (staggered surface hazards at X=20.0m and X=35.0m)
  4. Modes:
     - Standalone Autonomous Mode (--autonomous, default):
       Runs the adaptive path planning & pure pursuit controller inside Python.
     - Co-Simulation Bridge Mode (--bridge):
       Opens TCP JSON bridge server on port 20000 for MATLAB co-simulation.
     - Visual Mode (--gui):
       Opens sumo-gui with 2D/3D visualization and real-time tracking.

Usage:
  python sumo_sim.py [--gui] [--autonomous] [--bridge] [--port 20000]
                     [--max-steps 400] [--step-length 0.1]
=============================================================================
"""

import os
import sys
import time
import math
import json
import socket
import argparse
import subprocess
import threading

# ── 1. Locate and configure SUMO_HOME and TraCI ──────────────────────────────
def ensure_sumo_environment():
    """Locates SUMO binaries from system or eclipse-sumo package and sets SUMO_HOME."""
    if 'SUMO_HOME' not in os.environ:
        try:
            import sumolib
            # Check if eclipse_sumo package directory contains data
            pkg_dir = os.path.dirname(sumolib.__file__)
            candidate_home = os.path.abspath(os.path.join(pkg_dir, '..', 'sumo_data'))
            if os.path.exists(candidate_home):
                os.environ['SUMO_HOME'] = candidate_home
            else:
                # Try site-packages/sumo
                candidate_home2 = os.path.abspath(os.path.join(pkg_dir, '..', 'sumo'))
                if os.path.exists(candidate_home2):
                    os.environ['SUMO_HOME'] = candidate_home2
        except Exception:
            pass

    # Ensure SUMO bin folder is in PATH
    sumo_home = os.environ.get('SUMO_HOME', '')
    if sumo_home:
        bin_dir = os.path.join(sumo_home, 'bin')
        if os.path.exists(bin_dir) and bin_dir not in os.environ.get('PATH', ''):
            os.environ['PATH'] = bin_dir + os.pathsep + os.environ.get('PATH', '')

    try:
        import traci
        import sumolib
        return traci, sumolib
    except ImportError as e:
        sys.exit(f"[ERROR] TraCI / Sumolib not found. Run 'pip install eclipse-sumo traci sumolib'. Details: {e}")

traci, sumolib = ensure_sumo_environment()

# ── 2. Build or compile SUMO Network if needed ────────────────────────────────
def build_network_if_needed(sumo_dir):
    """Compiles indian_road.net.xml from .nod.xml and .edg.xml using netconvert if missing."""
    net_file = os.path.join(sumo_dir, 'indian_road.net.xml')
    nod_file = os.path.join(sumo_dir, 'indian_road.nod.xml')
    edg_file = os.path.join(sumo_dir, 'indian_road.edg.xml')

    if os.path.exists(net_file) and os.path.getsize(net_file) > 100:
        return net_file

    print("[SUMO] Compiling network geometry using netconvert...")
    netconvert_bin = sumolib.checkBinary('netconvert')
    cmd = [
        netconvert_bin,
        '--node-files', nod_file,
        '--edge-files', edg_file,
        '--output-file', net_file,
        '--no-turnarounds', 'true',
        '--opposites.guess', 'true'
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"[WARN] netconvert failed: {res.stderr}\nGenerating direct XML network...")
        generate_direct_net_xml(net_file)
    else:
        print(f"[SUMO] Network compiled successfully: {net_file}")
    return net_file

def generate_direct_net_xml(target_file):
    """Fallback generator for a 100m undivided two-lane rural road network."""
    xml_content = """<?xml version="1.0" encoding="UTF-8"?>
<net version="1.20" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/net_file.xsd">
    <location netOffset="0.00,0.00" convBoundary="0.00,-3.50,100.00,3.50" origBoundary="0.00,-3.50,100.00,3.50" projParameter="!"/>
    <edge id=":start_0" function="internal">
        <lane id=":start_0_0" index="0" speed="15.00" length="0.10" shape="0.00,-1.75 0.10,-1.75"/>
    </edge>
    <edge id="road_east" from="start" to="end" priority="1">
        <lane id="road_east_0" index="0" speed="15.00" length="100.00" width="3.50" shape="0.00,-1.75 100.00,-1.75"/>
    </edge>
    <edge id="road_west" from="end" to="start" priority="1">
        <lane id="road_west_0" index="0" speed="15.00" length="100.00" width="3.50" shape="100.00,1.75 0.00,1.75"/>
    </edge>
    <junction id="start" type="priority" x="0.00" y="0.00" incLanes="road_west_0" intLanes="" shape="0.00,3.50 0.00,-3.50"/>
    <junction id="end" type="priority" x="100.00" y="0.00" incLanes="road_east_0" intLanes="" shape="100.00,-3.50 100.00,3.50"/>
</net>
"""
    with open(target_file, 'w') as f:
        f.write(xml_content)
    print(f"[SUMO] Generated standalone network file at {target_file}")

# ── 3. Scenario Obstacle Definitions ─────────────────────────────────────────
SCENARIO_CONFIG = {
    "ego": {
        "id": "ego",
        "start_pos": [2.0, -1.75],
        "start_speed": 0.0,
        "target_speed": 5.0,
        "goal_x": 75.0,
        "length": 4.6,
        "width": 2.0
    },
    "static_roadblock": {
        "id": "roadblock_left",
        "pos": [20.0, -1.75],    # Centered in eastbound lane -> forces rightward nudge
        "length": 4.5,
        "width": 2.0
    },
    "construction_hazard": {
        "id": "hazard_debris",
        "pos": [38.0, 1.2],      # Located in westbound shoulder/lane edge
        "length": 3.0,
        "width": 1.5
    },
    "crossing_pedestrian": {
        "id": "pedestrian_cross",
        "start_pos": [28.0, -4.0],  # Starts off right road shoulder
        "end_pos": [28.0, 4.0],     # Crosses diagonally/perpendicularly
        "speed": 0.9,               # 0.9 m/s walking speed
        "current_pos": [28.0, -4.0]
    },
    "oncoming_rickshaw": {
        "id": "oncoming_rickshaw",
        "start_pos": [58.0, 1.75],  # Westbound oncoming lane
        "speed": -3.2,              # -3.2 m/s opposite ego
        "current_pos": [58.0, 1.75],
        "length": 3.0,
        "width": 1.4
    },
    "potholes": [
        {"x": 20.0, "y": 1.0, "radius": 0.8},
        {"x": 35.0, "y": -0.8, "radius": 1.0}
    ],
    "road_boundaries": [
        [0.0, -3.5], [100.0, -3.5],
        [0.0,  3.5], [100.0,  3.5]
    ]
}

# ── 4. Indian Road Autonomous Controller ─────────────────────────────────────
class IndianRoadAutonomousPlanner:
    """
    Standalone Python implementation of the SIH PS-26037 Adaptive Navigation Stack:
    Performs bottleneck detection, dynamic yielding, lane nudging, and pure pursuit steering.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = 'CRUISE'       # CRUISE, NUDGE, YIELD, RESUME
        self.prev_steer = 0.0
        self.replan_count = 0
        self.wb = 2.8               # Wheelbase (m)

    def plan_step(self, ego_x, ego_y, ego_yaw, ego_v, obstacles):
        """Calculates steer, throttle, and brake for current step."""
        # 1. Inspect dynamic obstacles
        rickshaw = next((o for o in obstacles if o['id'] == 'oncoming_rickshaw'), None)
        pedestrian = next((o for o in obstacles if o['id'] == 'pedestrian_cross'), None)

        dist_to_block = 20.0 - ego_x
        rickshaw_dist = (rickshaw['position'][0] - ego_x) if rickshaw else 999.0

        # 2. State Machine Transitions
        # Check Pedestrian Conflict
        ped_in_path = False
        if pedestrian:
            px, py = pedestrian['position']
            if 0.0 < (px - ego_x) < 7.0 and abs(py - ego_y) < 1.8:
                ped_in_path = True

        if ped_in_path:
            # Hard emergency yield for pedestrian
            self.state = 'YIELD_PED'
            target_v = 0.0
            accel = -4.0
        elif dist_to_block > 12.0:
            self.state = 'CRUISE'
            target_y = -1.75
            target_v = 4.5
            accel = 1.5 if ego_v < target_v else -0.5
        elif dist_to_block > -6.0:
            # In the roadblock approach zone [8m to 26m]
            # Decide whether to NUDGE or YIELD based on oncoming rickshaw distance
            if 0.0 < rickshaw_dist < 14.0 and ego_y < -0.5:
                # Oncoming conflict! Bottleneck decider triggers YIELD behind roadblock
                self.state = 'YIELD_OPPONENT'
                target_v = 0.0
                accel = -3.5 if ego_v > 0.3 else 0.0
                target_y = -1.75
            else:
                # Clear to nudge right into opposing corridor to bypass roadblock
                self.state = 'NUDGE_RIGHT'
                target_y = 0.7  # Shift into clear center/left lane
                target_v = 3.5
                accel = 1.0 if ego_v < target_v else -1.0
        elif ego_x < 36.0:
            # Clearing roadblock, avoiding right hazard at x=38
            self.state = 'RESUME_LANE'
            target_y = -0.5
            target_v = 4.0
            accel = 1.0 if ego_v < target_v else -0.5
        elif ego_x < self.cfg['ego']['goal_x']:
            # Normal lane cruising to finish line
            self.state = 'CRUISE_FINAL'
            target_y = -1.75
            target_v = 5.0
            accel = 1.5 if ego_v < target_v else -0.5
        else:
            # Reached Goal!
            self.state = 'ARRIVED'
            target_v = 0.0
            accel = -4.0
            target_y = -1.75

        # 3. Pure Pursuit Lateral Control
        lookahead = max(2.5, min(6.0, 1.5 + 0.8 * ego_v))
        target_x = ego_x + lookahead

        dx = target_x - ego_x
        dy = target_y - ego_y
        alpha = math.atan2(dy, dx) - ego_yaw
        steer_rad = math.atan2(2.0 * self.wb * math.sin(alpha), lookahead)
        steer_rad = max(-0.55, min(0.55, steer_rad))  # Clamp to +/- 31 degrees

        # 4. Longitude Throttle / Brake mapping
        if target_v == 0.0 and ego_v < 0.15:
            throttle = 0.0
            brake = 1.0
        elif accel >= 0.0:
            throttle = min(1.0, max(0.0, accel / 2.5))
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, max(0.0, -accel / 4.0))

        return steer_rad, throttle, brake, self.state

# ── 5. Main SUMO Simulation Runner ───────────────────────────────────────────
class SumoSimulation:
    def __init__(self, args):
        self.args = args
        self.sumo_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo')
        self.net_file = build_network_if_needed(self.sumo_dir)
        self.cfg_file = os.path.join(self.sumo_dir, 'indian_road.sumocfg')
        self.dt = args.step_length
        self.planner = IndianRoadAutonomousPlanner(SCENARIO_CONFIG)
        self.server_sock = None
        self.client_sock = None

        # Kinematic state of Ego: [x, y, yaw, v]
        self.ego_x = SCENARIO_CONFIG['ego']['start_pos'][0]
        self.ego_y = SCENARIO_CONFIG['ego']['start_pos'][1]
        self.ego_yaw = 0.0
        self.ego_v = 0.0

        # Dynamic obstacles state
        self.rickshaw_x = SCENARIO_CONFIG['oncoming_rickshaw']['start_pos'][0]
        self.rickshaw_y = SCENARIO_CONFIG['oncoming_rickshaw']['start_pos'][1]
        self.ped_x = SCENARIO_CONFIG['crossing_pedestrian']['start_pos'][0]
        self.ped_y = SCENARIO_CONFIG['crossing_pedestrian']['start_pos'][1]

    def start_sumo(self):
        """Launches SUMO process via TraCI (GUI or headless)."""
        binary = sumolib.checkBinary('sumo-gui' if self.args.gui else 'sumo')
        sumo_cmd = [
            binary,
            '-c', self.cfg_file,
            '--step-length', str(self.dt),
            '--collision.action', 'warn',
            '--lateral-resolution', '0.2',
            '--no-step-log', 'true',
            '--time-to-teleport', '-1'
        ]
        if self.args.gui:
            sumo_cmd.extend(['--start', 'true'])

        print(f"[SUMO] Starting TraCI simulation using {binary}...")
        traci.start(sumo_cmd)
        print("[SUMO] TraCI connected successfully.")

    def setup_bridge_server(self):
        """Initializes TCP server on port 20000 for MATLAB bridge co-simulation."""
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', self.args.port))
        self.server_sock.listen(1)
        print(f"\n=========================================================")
        print(f"  [SUMO BRIDGE] Listening on 0.0.0.0:{self.args.port}")
        print(f"  Waiting for MATLAB connection (carla/sumo_simulation_bridge.m)...")
        print(f"=========================================================\n")
        self.client_sock, addr = self.server_sock.accept()
        print(f"[SUMO BRIDGE] Client connected from {addr[0]}:{addr[1]}")

    def step_obstacles(self):
        """Updates positions of dynamic actors in SUMO and internal tracker."""
        # 1. Advance oncoming auto-rickshaw
        self.rickshaw_x += SCENARIO_CONFIG['oncoming_rickshaw']['speed'] * self.dt
        if self.rickshaw_x < 5.0:
            self.rickshaw_x = 75.0  # Loop back if needed

        try:
            traci.vehicle.moveToXY(
                "oncoming_rickshaw", "road_west", 0,
                self.rickshaw_x, self.rickshaw_y, angle=270, keepRoute=2
            )
        except Exception:
            pass

        # 2. Advance crossing pedestrian (crosses from y=-4.0 to y=+4.0)
        self.ped_y += SCENARIO_CONFIG['crossing_pedestrian']['speed'] * self.dt
        if self.ped_y > 4.5:
            self.ped_y = -4.0  # Recross

        try:
            traci.polygon.add(
                "ped_poly",
                [[self.ped_x-0.4, self.ped_y-0.4], [self.ped_x+0.4, self.ped_y-0.4],
                 [self.ped_x+0.4, self.ped_y+0.4], [self.ped_x-0.4, self.ped_y+0.4]],
                color=(255, 69, 0, 255), fill=True, layer=10
            )
        except Exception:
            pass

    def build_telemetry_package(self):
        """Constructs JSON sensor packet matching carla_bridge format."""
        obstacles = [
            {
                "id": 1,
                "type": "auto_rickshaw",
                "position": [round(self.rickshaw_x, 2), round(self.rickshaw_y, 2)],
                "velocity": [round(SCENARIO_CONFIG['oncoming_rickshaw']['speed'], 2), 0.0],
                "behavior_profile": "weaving"
            },
            {
                "id": 2,
                "type": "pedestrian",
                "position": [round(self.ped_x, 2), round(self.ped_y, 2)],
                "velocity": [0.0, round(SCENARIO_CONFIG['crossing_pedestrian']['speed'], 2)],
                "behavior_profile": "steady"
            },
            {
                "id": 3,
                "type": "static_hazard",
                "position": [SCENARIO_CONFIG['static_roadblock']['pos'][0],
                             SCENARIO_CONFIG['static_roadblock']['pos'][1]],
                "velocity": [0.0, 0.0],
                "behavior_profile": "stationary"
            },
            {
                "id": 4,
                "type": "static_hazard",
                "position": [SCENARIO_CONFIG['construction_hazard']['pos'][0],
                             SCENARIO_CONFIG['construction_hazard']['pos'][1]],
                "velocity": [0.0, 0.0],
                "behavior_profile": "stationary"
            }
        ]

        # Check collision threshold (1.0m to any obstacle)
        collision = False
        for obs in obstacles:
            ox, oy = obs['position']
            if math.hypot(self.ego_x - ox, self.ego_y - oy) < 1.2:
                collision = True
                break

        pkg = {
            "ego_state": [
                round(self.ego_x, 3),
                round(self.ego_y, 3),
                round(self.ego_yaw, 3),
                round(self.ego_v, 2)
            ],
            "obstacles": obstacles,
            "potholes": SCENARIO_CONFIG['potholes'],
            "road_boundaries": SCENARIO_CONFIG['road_boundaries'],
            "collision": collision,
            "camera": {"ready": True}
        }
        return pkg

    def apply_kinematics(self, steer_rad, throttle, brake):
        """Integrates Kinematic Bicycle Model and updates TraCI vehicle position."""
        accel = (throttle * 3.0) - (brake * 5.0)
        self.ego_v = max(0.0, min(8.0, self.ego_v + accel * self.dt))
        self.ego_x += self.ego_v * math.cos(self.ego_yaw) * self.dt
        self.ego_y += self.ego_v * math.sin(self.ego_yaw) * self.dt
        self.ego_yaw += (self.ego_v / 2.8) * math.tan(steer_rad) * self.dt

        # Project ego position into SUMO
        sumo_angle = 90.0 - math.degrees(self.ego_yaw)  # Convert math angle to SUMO heading
        try:
            traci.vehicle.moveToXY(
                "ego", "road_east", 0,
                self.ego_x, self.ego_y, angle=sumo_angle, keepRoute=2
            )
            traci.vehicle.setSpeed("ego", self.ego_v)
        except Exception:
            pass

    def run(self):
        """Main simulation loop."""
        self.start_sumo()

        if self.args.bridge:
            self.setup_bridge_server()

        step = 0
        t_sim = 0.0
        success = False

        print("\n[SIM] Running simulation loop...")
        try:
            while step < self.args.max_steps:
                step += 1
                t_sim += self.dt

                # 1. Step SUMO World
                traci.simulationStep()
                self.step_obstacles()

                # 2. Telemetry and control calculation
                pkg = self.build_telemetry_package()

                if self.args.bridge and self.client_sock:
                    # Bridge Mode: Exchange JSON with MATLAB
                    payload = (json.dumps(pkg) + "\n").encode('utf-8')
                    self.client_sock.sendall(payload)

                    resp = self.client_sock.recv(4096).decode('utf-8').strip()
                    if not resp:
                        print("[SUMO BRIDGE] MATLAB disconnected.")
                        break
                    ctrl = json.loads(resp)
                    steer = ctrl.get('steer', 0.0) * 0.55  # [-1, 1] to [-0.55, 0.55] rad
                    throttle = ctrl.get('throttle', 0.0)
                    brake = ctrl.get('brake', 0.0)
                    bsm_state = "MATLAB_CTL"
                else:
                    # Standalone Autonomous Mode
                    steer, throttle, brake, bsm_state = self.planner.plan_step(
                        self.ego_x, self.ego_y, self.ego_yaw, self.ego_v, pkg['obstacles']
                    )

                # 3. Apply physics
                self.apply_kinematics(steer, throttle, brake)

                # 4. Telemetry logging
                if step % 10 == 0 or bsm_state in ['NUDGE_RIGHT', 'YIELD_OPPONENT', 'YIELD_PED', 'ARRIVED']:
                    print(f"[t={t_sim:5.1f}s #{step:03d}] Pos:({self.ego_x:5.1f}, {self.ego_y:+4.2f}) "
                          f"V:{self.ego_v:4.1f}m/s | State: {bsm_state:<14} | "
                          f"Steer: {math.degrees(steer):+5.1f}° Thr:{throttle:.2f} Brk:{brake:.2f}")

                # 5. Goal check
                if self.ego_x >= SCENARIO_CONFIG['ego']['goal_x']:
                    print("\n" + "="*60)
                    print(f"  [SUCCESS] Goal reached at X={self.ego_x:.1f}m in t={t_sim:.1f}s!")
                    print(f"  Final state: {bsm_state}, Speed: {self.ego_v:.1f} m/s")
                    print("="*60 + "\n")
                    success = True
                    break

                if pkg['collision']:
                    print(f"\n[!!!] COLLISION detected at t={t_sim:.1f}s, X={self.ego_x:.1f}, Y={self.ego_y:.1f}!")
                    break

                if self.args.gui:
                    time.sleep(0.02)  # Smooth viewing pace for GUI

        except KeyboardInterrupt:
            print("\n[SIM] Interrupted by user.")
        finally:
            print("[SUMO] Closing TraCI...")
            try:
                traci.close()
            except Exception:
                pass
            if self.client_sock:
                self.client_sock.close()
            if self.server_sock:
                self.server_sock.close()
            print("[SIM] Finished.")

        return success


# ── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Eclipse SUMO Indian Road Simulator (SIH PS-26037)")
    parser.add_argument('--gui', action='store_true', help="Open sumo-gui visual window")
    parser.add_argument('--headless', action='store_true', help="Run headless in background (fastest)")
    parser.add_argument('--autonomous', action='store_true', default=True, help="Run standalone autonomous navigation")
    parser.add_argument('--bridge', action='store_true', help="Run as TCP bridge server for MATLAB co-simulation")
    parser.add_argument('--port', type=int, default=20000, help="TCP bridge port (default 20000)")
    parser.add_argument('--step-length', type=float, default=0.1, help="Simulation step length in seconds (default 0.1)")
    parser.add_argument('--max-steps', type=int, default=450, help="Maximum simulation steps (default 450 = 45s)")

    args = parser.parse_args()
    if args.bridge:
        args.autonomous = False

    sim = SumoSimulation(args)
    sim.run()
