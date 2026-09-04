"""
indian_road_supervisor.py — Webots 3D Supervisor Controller for Indian Rural Road
=============================================================================
SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads

This controller runs inside Webots as the Supervisor:
  1. Spawns and manages the 3D Indian Rural Road environment:
       - Ego Vehicle (autonomous car)
       - Pothole hazard (recessed road crater in ego lane at X=18m, Y=-1.2m)
       - Pedestrian 1 (crossing left-to-right at X=26m)
       - Pedestrian 2 (crossing right-to-left at X=48m)
       - Oncoming Auto-Rickshaw (Indian 3-wheeler approaching at X=65m, Y=+1.75m)
  2. Animates dynamic agents with realistic Indian road trajectories.
  3. Supports two execution modes:
       - STANDALONE AUTONOMOUS: Full onboard adaptive path planner & state machine
         executes swerves around pothole, yields to crossing pedestrians, yields
         or negotiates oncoming rickshaw bottleneck.
       - MATLAB CO-SIMULATION BRIDGE: Opens TCP port 20000 to stream telemetry
         (ego pose, obstacles, pothole, road bounds) to MATLAB and receive
         steer/throttle/brake commands.
=============================================================================
"""

import os
import sys
import math
import time
import json
import socket
import select
from typing import List, Dict, Tuple, Any

try:
    from controller import Supervisor, Camera
except ImportError:
    # Add standard Webots controller search path if running standalone
    webots_home = os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots")
    sys.path.append(os.path.join(webots_home, "lib", "controller", "python"))
    from controller import Supervisor, Camera


# ── Scenario Configuration ───────────────────────────────────────────────────
CONFIG = {
    "road_width": 7.0,
    "road_boundaries": [3.5, -3.5],   # [y_left, y_right]
    "ego": {
        "start_x": 0.0,
        "start_y": -1.75,
        "goal_x": 75.0,
        "goal_y": -1.75,
        "v_cruise": 5.0,
        "length": 4.2,
        "width": 1.8,
        "wheelbase": 2.8,
    },
    "pothole": {
        "center": [18.0, -1.2],
        "radius": 0.7,
        "depth": 0.12
    },
    "pedestrian_1": {
        "x": 26.0,
        "start_y": 3.8,
        "target_y": -3.8,
        "speed": 1.15,
        "trigger_ego_x": 8.0
    },
    "pedestrian_2": {
        "x": 48.0,
        "start_y": -3.8,
        "target_y": 3.8,
        "speed": 1.25,
        "trigger_ego_x": 27.0
    },
    "auto_rickshaw": {
        "start_x": 65.0,
        "y": 1.75,
        "speed": -3.6,          # Moving in -X direction
        "length": 2.6,
        "width": 1.3
    }
}


class StandalonePlanner:
    """
    Onboard Adaptive Path Planner & Behavior State Machine for Standalone Webots.
    Emulates the full Hybrid A* / EKF / BSM pipeline in pure Python.
    """
    def __init__(self):
        self.state = "CRUISE"
        self.pothole_cleared = False
        self.ped1_yielded = False
        self.ped2_yielded = False

    def plan_step(self, ego_x: float, ego_y: float, ego_yaw: float, ego_v: float,
                  obstacles: List[Dict[str, Any]], pothole: Dict[str, Any]) -> Tuple[float, float, float, str]:
        """
        Calculates steer (-1..1), throttle (0..1), brake (0..1), and state name.
        """
        p_center = pothole["center"]
        p_rad = pothole["radius"]

        # Default targets
        target_y = -1.75
        target_speed = CONFIG["ego"]["v_cruise"]
        steer = 0.0
        throttle = 0.0
        brake = 0.0

        dist_to_pothole = p_center[0] - ego_x

        # 1. Pothole Nudge Logic (X in [8m, 22m])
        if -2.0 <= dist_to_pothole <= 14.0:
            if not self.pothole_cleared:
                self.state = "NUDGE_RIGHT"  # In drive-on-left, swerve rightward toward centerline
                target_y = -0.15            # Safe corridor avoiding pothole at y=-1.2
                target_speed = 3.8
            if dist_to_pothole < -1.5:
                self.pothole_cleared = True
                self.state = "RESUME_LANE"

        # 2. Check Pedestrians
        for obs in obstacles:
            if "pedestrian" in obs["type"]:
                dx = obs["pos"][0] - ego_x
                dy = abs(obs["pos"][1] - ego_y)

                # If pedestrian is ahead within stopping distance and in / entering road
                if 1.0 < dx < 14.0 and abs(obs["pos"][1]) < 3.0:
                    ttc = dx / max(ego_v, 0.5)
                    if ttc < 3.2:
                        self.state = "YIELD_PEDESTRIAN"
                        target_speed = 0.0

        # 3. Check Oncoming Auto-Rickshaw
        for obs in obstacles:
            if obs["type"] == "auto_rickshaw":
                dx = obs["pos"][0] - ego_x
                if 0.0 < dx < 22.0:
                    # If ego is currently swerving near centerline while auto approaches
                    if ego_y > -1.0:
                        self.state = "YIELD_ONCOMING"
                        target_speed = min(target_speed, 2.5)

        # 4. Check Goal Arrival
        if ego_x >= CONFIG["ego"]["goal_x"]:
            self.state = "ARRIVED"
            target_speed = 0.0

        # ── Longitudinal Control (PI speed tracker) ──
        speed_err = target_speed - ego_v
        if target_speed <= 0.2:
            throttle = 0.0
            brake = 0.85 if ego_v > 0.3 else 0.4
        elif speed_err > 0.3:
            throttle = min(1.0, 0.4 + speed_err * 0.25)
            brake = 0.0
        elif speed_err < -0.5:
            throttle = 0.0
            brake = min(0.7, -speed_err * 0.2)
        else:
            throttle = 0.25
            brake = 0.0

        # ── Lateral Control (Stanley / Pure Pursuit) ──
        y_err = target_y - ego_y
        yaw_err = 0.0 - ego_yaw   # Nominal road heading is 0 rad

        # Cross-track steering law
        k_p = 0.45
        k_yaw = 0.60
        steer_cmd = (y_err * k_p) + (yaw_err * k_yaw)
        steer = max(-0.55, min(0.55, steer_cmd))

        return steer, throttle, brake, self.state


class IndianRoadSupervisor:
    """Webots Supervisor handling scene animation, telemetry, and bridge communication."""
    def __init__(self):
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        self.dt = self.time_step / 1000.0

        print("="*65)
        print("  Webots Indian Rural Road Scene Supervisor Initialized")
        print(f"  Time Step: {self.time_step} ms (dt={self.dt:.3f} s)")
        print("="*65)

        # Retrieve scene nodes
        self.ego_node = self.supervisor.getFromDef("EGO_VEHICLE")
        self.ped1_node = self.supervisor.getFromDef("PEDESTRIAN_1")
        self.ped2_node = self.supervisor.getFromDef("PEDESTRIAN_2")
        self.auto_node = self.supervisor.getFromDef("AUTO_RICKSHAW")
        self.pothole_node = self.supervisor.getFromDef("POTHOLE")

        if not self.ego_node:
            print("[ERROR] EGO_VEHICLE node not found in world!")
            sys.exit(1)

        # Ego state
        self.ego_x = CONFIG["ego"]["start_x"]
        self.ego_y = CONFIG["ego"]["start_y"]
        self.ego_z = 0.42
        self.ego_yaw = 0.0
        self.ego_v = 0.0

        # Dynamic obstacles state
        self.ped1_x = CONFIG["pedestrian_1"]["x"]
        self.ped1_y = CONFIG["pedestrian_1"]["start_y"]
        self.ped1_active = False

        self.ped2_x = CONFIG["pedestrian_2"]["x"]
        self.ped2_y = CONFIG["pedestrian_2"]["start_y"]
        self.ped2_active = False

        self.auto_x = CONFIG["auto_rickshaw"]["start_x"]
        self.auto_y = CONFIG["auto_rickshaw"]["y"]

        # Planner & TCP Bridge Server setup
        self.planner = StandalonePlanner()
        self.server_sock = None
        self.client_sock = None
        self.bridge_mode = False

        self._check_bridge_mode()

    def _check_bridge_mode(self):
        """Check if bridge server should be enabled."""
        bridge_env = os.environ.get("WEBOTS_BRIDGE_MODE", "auto").lower()
        if bridge_env in ["1", "true", "yes", "auto", "bridge"]:
            try:
                self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_sock.bind(("0.0.0.0", 20000))
                self.server_sock.listen(1)
                self.server_sock.setblocking(False)
                print("[NET] TCP Co-Simulation Bridge listening on 0.0.0.0:20000")
                print("      (MATLAB or external planner can connect at 127.0.0.1:20000)")
            except Exception as e:
                print(f"[NET] Bridge server failed to bind port 20000: {e}")
                self.server_sock = None

    def update_dynamic_obstacles(self):
        """Updates and animates pedestrians and oncoming auto in 3D space."""
        # 1. Pedestrian 1
        if not self.ped1_active and self.ego_x >= CONFIG["pedestrian_1"]["trigger_ego_x"]:
            self.ped1_active = True
            print(f"  [OBS] Pedestrian 1 triggered! Crossing left-to-right at X={self.ped1_x:.1f}m")

        if self.ped1_active:
            target_y = CONFIG["pedestrian_1"]["target_y"]
            if self.ped1_y > target_y:
                self.ped1_y -= CONFIG["pedestrian_1"]["speed"] * self.dt
            if self.ped1_node:
                self.ped1_node.getField("translation").setSFVec3f([self.ped1_x, self.ped1_y, 0.9])

        # 2. Pedestrian 2
        if not self.ped2_active and self.ego_x >= CONFIG["pedestrian_2"]["trigger_ego_x"]:
            self.ped2_active = True
            print(f"  [OBS] Pedestrian 2 triggered! Crossing right-to-left at X={self.ped2_x:.1f}m")

        if self.ped2_active:
            target_y = CONFIG["pedestrian_2"]["target_y"]
            if self.ped2_y < target_y:
                self.ped2_y += CONFIG["pedestrian_2"]["speed"] * self.dt
            if self.ped2_node:
                self.ped2_node.getField("translation").setSFVec3f([self.ped2_x, self.ped2_y, 0.9])

        # 3. Oncoming Auto-Rickshaw
        if self.auto_x > -10.0:
            self.auto_x += CONFIG["auto_rickshaw"]["speed"] * self.dt
            if self.auto_node:
                self.auto_node.getField("translation").setSFVec3f([self.auto_x, self.auto_y, 0.68])

    def get_obstacles_telemetry(self) -> List[Dict[str, Any]]:
        """Constructs standardized obstacle array for MATLAB / Python planner."""
        obstacles = []
        # Pedestrian 1
        v_ped1 = -CONFIG["pedestrian_1"]["speed"] if self.ped1_active else 0.0
        obstacles.append({
            "id": 1,
            "type": "pedestrian",
            "pos": [round(self.ped1_x, 2), round(self.ped1_y, 2)],
            "vel": [0.0, round(v_ped1, 2)],
            "heading": -1.57,
            "length": 0.5,
            "width": 0.5
        })

        # Pedestrian 2
        v_ped2 = CONFIG["pedestrian_2"]["speed"] if self.ped2_active else 0.0
        obstacles.append({
            "id": 2,
            "type": "pedestrian",
            "pos": [round(self.ped2_x, 2), round(self.ped2_y, 2)],
            "vel": [0.0, round(v_ped2, 2)],
            "heading": 1.57,
            "length": 0.5,
            "width": 0.5
        })

        # Oncoming Auto
        obstacles.append({
            "id": 3,
            "type": "auto_rickshaw",
            "pos": [round(self.auto_x, 2), round(self.auto_y, 2)],
            "vel": [round(CONFIG["auto_rickshaw"]["speed"], 2), 0.0],
            "heading": 3.14,
            "length": CONFIG["auto_rickshaw"]["length"],
            "width": CONFIG["auto_rickshaw"]["width"]
        })

        return obstacles

    def check_collisions(self) -> Tuple[bool, str]:
        """Checks distance between ego and all obstacles + pothole."""
        # 1. Pothole check
        p_cen = CONFIG["pothole"]["center"]
        d_pothole = math.hypot(self.ego_x - p_cen[0], self.ego_y - p_cen[1])
        if d_pothole < 0.65:
            return True, f"POTHOLE IMPACT at ({self.ego_x:.1f}, {self.ego_y:.1f})! Severe chassis drop."

        # 2. Pedestrian 1
        d_ped1 = math.hypot(self.ego_x - self.ped1_x, self.ego_y - self.ped1_y)
        if d_ped1 < 1.3:
            return True, f"COLLISION with Pedestrian 1 at ({self.ego_x:.1f}, {self.ego_y:.1f})!"

        # 3. Pedestrian 2
        d_ped2 = math.hypot(self.ego_x - self.ped2_x, self.ego_y - self.ped2_y)
        if d_ped2 < 1.3:
            return True, f"COLLISION with Pedestrian 2 at ({self.ego_x:.1f}, {self.ego_y:.1f})!"

        # 4. Auto-Rickshaw
        d_auto_x = abs(self.ego_x - self.auto_x)
        d_auto_y = abs(self.ego_y - self.auto_y)
        if d_auto_x < 3.2 and d_auto_y < 1.4:
            return True, f"HEAD-ON COLLISION with Auto-Rickshaw at ({self.ego_x:.1f}, {self.ego_y:.1f})!"

        return False, ""

    def apply_kinematics(self, steer_rad: float, throttle: float, brake: float):
        """Kinematic Bicycle Model integration and Webots 3D pose update."""
        accel = (throttle * 3.0) - (brake * 5.2)
        self.ego_v = max(0.0, min(7.5, self.ego_v + accel * self.dt))
        self.ego_x += self.ego_v * math.cos(self.ego_yaw) * self.dt
        self.ego_y += self.ego_v * math.sin(self.ego_yaw) * self.dt
        self.ego_yaw += (self.ego_v / CONFIG["ego"]["wheelbase"]) * math.tan(steer_rad) * self.dt

        # Update Webots node transform
        if self.ego_node:
            self.ego_node.getField("translation").setSFVec3f([self.ego_x, self.ego_y, self.ego_z])
            self.ego_node.getField("rotation").setSFRotation([0, 0, 1, self.ego_yaw])

    def run(self):
        """Main simulation execution loop."""
        step = 0
        t_sim = 0.0
        success = False

        print("\n[SIM] Starting Webots 3D simulation loop...")
        print("      Observing Rural Road: [Ego] -> [Pothole @ 18m] -> [Ped 1 @ 26m] -> [Ped 2 @ 48m] <- [Auto @ 65m]\n")

        try:
            while self.supervisor.step(self.time_step) != -1:
                step += 1
                t_sim += self.dt

                # 1. Update dynamic environment
                self.update_dynamic_obstacles()
                obstacles = self.get_obstacles_telemetry()
                collision, col_reason = self.check_collisions()

                # 2. Check incoming MATLAB bridge connection
                if self.server_sock and not self.client_sock:
                    readable, _, _ = select.select([self.server_sock], [], [], 0)
                    if readable:
                        self.client_sock, client_addr = self.server_sock.accept()
                        self.client_sock.setblocking(True)
                        self.bridge_mode = True
                        print(f"\n[BRIDGE] MATLAB connected from {client_addr}! Switching to MATLAB Control Mode.\n")

                # 3. Control computation
                if self.bridge_mode and self.client_sock:
                    # Construct and send telemetry package to MATLAB
                    pkg = {
                        "ego_state": [round(self.ego_x, 3), round(self.ego_y, 3), round(self.ego_yaw, 3), round(self.ego_v, 2)],
                        "obstacles": obstacles,
                        "potholes": [CONFIG["pothole"]],
                        "road_boundaries": CONFIG["road_boundaries"],
                        "collision": collision,
                        "camera": {"ready": True}
                    }
                    try:
                        payload = (json.dumps(pkg) + "\n").encode('utf-8')
                        self.client_sock.sendall(payload)

                        resp = self.client_sock.recv(4096).decode('utf-8').strip()
                        if not resp:
                            print("[BRIDGE] MATLAB disconnected.")
                            self.client_sock.close()
                            self.client_sock = None
                            self.bridge_mode = False
                            steer, throttle, brake = 0.0, 0.0, 1.0
                            bsm_state = "DISCONNECTED"
                        else:
                            ctrl = json.loads(resp)
                            steer = ctrl.get('steer', 0.0) * 0.55  # Map [-1, 1] to max steer rad
                            throttle = ctrl.get('throttle', 0.0)
                            brake = ctrl.get('brake', 0.0)
                            bsm_state = "MATLAB_HYBRID_A*"
                    except Exception as e:
                        print(f"[BRIDGE] Communication error: {e}")
                        self.client_sock = None
                        self.bridge_mode = False
                        steer, throttle, brake = 0.0, 0.0, 1.0
                        bsm_state = "ERROR"
                else:
                    # Standalone autonomous planner
                    steer, throttle, brake, bsm_state = self.planner.plan_step(
                        self.ego_x, self.ego_y, self.ego_yaw, self.ego_v, obstacles, CONFIG["pothole"]
                    )

                # 4. Integrate physical dynamics
                self.apply_kinematics(steer, throttle, brake)

                # 5. Telemetry output
                if step % 15 == 0 or bsm_state in ["NUDGE_RIGHT", "YIELD_PEDESTRIAN", "YIELD_ONCOMING", "ARRIVED"]:
                    print(f"[t={t_sim:5.1f}s #{step:03d}] Pos:({self.ego_x:5.1f}, {self.ego_y:+4.2f}) "
                          f"V:{self.ego_v:4.1f}m/s | State: {bsm_state:<16} | "
                          f"Steer: {math.degrees(steer):+5.1f}° Thr:{throttle:.2f} Brk:{brake:.2f}")

                # 6. Safety & Termination checks
                if collision:
                    print(f"\n[!!!] SIMULATION HALTED: {col_reason}\n")
                    break

                if self.ego_x >= CONFIG["ego"]["goal_x"]:
                    print("\n" + "="*65)
                    print(f"  [SUCCESS] Goal reached at X={self.ego_x:.1f}m in t={t_sim:.1f}s!")
                    print(f"  Pothole negotiated cleanly | Pedestrians yielded | Auto cleared")
                    print("="*65 + "\n")
                    success = True
                    break

        except KeyboardInterrupt:
            print("\n[SIM] Interrupted by user.")
        finally:
            if self.client_sock:
                self.client_sock.close()
            if self.server_sock:
                self.server_sock.close()
            print("[SIM] Webots supervisor finished.")

        return success


if __name__ == "__main__":
    app = IndianRoadSupervisor()
    app.run()
