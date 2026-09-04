"""
indian_road_supervisor.py — Webots 3D Supervisor Controller for Indian Rural Road
=============================================================================
SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads

Scenario Combination 3:
  - Pothole: X = 16.0m, Y = -1.50m (in ego lane) with orange hazard cone
    Ego executes early wide swerve to Y = +0.80m, completely avoiding the pothole!
  - Pedestrian 1: X = 30.0m (crossing right-to-left from Y = -3.8m to +3.8m)
  - Pedestrian 2: X = 50.0m (crossing left-to-right from Y = +3.8m to -3.8m)
  - Oncoming Auto-Rickshaw: X = 68.0m, Y = +1.80m (moving oncoming in -X direction)
  - Follow Camera: Active 3D third-person chase camera locked to ego vehicle.
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
    webots_home = os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots")
    sys.path.append(os.path.join(webots_home, "lib", "controller", "python"))
    from controller import Supervisor, Camera


# ── Scenario Configuration (Combination 3) ──────────────────────────────────
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
        "center": [16.0, -1.50],
        "radius": 0.85,
        "depth": 0.12
    },
    "pedestrian_1": {
        "x": 30.0,
        "start_y": -3.8,
        "target_y": 3.8,
        "speed": 1.20,
        "trigger_ego_x": 14.0
    },
    "pedestrian_2": {
        "x": 50.0,
        "start_y": 3.8,
        "target_y": -3.8,
        "speed": 1.20,
        "trigger_ego_x": 33.0
    },
    "auto_rickshaw": {
        "start_x": 68.0,
        "y": 1.80,
        "speed": -3.4,
        "length": 2.6,
        "width": 1.3
    }
}


class StandalonePlanner:
    """
    Onboard Adaptive Path Planner & Behavior State Machine.
    Executes decisive wide swerves around potholes and yields for crossing agents.
    """
    def __init__(self):
        self.state = "CRUISE"
        self.pothole_cleared = False

    def plan_step(self, ego_x: float, ego_y: float, ego_yaw: float, ego_v: float,
                  obstacles: List[Dict[str, Any]], pothole: Dict[str, Any]) -> Tuple[float, float, float, str]:
        p_center = pothole["center"]
        target_y = -1.75
        target_speed = CONFIG["ego"]["v_cruise"]
        steer = 0.0
        throttle = 0.0
        brake = 0.0

        # 1. Pothole Avoidance (Early and wide swerve to Y = +0.80m)
        dist_to_pothole = p_center[0] - ego_x
        if not self.pothole_cleared:
            if -2.5 <= dist_to_pothole <= 13.0:
                self.state = "NUDGE_RIGHT"
                target_y = 0.80   # Wide clearance: ego right wheel stays at Y = -0.10m, well clear of pothole at -1.50m!
                target_speed = 4.0
            if dist_to_pothole < -2.5:
                self.pothole_cleared = True
                self.state = "RESUME_LANE"
        else:
            if ego_x < 24.0:
                self.state = "RESUME_LANE"
                target_y = -1.40

        # 2. Crossing Pedestrians
        for obs in obstacles:
            if "pedestrian" in obs["type"]:
                dx = obs["pos"][0] - ego_x
                # If pedestrian is ahead and crossing within vehicle corridor
                if 1.0 < dx < 14.0 and abs(obs["pos"][1]) < 3.2:
                    ttc = dx / max(ego_v, 0.5)
                    if ttc < 3.5:
                        self.state = "YIELD_PEDESTRIAN"
                        target_speed = 0.0

        # 3. Oncoming Auto-Rickshaw
        for obs in obstacles:
            if obs["type"] == "auto_rickshaw":
                dx = obs["pos"][0] - ego_x
                if 0.0 < dx < 22.0 and ego_y > -0.6:
                    self.state = "YIELD_ONCOMING"
                    target_speed = min(target_speed, 2.8)

        # 4. Goal Arrival Check
        if ego_x >= CONFIG["ego"]["goal_x"]:
            self.state = "ARRIVED"
            target_speed = 0.0

        # ── Longitudinal Control ──
        speed_err = target_speed - ego_v
        if target_speed <= 0.2:
            throttle = 0.0
            brake = 0.88 if ego_v > 0.25 else 0.4
        elif speed_err > 0.3:
            throttle = min(1.0, 0.45 + speed_err * 0.28)
            brake = 0.0
        elif speed_err < -0.5:
            throttle = 0.0
            brake = min(0.7, -speed_err * 0.22)
        else:
            throttle = 0.28
            brake = 0.0

        # ── Lateral Stanley Controller with Heading Damping ──
        y_err = target_y - ego_y
        yaw_err = 0.0 - ego_yaw
        k_p = 0.55
        k_yaw = 0.70
        steer_cmd = (y_err * k_p) + (yaw_err * k_yaw)
        steer = max(-0.55, min(0.55, steer_cmd))

        return steer, throttle, brake, self.state


class IndianRoadSupervisor:
    """Webots Supervisor handling follow camera, dynamic agent animation, and bridge."""
    def __init__(self):
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        self.dt = self.time_step / 1000.0

        print("="*65)
        print("  Webots Indian Rural Road Scene Supervisor (Combination 3)")
        print(f"  Pothole @ X={CONFIG['pothole']['center'][0]}m | Ped 1 @ X={CONFIG['pedestrian_1']['x']}m")
        print(f"  Ped 2 @ X={CONFIG['pedestrian_2']['x']}m | Auto @ X={CONFIG['auto_rickshaw']['start_x']}m")
        print(f"  Time Step: {self.time_step} ms (dt={self.dt:.3f} s)")
        print("="*65)

        # Scene nodes
        self.ego_node = self.supervisor.getFromDef("EGO_VEHICLE")
        self.viewpoint_node = self.supervisor.getFromDef("VIEWPOINT")
        self.ped1_node = self.supervisor.getFromDef("PEDESTRIAN_1")
        self.ped2_node = self.supervisor.getFromDef("PEDESTRIAN_2")
        self.auto_node = self.supervisor.getFromDef("AUTO_RICKSHAW")
        self.pothole_node = self.supervisor.getFromDef("POTHOLE")

        if not self.ego_node:
            print("[ERROR] EGO_VEHICLE node not found!")
            sys.exit(1)

        # Ego vehicle state
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

        # Planner & TCP Bridge Server
        self.planner = StandalonePlanner()
        self.server_sock = None
        self.client_sock = None
        self.bridge_mode = False
        self._check_bridge_mode()

    def _check_bridge_mode(self):
        bridge_env = os.environ.get("WEBOTS_BRIDGE_MODE", "auto").lower()
        if bridge_env in ["1", "true", "yes", "auto", "bridge"]:
            try:
                self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_sock.bind(("0.0.0.0", 20000))
                self.server_sock.listen(1)
                self.server_sock.setblocking(False)
                print("[NET] TCP Co-Simulation Bridge listening on port 20000")
            except Exception as e:
                self.server_sock = None

    def update_camera_follow(self):
        """Actively update the Viewpoint position to follow the car in third-person view."""
        if self.viewpoint_node:
            cam_dist = 6.8
            cam_h = 3.2
            cam_x = self.ego_x - cam_dist * math.cos(self.ego_yaw)
            cam_y = self.ego_y - cam_dist * math.sin(self.ego_yaw)
            cam_z = self.ego_z + cam_h
            self.viewpoint_node.getField("position").setSFVec3f([cam_x, cam_y, cam_z])

    def update_dynamic_obstacles(self):
        """Updates and animates pedestrians and oncoming auto in 3D space."""
        # 1. Pedestrian 1 (at X = 30.0m, crossing right to left)
        if not self.ped1_active and self.ego_x >= CONFIG["pedestrian_1"]["trigger_ego_x"]:
            self.ped1_active = True
            print(f"  [OBS] Pedestrian 1 triggered! Crossing right-to-left at X={self.ped1_x:.1f}m")

        if self.ped1_active:
            target_y = CONFIG["pedestrian_1"]["target_y"]
            if self.ped1_y < target_y:
                self.ped1_y += CONFIG["pedestrian_1"]["speed"] * self.dt
            if self.ped1_node:
                self.ped1_node.getField("translation").setSFVec3f([self.ped1_x, self.ped1_y, 0.9])

        # 2. Pedestrian 2 (at X = 50.0m, crossing left to right)
        if not self.ped2_active and self.ego_x >= CONFIG["pedestrian_2"]["trigger_ego_x"]:
            self.ped2_active = True
            print(f"  [OBS] Pedestrian 2 triggered! Crossing left-to-right at X={self.ped2_x:.1f}m")

        if self.ped2_active:
            target_y = CONFIG["pedestrian_2"]["target_y"]
            if self.ped2_y > target_y:
                self.ped2_y -= CONFIG["pedestrian_2"]["speed"] * self.dt
            if self.ped2_node:
                self.ped2_node.getField("translation").setSFVec3f([self.ped2_x, self.ped2_y, 0.9])

        # 3. Oncoming Auto-Rickshaw (from X = 68.0m)
        if self.auto_x > -10.0:
            self.auto_x += CONFIG["auto_rickshaw"]["speed"] * self.dt
            if self.auto_node:
                self.auto_node.getField("translation").setSFVec3f([self.auto_x, self.auto_y, 0.68])

    def check_collisions(self) -> Tuple[bool, str]:
        p_cen = CONFIG["pothole"]["center"]
        # Exact wheel track check: ego right wheel position is ego_y - 0.9*cos(yaw)
        right_wheel_y = self.ego_y - 0.85 * math.cos(self.ego_yaw)
        left_wheel_y = self.ego_y + 0.85 * math.cos(self.ego_yaw)
        dx_pothole = abs(self.ego_x - p_cen[0])

        if dx_pothole < 1.0:
            if (p_cen[1] - 0.85) <= right_wheel_y <= (p_cen[1] + 0.85) or \
               (p_cen[1] - 0.85) <= left_wheel_y <= (p_cen[1] + 0.85):
                return True, f"POTHOLE IMPACT at X={self.ego_x:.1f}, Y={self.ego_y:.1f}!"

        if math.hypot(self.ego_x - self.ped1_x, self.ego_y - self.ped1_y) < 1.3:
            return True, f"COLLISION with Pedestrian 1 at ({self.ego_x:.1f}, {self.ego_y:.1f})!"
        if math.hypot(self.ego_x - self.ped2_x, self.ego_y - self.ped2_y) < 1.3:
            return True, f"COLLISION with Pedestrian 2 at ({self.ego_x:.1f}, {self.ego_y:.1f})!"
        if abs(self.ego_x - self.auto_x) < 3.2 and abs(self.ego_y - self.auto_y) < 1.4:
            return True, f"COLLISION with Auto-Rickshaw at ({self.ego_x:.1f}, {self.ego_y:.1f})!"

        return False, ""

    def get_obstacles_telemetry(self) -> List[Dict[str, Any]]:
        v_ped1 = CONFIG["pedestrian_1"]["speed"] if self.ped1_active else 0.0
        v_ped2 = -CONFIG["pedestrian_2"]["speed"] if self.ped2_active else 0.0
        return [
            {"id": 1, "type": "pedestrian", "pos": [round(self.ped1_x, 2), round(self.ped1_y, 2)],
             "vel": [0.0, round(v_ped1, 2)], "heading": 1.57, "length": 0.5, "width": 0.5},
            {"id": 2, "type": "pedestrian", "pos": [round(self.ped2_x, 2), round(self.ped2_y, 2)],
             "vel": [0.0, round(v_ped2, 2)], "heading": -1.57, "length": 0.5, "width": 0.5},
            {"id": 3, "type": "auto_rickshaw", "pos": [round(self.auto_x, 2), round(self.auto_y, 2)],
             "vel": [round(CONFIG["auto_rickshaw"]["speed"], 2), 0.0], "heading": 3.14,
             "length": CONFIG["auto_rickshaw"]["length"], "width": CONFIG["auto_rickshaw"]["width"]}
        ]

    def apply_kinematics(self, steer_rad: float, throttle: float, brake: float):
        accel = (throttle * 3.0) - (brake * 5.2)
        self.ego_v = max(0.0, min(7.5, self.ego_v + accel * self.dt))
        self.ego_x += self.ego_v * math.cos(self.ego_yaw) * self.dt
        self.ego_y += self.ego_v * math.sin(self.ego_yaw) * self.dt
        self.ego_yaw += (self.ego_v / CONFIG["ego"]["wheelbase"]) * math.tan(steer_rad) * self.dt

        if self.ego_node:
            self.ego_node.getField("translation").setSFVec3f([self.ego_x, self.ego_y, self.ego_z])
            self.ego_node.getField("rotation").setSFRotation([0, 0, 1, self.ego_yaw])

    def run(self):
        step, t_sim, success = 0, 0.0, False

        print("\n[SIM] Starting Webots 3D simulation (Combination 3)...")
        print(f"      Layout: [Ego] -> [Pothole @ {CONFIG['pothole']['center'][0]}m] -> "
              f"[Ped 1 @ {CONFIG['pedestrian_1']['x']}m] -> [Ped 2 @ {CONFIG['pedestrian_2']['x']}m] "
              f"<- [Auto @ {CONFIG['auto_rickshaw']['start_x']}m]\n")

        try:
            while self.supervisor.step(self.time_step) != -1:
                step += 1
                t_sim += self.dt

                # 1. Update camera to follow car
                self.update_camera_follow()

                # 2. Update dynamic scene obstacles
                self.update_dynamic_obstacles()
                obstacles = self.get_obstacles_telemetry()
                collision, col_reason = self.check_collisions()

                # 3. Check incoming MATLAB bridge connection
                if self.server_sock and not self.client_sock:
                    readable, _, _ = select.select([self.server_sock], [], [], 0)
                    if readable:
                        self.client_sock, client_addr = self.server_sock.accept()
                        self.client_sock.setblocking(True)
                        self.bridge_mode = True
                        print(f"\n[BRIDGE] MATLAB connected from {client_addr}!\n")

                # 4. Control computation
                if self.bridge_mode and self.client_sock:
                    pkg = {
                        "ego_state": [round(self.ego_x, 3), round(self.ego_y, 3), round(self.ego_yaw, 3), round(self.ego_v, 2)],
                        "obstacles": obstacles,
                        "potholes": [CONFIG["pothole"]],
                        "road_boundaries": CONFIG["road_boundaries"],
                        "collision": collision,
                        "camera": {"ready": True}
                    }
                    try:
                        self.client_sock.sendall((json.dumps(pkg) + "\n").encode('utf-8'))
                        resp = self.client_sock.recv(4096).decode('utf-8').strip()
                        if not resp:
                            break
                        ctrl = json.loads(resp)
                        steer = ctrl.get('steer', 0.0) * 0.55
                        throttle, brake = ctrl.get('throttle', 0.0), ctrl.get('brake', 0.0)
                        bsm_state = "MATLAB_CTL"
                    except Exception:
                        break
                else:
                    steer, throttle, brake, bsm_state = self.planner.plan_step(
                        self.ego_x, self.ego_y, self.ego_yaw, self.ego_v, obstacles, CONFIG["pothole"]
                    )

                # 5. Integrate vehicle dynamics
                self.apply_kinematics(steer, throttle, brake)

                # 6. Telemetry display
                if step % 15 == 0 or bsm_state in ["NUDGE_RIGHT", "YIELD_PEDESTRIAN", "YIELD_ONCOMING", "ARRIVED"]:
                    print(f"[t={t_sim:5.1f}s #{step:03d}] Pos:({self.ego_x:5.1f}, {self.ego_y:+4.2f}) "
                          f"V:{self.ego_v:4.1f}m/s | State: {bsm_state:<16} | "
                          f"Steer: {math.degrees(steer):+5.1f}° Thr:{throttle:.2f} Brk:{brake:.2f}")

                if collision:
                    print(f"\n[!!!] SIMULATION HALTED: {col_reason}\n")
                    break

                if self.ego_x >= CONFIG["ego"]["goal_x"]:
                    print("\n" + "="*65)
                    print(f"  [SUCCESS] Goal reached at X={self.ego_x:.1f}m in t={t_sim:.1f}s!")
                    print("  Pothole bypassed completely | Pedestrians yielded | Auto cleared")
                    print("="*65 + "\n")
                    success = True
                    break

        except KeyboardInterrupt:
            pass
        finally:
            if self.client_sock: self.client_sock.close()
            if self.server_sock: self.server_sock.close()

        return success


if __name__ == "__main__":
    app = IndianRoadSupervisor()
    app.run()
