"""
obs_sim.py  —  CARLA Indian Road Scene Generator & Autonomous Bridge Runner
=============================================================================
SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads

This script runs on the machine hosting CARLA (e.g. your friend's PC).
It performs the following end-to-end tasks:
  1. Connects to CARLA Simulator (default port 2000)
  2. Sets up the Indian Road Scene:
       - Spawns Ego Vehicle (role_name='hero') on a straight corridor
       - Static Obstacle 1: Parked car / roadblock on left lane (+18m)
       - Static Obstacle 2: Construction hazard / debris on right lane (+38m)
       - Dynamic Obstacle 1: Oncoming auto-rickshaw / vehicle (+55m)
       - Dynamic Obstacle 2: Crossing pedestrian / cattle surrogate (+28m)
  3. Mounts the Camera Sensor Rig on Ego (RGB + Metric Depth + Collision)
  4. Positions the CARLA Spectator in a 3D chase camera view behind the car
  5. Starts the TCP Bridge Server on 0.0.0.0:20000 to communicate with MATLAB
  6. Steps CARLA world synchronously with MATLAB path planning commands

Usage:
  python obs_sim.py [--carla-host 127.0.0.1] [--carla-port 2000]
                    [--matlab-port 20000] [--map Town01]
                    [--vehicle vehicle.tesla.model3] [--show-cam]
                    [--standalone]
=============================================================================
"""

import sys
import time
import json
import socket
import argparse
import threading
import numpy as np

# Discover CARLA egg if present in carla/dist
import glob, os
try:
    sys.path.append(glob.glob(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    carla = None
    CARLA_AVAILABLE = False
    print('[WARN] CARLA Python package not found. Running in MOCK fallback mode.')

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False

from carla_scene import (
    find_suitable_spawn_point,
    spawn_corridor_obstacles,
    update_spectator,
    set_indian_road_weather
)

# Import bridge components from carla_bridge
from carla_bridge import (
    CarlaSensorSuite,
    PerceptionFuser,
    build_detector,
    get_ego_telemetry,
    get_road_boundaries,
    get_camera_meta,
    MockWorld,
    IM_WIDTH, IM_HEIGHT, CAM_FOV, CAM_FX, CAM_FY, CAM_CX, CAM_CY
)


def run_simulation(args):
    print("=" * 70)
    print("  CARLA INDIAN ROAD SCENE SIMULATOR  (SIH PS-26037)")
    print("=" * 70)

    if not CARLA_AVAILABLE:
        print("[ERROR] carla module is required to run CARLA simulation.")
        print("        If you are testing offline without CARLA, use carla_bridge.py")
        sys.exit(1)

    print(f"[CONNECT] Connecting to CARLA at {args.carla_host}:{args.carla_port}...")
    try:
        client = carla.Client(args.carla_host, args.carla_port)
        client.set_timeout(15.0)
        world = client.get_world()
        curr_map = world.get_map().name
        print(f"[CONNECT] Connected to CARLA Server (Map: {curr_map})")

        if args.map and args.map.lower() not in curr_map.lower():
            print(f"[MAP] Loading requested map: {args.map}...")
            world = client.load_world(args.map)
            print(f"[MAP] Map loaded: {args.map}")
    except Exception as e:
        print(f"[ERROR] Failed to connect to CARLA server: {e}")
        print("        Ensure CarlaUE4.exe is running on the host machine.")
        sys.exit(1)

    # Configure synchronous mode
    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1  # 10 Hz matching MATLAB planning cycle
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_synchronous_mode(True)

    set_indian_road_weather(world)

    actor_list = []

    try:
        # Check if an ego vehicle already exists
        ego_vehicle = None
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') in ['hero', 'ego_vehicle']:
                ego_vehicle = actor
                print(f"[EGO] Found existing ego vehicle: id={ego_vehicle.id} ({ego_vehicle.type_id})")
                break

        # Spawn ego vehicle if not present
        if ego_vehicle is None:
            bp_lib = world.get_blueprint_library()
            ego_bp = bp_lib.filter(args.vehicle)[0]
            ego_bp.set_attribute('role_name', 'hero')

            spawn_pt = find_suitable_spawn_point(world, min_forward_dist=80.0)
            ego_vehicle = world.try_spawn_actor(ego_bp, spawn_pt)
            if ego_vehicle is None:
                # Retry with standard spawn point 0
                spawn_pt = world.get_map().get_spawn_points()[0]
                ego_vehicle = world.spawn_actor(ego_bp, spawn_pt)

            actor_list.append(ego_vehicle)
            print(f"[EGO] Spawned ego vehicle: {ego_vehicle.type_id} at {ego_vehicle.get_location()}")

        # Settle physics
        for _ in range(10):
            world.tick()
        time.sleep(0.5)

        # Spawn Indian Road Obstacles along corridor
        print("[SCENE] Generating Indian Road corridor obstacles...")
        obstacles = spawn_corridor_obstacles(world, ego_vehicle, traffic_manager)
        actor_list.extend(obstacles)

        # Update spectator view
        update_spectator(world, ego_vehicle)

        # Mount camera sensor suite on ego vehicle
        print("[SENSORS] Mounting synchronized RGB + Depth camera rig...")
        sensors = CarlaSensorSuite(world, ego_vehicle, show_cam=args.show_cam)

        # Build neural perception detector
        print(f"[PERCEPTION] Initializing object detector (confidence >= {args.det_conf})...")
        detector = build_detector(args.det_conf)
        fuser = PerceptionFuser(IM_WIDTH, IM_HEIGHT, CAM_FX, CAM_FY, CAM_CX, CAM_CY)

        # Wait for camera sync
        print("[SENSORS] Waiting for camera streams to produce frames...")
        for _ in range(300):
            world.tick()
            rgb, depth = sensors.snapshot()
            if rgb is not None and depth is not None:
                break
            time.sleep(0.05)
        print("[SENSORS] Camera stream synchronized successfully.")

        if args.autonomous:
            print("\n" + "=" * 70)
            print("  RUNNING AUTONOMOUS PYTHON CONTROLLER (NO MATLAB REQUIRED)")
            print("  Vehicle uses Pure Pursuit, Camera Perception, and Obstacle Nudge/Yield.")
            print("=" * 70 + "\n")

            carla_map = world.get_map()
            sim_time = 0.0
            dt = 0.1

            while sim_time < args.duration:
                world.tick()
                update_spectator(world, ego_vehicle)
                sim_time += dt

                rgb_bgr, depth_m = sensors.snapshot()
                detections = []
                if rgb_bgr is not None and depth_m is not None:
                    detections = detector.detect(rgb_bgr)

                projected_obs = []
                if depth_m is not None:
                    projected_obs = fuser.project(detections, depth_m)

                # Ego state
                tf = ego_vehicle.get_transform()
                vel = ego_vehicle.get_velocity()
                speed = math.sqrt(vel.x**2 + vel.y**2)
                curr_wp = carla_map.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)

                # Target waypoint ahead along road
                target_lookahead = max(5.0, min(14.0, speed * 1.5 + 4.5))
                nxt_list = curr_wp.next(target_lookahead) if curr_wp else []
                target_wp = nxt_list[0] if nxt_list else curr_wp

                # Check obstacles in corridor ahead (X in [1.5, 25.0], |Y| < 2.2m)
                target_lat_offset = 0.0
                target_speed = 4.2  # ~15 km/h
                brake_val = 0.0
                state_str = "CRUISE"

                for obs in projected_obs:
                    ox, oy = obs['position']
                    if 1.5 < ox < 25.0 and abs(oy) < 2.2:
                        if obs['type'] in ['auto_rickshaw', 'pedestrian', 'cattle'] and ox < 12.0:
                            state_str = "YIELD_WAIT"
                            target_speed = 0.0
                            brake_val = 0.7
                            break
                        elif oy <= 0.0:
                            state_str = "NUDGE_RIGHT"
                            target_lat_offset = 0.95
                            target_speed = 3.0
                        else:
                            state_str = "NUDGE_LEFT"
                            target_lat_offset = -0.95
                            target_speed = 3.0

                # Pure pursuit lateral steering to target with offset
                right_v = target_wp.transform.get_right_vector() if target_wp else carla.Vector3D(0, 0, 0)
                goal_loc = (target_wp.transform.location + right_v * target_lat_offset) if target_wp else tf.location

                yaw_rad = math.radians(tf.rotation.yaw)
                dx = goal_loc.x - tf.location.x
                dy = goal_loc.y - tf.location.y
                local_x = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
                local_y = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

                L = 2.7
                Ld2 = local_x**2 + local_y**2
                steer_rad = math.atan2(2.0 * L * local_y, Ld2) if Ld2 > 0.1 else 0.0
                steer_norm = max(-1.0, min(1.0, steer_rad / (math.pi / 6.0)))

                # Speed control
                if target_speed <= 0.1:
                    throttle_val = 0.0
                    brake_val = max(0.5, brake_val)
                elif speed < target_speed:
                    throttle_val = min(0.6, 0.35 + 0.3 * (target_speed - speed))
                    brake_val = 0.0
                else:
                    throttle_val = 0.0
                    brake_val = 0.2

                ctrl = carla.VehicleControl(steer=steer_norm, throttle=throttle_val, brake=brake_val, hand_brake=False)
                ego_vehicle.apply_control(ctrl)

                if int(sim_time / dt) % 10 == 0:
                    print(f"[AUTO t={sim_time:4.1f}s] Speed={speed:4.1f}m/s | State={state_str:<12s} | Steer={steer_norm:+.2f} Thr={throttle_val:.2f} Brk={brake_val:.2f} | Obs={len(projected_obs)}")

            print("\n[AUTO] Autonomous run completed successfully.")
            return

        if args.standalone:
            print("\n[SCENE] Running in STANDALONE mode (idle simulation).")
            print("        Press Ctrl+C to terminate and clean up actors.\n")
            while True:
                world.tick()
                update_spectator(world, ego_vehicle)
                time.sleep(0.1)

        # ---------------------------------------------------------------------
        # TCP Bridge Server for MATLAB
        # ---------------------------------------------------------------------
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to 0.0.0.0 so MATLAB on another PC across LAN/WiFi can connect!
        srv.bind(('0.0.0.0', args.matlab_port))
        srv.listen(1)

        print("\n" + "=" * 70)
        print(f"  [BRIDGE READY] WAITING FOR MATLAB ON PORT {args.matlab_port}...")
        print(f"  --> If running with MATLAB: Open MATLAB now and run 'carla_simulation_bridge'!")
        print(f"  --> If you do NOT have MATLAB and want the car to drive autonomously:")
        print(f"      Press Ctrl+C and run:  python obs_sim.py --autonomous")
        print("=" * 70 + "\n")

        conn, addr = srv.accept()
        conn.setblocking(True)
        print(f"[BRIDGE] MATLAB connected successfully from {addr}!\n")

        def generate_state_packet():
            world.tick()
            update_spectator(world, ego_vehicle)

            rgb_bgr, depth_m = sensors.snapshot()
            detections = []
            if rgb_bgr is not None and depth_m is not None:
                detections = detector.detect(rgb_bgr)

            projected_obs = []
            if depth_m is not None:
                projected_obs = fuser.project(detections, depth_m)

            with sensors._lock if hasattr(sensors, '_lock') else threading.Lock():
                collision = sensors.get_collision_state() if hasattr(sensors, 'get_collision_state') else False

            return {
                "ego": get_ego_telemetry(ego_vehicle),
                "obstacles": projected_obs,
                "road_boundaries": get_road_boundaries(world, ego_vehicle),
                "collision": collision,
                "camera": get_camera_meta(rgb_bgr)
            }

        buf = ''
        step_count = 0
        while True:
            chunk = conn.recv(4096).decode('utf-8', errors='replace')
            if not chunk:
                print("[BRIDGE] MATLAB disconnected.")
                break
            buf += chunk

            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue

                # Handle STEP (combined single-trip command)
                if msg.get('request') == 'STEP':
                    steer = max(-1.0, min(1.0, float(msg.get('steer', 0.0))))
                    throttle = max(0.0, min(1.0, float(msg.get('throttle', 0.0))))
                    brake = max(0.0, min(1.0, float(msg.get('brake', 0.0))))

                    ctrl = carla.VehicleControl()
                    ctrl.steer = steer
                    ctrl.throttle = throttle
                    ctrl.brake = brake
                    ctrl.hand_brake = False
                    ego_vehicle.apply_control(ctrl)

                    packet = generate_state_packet()
                    conn.sendall((json.dumps(packet) + '\n').encode('utf-8'))
                    step_count += 1
                    if step_count % 10 == 0:
                        print(f"[CO-SIM step={step_count:4d}] Steer={steer:+.2f} Throttle={throttle:.2f} Brake={brake:.2f} | Ego: [{packet['ego']['x']:.1f}, {packet['ego']['y']:.1f}] v={packet['ego']['v']:.1f}m/s")

                # Handle GET_STATE
                elif msg.get('request') == 'GET_STATE':
                    packet = generate_state_packet()
                    conn.sendall((json.dumps(packet) + '\n').encode('utf-8'))

                # Handle CONTROL
                elif 'steer' in msg:
                    steer = max(-1.0, min(1.0, float(msg.get('steer', 0.0))))
                    throttle = max(0.0, min(1.0, float(msg.get('throttle', 0.0))))
                    brake = max(0.0, min(1.0, float(msg.get('brake', 0.0))))

                    ctrl = carla.VehicleControl()
                    ctrl.steer = steer
                    ctrl.throttle = throttle
                    ctrl.brake = brake
                    ctrl.hand_brake = False
                    ego_vehicle.apply_control(ctrl)

                    conn.sendall((json.dumps({"ack": "ok"}) + '\n').encode('utf-8'))

                # Handle RESET
                elif msg.get('request') == 'RESET':
                    if sensors:
                        sensors.reset_collision()
                    conn.sendall((json.dumps({"ack": "reset_ok"}) + '\n').encode('utf-8'))

    except KeyboardInterrupt:
        print("\n[SIMULATION] Interrupted by user.")
    finally:
        print("\n[CLEANUP] Restoring world settings and destroying spawned actors...")
        try:
            world.apply_settings(orig_settings)
        except Exception:
            pass

        if 'sensors' in locals() and sensors:
            sensors.destroy()

        for a in reversed(actor_list):
            try:
                if a is not None and a.is_alive:
                    a.destroy()
            except Exception:
                pass
        print("[CLEANUP] Done. Simulation finished safely.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='CARLA Indian Road Scene & Simulation Bridge (SIH PS-26037)')
    parser.add_argument('--carla-host',  default='127.0.0.1',
                        help='CARLA server IP (default: 127.0.0.1)')
    parser.add_argument('--carla-port',  type=int, default=2000,
                        help='CARLA server port (default: 2000)')
    parser.add_argument('--matlab-port', type=int, default=20000,
                        help='TCP port for MATLAB co-simulation (default: 20000)')
    parser.add_argument('--tm-port',     type=int, default=8000,
                        help='CARLA Traffic Manager port (default: 8000)')
    parser.add_argument('--map',         default=None,
                        help='Map name to load (e.g. Town01, Town04)')
    parser.add_argument('--vehicle',     default='vehicle.tesla.model3',
                        help='Ego vehicle blueprint name')
    parser.add_argument('--show-cam',    action='store_true',
                        help='Show live forward camera stream in OpenCV window')
    parser.add_argument('--det-conf',    type=float, default=0.40,
                        help='Object detector confidence threshold')
    parser.add_argument('--standalone',  action='store_true',
                        help='Run scene only without waiting for MATLAB bridge')
    parser.add_argument('--autonomous',  action='store_true',
                        help='Run standalone autonomous Python controller directly in CARLA (No MATLAB required)')
    parser.add_argument('--duration',    type=float, default=60.0,
                        help='Duration in seconds for autonomous run (default: 60s)')
    args = parser.parse_args()
    run_simulation(args)