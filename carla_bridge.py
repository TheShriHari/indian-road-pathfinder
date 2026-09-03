"""
carla_bridge.py  —  CARLA ↔ MATLAB Connector for SIH PS-26037
=================================================================
Responsibilities (Python side — NO path planning or control logic here):
  1. Connect to CARLA, spawn ego vehicle + RGB camera + collision sensor.
  2. Each tick: pack ego pose/speed, all dynamic actors, road boundaries,
     latest RGB frame shape/stats, and collision flag into a JSON line
     and send it to MATLAB over TCP.
  3. Receive the JSON control command from MATLAB (steer, throttle, brake)
     and apply it to the ego vehicle.
  4. All intelligence (EKF, Hybrid A*, BSM, Pure Pursuit) lives in MATLAB.

Usage:
    python carla_bridge.py [--carla-host 127.0.0.1] [--carla-port 2000]
                           [--matlab-port 20000] [--map Town01]
                           [--vehicle model3] [--show-cam]

Protocol (newline-delimited JSON over TCP):
    MATLAB  →  Python  :  {"request": "GET_STATE"}
    Python  →  MATLAB  :  { "ego": {...}, "obstacles": [...],
                            "road_boundaries": [...], "collision": false,
                            "camera": {"width":640,"height":480,"mean_rgb":[r,g,b]} }
    MATLAB  →  Python  :  {"steer": 0.0, "throttle": 0.5, "brake": 0.0}
"""

import glob
import os
import sys
import math
import time
import json
import socket
import argparse
import threading
import numpy as np

# ── CARLA Python egg discovery ──────────────────────────────────────────────
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
    print('[WARN] CARLA Python package not found — running in MOCK mode.')

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False

# ── Constants ────────────────────────────────────────────────────────────────
IM_WIDTH  = 640
IM_HEIGHT = 480
CAM_FOV   = 110    # degrees

# ── Shared state (written by sensor callbacks, read by main loop) ────────────
_lock              = threading.Lock()
_latest_rgb        = None     # numpy (H, W, 3) uint8
_collision_hist    = []
_collision_flag    = False


# ═══════════════════════════════════════════════════════════════════════════════
#  CarlaSensorSuite  — spawns & manages all sensors on the ego vehicle
# ═══════════════════════════════════════════════════════════════════════════════
class CarlaSensorSuite:
    def __init__(self, world, vehicle, show_cam=False):
        self.world       = world
        self.vehicle     = vehicle
        self.show_cam    = show_cam
        self.actor_list  = []
        self._spawn_rgb_camera()
        self._spawn_collision_sensor()

    # ── RGB Camera ─────────────────────────────────────────────────────────
    def _spawn_rgb_camera(self):
        global _latest_rgb
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IM_WIDTH))
        bp.set_attribute('image_size_y', str(IM_HEIGHT))
        bp.set_attribute('fov',          str(CAM_FOV))

        # Mount on hood: 2.5 m forward, 0.7 m above vehicle origin
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._process_rgb)
        self.actor_list.append(sensor)
        print('[SENSORS] RGB camera spawned')

    def _process_rgb(self, image):
        """Convert raw CARLA RGBA flat array → (H, W, 3) uint8 numpy array."""
        global _latest_rgb
        arr  = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr  = arr.reshape((IM_HEIGHT, IM_WIDTH, 4))   # RGBA
        rgb  = arr[:, :, :3]                            # drop alpha
        with _lock:
            _latest_rgb = rgb
        if self.show_cam and CV2_AVAILABLE:
            cv2.imshow('CARLA Camera', rgb)
            cv2.waitKey(1)

    # ── Collision Sensor ────────────────────────────────────────────────────
    def _spawn_collision_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.collision')
        transform = carla.Transform(carla.Location(x=0.0, z=0.0))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._on_collision)
        self.actor_list.append(sensor)
        print('[SENSORS] Collision sensor spawned')

    def _on_collision(self, event):
        global _collision_flag, _collision_hist
        with _lock:
            _collision_hist.append({
                'frame':   event.frame,
                'other_id': int(event.other_actor.id),
                'impulse': [event.normal_impulse.x,
                            event.normal_impulse.y,
                            event.normal_impulse.z]
            })
            _collision_flag = True
        print(f'[COLLISION] with actor {event.other_actor.type_id}')

    def reset_collision(self):
        global _collision_flag, _collision_hist
        with _lock:
            _collision_flag = False
            _collision_hist.clear()

    def destroy(self):
        for a in self.actor_list:
            a.destroy()
        self.actor_list.clear()
        if CV2_AVAILABLE:
            cv2.destroyAllWindows()


# ═══════════════════════════════════════════════════════════════════════════════
#  Helpers — telemetry extraction
# ═══════════════════════════════════════════════════════════════════════════════

def _carla_to_iso(x_c, y_c, yaw_deg):
    """CARLA left-handed (X fwd, Y right) → ISO 8855 (X fwd, Y left)."""
    return float(x_c), float(-y_c), float(-math.radians(yaw_deg))


def get_ego_telemetry(vehicle):
    t     = vehicle.get_transform()
    v     = vehicle.get_velocity()
    speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
    x, y, yaw = _carla_to_iso(t.location.x, t.location.y, t.rotation.yaw)
    return {"x": x, "y": y, "yaw": yaw, "v": speed}


def get_dynamic_obstacles(world, ego_id):
    """Return list of all non-ego walkers and vehicles as obstacle dicts."""
    obstacles = []
    for actor in world.get_actors():
        tid = actor.type_id.lower()
        if actor.id == ego_id:
            continue
        is_walker  = 'walker' in tid
        is_vehicle = 'vehicle' in tid
        if not (is_walker or is_vehicle):
            continue

        t = actor.get_transform()
        v = actor.get_velocity()
        px, py, _ = _carla_to_iso(t.location.x, t.location.y, t.rotation.yaw)
        # Velocity: only flip Y component
        vx, vy = float(v.x), float(-v.y)

        # Classify type for MATLAB EKF noise tuning
        if is_walker:
            obs_type = 'pedestrian'
        elif any(k in tid for k in ['cow', 'cattle', 'animal', 'sheep']):
            obs_type = 'cattle'
        elif any(k in tid for k in ['tricycle', 'rickshaw', 'bh.crossbike']):
            obs_type = 'auto_rickshaw'
        else:
            obs_type = 'auto_rickshaw'   # generic vehicle

        behavior = 'erratic' if obs_type == 'cattle' else 'weaving'

        obstacles.append({
            "id":               int(actor.id),
            "type":             obs_type,
            "position":         [px, py],
            "velocity":         [vx, vy],
            "behavior_profile": behavior
        })
    return obstacles


def get_road_boundaries(world, ego_vehicle, radius=50.0, max_segments=40):
    """
    Sample OpenDRIVE topology waypoints near ego and return their centre-lane
    positions as road boundary reference points for the MATLAB rolling costmap.
    Works on ANY loaded CARLA map — no hardcoded road names or coords.
    """
    cmap    = world.get_map()
    ego_loc = ego_vehicle.get_transform().location
    points  = []
    for seg in cmap.get_topology()[:max_segments]:
        wp = seg[0]
        loc = wp.transform.location
        if math.hypot(loc.x - ego_loc.x, loc.y - ego_loc.y) <= radius:
            px, py, _ = _carla_to_iso(loc.x, loc.y, 0)
            points.append([px, py])
            # Also append lane edges (left width + right width)
            hw = wp.lane_width / 2.0
            lx, ly, _ = _carla_to_iso(loc.x - hw * math.sin(math.radians(wp.transform.rotation.yaw)),
                                       loc.y + hw * math.cos(math.radians(wp.transform.rotation.yaw)), 0)
            rx, ry, _ = _carla_to_iso(loc.x + hw * math.sin(math.radians(wp.transform.rotation.yaw)),
                                       loc.y - hw * math.cos(math.radians(wp.transform.rotation.yaw)), 0)
            points.append([lx, ly])
            points.append([rx, ry])
    return points


def get_camera_stats():
    """Return lightweight camera metadata (not raw pixels) for MATLAB HUD."""
    with _lock:
        frame = _latest_rgb
    if frame is None:
        return {"width": IM_WIDTH, "height": IM_HEIGHT, "mean_rgb": [0, 0, 0], "ready": False}
    mean = frame.mean(axis=(0, 1)).tolist()   # [R_mean, G_mean, B_mean]
    return {"width": IM_WIDTH, "height": IM_HEIGHT,
            "mean_rgb": [round(m, 1) for m in mean], "ready": True}


# ═══════════════════════════════════════════════════════════════════════════════
#  Mock world — used when CARLA is not running (unit-test the bridge protocol)
# ═══════════════════════════════════════════════════════════════════════════════
class MockWorld:
    def __init__(self):
        self._step = 0

    def tick(self):
        self._step += 1

    def get_ego_telemetry(self):
        # Ego drives forward at 4 m/s
        x = 2.0 + self._step * 0.4
        return {"x": x, "y": 0.0, "yaw": 0.0, "v": 4.0}

    def get_obstacles(self):
        cattle_y = -3.5 + self._step * 0.07
        auto_x   = 45.0 - self._step * 0.32
        return [
            {"id": 1, "type": "cattle",
             "position": [30.0, cattle_y], "velocity": [0.0, 0.7],
             "behavior_profile": "erratic"},
            {"id": 2, "type": "auto_rickshaw",
             "position": [auto_x, 1.2],  "velocity": [-3.2, 0.0],
             "behavior_profile": "weaving"}
        ]

    def get_road_boundaries(self):
        xs = list(range(-5, 65))
        return ([[x, -2.5] for x in xs] + [[x, 2.5] for x in xs])

    def apply_control(self, steer, throttle, brake):
        pass   # No-op in mock


# ═══════════════════════════════════════════════════════════════════════════════
#  Main bridge loop
# ═══════════════════════════════════════════════════════════════════════════════

def run_bridge(args):
    print('=' * 60)
    print('  CARLA ↔ MATLAB Bridge  (SIH PS-26037)')
    print('=' * 60)

    # ── Connect to CARLA ───────────────────────────────────────────────────
    mock_mode = not CARLA_AVAILABLE
    world_obj, ego_vehicle, sensors = None, None, None

    if not mock_mode:
        try:
            client = carla.Client(args.carla_host, args.carla_port)
            client.set_timeout(10.0)
            world_obj = client.get_world()
            print(f'[CARLA] Connected — map: {world_obj.get_map().name}')
            if args.map:
                world_obj = client.load_world(args.map)
                print(f'[CARLA] Loaded map: {args.map}')
        except Exception as e:
            print(f'[WARN] CARLA connection failed ({e}) — falling back to MOCK mode.')
            mock_mode = True

    mock = MockWorld() if mock_mode else None

    actor_list = []
    if not mock_mode:
        # Spawn ego vehicle
        bp_lib  = world_obj.get_blueprint_library()
        ego_bp  = bp_lib.filter(args.vehicle)[0]
        ego_bp.set_attribute('role_name', 'hero')
        spawn_pt = world_obj.get_map().get_spawn_points()[0]
        ego_vehicle = world_obj.spawn_actor(ego_bp, spawn_pt)
        actor_list.append(ego_vehicle)
        print(f'[CARLA] Ego vehicle spawned: {ego_vehicle.type_id}')

        # Give physics a moment to settle
        for _ in range(10):
            world_obj.tick()
        time.sleep(1.0)

        # Spawn sensors
        sensors = CarlaSensorSuite(world_obj, ego_vehicle, show_cam=args.show_cam)

        # Wait for first camera frame
        print('[CARLA] Waiting for first camera frame...')
        for _ in range(200):
            with _lock:
                if _latest_rgb is not None:
                    break
            time.sleep(0.05)
        print('[CARLA] Camera ready.')

    # ── Start TCP server for MATLAB ────────────────────────────────────────
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', args.matlab_port))
    srv.listen(1)
    print(f'[BRIDGE] Listening for MATLAB on port {args.matlab_port}...')
    conn, addr = srv.accept()
    conn.setblocking(True)
    print(f'[BRIDGE] MATLAB connected from {addr}')

    buf = ''
    try:
        while True:
            # Receive message from MATLAB (newline-terminated JSON)
            chunk = conn.recv(4096).decode('utf-8', errors='replace')
            if not chunk:
                print('[BRIDGE] MATLAB disconnected.')
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
                    print(f'[BRIDGE] Bad JSON from MATLAB: {line[:80]}')
                    continue

                # ── Handle GET_STATE ───────────────────────────────────────
                if msg.get('request') == 'GET_STATE':
                    if mock_mode:
                        mock.tick()
                        packet = {
                            "ego":            mock.get_ego_telemetry(),
                            "obstacles":      mock.get_obstacles(),
                            "road_boundaries": mock.get_road_boundaries(),
                            "collision":      False,
                            "camera":         {"width": IM_WIDTH, "height": IM_HEIGHT,
                                               "mean_rgb": [120, 100, 80], "ready": True}
                        }
                    else:
                        world_obj.tick()  # advance simulation one step
                        with _lock:
                            collision = _collision_flag
                        packet = {
                            "ego":            get_ego_telemetry(ego_vehicle),
                            "obstacles":      get_dynamic_obstacles(world_obj, ego_vehicle.id),
                            "road_boundaries": get_road_boundaries(world_obj, ego_vehicle),
                            "collision":      collision,
                            "camera":         get_camera_stats()
                        }

                    conn.sendall((json.dumps(packet) + '\n').encode('utf-8'))

                # ── Handle CONTROL command from MATLAB ─────────────────────
                elif 'steer' in msg:
                    steer    = max(-1.0, min(1.0, float(msg.get('steer',    0.0))))
                    throttle = max( 0.0, min(1.0, float(msg.get('throttle', 0.0))))
                    brake    = max( 0.0, min(1.0, float(msg.get('brake',    0.0))))

                    if mock_mode:
                        mock.apply_control(steer, throttle, brake)
                    else:
                        ctrl           = carla.VehicleControl()
                        ctrl.steer     = steer
                        ctrl.throttle  = throttle
                        ctrl.brake     = brake
                        ctrl.hand_brake = False
                        ego_vehicle.apply_control(ctrl)

                    # ACK back to MATLAB
                    conn.sendall((json.dumps({"ack": "ok"}) + '\n').encode('utf-8'))

                # ── Handle RESET (end of episode / new map) ────────────────
                elif msg.get('request') == 'RESET':
                    if not mock_mode and sensors:
                        sensors.reset_collision()
                    conn.sendall((json.dumps({"ack": "reset_ok"}) + '\n').encode('utf-8'))

                else:
                    print(f'[BRIDGE] Unknown message: {line[:80]}')

    except KeyboardInterrupt:
        print('\n[BRIDGE] Interrupted by user.')
    except (ConnectionResetError, BrokenPipeError):
        print('[BRIDGE] MATLAB disconnected unexpectedly.')
    finally:
        conn.close()
        srv.close()
        if sensors:
            sensors.destroy()
        for a in reversed(actor_list):
            a.destroy()
        print('[BRIDGE] Cleaned up. Goodbye.')


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CARLA ↔ MATLAB Bridge (PS-26037)')
    parser.add_argument('--carla-host',  default='127.0.0.1',
                        help='CARLA server host (default: 127.0.0.1)')
    parser.add_argument('--carla-port',  type=int, default=2000,
                        help='CARLA server port (default: 2000)')
    parser.add_argument('--matlab-port', type=int, default=20000,
                        help='TCP port MATLAB connects to (default: 20000)')
    parser.add_argument('--map',         default=None,
                        help='CARLA map to load, e.g. Town01 (default: current map)')
    parser.add_argument('--vehicle',     default='vehicle.tesla.model3',
                        help='CARLA vehicle blueprint filter (default: vehicle.tesla.model3)')
    parser.add_argument('--show-cam',    action='store_true',
                        help='Display live RGB camera feed via OpenCV')
    args = parser.parse_args()
    run_bridge(args)
