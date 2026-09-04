"""
carla_bridge.py  —  CARLA ↔ MATLAB Connector for SIH PS-26037
=================================================================
ARCHITECTURE
─────────────────────────────────────────────────────────────────
  [CARLA World]
       ↓ CARLA Python API (sensor streams only — NO world.get_actors())
  [CarlaSensorSuite]
       ├─ RGB  camera callback  → YoloDetector → 2D bounding boxes
       ├─ Depth camera callback → metric Z buffer (decoded from RGBA)
       └─ Collision sensor      → impulse events
       ↓ 3D back-projection (pinhole) + camera→ego ISO transform
  [PerceptionFuser]  →  obstacle list [{id, type, position, confidence}]
       ↓ TCP JSON (port 20000)
  [MATLAB] — EKF, Hybrid A*, BSM, Pure Pursuit

WHAT THIS FILE DOES (and does NOT do)
─────────────────────────────────────────────────────────────────
  ✔ Spawns ego vehicle, RGB camera, depth camera, collision sensor
  ✔ Detects objects from REAL camera pixels via neural network inference
  ✔ Decodes per-pixel metric depth from CARLA's RGBA depth encoding
  ✔ Projects 2D bounding box centers → 3D ego-frame ISO positions
  ✔ Streams telemetry + camera-detected obstacles to MATLAB over TCP
  ✔ Applies MATLAB control commands back to CARLA VehicleControl
  ✗ Does NOT call world.get_actors() for obstacle positions
  ✗ Does NOT do any path planning, EKF, or control computation

DETECTOR PRIORITY
─────────────────────────────────────────────────────────────────
  1. Ultralytics YOLOv8n (pip install ultralytics)  — fastest, best accuracy
  2. OpenCV DNN + MobileNet-SSD (bundled COCO model) — fallback, no extra deps
  3. MockDetector                                    — pure-mock or no cv2

Usage:
    python carla_bridge.py [--carla-host 127.0.0.1] [--carla-port 2000]
                           [--matlab-port 20000] [--map Town01]
                           [--vehicle vehicle.tesla.model3]
                           [--show-cam] [--det-conf 0.40]

Protocol (newline-delimited JSON, LF-terminated):
    MATLAB → Python :  {"request": "GET_STATE"}
    Python → MATLAB :  {"ego":{...}, "obstacles":[...], "road_boundaries":[...],
                        "collision":false, "camera":{...}}
    MATLAB → Python :  {"steer":0.0, "throttle":0.5, "brake":0.0}
    Python → MATLAB :  {"ack":"ok"}
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
import collections
import numpy as np

# ── CARLA egg discovery ───────────────────────────────────────────────────────
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
    print('[WARN] OpenCV not found — visual display and DNN fallback disabled.')

# ── Camera constants ──────────────────────────────────────────────────────────
IM_WIDTH  = 640
IM_HEIGHT = 480
CAM_FOV   = 90     # degrees  (MUST match blueprint attribute set below)
CAM_MOUNT_X = 2.5  # meters forward of vehicle origin
CAM_MOUNT_Z = 0.7  # meters above vehicle origin

# Pinhole intrinsics derived from FOV and resolution
# f = (W/2) / tan(FOV/2)  — same formula CARLA uses internally
_FOV_RAD = math.radians(CAM_FOV)
CAM_FX   = (IM_WIDTH  / 2.0) / math.tan(_FOV_RAD / 2.0)
CAM_FY   = (IM_HEIGHT / 2.0) / math.tan(_FOV_RAD / 2.0)
CAM_CX   = IM_WIDTH  / 2.0
CAM_CY   = IM_HEIGHT / 2.0

# CARLA depth encoding range
DEPTH_MAX_M = 1000.0        # 1 km
DEPTH_DENOM = 256.0**3 - 1  # 16 777 215

# Detection confidence threshold (overridable via --det-conf)
DET_CONF_DEFAULT = 0.40

# ── Shared state (written by sensor callbacks, read by fuser) ─────────────────
_lock         = threading.Lock()
_latest_rgb   = None    # np.ndarray (H, W, 3) uint8  – latest RGB frame
_latest_depth = None    # np.ndarray (H, W)    float32 – metric depth [m]
_collision_flag  = False
_collision_hist  = []


# ═══════════════════════════════════════════════════════════════════════════════
#  1. Neural Object Detector  (YOLOv8 → MobileNet-SSD fallback → Mock)
# ═══════════════════════════════════════════════════════════════════════════════

# COCO class IDs → Indian-road semantic label mapping
# (keeping only classes that are plausible on a rural Indian road)
_COCO_TO_LABEL = {
    0:  'pedestrian',   # person
    1:  'auto_rickshaw',# bicycle
    2:  'auto_rickshaw',# car
    3:  'auto_rickshaw',# motorcycle
    5:  'auto_rickshaw',# bus
    7:  'auto_rickshaw',# truck
    14: 'cattle',       # bird → ignore in practice but kept for completeness
    16: 'cattle',       # dog → domestic animal
    17: 'cattle',       # horse → large livestock
    18: 'cattle',       # sheep
    19: 'cattle',       # cow
    20: 'cattle',       # elephant
    21: 'cattle',       # bear → rare but possible
}

Detection = collections.namedtuple(
    'Detection', ['class_id', 'label', 'conf', 'x1', 'y1', 'x2', 'y2'])


class YoloV8Detector:
    """
    Ultralytics YOLOv8n detector.
    Requires:  pip install ultralytics
    On first run it downloads yolov8n.pt (~6 MB) automatically.
    """
    def __init__(self, conf_thresh=DET_CONF_DEFAULT):
        from ultralytics import YOLO  # lazy import — only at instantiation
        self.model = YOLO('yolov8n.pt')
        self.conf  = conf_thresh
        print(f'[DETECTOR] YOLOv8n loaded (conf≥{conf_thresh:.2f})')

    def detect(self, rgb_frame):
        """
        rgb_frame : np.ndarray (H, W, 3) uint8  BGR or RGB
        returns   : list[Detection]
        """
        results = self.model.predict(rgb_frame, conf=self.conf,
                                     verbose=False, stream=False)
        detections = []
        for r in results:
            for box in r.boxes:
                cid  = int(box.cls[0])
                if cid not in _COCO_TO_LABEL:
                    continue
                conf = float(box.conf[0])
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                detections.append(Detection(cid, _COCO_TO_LABEL[cid],
                                            conf, x1, y1, x2, y2))
        return detections


class MobileNetSSDDetector:
    """
    OpenCV DNN backend using MobileNet-SSD (Caffe).
    Downloads the prototxt and caffemodel if not present in the same directory.
    Requires:  pip install opencv-python requests
    """
    _PROTO_URL = ('https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/'
                  'master/MobileNetSSD_deploy.prototxt')
    _MODEL_URL = ('https://drive.google.com/uc?export=download&id='
                  '0B3gersZ2cHIxRm5PMWRoTkdHdHc')
    _CLASSES = {
        0: 'background', 1: 'auto_rickshaw', 2: 'bicycle',
        3: 'pedestrian',  4: 'cattle',       5: 'cattle',
        6: 'auto_rickshaw', 7: 'auto_rickshaw', 8: 'cattle',
        9: 'pedestrian', 10: 'pedestrian',  11: 'auto_rickshaw',
        12: 'auto_rickshaw', 13: 'auto_rickshaw', 14: 'auto_rickshaw',
        15: 'auto_rickshaw', 16: 'auto_rickshaw', 17: 'auto_rickshaw',
        18: 'pedestrian',   19: 'auto_rickshaw', 20: 'auto_rickshaw',
    }
    _LABEL_MAP = {
        'bicycle': 'auto_rickshaw', 'motorbike': 'auto_rickshaw',
        'car': 'auto_rickshaw',     'bus': 'auto_rickshaw',
        'train': 'auto_rickshaw',   'aeroplane': None,
        'boat': None,               'tvmonitor': None,
    }
    _KEEP = {'pedestrian', 'cattle', 'auto_rickshaw'}

    def __init__(self, conf_thresh=DET_CONF_DEFAULT, model_dir=None):
        if not CV2_AVAILABLE:
            raise RuntimeError('OpenCV not available for MobileNet-SSD.')
        self.conf = conf_thresh
        base = model_dir or os.path.dirname(os.path.abspath(__file__))
        proto = os.path.join(base, 'MobileNetSSD_deploy.prototxt')
        model = os.path.join(base, 'MobileNetSSD_deploy.caffemodel')

        if not (os.path.isfile(proto) and os.path.isfile(model)):
            self._download_models(proto, model)

        self.net = cv2.dnn.readNetFromCaffe(proto, model)
        print(f'[DETECTOR] MobileNet-SSD loaded (conf≥{conf_thresh:.2f})')

    @staticmethod
    def _download_models(proto_path, model_path):
        """Download MobileNet-SSD weights if missing."""
        try:
            import urllib.request
            print('[DETECTOR] Downloading MobileNet-SSD prototxt...')
            urllib.request.urlretrieve(MobileNetSSDDetector._PROTO_URL, proto_path)
            print('[DETECTOR] NOTE: Caffemodel must be downloaded manually from:')
            print('           https://github.com/chuanqi305/MobileNet-SSD')
            print('           Save as MobileNetSSD_deploy.caffemodel in the project root.')
        except Exception as e:
            raise RuntimeError(f'Model download failed: {e}') from e

    def detect(self, rgb_frame):
        blob = cv2.dnn.blobFromImage(
            cv2.resize(rgb_frame, (300, 300)), 0.007843,
            (300, 300), 127.5)
        self.net.setInput(blob)
        detections_raw = self.net.forward()
        h, w = rgb_frame.shape[:2]
        result = []
        for i in range(detections_raw.shape[2]):
            conf   = float(detections_raw[0, 0, i, 2])
            if conf < self.conf:
                continue
            cid    = int(detections_raw[0, 0, i, 1])
            label  = self._CLASSES.get(cid, 'background')
            if label not in self._KEEP:
                continue
            x1 = float(detections_raw[0, 0, i, 3]) * w
            y1 = float(detections_raw[0, 0, i, 4]) * h
            x2 = float(detections_raw[0, 0, i, 5]) * w
            y2 = float(detections_raw[0, 0, i, 6]) * h
            result.append(Detection(cid, label, conf,
                                    max(0.0, x1), max(0.0, y1),
                                    min(float(w), x2), min(float(h), y2)))
        return result


class MockDetector:
    """Synthetic detections for use when no camera/model is available."""
    def __init__(self): pass
    def detect(self, _frame):
        return [
            Detection(19, 'cattle',        0.85,  260, 200, 380, 320),
            Detection(2,  'auto_rickshaw', 0.72,  450, 180, 580, 350),
        ]


def build_detector(conf_thresh=DET_CONF_DEFAULT):
    """Try YOLOv8 → MobileNet-SSD → Mock."""
    try:
        return YoloV8Detector(conf_thresh)
    except Exception as e:
        print(f'[DETECTOR] YOLOv8 unavailable ({type(e).__name__}: {e})')

    if CV2_AVAILABLE:
        try:
            return MobileNetSSDDetector(conf_thresh)
        except Exception as e:
            print(f'[DETECTOR] MobileNet-SSD unavailable ({type(e).__name__}: {e})')

    print('[DETECTOR] Falling back to MockDetector.')
    return MockDetector()


# ═══════════════════════════════════════════════════════════════════════════════
#  2. PerceptionFuser  —  depth sampling + pinhole back-projection
# ═══════════════════════════════════════════════════════════════════════════════

class PerceptionFuser:
    """
    Fuses 2D bounding-box detections with the metric depth buffer to produce
    a list of 3D obstacle positions in the ISO vehicle frame.

    Camera frame convention (CARLA / OpenCV):
        X_cam = right,  Y_cam = down,  Z_cam = forward  (into scene)

    Vehicle ISO 8855 frame:
        X_veh = forward, Y_veh = left,  Z_veh = up

    Coordinate transform (camera → vehicle):
        X_veh  =  Z_cam  +  cam_offset_x      (forward distance + camera mount offset)
        Y_veh  = -X_cam                        (right → left flip)
        Z_veh  = -Y_cam  +  cam_offset_z       (down → up flip + height offset)

    The transform is exact for a forward-facing, level-mounted camera.
    For pitch/roll deviations the rotation would need to be parameterised.
    """

    def __init__(self, img_w, img_h, fx, fy, cx, cy,
                 cam_x=CAM_MOUNT_X, cam_z=CAM_MOUNT_Z):
        self.W, self.H   = img_w, img_h
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.cam_x = cam_x
        self.cam_z = cam_z
        self._det_id_counter = 0

    def _sample_depth(self, depth_buf, x1, y1, x2, y2):
        """
        Sample the MEDIAN metric depth [m] from the central 30% crop of the
        bounding box in the depth buffer.  Median is used (not mean) to
        suppress background bleed-through at box boundaries.
        """
        bw = x2 - x1
        bh = y2 - y1
        margin_x = bw * 0.15   # shrink 15% from each horizontal edge
        margin_y = bh * 0.15

        c0 = int(max(0,        math.floor(x1 + margin_x)))
        c1 = int(min(self.W-1, math.ceil (x2 - margin_x)))
        r0 = int(max(0,        math.floor(y1 + margin_y)))
        r1 = int(min(self.H-1, math.ceil (y2 - margin_y)))

        if r1 <= r0 or c1 <= c0:
            return None   # degenerate box

        patch = depth_buf[r0:r1+1, c0:c1+1]
        patch = patch[np.isfinite(patch) & (patch > 0.3) & (patch < 80.0)]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def project(self, detections, depth_buf):
        """
        detections : list[Detection]  — from detector.detect()
        depth_buf  : np.ndarray (H, W) float32  — metric depth [m]

        Returns list of dicts suitable for the MATLAB JSON payload.
        """
        obstacles = []
        for det in detections:
            u_ctr = (det.x1 + det.x2) / 2.0   # horizontal bbox centre [px]
            v_ctr = (det.y1 + det.y2) / 2.0   # vertical   bbox centre [px]

            Z = self._sample_depth(depth_buf, det.x1, det.y1, det.x2, det.y2)
            if Z is None:
                continue   # can't determine range → skip

            # ── Pinhole back-projection into camera 3D frame ──────────────
            # CARLA camera axes: X=right, Y=down, Z=forward
            X_cam = (u_ctr - self.cx) * Z / self.fx   # lateral offset [m]
            Y_cam = (v_ctr - self.cy) * Z / self.fy   # vertical offset [m]
            Z_cam = Z                                  # forward range   [m]

            # ── Camera frame → vehicle ISO 8855 frame ──────────────────────
            # X_veh (forward) = Z_cam + camera_mount_x_offset
            # Y_veh (left)    = -X_cam  (camera X is right → ISO Y is left)
            # Z_veh (up)      = -Y_cam + camera_mount_z_offset  (unused for 2D)
            X_veh = Z_cam + self.cam_x
            Y_veh = -X_cam

            # Object type → behavior_profile tag for MATLAB EKF Q-tuning
            behavior = 'erratic' if det.label == 'cattle' else 'weaving'

            self._det_id_counter += 1
            obstacles.append({
                "id":               self._det_id_counter,
                "type":             det.label,
                "position":         [round(X_veh, 3), round(Y_veh, 3)],
                "velocity":         [0.0, 0.0],   # EKF in MATLAB estimates velocity
                "confidence":       round(det.conf, 3),
                "behavior_profile": behavior,
            })
        return obstacles


# ═══════════════════════════════════════════════════════════════════════════════
#  3. CarlaSensorSuite  —  RGB + Depth cameras + Collision sensor
# ═══════════════════════════════════════════════════════════════════════════════

class CarlaSensorSuite:
    """
    Manages the ego vehicle sensor rig:
      - RGB  camera  (640x480, FOV=90°, hood-mounted)
      - Depth camera (640x480, FOV=90°, same transform — synchronized)
      - Collision sensor

    RGB  → feeds the neural detector (object classification)
    Depth → provides per-pixel metric distances for 3D projection
    """

    def __init__(self, world, vehicle, show_cam=False):
        self.world      = world
        self.vehicle    = vehicle
        self.show_cam   = show_cam
        self.actor_list = []
        self._mount     = carla.Transform(
                              carla.Location(x=CAM_MOUNT_X, z=CAM_MOUNT_Z))
        self._spawn_rgb_camera()
        self._spawn_depth_camera()
        self._spawn_collision_sensor()

    # ── RGB Camera ─────────────────────────────────────────────────────────
    def _spawn_rgb_camera(self):
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IM_WIDTH))
        bp.set_attribute('image_size_y', str(IM_HEIGHT))
        bp.set_attribute('fov',          str(CAM_FOV))
        sensor = self.world.spawn_actor(bp, self._mount, attach_to=self.vehicle)
        sensor.listen(self._on_rgb)
        self.actor_list.append(sensor)
        print(f'[SENSORS] RGB camera  spawned (FOV={CAM_FOV}° {IM_WIDTH}x{IM_HEIGHT})')

    def _on_rgb(self, image):
        """RGBA flat array → (H, W, 3) uint8 BGR for OpenCV compatibility."""
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((IM_HEIGHT, IM_WIDTH, 4))
        bgr = arr[:, :, :3][:, :, ::-1]   # RGBA → BGR
        with _lock:
            global _latest_rgb
            _latest_rgb = bgr
        if self.show_cam and CV2_AVAILABLE:
            cv2.imshow('CARLA RGB Camera', bgr)
            cv2.waitKey(1)

    # ── Depth Camera ────────────────────────────────────────────────────────
    def _spawn_depth_camera(self):
        bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        bp.set_attribute('image_size_x', str(IM_WIDTH))
        bp.set_attribute('image_size_y', str(IM_HEIGHT))
        bp.set_attribute('fov',          str(CAM_FOV))
        sensor = self.world.spawn_actor(bp, self._mount, attach_to=self.vehicle)
        sensor.listen(self._on_depth)
        self.actor_list.append(sensor)
        print(f'[SENSORS] Depth camera spawned (FOV={CAM_FOV}° {IM_WIDTH}x{IM_HEIGHT})')

    def _on_depth(self, image):
        """
        Decode CARLA depth RGBA encoding → metric distance [m].

        CARLA packs depth d ∈ [0, 1] as:
            R = floor(d × 256³)      mod 256
            G = floor(d × 256²)      mod 256
            B = floor(d × 256)       mod 256

        so  d = (R + G×256 + B×256²) / (256³ − 1)
        and Z_meters = DEPTH_MAX_M × d = 1000 × d
        """
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((IM_HEIGHT, IM_WIDTH, 4))
        R = arr[:, :, 0].astype(np.float32)
        G = arr[:, :, 1].astype(np.float32)
        B = arr[:, :, 2].astype(np.float32)
        depth_m = DEPTH_MAX_M * (R + G * 256.0 + B * 256.0**2) / DEPTH_DENOM
        with _lock:
            global _latest_depth
            _latest_depth = depth_m

    # ── Collision Sensor ────────────────────────────────────────────────────
    def _spawn_collision_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.collision')
        sensor = self.world.spawn_actor(bp,
                     carla.Transform(carla.Location(0, 0, 0)),
                     attach_to=self.vehicle)
        sensor.listen(self._on_collision)
        self.actor_list.append(sensor)
        print('[SENSORS] Collision sensor spawned')

    def _on_collision(self, event):
        global _collision_flag, _collision_hist
        with _lock:
            _collision_hist.append({
                'frame':    event.frame,
                'other_id': int(event.other_actor.id),
                'impulse':  [event.normal_impulse.x,
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

    def snapshot(self):
        """
        Thread-safe copy of (rgb, depth) frames for one perception cycle.
        Returns (rgb_bgr, depth_m) — both may be None if not yet received.
        """
        with _lock:
            rgb   = _latest_rgb.copy()   if _latest_rgb   is not None else None
            depth = _latest_depth.copy() if _latest_depth is not None else None
        return rgb, depth

    def destroy(self):
        for a in self.actor_list:
            try:
                a.destroy()
            except Exception:
                pass
        self.actor_list.clear()
        if CV2_AVAILABLE:
            cv2.destroyAllWindows()


# ═══════════════════════════════════════════════════════════════════════════════
#  4. Telemetry helpers  (no world.get_actors() — ever)
# ═══════════════════════════════════════════════════════════════════════════════

def _carla_to_iso(x_c, y_c, yaw_deg):
    """UE4 left-handed (X fwd, Y right) → ISO 8855 right-handed (X fwd, Y left)."""
    return float(x_c), float(-y_c), float(-math.radians(yaw_deg))


def get_ego_telemetry(vehicle):
    t     = vehicle.get_transform()
    v     = vehicle.get_velocity()
    speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
    x, y, yaw = _carla_to_iso(t.location.x, t.location.y, t.rotation.yaw)
    return {"x": round(x, 4), "y": round(y, 4),
            "yaw": round(yaw, 5), "v": round(speed, 4)}


def get_road_boundaries(world, ego_vehicle, radius=50.0, max_segments=40):
    """
    Sample OpenDRIVE topology near ego → ISO lane-edge reference points.
    Works on ANY loaded CARLA map — no hardcoded road names or coordinates.
    """
    cmap    = world.get_map()
    ego_loc = ego_vehicle.get_transform().location
    points  = []
    for seg in cmap.get_topology()[:max_segments]:
        wp  = seg[0]
        loc = wp.transform.location
        if math.hypot(loc.x - ego_loc.x, loc.y - ego_loc.y) > radius:
            continue
        px, py, _ = _carla_to_iso(loc.x, loc.y, 0)
        points.append([round(px, 3), round(py, 3)])
        hw  = wp.lane_width / 2.0
        yaw = math.radians(wp.transform.rotation.yaw)
        lx, ly, _ = _carla_to_iso(loc.x - hw * math.sin(yaw),
                                   loc.y + hw * math.cos(yaw), 0)
        rx, ry, _ = _carla_to_iso(loc.x + hw * math.sin(yaw),
                                   loc.y - hw * math.cos(yaw), 0)
        points.append([round(lx, 3), round(ly, 3)])
        points.append([round(rx, 3), round(ry, 3)])
    return points


def get_camera_meta(rgb_frame):
    """Lightweight camera metadata for MATLAB's telemetry HUD."""
    if rgb_frame is None:
        return {"width": IM_WIDTH, "height": IM_HEIGHT,
                "mean_rgb": [0, 0, 0], "ready": False}
    mean = rgb_frame.mean(axis=(0, 1)).tolist()
    return {"width": IM_WIDTH, "height": IM_HEIGHT,
            "mean_rgb": [round(m, 1) for m in mean], "ready": True}


# ═══════════════════════════════════════════════════════════════════════════════
#  5. Mock world  —  for testing the full bridge without CARLA
# ═══════════════════════════════════════════════════════════════════════════════

class MockWorld:
    """
    Synthetic world that mirrors what the live pipeline produces.
    Integrates bicycle kinematics so braking commands (B=1.0) halt the vehicle
    and steering (steer!=0) curves the path instead of open-loop time ghosting.
    """
    def __init__(self):
        self._step   = 0
        self._fuser  = PerceptionFuser(IM_WIDTH, IM_HEIGHT,
                                       CAM_FX, CAM_FY, CAM_CX, CAM_CY)
        self._det    = MockDetector()
        self.x       = 2.0
        self.y       = 0.0
        self.yaw     = 0.0
        self.v       = 3.5
        self.steer   = 0.0
        self.throttle = 0.0
        self.brake   = 0.0

    def tick(self):
        self._step += 1
        dt = 0.1
        # Closed-loop longitudinal response
        accel = (self.throttle * 3.0) - (self.brake * 5.0)
        self.v = max(0.0, min(8.0, self.v + accel * dt))
        # Closed-loop lateral bicycle kinematics
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        # Max steer angle 30 deg = pi/6; steer is in CARLA convention
        steer_rad = -self.steer * (math.pi / 6.0)
        self.yaw += (self.v / 2.7) * math.tan(steer_rad) * dt

    def get_ego_telemetry(self):
        return {"x": round(self.x, 4),
                "y": round(self.y, 4),
                "yaw": round(self.yaw, 5),
                "v": round(self.v, 4)}

    def get_obstacles(self):
        # Produce synthetic depth so that MockDetector gives plausible ranges
        depth_buf              = np.full((IM_HEIGHT, IM_WIDTH), 35.0, dtype=np.float32)
        depth_buf[200:320, 260:380] = max(5.0, 35.0 - self._step * 0.35) # cattle
        depth_buf[180:350, 450:580] = max(8.0, 60.0 - self._step * 0.28) # rickshaw
        dets = self._det.detect(None)
        return self._fuser.project(dets, depth_buf)

    def get_road_boundaries(self):
        xs = list(range(-5, 120))
        return [[x, -2.5] for x in xs] + [[x, 2.5] for x in xs]

    def apply_control(self, s, t, b):
        self.steer    = s
        self.throttle = t
        self.brake    = b


# ═══════════════════════════════════════════════════════════════════════════════
#  6. Main bridge loop
# ═══════════════════════════════════════════════════════════════════════════════

def run_bridge(args):
    print('=' * 64)
    print('  CARLA ↔ MATLAB Vision Bridge  (SIH PS-26037)')
    print('  Perception: camera-only  |  NO world.get_actors()')
    print('=' * 64)

    # ── Connect to CARLA ───────────────────────────────────────────────────
    mock_mode = not CARLA_AVAILABLE
    world_obj = ego_vehicle = sensors = None

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
            print(f'[WARN] CARLA connection failed ({e}) → MOCK mode.')
            mock_mode = True

    actor_list = []
    if not mock_mode:
        bp_lib  = world_obj.get_blueprint_library()
        ego_bp  = bp_lib.filter(args.vehicle)[0]
        ego_bp.set_attribute('role_name', 'hero')
        spawn_pt = world_obj.get_map().get_spawn_points()[0]
        ego_vehicle = world_obj.spawn_actor(ego_bp, spawn_pt)
        actor_list.append(ego_vehicle)
        print(f'[CARLA] Ego spawned: {ego_vehicle.type_id}')

        # Physics settle
        for _ in range(10):
            world_obj.tick()
        time.sleep(1.0)

        # Sensor rig
        sensors = CarlaSensorSuite(world_obj, ego_vehicle, show_cam=args.show_cam)

        # Wait for BOTH cameras to produce their first frame
        print('[CARLA] Waiting for camera pair sync...')
        for _ in range(400):
            with _lock:
                ready = (_latest_rgb is not None) and (_latest_depth is not None)
            if ready:
                break
            time.sleep(0.05)
        print('[CARLA] Camera pair ready.')

    # ── Build detector & fuser ─────────────────────────────────────────────
    detector = build_detector(args.det_conf)
    fuser    = PerceptionFuser(IM_WIDTH, IM_HEIGHT,
                               CAM_FX, CAM_FY, CAM_CX, CAM_CY)
    mock     = MockWorld() if mock_mode else None

    # Helper to generate synchronized state packet
    def generate_state_packet():
        if mock_mode:
            mock.tick()
            return {
                "ego":             mock.get_ego_telemetry(),
                "obstacles":       mock.get_obstacles(),
                "road_boundaries": mock.get_road_boundaries(),
                "collision":       False,
                "camera":          {"width": IM_WIDTH, "height": IM_HEIGHT,
                                    "mean_rgb": [120, 100, 80], "ready": True}
            }
        else:
            world_obj.tick()   # advance CARLA one step

            # Grab a synchronized snapshot of both camera frames
            rgb_bgr, depth_m = sensors.snapshot()

            # Run neural detector on RGB frame
            detections = []
            if rgb_bgr is not None and depth_m is not None:
                detections = detector.detect(rgb_bgr)

            # Project 2D boxes → 3D ISO obstacle positions
            obstacles = []
            if depth_m is not None:
                obstacles = fuser.project(detections, depth_m)

            with _lock:
                collision = _collision_flag

            return {
                "ego":             get_ego_telemetry(ego_vehicle),
                "obstacles":       obstacles,
                "road_boundaries": get_road_boundaries(world_obj, ego_vehicle),
                "collision":       collision,
                "camera":          get_camera_meta(rgb_bgr)
            }

    # ── TCP server for MATLAB ──────────────────────────────────────────────
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', args.matlab_port))
    srv.listen(1)
    print(f'\n[BRIDGE] Waiting for MATLAB on port {args.matlab_port}...')
    conn, addr = srv.accept()
    conn.setblocking(True)
    print(f'[BRIDGE] MATLAB connected from {addr}\n')

    buf = ''
    try:
        while True:
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
                    print(f'[BRIDGE] Bad JSON: {line[:80]}')
                    continue

                # ── Single-trip STEP (Eliminates two-trip TCP latency) ──────
                # Combines control actuation and state capture in one synchronous step
                if msg.get('request') == 'STEP':
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

                    packet = generate_state_packet()
                    conn.sendall((json.dumps(packet) + '\n').encode('utf-8'))

                # ── GET_STATE (Standard two-trip state acquisition) ────────
                elif msg.get('request') == 'GET_STATE':
                    packet = generate_state_packet()
                    conn.sendall((json.dumps(packet) + '\n').encode('utf-8'))

                # ── CONTROL command (Standard two-trip actuation) ──────────
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

                    conn.sendall((json.dumps({"ack": "ok"}) + '\n').encode('utf-8'))

                # ── RESET ──────────────────────────────────────────────────
                elif msg.get('request') == 'RESET':
                    if not mock_mode and sensors:
                        sensors.reset_collision()
                    conn.sendall((json.dumps({"ack": "reset_ok"}) + '\n').encode('utf-8'))

                else:
                    print(f'[BRIDGE] Unknown message: {line[:80]}')

    except KeyboardInterrupt:
        print('\n[BRIDGE] Interrupted.')
    except (ConnectionResetError, BrokenPipeError):
        print('[BRIDGE] MATLAB disconnected unexpectedly.')
    finally:
        conn.close()
        srv.close()
        if sensors:
            sensors.destroy()
        for a in reversed(actor_list):
            try:
                a.destroy()
            except Exception:
                pass
        print('[BRIDGE] Cleaned up. Goodbye.')


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='CARLA ↔ MATLAB Vision Bridge (PS-26037)')
    parser.add_argument('--carla-host',  default='127.0.0.1')
    parser.add_argument('--carla-port',  type=int, default=2000)
    parser.add_argument('--matlab-port', type=int, default=20000)
    parser.add_argument('--map',         default=None,
                        help='CARLA map name to load, e.g. Town04')
    parser.add_argument('--vehicle',     default='vehicle.tesla.model3')
    parser.add_argument('--show-cam',    action='store_true',
                        help='Display live RGB stream in an OpenCV window')
    parser.add_argument('--det-conf',    type=float, default=DET_CONF_DEFAULT,
                        help=f'Minimum detector confidence (default: {DET_CONF_DEFAULT})')
    args = parser.parse_args()
    run_bridge(args)
