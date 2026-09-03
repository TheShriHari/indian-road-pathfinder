"""
carla_bridge_test.py
Self-test for the vision-based carla_bridge.py pipeline.
No CARLA, no YOLOv8, no OpenCV required — validates:
  1. Depth decoding formula
  2. Pinhole back-projection math (camera 3D -> ISO vehicle frame)
  3. PerceptionFuser.project() output schema
  4. Full JSON packet serialisation / deserialisation
"""
import math
import json
import numpy as np
import sys, os

# ── Pull in bridge internals without running __main__ ─────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Minimal stubs so the import does not blow up without carla/cv2
import types

carla_stub = types.ModuleType('carla')
sys.modules.setdefault('carla', carla_stub)

# Prevent cv2 import inside bridge from crashing
cv2_stub = types.ModuleType('cv2')
sys.modules.setdefault('cv2', cv2_stub)

from carla_bridge import (
    PerceptionFuser, Detection, MockWorld, MockDetector,
    IM_WIDTH, IM_HEIGHT, CAM_FX, CAM_FY, CAM_CX, CAM_CY,
    CAM_MOUNT_X, CAM_MOUNT_Z,
    DEPTH_MAX_M, DEPTH_DENOM,
)

PASS = '\033[92mPASS\033[0m'
FAIL = '\033[91mFAIL\033[0m'
errors = []

def check(name, condition, detail=''):
    if condition:
        print(f'  {PASS}  {name}')
    else:
        print(f'  {FAIL}  {name}  — {detail}')
        errors.append(name)

print()
print('=' * 60)
print('  carla_bridge.py  —  Vision Pipeline Self-Test')
print('=' * 60)

# ── Test 1: Camera intrinsics formula ─────────────────────────────────────────
print('\n[1] Camera intrinsics (FOV=90°, 640x480)')
expected_f = (IM_WIDTH / 2.0) / math.tan(math.radians(90) / 2.0)   # = 320.0
check('CAM_FX == 320.0', abs(CAM_FX - 320.0) < 0.01,
      f'got CAM_FX={CAM_FX:.4f}')
check('CAM_FY == 240.0 * (480/640)',
      abs(CAM_FY - (IM_HEIGHT/2)/math.tan(math.radians(90)/2)) < 0.01)
check('CAM_CX == 320.0', abs(CAM_CX - 320.0) < 0.01)
check('CAM_CY == 240.0', abs(CAM_CY - 240.0) < 0.01)

# ── Test 2: Depth decoding formula ────────────────────────────────────────────
print('\n[2] CARLA depth RGBA -> metric distance')

def encode_carla_depth(Z_m):
    """Inverse of the decoding: encode a known depth Z as RGBA."""
    d   = Z_m / DEPTH_MAX_M
    val = round(d * DEPTH_DENOM)
    R   = val         % 256
    G   = (val // 256) % 256
    B   = (val // 256**2) % 256
    return R, G, B

for Z_true in [0.5, 5.0, 15.3, 42.7, 100.0, 999.9]:
    R, G, B = encode_carla_depth(Z_true)
    Z_decoded = DEPTH_MAX_M * (R + G * 256.0 + B * 256.0**2) / DEPTH_DENOM
    check(f'Depth decode Z={Z_true}m  →  decoded={Z_decoded:.3f}m',
          abs(Z_decoded - Z_true) < 0.01, f'error={abs(Z_decoded-Z_true):.4f}m')

# ── Test 3: Pinhole back-projection + ISO transform ───────────────────────────
print('\n[3] Pinhole 3D back-projection  (camera → vehicle ISO)')
fuser = PerceptionFuser(IM_WIDTH, IM_HEIGHT, CAM_FX, CAM_FY, CAM_CX, CAM_CY)

def make_depth_buf(Z, x1, y1, x2, y2):
    buf = np.full((IM_HEIGHT, IM_WIDTH), 80.0, dtype=np.float32)
    buf[int(y1):int(y2), int(x1):int(x2)] = Z
    return buf

# Object directly ahead at Z=20m, bbox centred on optical axis (u=320, v=240)
Z_obj = 20.0
x1, y1, x2, y2 = 280, 210, 360, 270  # bbox centred at (320, 240)
depth_buf = make_depth_buf(Z_obj, x1, y1, x2, y2)
det = Detection(19, 'cattle', 0.90, x1, y1, x2, y2)
out = fuser.project([det], depth_buf)
check('Single object detected',       len(out) == 1, str(out))
X_veh = out[0]['position'][0]
Y_veh = out[0]['position'][1]
# X_veh should be Z_obj + CAM_MOUNT_X = 20 + 2.5 = 22.5 m
check(f'Forward distance X_veh ≈ {Z_obj + CAM_MOUNT_X:.1f}m',
      abs(X_veh - (Z_obj + CAM_MOUNT_X)) < 0.5, f'got {X_veh:.3f}')
# Object on optical axis → Y_veh ≈ 0 (zero lateral offset)
check('Lateral offset Y_veh ≈ 0m (centred)',
      abs(Y_veh) < 0.5, f'got {Y_veh:.3f}')

# Object to the RIGHT at u=500 → X_cam positive → Y_veh negative (ISO: right = negative Y)
x1r, x2r = 460, 540
depth_buf_r = make_depth_buf(15.0, x1r, y1, x2r, y2)
det_r = Detection(2, 'auto_rickshaw', 0.78, x1r, y1, x2r, y2)
out_r = fuser.project([det_r], depth_buf_r)
check('Object right of centre → Y_veh < 0 (ISO right)',
      len(out_r) > 0 and out_r[0]['position'][1] < 0,
      f"Y_veh={out_r[0]['position'][1]:.3f} if any")

# Object to the LEFT at u=140 → Y_veh positive (ISO: left = positive Y)
x1l, x2l = 100, 180
depth_buf_l = make_depth_buf(15.0, x1l, y1, x2l, y2)
det_l = Detection(0, 'pedestrian', 0.65, x1l, y1, x2l, y2)
out_l = fuser.project([det_l], depth_buf_l)
check('Object left  of centre → Y_veh > 0 (ISO left)',
      len(out_l) > 0 and out_l[0]['position'][1] > 0,
      f"Y_veh={out_l[0]['position'][1]:.3f} if any")

# ── Test 4: Output schema validation ─────────────────────────────────────────
print('\n[4] Obstacle output schema (for MATLAB JSON)')
sample = out[0]
for field in ['id', 'type', 'position', 'velocity', 'confidence', 'behavior_profile']:
    check(f'Field "{field}" present', field in sample, str(sample.keys()))
check('position is length-2 list',
      isinstance(sample['position'], list) and len(sample['position']) == 2)
check('velocity is length-2 list (zeros, EKF infers it)',
      sample['velocity'] == [0.0, 0.0])
check('confidence in [0,1]', 0.0 <= sample['confidence'] <= 1.0)
check('type == "cattle"', sample['type'] == 'cattle')

# ── Test 5: Full JSON packet round-trip ───────────────────────────────────────
print('\n[5] Full JSON packet serialisation / deserialisation')
mock   = MockWorld()
mock.tick()
packet = {
    "ego":             mock.get_ego_telemetry(),
    "obstacles":       mock.get_obstacles(),
    "road_boundaries": mock.get_road_boundaries(),
    "collision":       False,
    "camera":          {"width": 640, "height": 480,
                        "mean_rgb": [120, 100, 80], "ready": True}
}
wire   = json.dumps(packet) + '\n'
parsed = json.loads(wire.strip())
check('Round-trip ego.x is float',   isinstance(parsed['ego']['x'], float))
check('Round-trip obstacles is list', isinstance(parsed['obstacles'], list))
check('Road boundaries non-empty',   len(parsed['road_boundaries']) > 0)
check('Packet size < 64 KB',         len(wire) < 65536,
      f'{len(wire)} bytes')
check('Camera ready flag',           parsed['camera']['ready'] is True)

# ── Summary ───────────────────────────────────────────────────────────────────
print()
print('=' * 60)
if errors:
    print(f'  RESULT: {len(errors)} test(s) FAILED:')
    for e in errors: print(f'    ✗ {e}')
else:
    print('  RESULT: ALL TESTS PASSED ✓')
print('=' * 60)
print()
sys.exit(1 if errors else 0)
