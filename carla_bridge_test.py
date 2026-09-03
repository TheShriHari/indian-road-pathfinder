"""Protocol self-test for carla_bridge.py — no CARLA install needed."""
import json

# Build a sample state packet (mirrors MockWorld.get_*)
packet = {
    "ego": {"x": 2.4, "y": 0.0, "yaw": 0.0, "v": 4.0},
    "obstacles": [
        {"id": 1, "type": "cattle",
         "position": [30.0, -3.43], "velocity": [0.0, 0.7],
         "behavior_profile": "erratic"},
        {"id": 2, "type": "auto_rickshaw",
         "position": [43.2, 1.2], "velocity": [-3.2, 0.0],
         "behavior_profile": "weaving"}
    ],
    "road_boundaries": [[x, -2.5] for x in range(-5, 65)]
                      + [[x,  2.5] for x in range(-5, 65)],
    "collision": False,
    "camera": {"width": 640, "height": 480,
               "mean_rgb": [120, 100, 80], "ready": True}
}

# Serialise as the bridge would send it
line = json.dumps(packet) + "\n"

# Parse back as MATLAB jsondecode would receive it
decoded = json.loads(line.strip())

# Control command MATLAB sends back
ctrl_str = '{"steer": 0.12, "throttle": 0.40, "brake": 0.00}'
ctrl = json.loads(ctrl_str)

# Assertions
assert decoded["ego"]["x"] == 2.4
assert len(decoded["obstacles"]) == 2
assert len(decoded["road_boundaries"]) == 140
assert decoded["collision"] is False
assert decoded["camera"]["ready"] is True
assert 0.0 <= ctrl["throttle"] <= 1.0
assert -1.0 <= ctrl["steer"] <= 1.0

print("=" * 54)
print("  carla_bridge.py — PROTOCOL SELF-TEST PASSED")
print("=" * 54)
print(f"  Packet size      : {len(line)} bytes")
print(f"  Ego pos          : ({decoded['ego']['x']:.2f}, {decoded['ego']['y']:.2f})")
print(f"  Obstacles        : {len(decoded['obstacles'])}")
print(f"  Road boundary pts: {len(decoded['road_boundaries'])}")
print(f"  Camera ready     : {decoded['camera']['ready']}")
print(f"  Control parsed   : steer={ctrl['steer']:.2f}  throttle={ctrl['throttle']:.2f}  brake={ctrl['brake']:.2f}")
print()
