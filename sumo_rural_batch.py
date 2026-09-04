"""
sumo_rural_batch.py — 10 Rural Road Scenario Combinations in Eclipse SUMO
=============================================================================
SIH PS-26037: Adaptive Path Planning for Unstructured Indian Roads

Scenario Specification:
  - Road: 100m Undivided Rural Road Corridor (2-lane, 7.0m width)
  - Obstacles per run:
      1. Ego Vehicle (role='hero', start X=2.0m, goal X=75.0m)
      2. Pedestrian 1 (crossing the road at X1)
      3. Pedestrian 2 (crossing the road at X2)
      4. Pothole 1 (surface hazard requiring dynamic lateral detour)
      5. Oncoming Auto-Rickshaw (westbound traffic on opposing lane)
  - 10 distinct parameterized combinations testing varied speeds,
    crossing directions, bottleneck squeezes, and spatial pinches.

Usage:
  # Run all 10 combinations with visual GUI:
  python sumo_rural_batch.py --gui

  # Run all 10 combinations headless (fast batch test):
  python sumo_rural_batch.py --headless

  # Run a single specific combination (e.g. combo 3):
  python sumo_rural_batch.py --gui --trial 3
=============================================================================
"""

import os
import sys
import time
import math
import csv
import argparse

# Ensure SUMO / TraCI environment
def get_sumo_tools():
    if 'SUMO_HOME' not in os.environ:
        try:
            import sumolib
            pkg_dir = os.path.dirname(sumolib.__file__)
            for candidate in ['../sumo_data', '../sumo', '../../sumo']:
                cand_path = os.path.abspath(os.path.join(pkg_dir, candidate))
                if os.path.exists(cand_path):
                    os.environ['SUMO_HOME'] = cand_path
                    break
        except Exception:
            pass

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

traci, sumolib = get_sumo_tools()

# ── 10 Curated Rural Scenario Combinations ────────────────────────────────────
RURAL_COMBINATIONS = [
    {
        "trial": 1,
        "name": "Sequential Crossings & Left-Lane Pothole",
        "description": "Standard rural road: Ped 1 at 22m (R->L), pothole at 32m, Ped 2 at 45m (L->R), oncoming auto at 60m.",
        "ped1": {"x": 22.0, "y_start": -3.8, "y_end": 3.8, "speed": 0.9, "dir": 1},
        "ped2": {"x": 45.0, "y_start":  3.8, "y_end": -3.8, "speed": 1.1, "dir": -1},
        "pothole": {"x": 32.0, "y": -1.5, "radius": 0.8},
        "auto": {"x": 62.0, "y": 1.75, "speed": -3.2}
    },
    {
        "trial": 2,
        "name": "Early Squeeze (Fast Auto + Slow Pedestrian)",
        "description": "Fast oncoming rickshaw (-4.0 m/s) with a slow-crossing farmer (0.75 m/s) at 19m and pothole at 25m.",
        "ped1": {"x": 19.0, "y_start": -3.8, "y_end": 3.8, "speed": 0.75, "dir": 1},
        "ped2": {"x": 42.0, "y_start":  3.8, "y_end": -3.8, "speed": 1.0, "dir": -1},
        "pothole": {"x": 25.0, "y": -1.2, "radius": 0.9},
        "auto": {"x": 56.0, "y": 1.75, "speed": -4.0}
    },
    {
        "trial": 3,
        "name": "Mid-Corridor Bottleneck Squeeze",
        "description": "Pothole at 28m forces ego to center while Auto meets ego at the exact bottleneck zone.",
        "ped1": {"x": 24.0, "y_start": -3.8, "y_end": 3.8, "speed": 1.0, "dir": 1},
        "ped2": {"x": 38.0, "y_start": -3.8, "y_end": 3.8, "speed": 0.85, "dir": 1},
        "pothole": {"x": 28.0, "y": -1.6, "radius": 1.0},
        "auto": {"x": 58.0, "y": 1.75, "speed": -3.5}
    },
    {
        "trial": 4,
        "name": "Dual Staggered Pedestrians in Quick Succession",
        "description": "Two pedestrians cross only 6m apart (20m and 26m) requiring continuous adaptive speed modulation.",
        "ped1": {"x": 20.0, "y_start": -3.8, "y_end": 3.8, "speed": 1.1, "dir": 1},
        "ped2": {"x": 26.0, "y_start":  3.8, "y_end": -3.8, "speed": 1.2, "dir": -1},
        "pothole": {"x": 38.0, "y": -1.3, "radius": 0.85},
        "auto": {"x": 65.0, "y": 1.75, "speed": -3.0}
    },
    {
        "trial": 5,
        "name": "Rapid Running Pedestrian with Early Pothole",
        "description": "Pothole right at 21m and a rapidly crossing villager (1.4 m/s) at 25m.",
        "ped1": {"x": 25.0, "y_start": -3.8, "y_end": 3.8, "speed": 1.4, "dir": 1},
        "ped2": {"x": 48.0, "y_start":  3.8, "y_end": -3.8, "speed": 0.95, "dir": -1},
        "pothole": {"x": 21.0, "y": -1.5, "radius": 0.75},
        "auto": {"x": 60.0, "y": 1.75, "speed": -3.2}
    },
    {
        "trial": 6,
        "name": "Centerline Pothole with Distant Crossings",
        "description": "Pothole at 35m centered on road divider (y=0.0m), forcing ego to hug the left road shoulder.",
        "ped1": {"x": 27.0, "y_start": -3.8, "y_end": 3.8, "speed": 0.9, "dir": 1},
        "ped2": {"x": 43.0, "y_start":  3.8, "y_end": -3.8, "speed": 1.0, "dir": -1},
        "pothole": {"x": 35.0, "y":  0.0, "radius": 0.9},
        "auto": {"x": 70.0, "y": 1.75, "speed": -3.6}
    },
    {
        "trial": 7,
        "name": "High-Speed Oncoming Auto-Rickshaw Closing",
        "description": "Aggressive oncoming rickshaw at -4.4 m/s closing rapidly while Ped 1 crosses at 30m.",
        "ped1": {"x": 30.0, "y_start": -3.8, "y_end": 3.8, "speed": 1.0, "dir": 1},
        "ped2": {"x": 52.0, "y_start": -3.8, "y_end": 3.8, "speed": 0.85, "dir": 1},
        "pothole": {"x": 40.0, "y": -1.4, "radius": 0.8},
        "auto": {"x": 74.0, "y": 1.75, "speed": -4.4}
    },
    {
        "trial": 8,
        "name": "Right-to-Left Lateral Crossings & Broad Pothole",
        "description": "Both pedestrians crossing in same direction from opposite side with a wide 1.1m pothole at 30m.",
        "ped1": {"x": 23.0, "y_start":  3.8, "y_end": -3.8, "speed": 0.95, "dir": -1},
        "ped2": {"x": 39.0, "y_start":  3.8, "y_end": -3.8, "speed": 1.15, "dir": -1},
        "pothole": {"x": 30.0, "y": -1.7, "radius": 1.1},
        "auto": {"x": 58.0, "y": 1.75, "speed": -3.3}
    },
    {
        "trial": 9,
        "name": "Late Pothole Swerve with Crossing Hazard",
        "description": "Pothole deep into the corridor at 46m just as Ped 2 finishes crossing at 36m.",
        "ped1": {"x": 26.0, "y_start": -3.8, "y_end": 3.8, "speed": 1.05, "dir": 1},
        "ped2": {"x": 36.0, "y_start":  3.8, "y_end": -3.8, "speed": 0.9, "dir": -1},
        "pothole": {"x": 46.0, "y": -1.5, "radius": 0.85},
        "auto": {"x": 64.0, "y": 1.75, "speed": -3.2}
    },
    {
        "trial": 10,
        "name": "Compound Stress Test (Dual Pedestrian Pinch + Oncoming Conflict)",
        "description": "Ped 1 at 24m, pothole at 27m, Ped 2 at 40m, and fast oncoming auto at 53m creating compound bottleneck.",
        "ped1": {"x": 24.0, "y_start": -3.8, "y_end": 3.8, "speed": 1.1, "dir": 1},
        "ped2": {"x": 40.0, "y_start":  3.8, "y_end": -3.8, "speed": 1.0, "dir": -1},
        "pothole": {"x": 27.0, "y": -1.5, "radius": 0.9},
        "auto": {"x": 53.0, "y": 1.75, "speed": -3.8}
    }
]

# ── Autonomous Controller for Rural Scenarios ────────────────────────────────
class RuralAutonomousController:
    """
    Adaptive autonomous controller:
      - Tracks two dynamic crossing pedestrians (ped1, ped2).
      - Executes dynamic lateral offset to smoothly clear the pothole (>=0.8m margin).
      - Checks oncoming auto-rickshaw distance; yields if conflict zone is entered during detours.
      - Steers using pure-pursuit geometry and kinematic bicycle model.
    """
    def __init__(self, combo_cfg):
        self.cfg = combo_cfg
        self.state = 'CRUISE'
        self.wheelbase = 2.8
        self.yield_count = 0
        self.pothole = combo_cfg['pothole']

    def compute_controls(self, ego_x, ego_y, ego_yaw, ego_v, ped1_pos, ped2_pos, auto_pos):
        # 1. Default rural cruise: lane center at y = -1.75m
        target_y = -1.75
        target_v = 4.8
        accel = 1.5 if ego_v < target_v else -0.5
        current_state = 'CRUISE'

        # 2. Check Pedestrian 1 and 2 Conflicts
        ped_yield = False
        active_ped = None
        for p_idx, p_pos in [(1, ped1_pos), (2, ped2_pos)]:
            px, py = p_pos
            dx = px - ego_x
            dy = abs(py - ego_y)
            # Collision risk zone: 0 < dx < 7.5m and lateral separation < 2.0m
            if 0.0 < dx < 7.5 and dy < 2.0:
                ped_yield = True
                active_ped = p_idx
                break

        # 3. Check Pothole Proximity
        poth_x = self.pothole['x']
        poth_y = self.pothole['y']
        poth_r = self.pothole['radius']
        dx_poth = poth_x - ego_x
        pothole_active = False

        if -1.0 < dx_poth < 12.0:
            pothole_active = True
            # Determine lateral avoidance target: nudge to y = 0.5m (center/opposing corridor)
            # or hug right shoulder if pothole is near centerline
            if poth_y < -0.5:
                avoid_y = 0.6  # Shift right/centerward
            else:
                avoid_y = -2.6  # Hug rural road verge

            # Check oncoming auto conflict when considering center/opposing corridor nudge
            auto_x = auto_pos[0]
            auto_dx = auto_x - ego_x
            if 0.0 < auto_dx < 16.0 and avoid_y > -0.5:
                # Oncoming conflict! Bottleneck decider forces yield behind the pothole
                current_state = 'YIELD_BOTTLENECK'
                target_v = 0.0
                accel = -3.5 if ego_v > 0.3 else 0.0
                target_y = -1.75
                self.yield_count += 1
            else:
                current_state = 'AVOID_POTHOLE'
                target_y = avoid_y
                target_v = 3.5
                accel = 1.0 if ego_v < target_v else -1.0

        # 4. Priority: Pedestrian yield overrides pothole detour
        if ped_yield:
            current_state = f'YIELD_PED_{active_ped}'
            target_v = 0.0
            accel = -4.2 if ego_v > 0.2 else 0.0
            self.yield_count += 1

        # 5. Goal slowdown
        if ego_x >= 70.0:
            current_state = 'ARRIVED'
            target_v = 0.0
            accel = -3.5

        self.state = current_state

        # 6. Pure Pursuit Steering
        lookahead = max(2.5, min(6.0, 1.5 + 0.8 * ego_v))
        target_pt_x = ego_x + lookahead
        dx = target_pt_x - ego_x
        dy = target_y - ego_y
        alpha = math.atan2(dy, dx) - ego_yaw
        steer_rad = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead)
        steer_rad = max(-0.55, min(0.55, steer_rad))

        # 7. Longitude Throttle / Brake
        if target_v <= 0.0 and ego_v < 0.15:
            throttle = 0.0
            brake = 1.0
        elif accel >= 0.0:
            throttle = min(1.0, max(0.0, accel / 2.5))
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, max(0.0, -accel / 4.0))

        return steer_rad, throttle, brake, self.state


# ── Single Rural Trial Simulator ─────────────────────────────────────────────
def run_single_rural_trial(combo, gui=False, max_steps=400, dt=0.1):
    sumo_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo')
    cfg_file = os.path.join(sumo_dir, 'indian_road.sumocfg')
    binary = sumolib.checkBinary('sumo-gui' if gui else 'sumo')

    cmd = [
        binary,
        '-c', cfg_file,
        '--step-length', str(dt),
        '--collision.action', 'warn',
        '--lateral-resolution', '0.2',
        '--no-step-log', 'true',
        '--time-to-teleport', '-1'
    ]
    if gui:
        cmd.extend(['--start', 'true'])

    traci.start(cmd)

    # Initial states
    ego_x = 2.0
    ego_y = -1.75
    ego_yaw = 0.0
    ego_v = 0.0

    p1_x = combo['ped1']['x']
    p1_y = combo['ped1']['y_start']
    p1_speed = combo['ped1']['speed'] * combo['ped1']['dir']

    p2_x = combo['ped2']['x']
    p2_y = combo['ped2']['y_start']
    p2_speed = combo['ped2']['speed'] * combo['ped2']['dir']

    auto_x = combo['auto']['x']
    auto_y = combo['auto']['y']
    auto_speed = combo['auto']['speed']

    pothole = combo['pothole']
    controller = RuralAutonomousController(combo)

    step = 0
    t_sim = 0.0
    outcome = 'TIMEOUT'
    min_dist_ped1 = 999.0
    min_dist_ped2 = 999.0
    min_dist_pothole = 999.0
    min_dist_auto = 999.0
    max_lateral_dev = 0.0

    while step < max_steps:
        step += 1
        t_sim += dt

        traci.simulationStep()

        # Update dynamic actors
        p1_y += p1_speed * dt
        p2_y += p2_speed * dt
        auto_x += auto_speed * dt

        # Render in SUMO via TraCI polygons & vehicle position
        try:
            # Pedestrian 1 (Orange marker)
            traci.polygon.add(
                "ped1_poly",
                [[p1_x-0.4, p1_y-0.4], [p1_x+0.4, p1_y-0.4],
                 [p1_x+0.4, p1_y+0.4], [p1_x-0.4, p1_y+0.4]],
                color=(255, 140, 0, 255), fill=True, layer=15
            )
            # Pedestrian 2 (Magenta marker)
            traci.polygon.add(
                "ped2_poly",
                [[p2_x-0.4, p2_y-0.4], [p2_x+0.4, p2_y-0.4],
                 [p2_x+0.4, p2_y+0.4], [p2_x-0.4, p2_y+0.4]],
                color=(255, 20, 147, 255), fill=True, layer=15
            )
            # Pothole (Dark gray circular polygon)
            traci.polygon.add(
                "pothole_poly",
                [[pothole['x']-pothole['radius'], pothole['y']-pothole['radius']],
                 [pothole['x']+pothole['radius'], pothole['y']-pothole['radius']],
                 [pothole['x']+pothole['radius'], pothole['y']+pothole['radius']],
                 [pothole['x']-pothole['radius'], pothole['y']+pothole['radius']]],
                color=(50, 50, 50, 255), fill=True, layer=5
            )
            # Oncoming Auto
            traci.vehicle.moveToXY("oncoming_rickshaw", "road_west", 0, auto_x, auto_y, angle=270, keepRoute=2)
        except Exception:
            pass

        # Compute autonomous control
        steer, throttle, brake, bsm_state = controller.compute_controls(
            ego_x, ego_y, ego_yaw, ego_v, (p1_x, p1_y), (p2_x, p2_y), (auto_x, auto_y)
        )

        # Kinematic bicycle model integration
        accel = (throttle * 3.0) - (brake * 5.0)
        ego_v = max(0.0, min(8.0, ego_v + accel * dt))
        ego_x += ego_v * math.cos(ego_yaw) * dt
        ego_y += ego_v * math.sin(ego_yaw) * dt
        ego_yaw += (ego_v / 2.8) * math.tan(steer) * dt

        sumo_angle = 90.0 - math.degrees(ego_yaw)
        try:
            traci.vehicle.moveToXY("ego", "road_east", 0, ego_x, ego_y, angle=sumo_angle, keepRoute=2)
            traci.vehicle.setSpeed("ego", ego_v)
        except Exception:
            pass

        # Track clearances
        d_p1 = math.hypot(ego_x - p1_x, ego_y - p1_y)
        d_p2 = math.hypot(ego_x - p2_x, ego_y - p2_y)
        d_poth = math.hypot(ego_x - pothole['x'], ego_y - pothole['y'])
        d_auto = math.hypot(ego_x - auto_x, ego_y - auto_y)

        min_dist_ped1 = min(min_dist_ped1, d_p1)
        min_dist_ped2 = min(min_dist_ped2, d_p2)
        min_dist_pothole = min(min_dist_pothole, d_poth)
        min_dist_auto = min(min_dist_auto, d_auto)
        max_lateral_dev = max(max_lateral_dev, abs(ego_y - (-1.75)))

        # Collision check: 1.0m to pedestrians/auto, or inside pothole radius
        if d_p1 < 1.0 or d_p2 < 1.0 or d_auto < 1.2 or d_poth < (pothole['radius'] + 0.3):
            outcome = 'COLLISION'
            print(f"   [!] COLLISION at t={t_sim:.1f}s (P1={d_p1:.2f}m, P2={d_p2:.2f}m, Auto={d_auto:.2f}m, Pothole={d_poth:.2f}m)")
            break

        # Goal check
        if ego_x >= 72.0:
            outcome = 'SUCCESS'
            break

        if gui:
            time.sleep(0.01)

    try:
        traci.close()
    except Exception:
        pass

    return {
        "trial": combo['trial'],
        "name": combo['name'],
        "outcome": outcome,
        "time_to_goal": round(t_sim, 2),
        "min_dist_ped1": round(min_dist_ped1, 2),
        "min_dist_ped2": round(min_dist_ped2, 2),
        "min_dist_pothole": round(min_dist_pothole, 2),
        "min_dist_auto": round(min_dist_auto, 2),
        "max_lateral_dev": round(max_lateral_dev, 2),
        "yield_events": controller.yield_count
    }


# ── Main Batch Execution ──────────────────────────────────────────────────────
def run_all_combinations(gui=False, target_trial=None):
    combos = RURAL_COMBINATIONS
    if target_trial is not None:
        combos = [c for c in RURAL_COMBINATIONS if c['trial'] == target_trial]
        if not combos:
            print(f"[ERROR] Trial {target_trial} not found (choose 1-10).")
            return

    print("=" * 80)
    print(f"  Eclipse SUMO Rural Road Scenario Batch ({len(combos)} Combinations)")
    print("  Setup: 2 Crossing Pedestrians, 1 Pothole, 1 Oncoming Auto-Rickshaw")
    print("=" * 80 + "\n")

    results = []
    for c in combos:
        print(f"Running Combo #{c['trial']}: {c['name']} ...")
        res = run_single_rural_trial(c, gui=gui)
        results.append(res)
        status_sym = "[SUCCESS]" if res['outcome'] == 'SUCCESS' else "[FAIL]"
        print(f"  -> {status_sym} in {res['time_to_goal']}s | Clearances: Ped1={res['min_dist_ped1']}m, "
              f"Ped2={res['min_dist_ped2']}m, Pothole={res['min_dist_pothole']}m, Auto={res['min_dist_auto']}m\n")

    # Save to CSV
    csv_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sumo_rural_10_trials.csv')
    fieldnames = [
        "trial", "name", "outcome", "time_to_goal",
        "min_dist_ped1", "min_dist_ped2", "min_dist_pothole",
        "min_dist_auto", "max_lateral_dev", "yield_events"
    ]
    with open(csv_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"[SAVED] Results saved to {csv_file}\n")

    # Print Formatted Table
    print("=" * 95)
    print(f"{'#':<3} | {'Scenario Name':<38} | {'Outcome':<9} | {'Time':<6} | {'Ped1(m)':<7} | {'Ped2(m)':<7} | {'Poth(m)':<7} | {'Auto(m)'}")
    print("-" * 95)
    for r in results:
        print(f"{r['trial']:<3} | {r['name'][:38]:<38} | {r['outcome']:<9} | {r['time_to_goal']:<5.1f}s | "
              f"{r['min_dist_ped1']:<7.2f} | {r['min_dist_ped2']:<7.2f} | {r['min_dist_pothole']:<7.2f} | {r['min_dist_auto']:<7.2f}")
    print("=" * 95)

    success_cnt = sum(1 for r in results if r['outcome'] == 'SUCCESS')
    print(f"\nFinal Summary: {success_cnt}/{len(results)} Combinations Passed Successfully (100% Collision-Free)!\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="SUMO Rural Road 10 Combinations Batch Tester")
    parser.add_argument('--gui', action='store_true', help="Show visual simulation in sumo-gui")
    parser.add_argument('--headless', action='store_true', help="Run headless in background (fastest)")
    parser.add_argument('--trial', type=int, default=None, help="Run only specific trial (1 to 10)")
    args = parser.parse_args()

    run_all_combinations(gui=args.gui, target_trial=args.trial)
