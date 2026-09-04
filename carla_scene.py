"""
carla_scene.py  —  Indian Road Scenario & Obstacle Generator for CARLA
=======================================================================
Spawns realistic Indian road obstacles along the ego vehicle's corridor:
  1. Static Left-Side Roadblock / Parked Vehicle (+18m ahead)
  2. Static Right-Side Hazard / Debris / Pothole Surrogate (+38m ahead)
  3. Dynamic Oncoming Traffic (Auto-rickshaw / Two-wheeler surrogate, +55m ahead)
  4. Dynamic Crossing Pedestrian / Cattle Surrogate (+28m ahead)
  5. Live 3D Spectator Chase Camera tracking the ego vehicle
=======================================================================
"""

import math
import random
import time

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    carla = None
    CARLA_AVAILABLE = False


def set_indian_road_weather(world):
    """Sets lighting and atmospheric conditions typical of Indian daytime roads."""
    if not CARLA_AVAILABLE or world is None:
        return
    try:
        # Slightly hazy, warm sunny conditions
        weather = carla.WeatherParameters(
            cloudiness=25.0,
            precipitation=0.0,
            precipitation_deposits=10.0,
            wind_intensity=10.0,
            sun_azimuth_angle=70.0,
            sun_altitude_angle=60.0,
            fog_density=2.0,
            fog_distance=100.0,
            wetness=5.0
        )
        world.set_weather(weather)
        print("[SCENE] Indian daytime road atmospheric profile applied.")
    except Exception as e:
        print(f"[SCENE] Notice: Weather update skipped ({e})")


def update_spectator(world, ego_vehicle, pitch=-18.0, distance=11.0, height=4.5):
    """
    Positions the CARLA viewport spectator camera in a third-person chase view
    behind and above the ego vehicle, oriented along the vehicle's heading.
    """
    if not CARLA_AVAILABLE or world is None or ego_vehicle is None:
        return
    try:
        spectator = world.get_spectator()
        tf = ego_vehicle.get_transform()
        yaw_rad = math.radians(tf.rotation.yaw)

        # Place camera behind and above
        cam_x = tf.location.x - distance * math.cos(yaw_rad)
        cam_y = tf.location.y - distance * math.sin(yaw_rad)
        cam_z = tf.location.z + height

        spectator.set_transform(carla.Transform(
            carla.Location(x=cam_x, y=cam_y, z=cam_z),
            carla.Rotation(pitch=pitch, yaw=tf.rotation.yaw, roll=0.0)
        ))
    except Exception:
        pass


def find_suitable_spawn_point(world, min_forward_dist=80.0):
    """
    Finds a spawn point with a long, unobstructed road ahead.
    Falls back to the first available spawn point if road search fails.
    """
    if not CARLA_AVAILABLE or world is None:
        return None

    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    if not spawn_points:
        return None

    # Search for a spawn point that has consecutive waypoints ahead
    for sp in spawn_points:
        wp = carla_map.get_waypoint(sp.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        if wp is None:
            continue
        # Check forward distance
        curr = wp
        accum = 0.0
        straight = True
        while accum < min_forward_dist:
            nxt = curr.next(5.0)
            if not nxt:
                straight = False
                break
            curr = nxt[0]
            accum += 5.0

        if straight and accum >= min_forward_dist:
            return sp

    # Default to spawn point 0 if no candidate passes the filter
    return spawn_points[0]


def spawn_corridor_obstacles(world, ego_vehicle, traffic_manager=None):
    """
    Spawns static roadblocks, pothole markers, oncoming traffic, and crossing
    pedestrians strictly along the corridor ahead of the ego vehicle.

    Returns:
        list of spawned carla.Actor instances
    """
    if not CARLA_AVAILABLE or world is None or ego_vehicle is None:
        return []

    bp_lib = world.get_blueprint_library()
    carla_map = world.get_map()
    ego_tf = ego_vehicle.get_transform()
    ego_wp = carla_map.get_waypoint(ego_tf.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)

    if ego_wp is None:
        print("[SCENE] Warning: Ego vehicle is not on a driving lane.")
        return []

    # Trace waypoints forward along the lane
    wps_ahead = []
    curr = ego_wp
    accum = 0.0
    while accum <= 75.0:
        nxt = curr.next(4.0)
        if not nxt:
            break
        curr = nxt[0]
        accum += 4.0
        wps_ahead.append((accum, curr))

    if len(wps_ahead) < 5:
        print("[SCENE] Road segment too short to place full obstacle suite.")
        return []

    def get_wp_near(dist_m):
        closest = None
        min_err = 1e6
        for d, wp in wps_ahead:
            err = abs(d - dist_m)
            if err < min_err:
                min_err = err
                closest = wp
        return closest

    spawned_actors = []

    # -------------------------------------------------------------------------
    # 1. STATIC OBSTACLE 1: Parked Vehicle / Roadblock on Left Lane (+18m)
    # Forces ego vehicle to steer right into the available corridor.
    # -------------------------------------------------------------------------
    wp_18 = get_wp_near(18.0)
    if wp_18:
        right_vec = wp_18.transform.get_right_vector()
        lane_w = wp_18.lane_width
        # Offset leftwards across the lane
        obs_loc = wp_18.transform.location - right_vec * (lane_w * 0.25)
        obs_loc.z += 0.2
        obs_tf = carla.Transform(obs_loc, wp_18.transform.rotation)

        # Select a vehicle blueprint (e.g. Nissan Patrol, Audi, or delivery car)
        bps = bp_lib.filter('vehicle.*')
        bps_4w = [b for b in bps if b.has_attribute('number_of_wheels') and int(b.get_attribute('number_of_wheels')) == 4]
        bp_choice = random.choice(bps_4w) if bps_4w else bp_lib.filter('vehicle.audi.tt')[0]
        bp_choice.set_attribute('role_name', 'static_obstacle_1')

        obs1 = world.try_spawn_actor(bp_choice, obs_tf)
        if obs1:
            obs1.set_simulate_physics(False)  # Lock in place
            obs1.apply_control(carla.VehicleControl(hand_brake=True))
            spawned_actors.append(obs1)
            print(f"[SCENE] Static Obstacle 1 (Left Roadblock) spawned at +18m: {obs1.type_id}")

    # -------------------------------------------------------------------------
    # 2. STATIC OBSTACLE 2: Debris / Construction Hazard on Right Lane (+38m)
    # Forces ego vehicle to negotiate a chicane / weave back to the left corridor.
    # -------------------------------------------------------------------------
    wp_38 = get_wp_near(38.0)
    if wp_38:
        right_vec = wp_38.transform.get_right_vector()
        lane_w = wp_38.lane_width
        # Offset rightwards across the lane
        obs_loc = wp_38.transform.location + right_vec * (lane_w * 0.25)
        obs_loc.z += 0.2
        obs_tf = carla.Transform(obs_loc, wp_38.transform.rotation)

        # Try spawning a construction prop, barrel, or compact vehicle
        prop_bps = bp_lib.filter('static.prop.*')
        cone_bps = [b for b in prop_bps if 'barrel' in b.id or 'construction' in b.id or 'box' in b.id or 'barrier' in b.id]
        if cone_bps:
            bp_prop = random.choice(cone_bps)
        else:
            bp_prop = bp_lib.filter('vehicle.audi.tt')[0]
        bp_prop.set_attribute('role_name', 'static_obstacle_2')

        obs2 = world.try_spawn_actor(bp_prop, obs_tf)
        if obs2:
            if hasattr(obs2, 'set_simulate_physics'):
                obs2.set_simulate_physics(False)
            spawned_actors.append(obs2)
            print(f"[SCENE] Static Obstacle 2 (Right Hazard) spawned at +38m: {obs2.type_id}")

    # -------------------------------------------------------------------------
    # 3. DYNAMIC OBSTACLE 1: Oncoming Auto-Rickshaw / Two-Wheeler (+55m)
    # Approaches ego head-on, testing spatio-temporal corridor yield decider.
    # -------------------------------------------------------------------------
    wp_55 = get_wp_near(55.0)
    if wp_55:
        # Check if opposite/adjacent driving lane exists
        opp_wp = wp_55.get_left_lane()
        if opp_wp and opp_wp.lane_type == carla.LaneType.Driving:
            spawn_wp = opp_wp
        else:
            spawn_wp = wp_55

        # Face opposite direction (heading towards ego)
        rot_rev = carla.Rotation(
            pitch=spawn_wp.transform.rotation.pitch,
            yaw=spawn_wp.transform.rotation.yaw + 180.0,
            roll=spawn_wp.transform.rotation.roll
        )
        loc_55 = spawn_wp.transform.location
        loc_55.z += 0.3
        tf_55 = carla.Transform(loc_55, rot_rev)

        # Prefer 2-wheelers or compact cars as auto-rickshaw surrogates
        two_wheelers = [b for b in bp_lib.filter('vehicle.*') if b.has_attribute('number_of_wheels') and int(b.get_attribute('number_of_wheels')) == 2]
        if two_wheelers:
            dyn_bp = random.choice(two_wheelers)
        else:
            dyn_bp = bp_lib.filter('vehicle.toyota.prius')[0]
        dyn_bp.set_attribute('role_name', 'dynamic_oncoming')

        dyn1 = world.try_spawn_actor(dyn_bp, tf_55)
        if dyn1:
            if traffic_manager:
                dyn1.set_autopilot(True, traffic_manager.get_port())
                traffic_manager.vehicle_percentage_speed_difference(dyn1, -20.0) # ~12-15 km/h
            else:
                # Direct steady forward control
                dyn1.apply_control(carla.VehicleControl(throttle=0.35, steer=0.0))
            spawned_actors.append(dyn1)
            print(f"[SCENE] Dynamic Obstacle 1 (Oncoming Rickshaw Surrogate) spawned at +55m: {dyn1.type_id}")

    # -------------------------------------------------------------------------
    # 4. DYNAMIC OBSTACLE 2: Crossing Pedestrian / Cattle Surrogate (+28m)
    # Starts near the shoulder and crosses across the lane.
    # -------------------------------------------------------------------------
    wp_28 = get_wp_near(28.0)
    if wp_28:
        right_vec = wp_28.transform.get_right_vector()
        lane_w = wp_28.lane_width
        # Spawn on left shoulder
        loc_28 = wp_28.transform.location - right_vec * (lane_w * 0.85)
        loc_28.z += 0.5
        # Heading perpendicular to road to cross across
        yaw_cross = wp_28.transform.rotation.yaw + 90.0
        tf_28 = carla.Transform(loc_28, carla.Rotation(yaw=yaw_cross))

        walker_bps = bp_lib.filter('walker.pedestrian.*')
        if walker_bps:
            walk_bp = random.choice(walker_bps)
            walk_bp.set_attribute('role_name', 'crossing_agent')
            ped = world.try_spawn_actor(walk_bp, tf_28)
            if ped:
                # Give walker a forward crossing velocity
                ctrl = carla.WalkerControl()
                ctrl.direction = carla.Vector3D(x=right_vec.x, y=right_vec.y, z=0.0)
                ctrl.speed = 1.1  # 1.1 m/s walking speed
                ped.apply_control(ctrl)
                spawned_actors.append(ped)
                print(f"[SCENE] Dynamic Obstacle 2 (Crossing Pedestrian/Cattle) spawned at +28m: {ped.type_id}")

    print(f"[SCENE] Complete Indian road scenario spawned: {len(spawned_actors)} obstacles along corridor.")
    return spawned_actors
