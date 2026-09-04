"""
CARLA simulation: 1 ego vehicle + 2 dynamic obstacles + 2 static obstacles.

Tested against:
    - Server (API) version: 0.9.15
    - Client (pip package)  version: 0.9.16

Notes on version mismatch:
    CARLA prints a warning like:
        "WARNING: Client and server versions mismatch"
    This is expected and usually harmless across one minor version gap.
    If you actually hit AttributeError / API-shape errors, install a
    matching client instead:
        pip install carla==0.9.15

Obstacle design:
    Dynamic obstacles  -> real vehicles driven by the CARLA Traffic Manager
                           (autopilot=True), so they move around the map.
    Static obstacles    -> vehicles spawned normally, then autopilot is
                           left off and physics simulation is disabled so
                           they behave as fixed, non-moving obstacles
                           (like parked cars / roadblocks).
"""

import random
import time
import carla


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
HOST = "127.0.0.1"
PORT = 2000
TIMEOUT_S = 10.0

TM_PORT = 8000          # Traffic Manager port
SYNC_MODE = True         # Run world in synchronous mode (recommended)
FIXED_DELTA_SECONDS = 0.05

SIM_DURATION_S = 30      # How long to run the loop before cleanup


def connect_to_carla():
    """Connect to the CARLA server and return (client, world)."""
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT_S)

    print(f"Client API version: {client.get_client_version()}")
    print(f"Server API version: {client.get_server_version()}")

    world = client.get_world()
    return client, world


def configure_sync_mode(client, world, enable=True):
    """Enable/disable synchronous mode for deterministic stepping."""
    settings = world.get_settings()
    settings.synchronous_mode = enable
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS if enable else None
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(TM_PORT)
    traffic_manager.set_synchronous_mode(enable)
    return traffic_manager


def spawn_vehicle(world, blueprint_library, spawn_point, role_name="obstacle"):
    """Spawn a single 4-wheeled vehicle at a given spawn point."""
    vehicle_bps = blueprint_library.filter("vehicle.*")
    # Restrict to 4-wheel vehicles to avoid motorbikes/bicycles for obstacles
    vehicle_bps = [bp for bp in vehicle_bps if int(bp.get_attribute("number_of_wheels")) == 4]
    bp = random.choice(vehicle_bps)
    bp.set_attribute("role_name", role_name)
    if bp.has_attribute("color"):
        color = random.choice(bp.get_attribute("color").recommended_values)
        bp.set_attribute("color", color)

    vehicle = world.try_spawn_actor(bp, spawn_point)
    return vehicle


def make_dynamic_obstacle(world, traffic_manager, blueprint_library, spawn_point, index):
    """Spawn a vehicle and hand control to the Traffic Manager (moving obstacle)."""
    vehicle = spawn_vehicle(world, blueprint_library, spawn_point, role_name=f"dynamic_obstacle_{index}")
    if vehicle is None:
        print(f"[dynamic obstacle {index}] Failed to spawn at {spawn_point.location}")
        return None

    vehicle.set_autopilot(True, traffic_manager.get_port())

    # Optional: tweak per-vehicle TM behavior
    traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(-10, 20))
    traffic_manager.distance_to_leading_vehicle(vehicle, 3.0)
    traffic_manager.auto_lane_change(vehicle, True)

    print(f"[dynamic obstacle {index}] Spawned id={vehicle.id} at {spawn_point.location}")
    return vehicle


def make_static_obstacle(world, blueprint_library, spawn_point, index):
    """Spawn a vehicle and freeze it in place (fixed obstacle)."""
    vehicle = spawn_vehicle(world, blueprint_library, spawn_point, role_name=f"static_obstacle_{index}")
    if vehicle is None:
        print(f"[static obstacle {index}] Failed to spawn at {spawn_point.location}")
        return None

    vehicle.set_autopilot(False)
    vehicle.set_simulate_physics(False)   # locks the actor in place, ignores forces
    vehicle.apply_control(carla.VehicleControl(hand_brake=True))

    print(f"[static obstacle {index}] Spawned id={vehicle.id} at {spawn_point.location}")
    return vehicle


def main():
    client, world = connect_to_carla()
    traffic_manager = configure_sync_mode(client, world, enable=SYNC_MODE)

    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < 5:
        raise RuntimeError("Map does not have enough spawn points for ego + 4 obstacles.")

    random.shuffle(spawn_points)
    ego_sp = spawn_points[0]
    dynamic_sps = spawn_points[1:3]
    static_sps = spawn_points[3:5]

    actor_list = []

    try:
        # --- Ego vehicle (optional, useful for testing obstacle avoidance) ---
        ego_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("role_name", "ego_vehicle")
        ego_vehicle = world.try_spawn_actor(ego_bp, ego_sp)
        if ego_vehicle is None:
            raise RuntimeError("Failed to spawn ego vehicle.")
        ego_vehicle.set_autopilot(True, traffic_manager.get_port())
        actor_list.append(ego_vehicle)
        print(f"[ego] Spawned id={ego_vehicle.id} at {ego_sp.location}")

        # --- Two dynamic obstacles ---
        for i, sp in enumerate(dynamic_sps, start=1):
            v = make_dynamic_obstacle(world, traffic_manager, blueprint_library, sp, i)
            if v:
                actor_list.append(v)

        # --- Two static obstacles ---
        for i, sp in enumerate(static_sps, start=1):
            v = make_static_obstacle(world, blueprint_library, sp, i)
            if v:
                actor_list.append(v)

        # --- Spectator follows the ego vehicle from above ---
        spectator = world.get_spectator()

        # --- Main simulation loop ---
        start_time = time.time()
        while time.time() - start_time < SIM_DURATION_S:
            if SYNC_MODE:
                world.tick()
            else:
                world.wait_for_tick()

            transform = ego_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location + carla.Location(z=30),
                    carla.Rotation(pitch=-90),
                )
            )

    finally:
        print("Cleaning up actors...")
        for actor in actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()

        # Restore world to asynchronous mode so it doesn't hang other clients
        configure_sync_mode(client, world, enable=False)
        print("Done.")


if __name__ == "__main__":
    main()