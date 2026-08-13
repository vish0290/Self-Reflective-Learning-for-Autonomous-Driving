"""Spawn an ego vehicle on autopilot, plus background traffic and pedestrians."""
import argparse
import random
import time

import carla


def spawn_ego(world, vehicle_filter="vehicle.tesla.model3"):
    bp = world.get_blueprint_library().filter(vehicle_filter)[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle.set_autopilot(True)
    print(f"Ego vehicle spawned ({vehicle.type_id}), autopilot enabled.")
    return vehicle


def spawn_traffic(client, world, n_vehicles, taken_spawn_point):
    blueprints = [bp for bp in world.get_blueprint_library().filter("vehicle.*")
                  if int(bp.get_attribute("number_of_wheels")) == 4]
    spawn_points = [p for p in world.get_map().get_spawn_points() if p != taken_spawn_point]
    random.shuffle(spawn_points)

    vehicles = []
    for spawn_point in spawn_points[:n_vehicles]:
        bp = random.choice(blueprints)
        vehicle = world.try_spawn_actor(bp, spawn_point)
        if vehicle is not None:
            vehicle.set_autopilot(True)
            vehicles.append(vehicle)
    print(f"Spawned {len(vehicles)} traffic vehicles.")
    return vehicles


def spawn_walkers(client, world, n_walkers):
    walker_bps = world.get_blueprint_library().filter("walker.pedestrian.*")
    walker_control_bp = world.get_blueprint_library().find("controller.ai.walker")

    walkers, controllers = [], []
    for _ in range(n_walkers):
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        walker = world.try_spawn_actor(random.choice(walker_bps), carla.Transform(loc))
        if walker is None:
            continue
        controller = world.spawn_actor(walker_control_bp, carla.Transform(), attach_to=walker)
        walkers.append(walker)
        controllers.append(controller)

    world.tick() if world.get_settings().synchronous_mode else time.sleep(0.5)
    for controller in controllers:
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        controller.set_max_speed(1 + random.random())
    print(f"Spawned {len(walkers)} pedestrians.")
    return walkers, controllers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("-n", "--number-of-vehicles", type=int, default=30)
    parser.add_argument("-w", "--number-of-walkers", type=int, default=15)
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    for actor in world.get_actors().filter("vehicle.*"):
        actor.destroy()
    for actor in world.get_actors().filter("walker.*"):
        actor.destroy()
    for actor in world.get_actors().filter("controller.ai.walker"):
        actor.destroy()

    ego = spawn_ego(world, args.vehicle_filter)
    traffic_vehicles = spawn_traffic(client, world, args.number_of_vehicles, ego.get_transform())
    walkers, walker_controllers = spawn_walkers(client, world, args.number_of_walkers)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        for controller in walker_controllers:
            controller.stop()
        client.apply_batch([carla.command.DestroyActor(a) for a in walker_controllers])
        client.apply_batch([carla.command.DestroyActor(a) for a in walkers])
        client.apply_batch([carla.command.DestroyActor(v) for v in traffic_vehicles])
        ego.destroy()


if __name__ == "__main__":
    main()
