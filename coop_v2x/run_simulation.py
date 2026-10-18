import argparse
import math
import random
from typing import List, Optional

import carla

from .carla_env import ensure_carla_path

ensure_carla_path()
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402

from .intersection_manager import POLICIES, FixedCycleSignal, IntersectionManager  # noqa: E402
from .junction import SIDE_NAMES, approach_side, classify_intent, to_math_frame  # noqa: E402
from .logging_utils import MetricsLogger  # noqa: E402
from .messaging import Mailbox  # noqa: E402
from .vehicle_agent import VehicleAgent  # noqa: E402
from .vehicle_client import V2XClient  # noqa: E402


# A vehicle that is waiting for permission brakes once it is within STOP_RADIUS of its stop point.
STOP_RADIUS = 2.0


def get_client(host: str, port: int) -> carla.Client:
    client = carla.Client(host, port)
    client.set_timeout(5.0)
    return client


def enable_synchronous_mode(world: carla.World, delta_seconds: float = 0.05):
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = delta_seconds
    settings.substepping = False
    world.apply_settings(settings)


def spawn_vehicles(world: carla.World, count: int, spawn_indices: Optional[List[int]] = None) -> List[carla.Vehicle]:
    blueprint_library = world.get_blueprint_library()
    bp_candidates = [
        bp for bp in blueprint_library.filter("vehicle.*")
        if int(bp.get_attribute("number_of_wheels").as_int()) == 4
    ]

    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < count:
        raise RuntimeError(f"Not enough spawn points for {count} vehicles")
    if spawn_indices is None:
        random.shuffle(spawn_points)
        selected_points = spawn_points[:count]
    else:
        if len(spawn_indices) < count:
            raise ValueError(f"Need {count} spawn indices but got {len(spawn_indices)}")
        selected_points = []
        for idx in spawn_indices[:count]:
            try:
                selected_points.append(spawn_points[idx])
            except IndexError as exc:
                raise ValueError(f"Spawn index {idx} is out of range (total {len(spawn_points)})") from exc

    vehicles = []
    for i in range(count):
        blueprint = random.choice(bp_candidates)
        blueprint.set_attribute("role_name", f"ego_{i}")
        vehicle = world.spawn_actor(blueprint, selected_points[i])
        vehicle.set_autopilot(False)
        vehicles.append(vehicle)
        print(f"Spawned vehicle {vehicle.id} ({vehicle.type_id}) at {selected_points[i].location}")
    return vehicles


def parse_indices(raw: Optional[str]) -> Optional[List[int]]:
    if raw is None:
        return None
    try:
        return [int(x) for x in raw.split(",") if x.strip() != ""]
    except ValueError:
        raise ValueError(f"Could not parse indices from '{raw}'")


def require_indices(raw: Optional[str], label: str, required: int, total_available: int) -> List[int]:
    indices = parse_indices(raw)
    if indices is None:
        raise ValueError(f"--{label}-indices must include at least {required} comma-separated values")
    if len(indices) < required:
        raise ValueError(f"--{label}-indices provided {len(indices)} values but {required} are required")
    validated: List[int] = []
    for idx in indices[:required]:
        if idx < 0 or idx >= total_available:
            raise ValueError(f"--{label}-indices includes invalid index {idx} (valid range 0-{total_available - 1})")
        validated.append(idx)
    return validated


def average_location(points: List[carla.Transform]) -> carla.Location:
    loc = carla.Location()
    for t in points:
        loc.x += t.location.x
        loc.y += t.location.y
        loc.z += t.location.z
    n = max(len(points), 1)
    loc.x /= n
    loc.y /= n
    loc.z /= n
    return loc


def movement_for(start: carla.Location, destination: carla.Location, center: carla.Location, yaw_deg: float):
    """Approach side and turn for a vehicle going from `start` through the junction to `destination`.

    The turn is read from the angle between the direction of approach and the direction of departure,
    which is an approximation: a destination on a road that bends away from the junction can be
    classified differently from the route CARLA's planner takes."""
    sx, sy = to_math_frame(start.x, start.y, center.x, center.y)
    dx, dy = to_math_frame(destination.x, destination.y, center.x, center.y)
    return approach_side(sx, sy, yaw_deg), classify_intent((-sx, -sy), (dx, dy))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default="Town05", help="CARLA map to load (e.g., Town05)")
    parser.add_argument("--vehicles", type=int, default=3, help="number of test vehicles to spawn")
    parser.add_argument("--delta", type=float, default=0.05, help="fixed delta seconds for sync mode")
    parser.add_argument("--approach", type=float, default=35.0, help="distance (m) from intersection center to stop line / join radius")
    parser.add_argument("--box", type=float, default=12.0, help="half-extent (m) of intersection box")
    parser.add_argument("--center-x", type=float, default=None, help="intersection center X (optional)")
    parser.add_argument("--center-y", type=float, default=None, help="intersection center Y (optional)")
    parser.add_argument("--center-z", type=float, default=None, help="intersection center Z (optional)")
    parser.add_argument("--spawn-indices", type=str, default=None, help="comma-separated spawn point indices")
    parser.add_argument("--dest-indices", type=str, default=None, help="comma-separated destination indices")
    parser.add_argument("--logfile", type=str, default="metrics.csv", help="CSV log output")
    parser.add_argument(
        "--controller", choices=("cooperative", "signal"), default="cooperative",
        help="cooperative intersection manager, or the fixed-cycle traffic signal baseline",
    )
    parser.add_argument(
        "--policy", choices=POLICIES, default="conflict-aware",
        help="cooperative controller: grant by movement conflicts, or plain first-come-first-served",
    )
    parser.add_argument(
        "--max-active", type=int, default=None,
        help="max vehicles holding a permission at once (default: unlimited for conflict-aware, 1 for fcfs)",
    )
    parser.add_argument("--green", type=float, default=10.0, help="signal: green seconds per phase")
    parser.add_argument("--all-red", type=float, default=2.0, help="signal: all-red seconds between phases")
    parser.add_argument(
        "--signal-phases", choices=("split", "axis"), default="split",
        help="signal: one approach at a time (conflict-free), or two axis phases",
    )
    parser.add_argument(
        "--junction-yaw", type=float, default=0.0,
        help="rotate the four approach sides by this many degrees for a junction not aligned with the x/y axes",
    )
    parser.add_argument("--duration", type=float, default=120.0, help="stop after this many simulated seconds")
    parser.add_argument(
        "--permission-timeout", type=float, default=None,
        help="revoke a permission the vehicle has not used after this many simulated seconds (default: never)",
    )
    parser.add_argument("--list-spawns", action="store_true", help="list spawn points and exit")
    parser.add_argument("--list-junctions", action="store_true", help="list junction centers/extents and exit")
    args = parser.parse_args()

    client = get_client(args.host, args.port)
    world = client.get_world()
    if args.map not in world.get_map().name:
        world = client.load_world(args.map)
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()

    if args.list_spawns:
        for idx, sp in enumerate(spawn_points):
            loc = sp.location
            print(f"{idx}: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
        return
    if args.list_junctions:
        junctions = world.get_map().get_junctions()
        for j_idx, j in enumerate(junctions):
            c = j.bounding_box.location
            e = j.bounding_box.extent
            print(
                f"{j_idx}: center=({c.x:.2f}, {c.y:.2f}, {c.z:.2f}) "
                f"extent=({e.x:.2f}, {e.y:.2f}, {e.z:.2f})"
            )
        return

    original_settings = world.get_settings()
    vehicles: List[carla.Vehicle] = []
    agents: List[VehicleAgent] = []
    manager: IntersectionManager | None = None
    logger: Optional[MetricsLogger] = None
    route_planner = GlobalRoutePlanner(carla_map, 1.5)
    try:
        enable_synchronous_mode(world, args.delta)

        spawn_indices = require_indices(args.spawn_indices, "spawn", args.vehicles, len(spawn_points))
        dest_indices = require_indices(args.dest_indices, "dest", args.vehicles, len(spawn_points))

        # Choose intersection center before spawning to share with agents.
        if args.center_x is not None and args.center_y is not None:
            center_location = carla.Location(
                x=args.center_x,
                y=args.center_y,
                z=args.center_z if args.center_z is not None else spawn_points[0].location.z,
            )
        elif spawn_indices:
            subset = [spawn_points[i] for i in spawn_indices]
            center_location = average_location(subset)
        else:
            center_location = spawn_points[0].location

        mailbox = Mailbox()
        movements = []
        for idx, dest_idx in zip(spawn_indices, dest_indices):
            movements.append(
                movement_for(spawn_points[idx].location, spawn_points[dest_idx].location, center_location, args.junction_yaw)
            )
            side, intent = movements[-1]
            print(f"Vehicle mapping: spawn idx {idx} -> dest idx {dest_idx} (side {SIDE_NAMES[side]}, {intent.value})")
        vehicles = spawn_vehicles(world, args.vehicles, spawn_indices=spawn_indices)

        agents = []
        def compute_stop_location(start_location: carla.Location) -> carla.Location:
            direction = carla.Location(
                x=start_location.x - center_location.x,
                y=start_location.y - center_location.y,
                z=0.0,
            )
            distance = math.sqrt(direction.x ** 2 + direction.y ** 2)
            if distance < 1e-3:
                return carla.Location(center_location.x, center_location.y, center_location.z)
            scale = min(args.approach, distance)
            direction.x = direction.x / distance * scale
            direction.y = direction.y / distance * scale
            return carla.Location(
                x=center_location.x + direction.x,
                y=center_location.y + direction.y,
                z=center_location.z,
            )

        for i, v in enumerate(vehicles):
            dest_idx = dest_indices[i]
            start_idx = spawn_indices[i]
            destination = spawn_points[dest_idx].location
            start_location = spawn_points[start_idx].location
            stop_location = compute_stop_location(start_location)
            side, intent = movements[i]
            agents.append(
                VehicleAgent(
                    world=world,
                    vehicle=v,
                    target_speed_kmh=25,
                    destination=destination,
                    start_location=start_location,
                    stop_location=stop_location,
                    center=center_location,
                    client=V2XClient(v.id, side, intent, mailbox, args.approach, STOP_RADIUS),
                    route_planner=route_planner,
                )
            )

        common = dict(
            approach_radius=args.approach, box_half_extent=args.box, permission_timeout=args.permission_timeout
        )
        if args.controller == "signal":
            manager = FixedCycleSignal(
                center_location, green=args.green, all_red=args.all_red, phases=args.signal_phases, **common
            )
        else:
            manager = IntersectionManager(center_location, policy=args.policy, max_active=args.max_active, **common)
        logger = MetricsLogger(args.logfile)

        start_sim_time: Optional[float] = None
        while True:
            world.tick()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds
            if start_sim_time is None:
                start_sim_time = timestamp

            for agent in agents:
                agent.publish(timestamp)
            manager.step(mailbox, timestamp)
            for agent in agents:
                agent.step()

            # Log any vehicles that have cleared the box.
            if logger is not None:
                completed = manager.poll_completed()
                for state in completed:
                    logger.log_state(state)

            # Stop after --duration simulated seconds to avoid runaway runs.
            if (timestamp - start_sim_time) > args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        for agent in agents:
            agent.destroy()
        if logger is not None:
            logger.close()
        for vehicle in vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        world.apply_settings(original_settings)


if __name__ == "__main__":
    main()
