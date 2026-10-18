"""Run the real IntersectionManager against simple simulated cars. No CARLA needed.

    python -m coop_v2x.demo

Four cars approach a crossroads from the four compass directions at a steady speed. Each stops at its
stop point unless the manager has granted it permission, exactly as VehicleAgent does in CARLA. This is a
logic demonstration (queueing order, one car in the box at a time, metrics), not a physics simulation.
"""
import argparse
import math
import tempfile
from dataclasses import dataclass
from typing import List

from .analyze_metrics import format_summary, load_rows, summarise
from .intersection_manager import IntersectionManager
from .logging_utils import MetricsLogger


@dataclass
class Point:
    x: float
    y: float

    def distance(self, other: "Point") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


class FakeVehicle:
    def __init__(self, vehicle_id: int, position: Point):
        self.id = vehicle_id
        self.position = position

    def get_location(self) -> Point:
        return self.position


class FakeAgent:
    """Drives straight through the centre at constant speed; holds at its stop point without permission."""

    def __init__(self, vehicle_id: int, start: Point, center: Point, speed: float, approach: float, stop_radius: float = 2.0):
        self.vehicle = FakeVehicle(vehicle_id, start)
        self.speed = speed
        self.stop_radius = stop_radius
        length = start.distance(center)
        self.heading = Point((center.x - start.x) / length, (center.y - start.y) / length)
        # Same placement as run_simulation: STOP_RADIUS inside the approach radius.
        stop_distance = approach - stop_radius
        self.stop_location = Point(center.x - self.heading.x * stop_distance, center.y - self.heading.y * stop_distance)

    def distance_to(self, location: Point) -> float:
        return self.vehicle.position.distance(location)

    def step(self, permission_to_go: bool, dt: float):
        if not permission_to_go and self.distance_to(self.stop_location) <= self.stop_radius:
            return
        pos = self.vehicle.position
        self.vehicle.position = Point(pos.x + self.heading.x * self.speed * dt, pos.y + self.heading.y * self.speed * dt)


def run_demo(
    logfile: str,
    start_distances=(40.0, 45.0, 50.0, 55.0),
    approach: float = 35.0,
    box: float = 12.0,
    speed: float = 7.0,
    max_active: int = 1,
    dt: float = 0.1,
    duration: float = 120.0,
) -> List:
    """Simulate until every car has cleared the junction. Returns the finished VehicleStates in order."""
    center = Point(0.0, 0.0)
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    agents = [
        FakeAgent(i, Point(dx * d, dy * d), center, speed, approach)
        for i, (d, (dx, dy)) in enumerate(zip(start_distances, directions))
    ]
    manager = IntersectionManager(center, approach_radius=approach, box_half_extent=box, max_active=max_active)
    logger = MetricsLogger(logfile)
    finished: List = []
    try:
        t = 0.0
        while t < duration and len(finished) < len(agents):
            manager.update(agents, t)
            permissions = manager.current_permissions()
            for agent in agents:
                agent.step(permissions.get(agent.vehicle.id, False), dt)
            for state in manager.poll_completed():
                logger.log_state(state)
                finished.append(state)
            t += dt
    finally:
        logger.close()
    return finished


def main():
    parser = argparse.ArgumentParser(description="Offline demo of the intersection manager.")
    parser.add_argument("--max-active", type=int, default=1, help="cars allowed through at once")
    parser.add_argument("--logfile", default=None, help="where to write the metrics CSV (default: a temp file)")
    args = parser.parse_args()

    logfile = args.logfile or tempfile.NamedTemporaryFile(suffix=".csv", delete=False).name
    finished = run_demo(logfile, max_active=args.max_active)
    print("car  arrived  allowed  entered  exited")
    for s in sorted(finished, key=lambda s: s.permission_time):
        print(f"{s.vehicle_id:>3}  {s.arrival_time:7.1f}  {s.permission_time:7.1f}  {s.enter_time:7.1f}  {s.exit_time:6.1f}")
    print()
    print(format_summary(summarise(load_rows(logfile))))
    print(f"\nMetrics written to {logfile}")


if __name__ == "__main__":
    main()
