"""A small kinematic simulation of the junction, for running the controllers without CARLA.

Cars drive along the real movement path (approach lane, through the box on their turn, away along the
exit lane) at a constant speed. They stop at the stop point without permission and keep a minimum gap to
the car ahead on their approach. There is no collision physics, acceleration or reaction time, so this
shows how the controllers queue and order traffic; it does not predict CARLA's numbers.
"""
import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

from .intersection_manager import FixedCycleSignal, IntersectionManager, VehicleState
from .junction import Intent, exit_heading, heading, lane_vector, movement_path
from .messaging import Mailbox
from .vehicle_client import V2XClient

STOP_RADIUS = 2.0
MIN_GAP = 7.0  # metres kept to the car ahead on the same approach


@dataclass
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class CarSpec:
    vehicle_id: int
    side: int
    intent: Intent
    start_distance: float  # metres from the centre along the approach


@dataclass(frozen=True)
class Geometry:
    approach: float = 20.0  # stop line a few metres before the box
    box: float = 12.0
    lane_offset: float = 3.5


class FakeCar:
    def __init__(self, spec: CarSpec, geometry: Geometry, speed: float, mailbox: Mailbox):
        self.spec = spec
        self.speed = speed
        hx, hy = heading(spec.side)
        rx, ry = lane_vector(spec.side)
        e = geometry.lane_offset
        start = (-spec.start_distance * hx + e * rx, -spec.start_distance * hy + e * ry)
        through = movement_path(spec.side, spec.intent, geometry.box, e)
        ex, ey = exit_heading(spec.side, spec.intent)
        away = geometry.approach * 1.5
        end = (through[-1][0] + ex * away, through[-1][1] + ey * away)
        self.path: List[Tuple[float, float]] = [start] + through + [end]
        self._lengths = [math.dist(self.path[i], self.path[i + 1]) for i in range(len(self.path) - 1)]
        self._total = sum(self._lengths)
        self._entry_s = self._lengths[0]  # the box entry is the second path point
        self.s = 0.0
        self.stop_point = (-geometry.approach * hx + e * rx, -geometry.approach * hy + e * ry)
        self.client = V2XClient(spec.vehicle_id, spec.side, spec.intent, mailbox, geometry.approach, STOP_RADIUS)

    @property
    def position(self) -> Point:
        remaining = self.s
        for i, length in enumerate(self._lengths):
            if remaining <= length or i == len(self._lengths) - 1:
                f = 0.0 if length == 0 else min(remaining / length, 1.0)
                (x1, y1), (x2, y2) = self.path[i], self.path[i + 1]
                return Point(x1 + (x2 - x1) * f, y1 + (y2 - y1) * f)
            remaining -= length
        return Point(*self.path[-1])

    @property
    def progress(self) -> float:
        """Distance along the path measured from the box entry (negative while still approaching)."""
        return self.s - self._entry_s

    def publish(self, timestamp: float) -> None:
        p = self.position
        self.client.publish(timestamp, p.x, p.y, self.speed, math.hypot(p.x, p.y))

    def move(self, dt: float, others: Sequence["FakeCar"]) -> None:
        self.client.receive()
        p = self.position
        if self.client.must_hold(math.dist((p.x, p.y), self.stop_point)):
            return
        if self.progress < 0:  # still on the shared approach lane: keep a gap to the car ahead
            for other in others:
                if other is not self and other.spec.side == self.spec.side and 0 < other.progress - self.progress < MIN_GAP:
                    return
        self.s = min(self.s + self.speed * dt, self._total)


ControllerFactory = Callable[[Point, Geometry], IntersectionManager]


def make_controller(name: str, geometry: Geometry, **options) -> IntersectionManager:
    """`name` is 'signal', 'fcfs' or 'conflict-aware'."""
    center = Point(0.0, 0.0)
    common = dict(approach_radius=geometry.approach, box_half_extent=geometry.box, lane_offset=geometry.lane_offset)
    if name == "signal":
        return FixedCycleSignal(center, **common, **options)
    return IntersectionManager(center, policy=name, **common, **options)


def build_scenario(vehicles: int = 16, geometry: Geometry = Geometry()) -> List[CarSpec]:
    """Vehicles spread evenly over the four approaches with a fixed mix of turns, staggered so that cars
    reach the junction one after another. Deterministic, so runs are comparable."""
    mix = [Intent.STRAIGHT, Intent.LEFT, Intent.STRAIGHT, Intent.RIGHT]
    specs = []
    for i in range(vehicles):
        side, rank = i % 4, i // 4
        specs.append(CarSpec(i, side, mix[(side + rank) % 4], geometry.approach + 5.0 + side * 3.0 + rank * 9.0))
    return specs


def run_scenario(
    controller: IntersectionManager,
    mailbox: Mailbox,
    specs: Sequence[CarSpec],
    geometry: Geometry = Geometry(),
    speed: float = 7.0,
    dt: float = 0.1,
    duration: float = 300.0,
) -> List[VehicleState]:
    """Run until every car has cleared the junction (or `duration` simulated seconds pass)."""
    cars = [FakeCar(spec, geometry, speed, mailbox) for spec in specs]
    finished: List[VehicleState] = []
    t = 0.0
    while t < duration and len(finished) < len(cars):
        for car in cars:
            car.publish(t)
        controller.step(mailbox, t)
        for car in cars:
            car.move(dt, cars)
        finished.extend(controller.poll_completed())
        t += dt
    return finished
