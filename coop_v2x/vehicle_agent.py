from __future__ import annotations

import math
from typing import Optional

import carla

from .carla_env import ensure_carla_path
from .vehicle_client import V2XClient

ensure_carla_path()
from agents.navigation.basic_agent import BasicAgent  # noqa: E402
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402


def planar_distance(a: carla.Location, b: carla.Location) -> float:
    """Distance on the ground plane. The manager measures in x and y only, so the agent must too;
    a junction on a slope would otherwise look further away to the vehicle than to the manager."""
    return math.hypot(a.x - b.x, a.y - b.y)


class VehicleAgent:
    """CARLA's BasicAgent plus the V2X protocol.

    Each tick the agent reports its state to the intersection manager (`publish`), then reads the manager's
    replies and either holds at the stop point or drives on (`step`)."""

    def __init__(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
        target_speed_kmh: float,
        destination: carla.Location,
        start_location: carla.Location,
        stop_location: carla.Location,
        center: carla.Location,
        client: V2XClient,
        route_planner: Optional[GlobalRoutePlanner] = None,
    ):
        self.world = world
        self.vehicle = vehicle
        self.stop_location = stop_location
        self.center = center
        self.client = client
        opt_dict = {
            "target_speed": target_speed_kmh,
            "ignore_traffic_lights": True,
            "ignore_stop_signs": True,
        }
        self.agent = BasicAgent(
            vehicle,
            target_speed=target_speed_kmh,
            opt_dict=opt_dict,
            map_inst=world.get_map(),
            grp_inst=route_planner,
        )
        self.agent.set_destination(destination, start_location=start_location, clean_queue=True)

    def publish(self, timestamp: float) -> None:
        """Send REQUEST/INTENT (once, on arrival) and STATE (every tick) to the manager."""
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        self.client.publish(timestamp, location.x, location.y, speed, planar_distance(location, self.center))

    def step(self) -> None:
        """Apply the manager's PERMIT/CLEARED replies: brake at the stop point, or follow the route."""
        self.client.receive()
        if self.client.must_hold(self.distance_to(self.stop_location)):
            control = carla.VehicleControl(throttle=0.0, brake=1.0)
        else:
            control = self.agent.run_step()
        self.vehicle.apply_control(control)

    def distance_to(self, location: carla.Location) -> float:
        return planar_distance(self.vehicle.get_location(), location)

    @property
    def is_alive(self) -> bool:
        return self.vehicle.is_alive

    def destroy(self):
        if self.vehicle.is_alive:
            self.vehicle.destroy()
