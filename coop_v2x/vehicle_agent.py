from __future__ import annotations

from typing import Optional

import carla

from .carla_env import ensure_carla_path

ensure_carla_path()
from agents.navigation.basic_agent import BasicAgent  # noqa: E402
from agents.navigation.global_route_planner import GlobalRoutePlanner


class VehicleAgent:
    """Wraps CARLA's BasicAgent with intersection stop/go overrides."""

    def __init__(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
        target_speed_kmh: float,
        destination: carla.Location,
        start_location: carla.Location,
        stop_location: carla.Location,
        stop_radius: float,
        route_planner: Optional[GlobalRoutePlanner] = None,
    ):
        self.world = world
        self.vehicle = vehicle
        self.stop_location = stop_location
        self.stop_radius = stop_radius
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

    def step(self, permission_to_go: bool):
        distance_to_stop = self.distance_to(self.stop_location)
        if not permission_to_go and distance_to_stop <= self.stop_radius:
            control = carla.VehicleControl(throttle=0.0, brake=1.0)
        else:
            control = self.agent.run_step()
        self.vehicle.apply_control(control)

    def distance_to(self, location: carla.Location) -> float:
        return self.vehicle.get_location().distance(location)

    @property
    def is_alive(self) -> bool:
        return self.vehicle.is_alive

    def destroy(self):
        if self.vehicle.is_alive:
            self.vehicle.destroy()
