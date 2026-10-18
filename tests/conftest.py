import sys
import types
from types import SimpleNamespace

import pytest


@pytest.fixture
def carla_stubs(monkeypatch):
    """Minimal stand-ins for `carla` and CARLA's `agents` package, so the CARLA-facing modules import."""
    carla = types.ModuleType("carla")
    carla.Location = SimpleNamespace
    carla.World = carla.Vehicle = carla.Client = object
    carla.VehicleControl = lambda **kwargs: SimpleNamespace(**kwargs)

    class BasicAgent:
        def __init__(self, vehicle, target_speed=None, opt_dict=None, map_inst=None, grp_inst=None):
            self.destination = None

        def set_destination(self, destination, start_location=None, clean_queue=True):
            self.destination = destination

        def run_step(self):
            return SimpleNamespace(throttle=0.5, brake=0.0)

    class GlobalRoutePlanner:
        pass

    agents = types.ModuleType("agents")
    navigation = types.ModuleType("agents.navigation")
    basic = types.ModuleType("agents.navigation.basic_agent")
    basic.BasicAgent = BasicAgent
    planner = types.ModuleType("agents.navigation.global_route_planner")
    planner.GlobalRoutePlanner = GlobalRoutePlanner
    for name, module in {
        "carla": carla,
        "agents": agents,
        "agents.navigation": navigation,
        "agents.navigation.basic_agent": basic,
        "agents.navigation.global_route_planner": planner,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    for cached in ("coop_v2x.vehicle_agent", "coop_v2x.run_simulation"):
        monkeypatch.delitem(sys.modules, cached, raising=False)
    return carla
