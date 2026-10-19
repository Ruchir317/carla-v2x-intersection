import sys
import types
from types import SimpleNamespace

import pytest


@pytest.fixture
def carla_stubs(monkeypatch):
    """Minimal stand-ins for `carla` and CARLA's `agents` package, so the CARLA-facing modules import."""
    class FakeCarla(types.ModuleType):
        """Any CARLA class the code merely names in a type hint (Transform, World, ...) resolves to a
        placeholder; Python before 3.14 evaluates those hints when the function is defined."""

        def __getattr__(self, name):
            if name.startswith("__"):
                raise AttributeError(name)
            return type(name, (), {})

    carla = FakeCarla("carla")
    carla.Location = SimpleNamespace
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
