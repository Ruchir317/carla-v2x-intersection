"""The CARLA-facing glue, exercised against stand-ins for CARLA (no simulator needed)."""
import math
from types import SimpleNamespace

from coop_v2x.junction import Intent
from coop_v2x.messaging import Mailbox, Message, MessageType
from coop_v2x.vehicle_client import V2XClient


class Loc:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))


class FakeVehicle:
    id = 5

    def __init__(self, x, y):
        self.location = Loc(x, y)
        self.controls = []
        self.alive = True

    def get_location(self):
        return self.location

    def get_velocity(self):
        return SimpleNamespace(x=7.0, y=0.0, z=0.0)

    def apply_control(self, control):
        self.controls.append(control)

    @property
    def is_alive(self):
        return self.alive

    def destroy(self):
        self.alive = False


def make_agent(carla_stubs, x, y):
    from coop_v2x.vehicle_agent import VehicleAgent

    mailbox = Mailbox()
    vehicle = FakeVehicle(x, y)
    world = SimpleNamespace(get_map=lambda: None)
    client = V2XClient(5, 3, Intent.STRAIGHT, mailbox, approach_radius=35.0, stop_radius=2.0)
    agent = VehicleAgent(
        world=world,
        vehicle=vehicle,
        target_speed_kmh=25,
        destination=Loc(0, 100),
        start_location=Loc(x, y),
        stop_location=Loc(0, -35.0),
        center=Loc(0, 0),
        client=client,
    )
    return agent, vehicle, mailbox


def test_publish_announces_the_vehicle_and_reports_its_state(carla_stubs):
    agent, _, mailbox = make_agent(carla_stubs, 0.0, -30.0)
    agent.publish(1.0)
    messages = mailbox.drain_manager()
    assert [m.type for m in messages] == [MessageType.REQUEST, MessageType.INTENT, MessageType.STATE]
    assert messages[-1].payload == {"x": 0.0, "y": -30.0, "speed": 7.0}


def test_a_waiting_vehicle_brakes_at_the_stop_point(carla_stubs):
    agent, vehicle, _ = make_agent(carla_stubs, 0.0, -34.0)  # 1 m from the stop point, inside the radius
    agent.publish(1.0)
    agent.step()
    assert vehicle.controls[-1].brake == 1.0 and vehicle.controls[-1].throttle == 0.0


def test_a_permitted_vehicle_follows_its_route(carla_stubs):
    agent, vehicle, mailbox = make_agent(carla_stubs, 0.0, -34.0)
    agent.publish(1.0)
    mailbox.send_to_vehicle(5, Message(MessageType.PERMIT, 5, 1.0))
    agent.step()
    assert vehicle.controls[-1].brake == 0.0 and vehicle.controls[-1].throttle == 0.5


def test_a_vehicle_that_has_not_announced_itself_keeps_driving(carla_stubs):
    agent, vehicle, _ = make_agent(carla_stubs, 0.0, -34.0)
    agent.step()  # never published, so it is not yet known to the manager
    assert vehicle.controls[-1].brake == 0.0


def test_destroy_removes_a_live_vehicle_once(carla_stubs):
    agent, vehicle, _ = make_agent(carla_stubs, 0.0, -34.0)
    agent.destroy()
    agent.destroy()
    assert not agent.is_alive


def test_movement_for_reads_side_and_turn_from_carla_coordinates(carla_stubs):
    from coop_v2x.run_simulation import movement_for

    center = SimpleNamespace(x=100.0, y=50.0)
    start = SimpleNamespace(x=100.0, y=80.0)  # CARLA +y is "down" in the math frame, so this is side 3

    def turn(dest):
        side, intent = movement_for(start, SimpleNamespace(x=dest[0], y=dest[1]), center, 0.0)
        return side, intent

    assert turn((100.0, 20.0)) == (3, Intent.STRAIGHT)
    assert turn((130.0, 50.0)) == (3, Intent.RIGHT)
    assert turn((70.0, 50.0)) == (3, Intent.LEFT)
