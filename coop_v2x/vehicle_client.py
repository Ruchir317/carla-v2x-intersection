from .junction import Intent
from .messaging import Mailbox, Message, MessageType


class V2XClient:
    """A vehicle's side of the protocol, independent of CARLA so it can be tested without it.

    It announces the vehicle once it is inside the approach radius, reports its state every tick, and
    remembers whether it has been given permission to cross."""

    def __init__(
        self,
        vehicle_id: int,
        side: int,
        intent: Intent,
        mailbox: Mailbox,
        approach_radius: float,
        stop_radius: float = 2.0,
    ):
        self.vehicle_id = vehicle_id
        self.side = side
        self.intent = intent
        self.mailbox = mailbox
        self.approach_radius = approach_radius
        self.stop_radius = stop_radius
        self.permitted = False
        self.cleared = False
        self._requested = False

    def publish(self, timestamp: float, x: float, y: float, speed: float, distance_to_center: float) -> None:
        if not self._requested and distance_to_center <= self.approach_radius:
            self._requested = True
            self._send(MessageType.REQUEST, timestamp)
            self._send(MessageType.INTENT, timestamp, side=self.side, intent=self.intent.value)
        self._send(MessageType.STATE, timestamp, x=x, y=y, speed=speed)

    def receive(self) -> None:
        for message in self.mailbox.drain_vehicle(self.vehicle_id):
            if message.type == MessageType.PERMIT:
                self.permitted = True
            elif message.type == MessageType.CLEARED:
                self.cleared = True

    def must_hold(self, distance_to_stop: float) -> bool:
        """Brake at the stop point while waiting for permission.

        Only a vehicle that has already announced itself holds, so it can never stall just outside the
        approach radius where the manager has not seen it. Once cleared a vehicle never holds again."""
        return self._requested and not (self.permitted or self.cleared) and distance_to_stop <= self.stop_radius

    def _send(self, type_: MessageType, timestamp: float, **payload) -> None:
        self.mailbox.send_to_manager(Message(type_, self.vehicle_id, timestamp, payload))
