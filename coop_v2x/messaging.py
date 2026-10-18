"""The V2X message layer.

Vehicles publish REQUEST (I have arrived), INTENT (which side I came from and where I am going) and STATE
(where I am, how fast). The intersection manager answers with PERMIT (you may cross), CROSSING (you are in
the box) and CLEARED (you are clear). `Mailbox` is an in-memory stand-in for the radio link: delivery is
instant and lossless, which is exactly the idealisation real V2X hardware would not give you.
"""
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List


class MessageType(str, Enum):
    REQUEST = "REQUEST"
    INTENT = "INTENT"
    STATE = "STATE"
    PERMIT = "PERMIT"
    CROSSING = "CROSSING"
    CLEARED = "CLEARED"


@dataclass(frozen=True)
class Message:
    type: MessageType
    vehicle_id: int
    timestamp: float
    payload: Dict[str, Any] = field(default_factory=dict)


class Mailbox:
    """One inbox for the manager and one per vehicle. Reading an inbox empties it."""

    def __init__(self):
        self._manager: List[Message] = []
        self._vehicles: Dict[int, List[Message]] = defaultdict(list)

    def send_to_manager(self, message: Message) -> None:
        self._manager.append(message)

    def send_to_vehicle(self, vehicle_id: int, message: Message) -> None:
        self._vehicles[vehicle_id].append(message)

    def drain_manager(self) -> List[Message]:
        messages, self._manager = self._manager, []
        return messages

    def drain_vehicle(self, vehicle_id: int) -> List[Message]:
        return self._vehicles.pop(vehicle_id, [])
