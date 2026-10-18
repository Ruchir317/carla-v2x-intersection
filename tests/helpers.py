from types import SimpleNamespace

from coop_v2x.junction import Intent
from coop_v2x.messaging import Mailbox, Message, MessageType

CENTER = SimpleNamespace(x=0.0, y=0.0)
APPROACH = 35.0
BOX = 12.0
E = 3.5  # lane offset

# Movement shorthand: a vehicle's side is 0 (right), 1 (up), 2 (left) or 3 (down); its turn is S, L or R.
S, L, R = Intent.STRAIGHT, Intent.LEFT, Intent.RIGHT

# A point on each approach lane 30 m out, and the matching point 30 m out the far side after going straight.
SIDE_START = {0: (30.0, E), 1: (-E, 30.0), 2: (-30.0, -E), 3: (E, -30.0)}
SIDE_FAR = {0: (-30.0, E), 1: (-E, -30.0), 2: (30.0, -E), 3: (E, 30.0)}


def send(mailbox: Mailbox, type_: MessageType, vid: int, t: float, **payload) -> None:
    mailbox.send_to_manager(Message(type_, vid, t, payload))


def arrive(mailbox, vid, t, side, intent, pos=None):
    """The vehicle announces itself (REQUEST + INTENT) and reports its position."""
    x, y = pos or SIDE_START[side]
    send(mailbox, MessageType.REQUEST, vid, t)
    send(mailbox, MessageType.INTENT, vid, t, side=side, intent=intent.value)
    send(mailbox, MessageType.STATE, vid, t, x=x, y=y, speed=7.0)


def report(mailbox, vid, t, pos):
    send(mailbox, MessageType.STATE, vid, t, x=pos[0], y=pos[1], speed=7.0)


def inbox_types(mailbox, vid):
    return [m.type for m in mailbox.drain_vehicle(vid)]
