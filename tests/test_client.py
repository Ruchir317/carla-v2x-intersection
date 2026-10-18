from coop_v2x.junction import Intent
from coop_v2x.messaging import Mailbox, Message, MessageType
from coop_v2x.vehicle_client import V2XClient


def make():
    mailbox = Mailbox()
    return V2XClient(7, side=3, intent=Intent.LEFT, mailbox=mailbox, approach_radius=35.0, stop_radius=2.0), mailbox


def sent(mailbox):
    return [(m.type, m.payload) for m in mailbox.drain_manager()]


def test_far_away_vehicle_only_reports_state():
    client, mailbox = make()
    client.publish(0.0, 0.0, -80.0, 7.0, 80.0)
    assert [t for t, _ in sent(mailbox)] == [MessageType.STATE]


def test_vehicle_announces_itself_once_inside_the_approach_radius():
    client, mailbox = make()
    client.publish(1.0, 0.0, -34.0, 7.0, 34.0)
    client.publish(1.1, 0.0, -33.0, 7.0, 33.0)
    types = [t for t, _ in sent(mailbox)]
    assert types == [MessageType.REQUEST, MessageType.INTENT, MessageType.STATE, MessageType.STATE]


def test_intent_message_carries_side_and_turn():
    client, mailbox = make()
    client.publish(0.0, 0.0, -30.0, 7.0, 30.0)
    payloads = dict(sent(mailbox))
    assert payloads[MessageType.INTENT] == {"side": 3, "intent": "left"}
    assert payloads[MessageType.STATE] == {"x": 0.0, "y": -30.0, "speed": 7.0}


def test_does_not_hold_before_it_has_announced_itself():
    client, _ = make()
    assert not client.must_hold(distance_to_stop=0.5)  # not yet requested, so it must keep driving


def test_holds_at_the_stop_point_until_permitted():
    client, mailbox = make()
    client.publish(0.0, 0.0, -34.0, 7.0, 34.0)
    assert client.must_hold(1.0)
    assert not client.must_hold(5.0)  # not at the stop point yet
    mailbox.send_to_vehicle(7, Message(MessageType.PERMIT, 7, 1.0))
    client.receive()
    assert client.permitted and not client.must_hold(1.0)


def test_never_holds_again_once_cleared():
    client, mailbox = make()
    client.publish(0.0, 0.0, -34.0, 7.0, 34.0)
    mailbox.send_to_vehicle(7, Message(MessageType.CLEARED, 7, 5.0))
    client.receive()
    assert client.cleared and not client.must_hold(0.0)
