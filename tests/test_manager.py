import pytest

from coop_v2x.intersection_manager import IntersectionManager
from coop_v2x.messaging import Mailbox, MessageType

from .helpers import APPROACH, BOX, CENTER, E, L, R, S, SIDE_FAR, arrive, inbox_types, report, send


def make_manager(**kwargs):
    kwargs.setdefault("stale_timeout", 60.0)  # these tests do not keep every vehicle reporting
    return IntersectionManager(CENTER, approach_radius=APPROACH, box_half_extent=BOX, **kwargs), Mailbox()


def cross(manager, mailbox, vid, side, t0):
    """Drive a vehicle that holds a permission through the box and out the far side (STATE messages)."""
    report(mailbox, vid, t0, (E, 0.0))  # inside the box
    manager.step(mailbox, t0)
    report(mailbox, vid, t0 + 1.0, SIDE_FAR[side])  # far side, beyond 75% of the approach radius
    manager.step(mailbox, t0 + 1.0)


def test_non_conflicting_vehicles_are_permitted_together():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.1, side=1, intent=S)  # opposing straights
    manager.step(mailbox, 0.2)
    assert manager.current_permissions() == {1: True, 2: True}


def test_a_conflicting_vehicle_waits_until_the_other_has_cleared():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.1, side=2, intent=S)  # crosses vehicle 1
    manager.step(mailbox, 0.2)
    assert manager.current_permissions() == {1: True}
    cross(manager, mailbox, 1, side=3, t0=5.0)
    assert manager.current_permissions() == {2: True}


def test_a_later_vehicle_may_not_overtake_an_earlier_one_it_conflicts_with():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)  # active
    arrive(mailbox, 2, 0.1, side=2, intent=S)  # conflicts with 1, waits
    arrive(mailbox, 3, 0.2, side=1, intent=S)  # fine with 1, but crosses 2's path, so it queues behind 2
    manager.step(mailbox, 0.3)
    assert manager.current_permissions() == {1: True}


def test_a_later_vehicle_that_conflicts_with_nobody_ahead_is_not_held_up():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.1, side=2, intent=S)  # waits for 1
    arrive(mailbox, 3, 0.2, side=1, intent=R)  # a right turn from above meets neither vehicle 1 nor vehicle 2
    manager.step(mailbox, 0.3)
    assert manager.current_permissions() == {1: True, 3: True}


def test_vehicles_on_the_same_side_queue_in_arrival_order():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.5, side=2, intent=S)  # waits for 1
    arrive(mailbox, 3, 0.6, side=2, intent=L)  # same side as 2: may not overtake it
    manager.step(mailbox, 0.7)
    assert manager.current_permissions() == {1: True}


def test_fcfs_policy_lets_one_vehicle_through_at_a_time():
    manager, mailbox = make_manager(policy="fcfs")
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.1, side=1, intent=S)  # would be fine together, but fcfs is serial
    manager.step(mailbox, 0.2)
    assert manager.current_permissions() == {1: True}


def test_fcfs_max_active_allows_more_than_one():
    manager, mailbox = make_manager(policy="fcfs", max_active=2)
    for vid in (1, 2, 3):
        arrive(mailbox, vid, vid * 0.1, side=vid % 4, intent=S)
    manager.step(mailbox, 1.0)
    assert manager.current_permissions() == {1: True, 2: True}


def test_unknown_policy_is_rejected():
    with pytest.raises(ValueError):
        IntersectionManager(CENTER, policy="yolo")


def test_an_approach_radius_that_does_not_clear_the_box_is_rejected():
    with pytest.raises(ValueError, match="approach radius"):
        IntersectionManager(CENTER, approach_radius=8.0, box_half_extent=8.0)
    IntersectionManager(CENTER, approach_radius=11.0, box_half_extent=8.0)  # 3 m clear of the box is enough


def test_a_vehicle_with_no_declared_intent_is_treated_as_conflicting_with_everyone():
    manager, mailbox = make_manager()
    send(mailbox, MessageType.REQUEST, 1, 0.0)  # no INTENT ever arrives
    send(mailbox, MessageType.STATE, 1, 0.0, x=0.0, y=-30.0, speed=7.0)
    arrive(mailbox, 2, 0.1, side=1, intent=S)
    manager.step(mailbox, 0.2)
    assert manager.current_permissions() == {1: True}  # first come, first served; 2 must wait for it


def test_crossing_and_cleared_are_sent_once_each_in_order():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    manager.step(mailbox, 0.1)
    cross(manager, mailbox, 1, side=3, t0=5.0)
    manager.step(mailbox, 7.0)
    assert inbox_types(mailbox, 1) == [MessageType.PERMIT, MessageType.CROSSING, MessageType.CLEARED]


def test_leaving_the_box_is_not_enough_the_vehicle_must_also_drive_clear():
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    manager.step(mailbox, 0.1)
    report(mailbox, 1, 1.0, (E, 0.0))
    manager.step(mailbox, 1.0)
    report(mailbox, 1, 2.0, (E, 15.0))  # out of the box but only 15 m from the centre (< 0.75 * 35)
    manager.step(mailbox, 2.0)
    assert manager.poll_completed() == []
    report(mailbox, 1, 3.0, (E, 30.0))
    manager.step(mailbox, 3.0)
    assert [s.vehicle_id for s in manager.poll_completed()] == [1]
    assert manager.poll_completed() == []  # each vehicle is reported once


def test_each_vehicle_is_judged_on_its_own_position():
    """Regression: an earlier version reused another vehicle's distance when deciding it had cleared."""
    manager, mailbox = make_manager()
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.0, side=2, intent=S, pos=(-20.0, -E))  # waiting 20 m out
    manager.step(mailbox, 0.1)
    report(mailbox, 1, 1.0, (E, 0.0))
    report(mailbox, 2, 1.0, (-20.0, -E))
    manager.step(mailbox, 1.0)
    report(mailbox, 1, 2.0, (E, 30.0))
    report(mailbox, 2, 2.0, (-20.0, -E))  # vehicle 2 is last in the batch and only 20 m out
    manager.step(mailbox, 2.0)
    assert [s.vehicle_id for s in manager.poll_completed()] == [1]
    assert manager.current_permissions() == {2: True}


def test_an_unused_permission_blocks_the_queue_unless_a_timeout_is_set():
    def run(timeout):
        manager, mailbox = make_manager(permission_timeout=timeout)
        arrive(mailbox, 1, 0.0, side=3, intent=S, pos=(E, -20.0))  # holds a permission, never moves
        arrive(mailbox, 2, 0.0, side=2, intent=S, pos=(-20.0, -E))
        for t in (0.0, 3.0, 6.0, 9.0):
            report(mailbox, 1, t, (E, -20.0))
            report(mailbox, 2, t, (-20.0, -E))
            manager.step(mailbox, t)
        return manager.current_permissions()

    assert run(None) == {1: True}
    assert run(5.0) == {2: True}


def test_a_vehicle_that_goes_silent_is_forgotten_and_frees_its_permission():
    manager, mailbox = make_manager(stale_timeout=2.0)
    arrive(mailbox, 1, 0.0, side=3, intent=S)
    arrive(mailbox, 2, 0.0, side=2, intent=S)
    manager.step(mailbox, 0.1)
    assert manager.current_permissions() == {1: True}
    for t in (1.0, 2.0, 3.0):  # only vehicle 2 keeps reporting
        report(mailbox, 2, t, (-30.0, -E))
        manager.step(mailbox, t)
    assert manager.current_permissions() == {2: True}


def test_messages_from_unannounced_vehicles_are_ignored():
    manager, mailbox = make_manager()
    report(mailbox, 9, 0.0, (0.0, 0.0))
    manager.step(mailbox, 0.0)
    assert manager.current_permissions() == {} and manager.poll_completed() == []
