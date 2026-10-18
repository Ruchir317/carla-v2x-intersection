import pytest

from coop_v2x.intersection_manager import FixedCycleSignal
from coop_v2x.messaging import Mailbox

from .helpers import APPROACH, BOX, CENTER, S, SIDE_START, arrive, report


def make_signal(**kwargs):
    return FixedCycleSignal(CENTER, approach_radius=APPROACH, box_half_extent=BOX, green=10.0, all_red=2.0, **kwargs)


def test_split_phases_give_each_approach_its_own_green():
    signal = make_signal()
    assert [signal.green_sides(t) for t in (0, 5, 12, 24, 36, 48)] == [(0,), (0,), (1,), (2,), (3,), (0,)]


def test_everything_is_red_between_phases():
    signal = make_signal()
    assert signal.green_sides(10.0) is None and signal.green_sides(11.9) is None
    assert signal.green_sides(22.5) is None


def test_axis_phases_alternate_between_two_axes():
    signal = make_signal(phases="axis")
    assert [signal.green_sides(t) for t in (0, 12, 24)] == [(0, 2), (1, 3), (0, 2)]


def test_a_vehicle_waits_for_its_green():
    signal, mailbox = make_signal(), Mailbox()
    arrive(mailbox, 1, 0.0, side=1, intent=S)  # side 1 gets green at t = 12
    signal.step(mailbox, 0.1)
    assert signal.current_permissions() == {}
    for t in (3.0, 6.0, 9.0, 11.0):  # side 0's green, then all red; the vehicle keeps reporting
        report(mailbox, 1, t, SIDE_START[1])
        signal.step(mailbox, t)
    assert signal.current_permissions() == {}
    report(mailbox, 1, 12.1, SIDE_START[1])
    signal.step(mailbox, 12.1)
    assert signal.current_permissions() == {1: True}


def test_a_vehicle_that_has_its_permission_keeps_it_when_the_light_changes():
    signal, mailbox = make_signal(), Mailbox()
    arrive(mailbox, 1, 0.0, side=0, intent=S)
    signal.step(mailbox, 0.1)
    report(mailbox, 1, 15.0, SIDE_START[0])
    signal.step(mailbox, 15.0)  # side 0's green is over
    assert signal.current_permissions() == {1: True}


def test_invalid_phase_mode_is_rejected():
    with pytest.raises(ValueError):
        make_signal(phases="roundabout")
