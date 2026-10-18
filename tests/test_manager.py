from coop_v2x.demo import FakeAgent, Point, run_demo
from coop_v2x.intersection_manager import IntersectionManager

CENTER = Point(0.0, 0.0)


def make_agent(vehicle_id, x, y):
    """A parked fake vehicle; tests move it by assigning vehicle.position."""
    agent = FakeAgent(vehicle_id, Point(40.0, 0.0), CENTER, speed=0.0, approach=35.0)  # any valid start gives a heading
    agent.vehicle.position = Point(x, y)
    return agent


def test_cars_cross_in_arrival_order_one_at_a_time(tmp_path):
    finished = run_demo(str(tmp_path / "m.csv"), max_active=1)
    assert len(finished) == 4
    by_permission = sorted(finished, key=lambda s: s.permission_time)
    assert [s.vehicle_id for s in by_permission] == [s.vehicle_id for s in sorted(finished, key=lambda s: s.arrival_time)]
    # Safety: no two cars are ever inside the box at the same time.
    spans = sorted((s.enter_time, s.exit_time) for s in finished)
    assert all(earlier_exit <= later_enter for (_, earlier_exit), (later_enter, _) in zip(spans, spans[1:]))


def test_max_active_two_lets_two_cars_through_together(tmp_path):
    finished = run_demo(str(tmp_path / "m.csv"), max_active=2)
    first_two = sorted(finished, key=lambda s: s.arrival_time)[:2]
    assert all(s.permission_time == s.arrival_time for s in first_two)  # neither had to wait
    assert max(s.enter_time for s in first_two) < min(s.exit_time for s in first_two)  # in the box together


def test_clearing_uses_each_vehicles_own_distance():
    """Regression: the clearing check once reused the distance of whichever vehicle was seen last."""
    a, b = make_agent(0, 0, 0), make_agent(1, -20, 0)  # a is inside the box, b is queued 20 m out
    manager = IntersectionManager(CENTER, approach_radius=35.0, box_half_extent=12.0)
    manager.update([a, b], 0.0)
    assert manager.current_permissions() == {0: True}

    a.vehicle.position = Point(30.0, 0.0)  # a has left the box and is 30 m away (> 75% of the radius)
    manager.update([a, b], 1.0)
    done = manager.poll_completed()
    assert [s.vehicle_id for s in done] == [0] and done[0].exit_time == 1.0
    assert manager.current_permissions() == {1: True}  # b is next


def test_unused_permission_blocks_the_queue_unless_a_timeout_is_set():
    def scenario(timeout):
        stuck, waiting = make_agent(0, -20, 0), make_agent(1, 0, -20)  # `stuck` never moves into the box
        manager = IntersectionManager(CENTER, approach_radius=35.0, box_half_extent=12.0, permission_timeout=timeout)
        for t in range(0, 10, 3):  # t = 0, 3, 6, 9
            manager.update([stuck, waiting], float(t))
        return manager.current_permissions()

    assert scenario(None) == {0: True}  # default: the queue stays blocked
    assert scenario(5.0) == {1: True}  # revoked after 5 s, so the next car gets its turn


def test_each_finished_vehicle_is_reported_once():
    a = make_agent(0, 0, 0)
    manager = IntersectionManager(CENTER, approach_radius=35.0, box_half_extent=12.0)
    manager.update([a], 0.0)
    a.vehicle.position = Point(30.0, 0.0)
    manager.update([a], 1.0)
    assert len(manager.poll_completed()) == 1
    assert manager.poll_completed() == []


def test_a_vehicle_removed_from_the_world_frees_its_permission():
    a, b = make_agent(0, -20, 0), make_agent(1, 0, -20)
    manager = IntersectionManager(CENTER, approach_radius=35.0, box_half_extent=12.0)
    manager.update([a, b], 0.0)
    assert manager.current_permissions() == {0: True}
    manager.update([b], 1.0)  # a was destroyed
    assert manager.current_permissions() == {1: True}
