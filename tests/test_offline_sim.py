import pytest

from coop_v2x.demo import run
from coop_v2x.junction import movements_conflict
from coop_v2x.offline_sim import Geometry


def conflicting_overlaps(states):
    """Pairs of conflicting movements that were inside the box (entry until clear) at the same time."""
    found = 0
    for i, a in enumerate(states):
        for b in states[i + 1 :]:
            overlap = a.enter_time < b.exit_time and b.enter_time < a.exit_time
            if overlap and movements_conflict(a.side, a.intent, b.side, b.intent):
                found += 1
    return found


def test_every_car_finishes_with_consistent_timestamps():
    finished, summary = run("conflict-aware")
    assert len(finished) == 16 and summary["incomplete"] == 0
    for s in finished:
        assert s.arrival_time <= s.permission_time <= s.enter_time <= s.exit_time


def test_the_conflict_aware_manager_never_lets_conflicting_movements_share_the_box():
    finished, _ = run("conflict-aware")
    assert conflicting_overlaps(finished) == 0


def test_the_split_phase_signal_is_conflict_free_but_two_phases_are_not():
    safe, _ = run("signal", phases="split")
    generous, _ = run("signal", phases="axis")
    assert conflicting_overlaps(safe) == 0
    assert conflicting_overlaps(generous) > 0  # why the split signal is the fair baseline


def test_fcfs_lets_one_car_through_at_a_time():
    finished, _ = run("fcfs")
    spans = sorted((s.enter_time, s.exit_time) for s in finished)
    assert len(finished) == 16
    assert all(earlier_exit <= later_enter for (_, earlier_exit), (later_enter, _) in zip(spans, spans[1:]))


def test_conflict_aware_clears_the_queue_faster_than_one_at_a_time():
    _, serial = run("fcfs")
    _, aware = run("conflict-aware")
    assert aware["clearance_s"] < serial["clearance_s"]
    assert aware["throughput_per_min"] > serial["throughput_per_min"]


@pytest.mark.parametrize("approach", [15.0, 17.5, 20.0, 27.3, 34.8, 34.9, 35.0, 40.0])
@pytest.mark.parametrize("speed", [3.5, 7.0])
def test_no_stop_point_position_can_strand_a_car(approach, speed):
    """Regression: a car that braked just outside the approach radius used to wait forever, unseen."""
    finished, summary = run("conflict-aware", speed=speed, geometry=Geometry(approach=approach))
    assert len(finished) == 16 and summary["incomplete"] == 0


def test_the_run_writes_a_metrics_csv_that_matches_the_summary(tmp_path):
    path = str(tmp_path / "m.csv")
    finished, summary = run("conflict-aware", logfile=path)
    rows = open(path).read().strip().splitlines()
    assert len(rows) == 1 + len(finished) and summary["vehicles"] == len(finished)
