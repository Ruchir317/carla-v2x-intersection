import pytest

from coop_v2x.junction import (
    Intent,
    approach_side,
    classify_intent,
    movement_path,
    movements_block,
    movements_conflict,
    segments_intersect,
    to_math_frame,
)

I = Intent
MOVES = [(side, intent) for side in range(4) for intent in I]


def conflict(a, b):
    return movements_conflict(a[0], a[1], b[0], b[1])


@pytest.mark.parametrize(
    "a,b,expected",
    [
        ((3, I.STRAIGHT), (2, I.STRAIGHT), True),  # crossing straights
        ((3, I.STRAIGHT), (1, I.STRAIGHT), False),  # opposing straights pass each other
        ((3, I.STRAIGHT), (1, I.LEFT), True),  # a left turn crosses oncoming traffic
        ((3, I.RIGHT), (2, I.STRAIGHT), True),  # a right turn merges into the cross street's lane
        ((3, I.RIGHT), (1, I.STRAIGHT), False),
        ((3, I.RIGHT), (0, I.STRAIGHT), False),
        ((3, I.RIGHT), (1, I.RIGHT), False),  # two right turns leave by different roads
        ((3, I.STRAIGHT), (3, I.LEFT), False),  # same side: one lane, handled by `block`
        ((3, I.LEFT), (1, I.LEFT), True),  # opposing lefts: conservative with sharp corners
    ],
)
def test_known_conflicts(a, b, expected):
    assert conflict(a, b) is expected


def test_conflicts_are_symmetric_and_independent_of_orientation():
    for a in MOVES:
        for b in MOVES:
            assert conflict(a, b) == conflict(b, a)
            assert conflict(a, b) == conflict(((a[0] + 1) % 4, a[1]), ((b[0] + 1) % 4, b[1]))


def test_same_side_vehicles_block_each_other_but_do_not_conflict():
    assert not movements_conflict(3, I.STRAIGHT, 3, I.RIGHT)
    assert movements_block(3, I.STRAIGHT, 3, I.RIGHT)
    assert not movements_block(3, I.STRAIGHT, 1, I.STRAIGHT)


def test_movement_paths_start_on_the_entry_edge_and_end_on_the_exit_edge():
    path = movement_path(3, I.LEFT, 12.0, 3.5)  # from below, turning left
    assert path[0] == pytest.approx((3.5, -12.0))
    assert path[-1] == pytest.approx((-12.0, 3.5))


def test_a_right_turn_exits_through_the_right_hand_edge():
    path = movement_path(3, I.RIGHT, 12.0, 3.5)  # from below, turning right
    assert path[0] == pytest.approx((3.5, -12.0))
    assert path[1] == pytest.approx((3.5, -3.5))
    assert path[-1] == pytest.approx((12.0, -3.5))


@pytest.mark.parametrize(
    "dx,dy,side", [(30, -2, 0), (-1, 30, 1), (-30, 2, 2), (0, -30, 3), (30, 29, 0), (29, 30, 1)]
)
def test_approach_side(dx, dy, side):
    assert approach_side(dx, dy) == side


def test_a_rotated_junction_can_be_aligned_with_the_yaw_option():
    assert approach_side(20, 20, yaw_deg=45) == 0
    assert approach_side(-20, 20, yaw_deg=45) == 1


def test_carla_y_axis_is_flipped():
    # CARLA's +y points to the right, so it is "down" in the math frame.
    assert to_math_frame(10.0, 30.0, 10.0, 0.0) == (0.0, -30.0)
    assert approach_side(*to_math_frame(10.0, 30.0, 10.0, 0.0)) == 3


@pytest.mark.parametrize(
    "approach,depart,intent",
    [((0, 1), (0, 1), I.STRAIGHT), ((0, 1), (-1, 0), I.LEFT), ((0, 1), (1, 0), I.RIGHT), ((0, 1), (0.3, 1), I.STRAIGHT)],
)
def test_classify_intent(approach, depart, intent):
    assert classify_intent(approach, depart) == intent


def test_segment_intersection_cases():
    assert segments_intersect((0, 0), (2, 2), (0, 2), (2, 0))  # crossing
    assert segments_intersect((0, 0), (2, 0), (1, 0), (3, 0))  # collinear overlap
    assert segments_intersect((0, 0), (1, 0), (1, 0), (1, 1))  # touching at an end
    assert not segments_intersect((0, 0), (1, 0), (2, 0), (3, 0))  # collinear, apart
    assert not segments_intersect((0, 0), (1, 0), (0, 1), (1, 1))  # parallel
